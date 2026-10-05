#!/usr/bin/env python3
# Console to the machine over TCP using the Makera frame protocol. Needs prompt_toolkit.
# Full screen: replies scroll above the input (PageUp/PageDown), the machine status sits below it.
# While a file or script runs, its lines around the current one show beside the log when the terminal
# is wide, above the input when it is tall; "job watch" names them, and each file is downloaded once.
# The running line is highlighted.
# Typed and pasted lines queue up and go out one at a time: each waits until the machine has taken
# the one before, so a paste never overruns its 128-byte line buffer. The prompt stays live meanwhile.
# Ctrl-X aborts, Ctrl-P holds, Ctrl-O resumes: sent the moment the key is pressed, no Enter needed.
# "?" "!" "~" and ^X typed as text also go out at once as realtime bytes, ahead of the queue.
# Ctrl-C drops the queue, or cancels a running transfer; with neither it clears the line. Ctrl-D quits.
# While a transfer runs the machine reads nothing else, so realtime keys are refused and lines wait.
# A dropped or silent connection is reopened on its own, so "reset" comes back by itself.
# Local commands start with "/", see /help.
# Tab completes commands, local paths after /upload and paths on the machine (listed on first Tab).
# History lives in ~/.carvera_cli_history; Up/Down, Ctrl-R and the usual line editing come from prompt_toolkit.
import argparse
import collections
import glob
import os
import re
import socket
import threading
import time

from prompt_toolkit.application import Application
from prompt_toolkit.buffer import Buffer
from prompt_toolkit.completion import Completer, Completion
from prompt_toolkit.data_structures import Point, Size
from prompt_toolkit.history import FileHistory
from prompt_toolkit.key_binding import KeyBindings
from prompt_toolkit.layout import (DynamicContainer, Float, FloatContainer, HSplit, Layout, VSplit, Window)
from prompt_toolkit.layout.controls import BufferControl, FormattedTextControl
from prompt_toolkit.layout.dimension import Dimension
from prompt_toolkit.layout.menus import CompletionsMenu
from prompt_toolkit.layout.processors import BeforeInput
from prompt_toolkit.lexers import Lexer
from prompt_toolkit.styles import Style
from prompt_toolkit.widgets import SearchToolbar

from upload import frame, Closed, Download, FrameReader, Upload, INFO, CTRL_MULTI, CAN

CTRL_SINGLE = 0xA1
STATUS = 0x81
LOAD_INFO, LOAD_FINISH, LOAD_ERROR, JOB = 0x83, 0x84, 0x85, 0x86
LOAD_DONE = 'Load directory finished.'  # the machine's word that a listing ended, not a line of it
FETCHED = 'Info: Download success:'  # follows each download the pane asked for
# upload and download are left out: typed, they start a transfer this console does not drive
SHELL = ('ls cd pwd cat echo rm mv mkdir reset dfu break help ftype version model mem task get '
         'set_temp switch net ap wlan diagnose sleep power remount calc_thermistor thermistors time test '
         'play progress abort suspend resume step goto job trace macro').split()
REMOTE_PATH = 'ls cd cat rm mv mkdir play'.split()  # commands taking a path on the machine
SUBCOMMANDS = {'job': 'status watch load', 'step': 'over out', 'trace': 'on off', 'macro': 'list params'}
REALTIME = {'?': b'?', '!': b'!', '~': b'~', '^X': b'\x18', '^x': b'\x18'}
REALTIME_NAMES = {b'\x18': '^X abort', b'!': 'feed hold', b'~': 'resume', b'?': 'status'}
MODAL_PRINTERS = (['$I'], ['get', 'state'])  # print a [G...] line of their own, like the $G receipt
LOCAL = {
    '/upload <local> [<remote>]': 'send a file, default /sd/gcodes/<name>',
    '/reconnect': 'drop the connection and open a new one',
    '/help': 'this list',
    '/quit': 'leave, as Ctrl-D does',
}
DEFAULT_PORT = 2222
# FrameConsole's line buffer holds 127 bytes: a line, its newline and the "$G\n" behind it must fit
LINE_MAX = 120
POLL_S = 0.5  # status poll, also keeps the machine from dropping an idle connection (wifi.tcp_timeout_s)
MODAL_S = 2.0  # $G poll for the modal state, which the status frame does not carry
CONNECT_S = 5.0
SEND_S = 10.0  # a send the machine does not take in this long ends the link
SILENT_S = 15.0  # polls answer twice a second, so this much silence means the link is gone
RETRY_S = (0.5, 1, 2, 4)  # backoff between attempts, the last one repeats
STABLE_S = 5.0  # a link that lived this long starts the backoff over
START_S = 10.0  # a transfer the machine has not begun by then was refused without a word
CANCEL_S = 5.0  # a cancel the machine has not answered by then is given up on
STUCK_S = 30.0  # a typed line with no receipt this long, while idle and quiet, is pointed out
RESYNC_S = 8.0  # one of our own queries unanswered this long, with nothing else heard, lost its reply
REFETCH_S = 30.0  # a file that could not be fetched is asked for again after this long
HISTORY_FILE = os.path.expanduser('~/.carvera_cli_history')
STATE_STYLE = {'Idle': 'idle', 'Run': 'run', 'Home': 'run', 'Jog': 'run', 'Hold': 'hold', 'Pause': 'hold',
               'Wait': 'hold', 'Alarm': 'alarm', 'Sleep': 'dim'}
STYLE = Style.from_dict({
    'bottom-toolbar': 'noreverse',
    'link': 'bold',
    'down': '#ff5555 bold',
    'idle': '#55cc55 bold',
    'run': '#55aaff bold',
    'hold': '#ddbb33 bold',
    'alarm': '#ff5555 bold',
    'dim': '#888888',
    'sep': '#444444',
    'prompt': '#55aaff bold',
    'header': 'bg:#2d2d30 #dddddd bold',
    'current': 'bg:#264f78',
    'caller': 'bg:#3a3d41',
    'gutter': '#606060',
    'comment': '#6a9955 italic',
    'g': '#569cd6 bold',
    'm': '#c586c0 bold',
    'tool': '#ce9178 bold',
    'feed': '#4ec9b0',
    'axis': '#dcdcaa',
    'lineno': '#808080',
    'oword': '#c586c0 italic',
    'param': '#9cdcfe',
    'cmd': '#4fc1ff bold',
})


class Lost(Exception):
    pass


class Stopped(Exception):
    pass


class Rejected(Exception):
    pass


class Ask:
    def __init__(self, kind, own_replies=0):
        # a typed line's receipt: 'show' (a typed $G) or 'hide'; our own queries: 'poll', 'watch', 'ls'
        self.kind = kind
        self.own_replies = own_replies  # [G lines the command itself prints before the $G's
        self.since = time.monotonic()


# P:lines,percent,seconds and the rest of <State|Key:v|...> as a dict
def parse_status(status):
    parts = status.strip('<>').split('|')
    return parts[0], dict(p.split(':', 1) for p in parts[1:] if ':' in p)


# a job frame: "<role> <phase> <outcome> <flags> <line> <read> <secs> <size> <path>" per level,
# outermost first, each perhaps followed by "args ..."
Head = collections.namedtuple('Head', 'role phase line size path')


def parse_job(text):
    heads = []
    for line in text.split('\n')[:-1]:  # a line without its end was cut off by the machine's buffer
        f = line.rstrip('\r').split(None, 8)
        if len(f) == 9 and f[0] in ('file', 'script'):
            heads.append(Head(f[0], f[1], int(f[4]), int(f[7], 16) if f[7] != '-' else 0, f[8]))
    return heads


class Console:
    def __init__(self, host, port, show=None):
        self.host, self.port = host, port
        if show is not None:
            self.show = show
        self.lock = threading.Lock()  # guards sock, the per-link state and every send
        # wakes the sender on replies, transfer ends, link changes and Ctrl-C; never held while taking lock
        self.cond = threading.Condition()
        self.sock = None
        self.link = 'connecting'
        self.stopping = False
        self.drop_reason = None  # set when we end the link ourselves, the reader only sees it close
        self.generation = 0  # counts link changes, so a wait knows its link is gone
        self.status = ''
        self.modal = ''  # [G0 G54 G17 G21 G90 G94 M0 M5 M9 T0 F0. S0.] from $G
        self.listing = {}  # remote directory -> entries, filled by list_remote for completion
        self.listing_dir = None  # directory an ls for completion is running for
        self.queue = collections.deque()  # lines and transfers waiting their turn
        self.current = None  # the item the sender is on
        self.current_sent = False  # it went out and waits for its receipt
        self.rejected = None  # an error the machine printed while the current line waited
        self.stops = 0  # Ctrl-C count, a wait started before a stop gives up
        self.heads = []  # what runs, from the job frames
        self.texts = {}  # (path, size) -> its lines; None while fetching, False when that failed
        self.failed = {}  # (path, size) -> when its fetch failed
        self.want_program = False  # the UI has room for the program pane
        self.last_modal = 0.0
        self.heard = time.monotonic()  # last frame other than a status
        self.reset_session()
        for target in (self.supervisor, self.poller, self.sender):
            threading.Thread(target=target, daemon=True).start()

    # per-link state, under lock: replies owed on the old link never arrive on the new one
    def reset_session(self):
        self.want_status = False  # show the next status: a typed ? answers the same way as our polls
        self.asks = collections.deque()  # the Ask in flight, at most one
        self.watching = False  # "job watch on" went out on this link
        self.asked = 0  # $G sent on this link
        self.answered = 0  # $G replies seen; the nth reply is the receipt of the line before the nth $G
        self.transfer = None
        if self.listing_dir is not None:
            self.listing.pop(self.listing_dir, None)
            self.listing_dir = None

    @property
    def connected(self):
        return self.sock is not None

    @property
    def state(self):
        return parse_status(self.status)[0] if self.status.startswith('<') else ''

    def send(self, data):
        with self.lock:
            self.send_locked(data)

    def send_locked(self, data):
        if self.sock is None:
            raise OSError('not connected')
        try:
            self.sock.sendall(data)
        except OSError as e:
            self.drop_locked(f'send failed: {e}')
            raise

    def drop_locked(self, reason):
        # the reader's recv returns empty and the supervisor takes over
        self.drop_reason = self.drop_reason or reason
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

    def drop(self, reason):
        with self.lock:
            if self.sock is not None:
                self.drop_locked(reason)

    def wake(self):
        with self.cond:
            self.cond.notify_all()

    def show(self, text):
        print(text)

    def supervisor(self):
        attempt = 0
        last_error = None
        while not self.stopping:
            self.link = f'connecting to {self.host}:{self.port}'
            if attempt:
                self.link += f' (attempt {attempt + 1})'
            started = time.monotonic()
            try:
                reason = self.session()
            except Exception as e:  # a refused connect, a bad address or a bug: none may end the retries
                error = str(e) or type(e).__name__
                if error != last_error:
                    self.show(f'cannot connect to {self.host}:{self.port}: {error}')
                    last_error = error
            else:
                last_error = None
                if self.stopping:
                    break
                self.show(f'connection lost: {reason}, reconnecting')
                if time.monotonic() - started >= STABLE_S:
                    attempt = 0
            # also after a lost link: a machine that accepts and closes at once must not be hammered
            time.sleep(RETRY_S[min(attempt, len(RETRY_S) - 1)])
            attempt += 1

    # one connection, from connect to loss; returns why it ended
    def session(self):
        sock = socket.create_connection((self.host, self.port), timeout=CONNECT_S)
        try:
            sock.settimeout(SEND_S)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.lock:
                self.reset_session()
                self.sock = sock
                self.drop_reason = None
            self.link = f'{self.host}:{self.port}'
            self.show(f'connected to {self.host}:{self.port}')
            with self.cond:
                self.cond.notify_all()  # lines held for the link go now
            return self.reader(sock)
        finally:
            with self.lock:
                self.sock = None
                up, self.transfer = self.transfer, None
            sock.close()
            with self.cond:
                self.generation += 1
                self.cond.notify_all()
            if up is not None:
                up.result = up.result or 'interrupted'
                if isinstance(up, Upload):
                    self.show(f'upload interrupted after {up.progress.done} of {up.progress.size} bytes')

    def reader(self, sock):
        r = FrameReader(sock)
        try:
            while True:
                ftype, payload = r.next(SILENT_S)
                if ftype is None:
                    return f'no reply for {SILENT_S:.0f} s'
                if ftype != STATUS:
                    self.heard = time.monotonic()
                try:
                    self.on_frame(ftype, payload)
                except OSError:
                    raise
                except Exception as e:  # one bad frame must not take the link down
                    self.show(f'could not handle frame {ftype:02x}: {e!r}')
        except Closed:
            return self.drop_reason or 'closed by the machine'
        except OSError as e:
            return self.drop_reason or str(e)

    def on_frame(self, ftype, payload):
        up = self.transfer
        if up is not None:
            reply = up.answer(ftype, payload)
            if reply:
                self.send(reply)
            if reply is None and up.result is not None:
                self.print_frame(ftype, payload)  # the machine's own word before ours
            if up.result is not None and isinstance(up, Upload):
                self.end_transfer(up, {'done': f'upload complete, {up.progress.summary()}',
                                       'failed': 'upload failed', 'refused': 'upload refused',
                                       'cancelled': 'upload cancelled by machine'}[up.result])
            elif up.result is not None:
                self.end_transfer(up, None)
            if reply is not None or up.result is not None:
                return
        if self.listing_dir is not None and ftype in (LOAD_INFO, LOAD_FINISH, LOAD_ERROR):
            if ftype == LOAD_INFO:
                lines = payload.decode(errors='replace').split('\n')
                entries = self.listing.setdefault(self.listing_dir, [])
                entries += [e.split()[0] for e in lines if e.strip()]
            else:
                self.listing_dir = None
            return
        if ftype == JOB:
            self.heads = parse_job(payload.decode(errors='replace'))
            return
        text = payload.decode(errors='replace').rstrip()
        ask = self.asks[0] if self.asks else None
        if ftype == INFO and ask is not None and ask.kind == 'watch' and text == 'job watch on':
            return
        # the modal state; "$#" and "get wcs" print [G54:x,y,z] and the like, which carry a colon
        if ftype == INFO and text.startswith('[G') and ':' not in text:
            self.modal = text
            if ask is not None and ask.own_replies:
                ask.own_replies -= 1  # "$I" or "get state" answering for itself; the $G's comes after
            elif ask is not None:
                self.asks.popleft()
                if ask.kind == 'ls':
                    self.listing_dir = None  # an ls that printed no LOAD_FINISH ends here all the same
                with self.cond:
                    self.answered += 1
                    self.cond.notify_all()
                if ask.kind != 'show':
                    return
        typed = ask is not None and ask.kind in ('show', 'hide')
        if ftype == INFO and typed and text.lower().startswith(('error', 'alarm')):
            self.rejected = self.rejected or text  # errors go to every console; any of them stops a paste
        if ftype == STATUS:
            self.status = text
            if not self.want_status:
                return
            self.want_status = False
        self.print_frame(ftype, payload)

    # the levels the pane shows: a finished job is no longer running
    def shown_heads(self):
        heads = self.heads
        return heads if heads and heads[0].phase != 'idle' else []

    # a download for the pane has ended, one way or the other
    def fetched(self, dl):
        if dl.result == 'done':
            # numbered as the machine numbers them: by newline alone
            self.texts[dl.key] = [l.rstrip('\r') for l in dl.data.decode(errors='replace').split('\n')]
            return
        if dl.key not in self.failed:
            self.show(f'could not fetch {dl.remote}: {dl.result}')
        self.texts[dl.key] = False
        self.failed[dl.key] = time.monotonic()

    # each file a head names, once
    def fetch_missing(self):
        now = time.monotonic()
        for h in self.shown_heads():
            key = (h.path, h.size)
            if key not in self.texts or (self.texts[key] is False
                                         and now - self.failed[key] >= REFETCH_S):
                self.texts[key] = None
                self.enqueue(Download(h.path, key))

    def print_frame(self, ftype, payload):
        text = payload.decode(errors='replace').replace('\r', '').rstrip()
        if ftype == LOAD_FINISH and text == LOAD_DONE:
            return
        if ftype == INFO and text.startswith(FETCHED):
            return
        if text:
            self.show(text)

    def poller(self):
        while not self.stopping:
            time.sleep(POLL_S)
            try:
                self.poll()
            except Exception as e:
                self.show(f'poll failed: {e!r}')

    # a file plays or a script runs: the program pane has something to show
    @property
    def running(self):
        return '|P:' in self.status or self.state in ('Run', 'Hold', 'Pause', 'Wait')

    def poll(self):
        up = self.transfer
        now = time.monotonic()
        if up is not None:
            if up.cancel_sent is not None and now - up.cancel_sent > CANCEL_S:
                self.end_transfer(up, f'{up.kind} cancel not answered by the machine, dropped here')
            elif not up.started and now - up.began > START_S:
                self.cancel_transfer(up, f'{up.kind} not begun by the machine within {START_S:.0f} s')
            return
        if not self.connected:
            return
        ask = self.asks[0] if self.asks else None
        # our own queries are answered at once and only go out when the line before was taken, so one
        # unanswered while nothing else arrives lost its reply; a reconnect starts the count over
        if (ask is not None and ask.kind in ('poll', 'watch', 'ls') and not self.running
                and now - ask.since > RESYNC_S and now - self.heard > RESYNC_S):
            self.drop('a reply went missing, reconnecting to resync')
            return
        try:
            self.poll_status()
            if not self.watching:
                if self.ask('job watch on', 'watch') is not None:
                    self.watching = True
            elif self.want_program:
                self.fetch_missing()
            if now - self.last_modal >= MODAL_S:
                if self.ask(None, 'poll') is not None:
                    self.last_modal = now
        except OSError:
            pass  # the supervisor reports it

    def realtime(self, byte):
        up = self.transfer
        if up is not None and up.started:
            if byte == b'?':
                raise OSError(f'{up.kind} running, the machine reads nothing else until it ends')
            # the machine hears nothing else during a transfer: end it, then send the key
            threading.Thread(target=self.realtime_after_cancel, args=(up, byte), daemon=True).start()
            if up.ended:
                return f'{up.kind} finishing on the machine, the key follows when it is done'
            return f'{up.kind} cancelled first, the key follows'
        with self.lock:
            if byte == b'?':
                self.want_status = True
            try:
                self.send_locked(frame(CTRL_SINGLE, byte))
            except OSError:
                self.want_status = False
                raise
        return None

    def realtime_after_cancel(self, up, byte):
        if not up.ended:
            self.cancel_transfer(up, f'{up.kind} cancelled for a realtime key')
        with self.cond:
            self.cond.wait_for(lambda: self.transfer is not up, SILENT_S if up.ended else CANCEL_S + 1)
        try:
            self.realtime(byte)
            self.show(f'{REALTIME_NAMES.get(byte, byte)} sent')
        except OSError as e:
            self.show(f'failed: {e}')

    def poll_status(self):
        self.send(frame(CTRL_SINGLE, b'?'))

    # sends line (None for a bare poll) with a $G behind it; the machine takes lines in order,
    # so the $G's reply is the line's receipt. One at a time, so the machine's 127-byte line buffer
    # never holds more than a line and its $G. Returns the answered count that marks the receipt,
    # None while another is in flight or a transfer holds the machine.
    def ask(self, line, kind):
        with self.lock:
            if self.asks or self.transfer is not None:
                return None
            if kind in ('show', 'hide'):
                self.rejected = None  # from here on an error is this line's
            if line is not None and line != '$G':
                self.send_locked(frame(CTRL_MULTI, line.encode()))
            self.asks.append(Ask(kind, 1 if line is not None and line.split() in MODAL_PRINTERS else 0))
            try:
                self.send_locked(frame(CTRL_MULTI, b'$G'))
            except OSError:
                self.asks.pop()
                raise
            self.asked += 1
            return self.asked

    def list_remote(self, directory):
        if self.listing_dir is not None or not self.connected or self.transfer is not None:
            return
        if len(f'ls {directory}'.encode()) > LINE_MAX:
            return
        self.listing[directory] = []
        self.listing_dir = directory
        try:
            sent = self.ask(f'ls {directory}', 'ls')
        except OSError:
            sent = None
        if sent is None:  # busy: the next Tab asks again
            self.listing_dir = None
            self.listing.pop(directory, None)

    def start_transfer(self, up, stops):
        with self.lock:
            # the slot is empty, so every line before it was taken and FILE_START is next in line
            if self.stops != stops:
                raise Stopped
            if self.sock is None:
                raise OSError('not connected')
            if self.asks or self.transfer is not None:
                return False
            frames = up.start()
            up.cancel_sent = None
            self.transfer = up
            try:
                for f in frames:
                    self.send_locked(f)
            except OSError:
                self.transfer = None
                raise
        if isinstance(up, Upload):
            self.show(f'uploading -> {up.remote}: {len(up.data)} bytes, {up.total} packets')
        return True

    def end_transfer(self, up, message):
        with self.lock:
            if self.transfer is not up:
                return
            self.transfer = None
        up.result = up.result or 'cancelled'
        if up.result == 'done' and isinstance(up, Upload):
            self.listing.clear()  # the card changed
            self.texts = {k: v for k, v in self.texts.items() if k[0] != up.remote}
        if message:
            self.show(message)
        self.wake()

    # the machine ends it and says so; only one that never began is ended here
    def cancel_transfer(self, up, message):
        if not up.started:
            try:
                self.send(frame(CAN))
            except OSError:
                pass
            self.end_transfer(up, message)
        elif up.ended:
            self.show(f'{up.kind} finishing on the machine, it cannot be cancelled now')
        elif up.cancel_sent is None:
            up.cancel_sent = time.monotonic()
            try:
                self.send(frame(CAN))
                self.show(f'{message}, waiting for the machine')
            except OSError:
                pass  # the lost link ends it

    def enqueue(self, item):
        with self.cond:
            self.queue.append(item)
            self.cond.notify_all()

    # Ctrl-C: returns what it did, or None when there was nothing to stop
    def interrupt(self):
        with self.cond:
            fetches = [i for i in self.queue if isinstance(i, Download)]
            dropped = len(self.queue) - len(fetches)
            self.queue.clear()
            current = self.current
            sent = self.current_sent
            self.stops += 1
            self.cond.notify_all()
        for dl in fetches:
            dl.result = 'cancelled'
            self.fetched(dl)
        up = self.transfer
        if up is not None:
            self.cancel_transfer(up, f'{up.kind} cancelled')
        if isinstance(current, str) and sent:
            return f'stopped: "{current}" went out, not yet taken; {dropped} queued not sent'
        if current is not None and current is not up and not isinstance(current, Download):
            dropped += 1
        if dropped:
            return f'stopped: {dropped} queued not sent'
        return None

    def sender(self):
        while not self.stopping:
            with self.cond:
                while not self.queue:
                    self.cond.wait()
                item = self.queue.popleft()
                self.current = item
                self.current_sent = False
                stops = self.stops
            try:
                self.run(item, stops)
            except Stopped:
                pass  # interrupt() has said so
            except Exception as e:
                with self.cond:
                    dropped = len(self.queue)
                    self.queue.clear()
                what = item if isinstance(item, str) else f'{item.kind} {item.remote}'
                rest = f'; {dropped} queued not sent' if dropped else ''
                # a reset or dfu ends the link by design, and an error the machine printed is on screen:
                # alone they need no word
                ends_link = isinstance(e, Lost) and what.split()[0] in ('reset', 'dfu')
                quiet = ends_link or isinstance(e, Rejected)
                if dropped or not quiet:
                    self.show(f'failed: "{what}": {e}{rest}')
            finally:
                self.current = None
                self.current_sent = False

    def run(self, item, stops):
        # lines wait for the link, a transfer and the slot rather than fail
        free = lambda: self.connected and not self.asks and self.transfer is None
        if isinstance(item, (Upload, Download)):
            try:
                while True:
                    self.wait_until(free, stops, None)
                    if self.start_transfer(item, stops):
                        break
                self.wait_until(lambda: self.transfer is not item, stops, None)
            finally:
                if isinstance(item, Download):
                    item.result = item.result or 'cancelled'
                    self.fetched(item)
            if isinstance(item, Upload) and item.result != 'done':
                raise Rejected(f'upload {item.result}')  # what follows may count on the file
            return
        while True:
            self.wait_until(free, stops, None)
            generation = self.generation
            try:
                receipt = self.ask(item, 'show' if item == '$G' else 'hide')
            except OSError:
                if not self.connected:
                    continue  # the link went between the wait and the send; it comes back
                raise
            if receipt is not None:
                break
        self.current_sent = True
        if item != '$G':
            try:
                self.poll_status()  # after the line, so the status shows its effect
            except OSError:
                pass
        self.wait_until(lambda: self.answered >= receipt, stops, generation)
        if self.rejected:
            raise Rejected(f'the machine answered "{self.rejected}"')

    def wait_until(self, done, stops, generation):
        with self.cond:
            while True:
                # the link first: a new link's counts start over and could pass for this one's receipt
                if generation is not None and self.generation != generation:
                    raise Lost('the link went down before the machine confirmed it')
                if done():
                    return
                if self.stops != stops:
                    raise Stopped
                self.cond.wait(POLL_S)

    def close(self):
        self.stopping = True
        self.drop('closed')


# P:lines,percent,seconds from the status while a file plays
def play_progress(field):
    played, percent, secs = (int(v) for v in field.split(',')[:3])
    return f'line {played}   {percent}%   {secs // 3600:02}:{secs // 60 % 60:02}:{secs % 60:02}'


def status_block(con, progress=True):
    # <Idle|MPos:x,y,z,a|WPos:x,y,z,a|F:cur,set,ovr|S:cur,set,ovr,..|T:n,tlo|H:reason> -> styled lines
    out = []
    live = con.connected
    body = '' if live else 'class:dim'
    if live:
        out.append(('class:link', con.link))
    else:
        out.append(('class:down', con.link))
    status = con.status
    if not status.startswith('<'):
        out.append((body, '\n' + (status or 'no status yet')))
    else:
        state, fields = parse_status(status)
        out.append(('', '   '))
        out.append(('class:' + STATE_STYLE.get(state, 'link') if live else body, state))
        if 'H' in fields:
            out.append((body, f'   halt reason {fields["H"]}'))
        lines = []
        axes = fields.get('MPos', '').split(',')
        if axes and axes[0]:
            lines.append('      ' + ''.join(f'{a:>11}' for a in 'XYZAB'[:len(axes)]))
            for key in ('MPos', 'WPos'):
                if key in fields:
                    lines.append(f'{key:5} ' + ''.join(f'{float(v):11.3f}' for v in fields[key].split(',')))
        info = []
        if 'F' in fields:
            f = fields['F'].split(',')
            info.append(f'F {float(f[0]):.0f} / {float(f[1]):.0f} mm/min'
                        + (f'  {float(f[2]):.0f}%' if len(f) > 2 else ''))
        if 'S' in fields:
            sp = fields['S'].split(',')
            info.append(f'S {float(sp[0]):.0f} / {float(sp[1]):.0f} rpm'
                        + (f'  {float(sp[2]):.0f}%' if len(sp) > 2 else ''))
        if 'T' in fields:
            t = fields['T'].split(',')
            info.append(f'T{t[0]}' + (f'  TLO {float(t[1]):.3f}' if len(t) > 1 else ''))
        if info:
            lines.append('    '.join(info))
        if 'P' in fields and progress:
            lines.append(play_progress(fields['P']))
        for line in lines:
            out.append((body, '\n' + line))
    if con.modal:
        out.append(('class:dim', '\n' + con.modal.strip('[]')))
    up = con.transfer
    if up is not None:
        out.append(('class:run', f'\n{up.kind} {up.progress.line()}   Ctrl-C cancels'))
    current, queued = con.current, len(con.queue)
    if isinstance(current, str):
        if con.current_sent:
            what = 'waiting for the machine to take'
        else:
            what = 'waiting for the link, then' if not live else 'waiting for the transfer, then'
        out.append(('class:hold', f'\n{what}: {current}'
                    + (f'   {queued} more queued' if queued else '') + '   Ctrl-C drops'))
        ask = con.asks[0] if con.asks else None
        quiet = time.monotonic() - max(con.heard, ask.since if ask else 0)
        if con.current_sent and con.state == 'Idle' and quiet > STUCK_S:
            # a dwell also sits Idle, so this is left to the user
            out.append(('class:dim', f'\nno receipt for {quiet:.0f} s while idle: /reconnect if it is lost'))
    return out


class CarveraCompleter(Completer):
    def __init__(self, con):
        self.con = con

    def get_completions(self, document, complete_event):
        words = document.text_before_cursor.split(' ')
        word = words[-1]
        if len(words) == 1:
            options = [c + ' ' for c in SHELL + [u.split()[0] for u in LOCAL] if c.startswith(word)]
        elif len(words) == 2 and words[0] in SUBCOMMANDS:
            options = [c + ' ' for c in SUBCOMMANDS[words[0]].split() if c.startswith(word)]
        elif words[0] == '/upload' and len(words) == 2:
            options = [p + '/' if os.path.isdir(p) else p for p in glob.glob(os.path.expanduser(word) + '*')]
        elif (words[0] in REMOTE_PATH or words[:2] == ['job', 'load']) and word.startswith('/'):
            directory, _, name = word.rpartition('/')
            directory = directory or '/'
            entries = self.con.listing.get(directory)  # the reader may clear it meanwhile
            if entries is None or self.con.listing_dir == directory:
                self.con.list_remote(directory)  # ready on the next Tab
                return
            base = directory.rstrip('/') + '/'
            options = [base + e for e in entries if e.startswith(name)]
        else:
            return
        for o in sorted(options):
            yield Completion(o, start_position=-len(word),
                             display=os.path.basename(o.rstrip('/ ')) + ('/' if o.endswith('/') else ''))


GCODE = re.compile(r'''
    (?P<comment>\([^)]*\)?|;.*)
  | (?P<oword>[oO](?:<[^>]*>|\d+)
        (?:\s+(?:sub|endsub|call|if|elseif|else|endif|while|endwhile|do|return
                 |repeat|endrepeat|break|continue)\b)?)
  | (?P<param>\#<[^>]*>|\#\d+)
  | (?P<open>\[) | (?P<close>\])
  | (?P<word>[A-Za-z](?:\s*[-+]?(?:\d+\.?\d*|\.\d+))?)
''', re.X)
WORD_STYLE = {'G': 'g', 'M': 'm', 'T': 'tool', 'F': 'feed', 'S': 'feed', 'N': 'lineno'}


# G-code, O-words and the parameters of scripts in colour; a shell command gets its name marked
def gcode_fragments(text, base=''):
    head = text.split(' ', 1)[0]
    if head in SHELL or head.startswith('/') or head.startswith('$'):
        return [(base + ' class:cmd', head), (base, text[len(head):])]
    out = []
    at = 0
    depth = 0  # inside [expressions] letters are operators and functions, not words
    for m in GCODE.finditer(text):
        if m.start() > at:
            out.append((base, text[at:m.start()]))
        kind = m.lastgroup
        if kind in ('open', 'close'):
            depth = max(0, depth + (1 if kind == 'open' else -1))
            kind = None
        elif kind == 'word' and depth:
            kind = None
        elif kind == 'word':
            letter = m.group()[0].upper()
            # a word has a value, a parameter or an expression
            value = len(m.group()) > 1 or text[m.end():].lstrip()[:1] in ('[', '#')
            kind = WORD_STYLE.get(letter, 'axis') if value else None
        out.append((base + (f' class:{kind}' if kind else ''), m.group()))
        at = m.end()
    out.append((base, text[at:]))
    return out


class GcodeLexer(Lexer):
    def lex_document(self, document):
        return lambda n: gcode_fragments(document.lines[n])


def log_style(text):
    low = text.lower()
    if low.startswith(('error', 'alarm', 'failed', 'halt')) or 'error:' in low:
        return 'class:alarm'
    if low.startswith(('connection lost', 'cannot connect', 'upload interrupted', 'upload refused',
                       'upload failed', 'upload cancelled', 'stopped:')):
        return 'class:hold'
    if low.startswith(('connected to', 'upload complete')):
        return 'class:idle'
    return ''


class Ui:
    LOG_MAX = 5000  # lines kept for scrolling back
    LOG_RENDER = 400  # lines handed to the window; it shows the bottom of them
    WIDE_COLS = 130  # from here the program sits beside the log
    STACK_ROWS = 28  # from here, when narrower, it sits above the input

    def __init__(self):
        self.lock = threading.Lock()
        self.log = collections.deque(maxlen=self.LOG_MAX)  # one fragment list per line
        self.scroll = 0  # lines back from the newest
        self.con = None
        self.app = None

    def attach(self, con):
        self.con = con
        self.input = Buffer(history=FileHistory(HISTORY_FILE), completer=CarveraCompleter(con),
                            complete_while_typing=False, multiline=False, accept_handler=self.accept)
        search = SearchToolbar()
        self.input_window = Window(
            # preview_search shows the Ctrl-R match as it is typed, as PromptSession does
            BufferControl(self.input, lexer=GcodeLexer(), search_buffer_control=search.control,
                          preview_search=True, input_processors=[BeforeInput('> ', style='class:prompt')]),
            height=lambda: Dimension.exact(min(10, self.input.document.line_count)), wrap_lines=True)
        self.log_window = Window(FormattedTextControl(self.log_text, get_cursor_position=self.log_end),
                                 wrap_lines=True)
        self.program_window = Window(FormattedTextControl(self.program_text), wrap_lines=False)
        # the program pane's header carries the play progress when it shows
        plain = lambda: self.layout() is self.layouts['plain']
        status = Window(FormattedTextControl(lambda: status_block(con, plain())), dont_extend_height=True)
        hline = Window(height=1, char='─', style='class:sep')
        vline = Window(width=1, char='│', style='class:sep')
        typing = [self.log_window, self.input_window, search]
        self.layouts = {
            'side': VSplit([HSplit(typing), vline,
                            HSplit([status, hline, self.program_window], width=self.side_width)]),
            'stack': HSplit([self.log_window, hline,
                             HSplit([self.program_window], height=self.stack_height), hline,
                             self.input_window, search, hline, status]),
            'plain': HSplit(typing[:2] + [search, hline, status]),
        }
        root = FloatContainer(DynamicContainer(self.layout),
                              floats=[Float(xcursor=True, ycursor=True,
                                            content=CompletionsMenu(max_height=12, scroll_offset=1))])
        self.app = Application(layout=Layout(root, focused_element=self.input_window),
                               key_bindings=self.keys(), style=STYLE, full_screen=True,
                               refresh_interval=POLL_S)

    def size(self):
        if self.app is None:  # the layout is walked once while the app is being built
            return Size(rows=24, columns=80)
        return self.app.output.get_size()

    def layout(self):
        size = self.size()
        # the files are fetched only while there is room to show them
        self.con.want_program = size.columns >= self.WIDE_COLS or size.rows >= self.STACK_ROWS
        if not self.con.shown_heads():
            return self.layouts['plain']
        if size.columns >= self.WIDE_COLS:
            return self.layouts['side']
        return self.layouts['stack' if size.rows >= self.STACK_ROWS else 'plain']

    def side_width(self):
        return Dimension.exact(min(96, self.size().columns * 45 // 100))

    def stack_height(self):
        return Dimension.exact(min(14, self.size().rows // 3))

    def say(self, text, style=None):
        lines = [[(log_style(line) if style is None else style, line)] for line in text.split('\n')]
        self.add(lines)

    def add(self, lines):
        with self.lock:
            self.log.extend(lines)
            if self.scroll:
                self.scroll = min(self.scroll + len(lines), len(self.log) - 1)  # hold the view still
        if self.app is not None:
            self.app.invalidate()
        else:
            for line in lines:
                print(''.join(t for _, t in line))

    def log_text(self):
        with self.lock:
            end = len(self.log) - self.scroll
            lines = [self.log[i] for i in range(max(0, end - self.LOG_RENDER), end)]
        out = []
        for i, line in enumerate(lines):
            out.extend(line)
            if i < len(lines) - 1:
                out.append(('', '\n'))
        if self.scroll:
            out.append(('class:hold', f'\n-- {self.scroll} newer lines below, PageDown --'))
        return out

    def log_end(self):
        return Point(0, max(0, min(len(self.log) - self.scroll, self.LOG_RENDER) - (0 if self.scroll else 1)))

    def page(self, direction):
        rows = max(1, self.log_window.render_info.window_height - 2) if self.log_window.render_info else 10
        with self.lock:
            self.scroll = max(0, min(self.scroll + direction * rows, len(self.log) - 1))
        self.app.invalidate()

    def program_text(self):
        con = self.con
        heads = con.shown_heads()
        info = self.program_window.render_info
        height = info.window_height if info else 20
        width = info.window_width if info else 80
        # as many lines as the pane holds; each source gets a header and an equal share
        per = max(3, height // max(1, len(heads)) - 1)
        around = min(30, (per - 1) // 2)
        fields = parse_status(con.status)[1]
        out = []
        for index, h in enumerate(heads):
            active = index == len(heads) - 1  # the deepest source is the one running
            head = f' {os.path.basename(h.path)}'
            if index == 0 and 'P' in fields:
                head += '   ' + play_progress(fields['P'])
            if out:
                out.append(('', '\n'))
            out.append(('class:header', head.ljust(width)))
            lines = con.texts.get((h.path, h.size))
            if lines is None or lines is False:
                out.append(('class:dim', '\n fetching' if lines is None else '\n not readable'))
                continue
            for number in range(max(1, h.line - around), min(len(lines), h.line + around) + 1):
                here = number == h.line
                base = ('class:current' if active else 'class:caller') if here else ''
                out.append(('', '\n'))
                out.append((base + ' class:gutter', f'{"▶" if here else " "}{number:>6}  '))
                out.extend(gcode_fragments(lines[number - 1], base))
                if here:
                    out.append((base, ' ' * width))  # the bar runs the full width; the window cuts it
        return out or [('class:dim', ' nothing running')]

    def accept(self, buffer):
        text = buffer.text
        self.scroll = 0
        for line in text.splitlines():
            if line.strip():
                self.add([[('class:prompt', '> ')] + gcode_fragments(line.strip())])
        try:
            enter(self.con, text)
        except Quit:
            self.app.exit()
        return False

    def keys(self):
        keys = KeyBindings()
        con = self.con

        def send(byte, what):
            try:
                self.say(con.realtime(byte) or what, 'class:hold')
            except OSError as e:
                self.say(f'failed: {e}')

        @keys.add('c-x', eager=True)  # prompt_toolkit uses c-x as a prefix, eager stops it waiting for a second key
        def _(event):
            send(b'\x18', '^X abort sent')

        @keys.add('c-p')
        def _(event):
            send(b'!', 'feed hold sent')

        @keys.add('c-o')
        def _(event):
            send(b'~', 'resume sent')

        @keys.add('c-c')
        def _(event):
            self.input.reset()
            what = con.interrupt()
            if what:
                self.say(what)

        @keys.add('c-d')
        def _(event):
            if not self.input.text:
                event.app.exit()

        @keys.add('pageup')
        def _(event):
            self.page(1)

        @keys.add('pagedown')
        def _(event):
            self.page(-1)

        return keys


class Quit(Exception):
    pass


def local_command(con, line):
    words = line.split()
    if words[0] == '/upload' and len(words) in (2, 3):
        local = os.path.expanduser(words[1])
        remote = words[2] if len(words) == 3 else '/sd/gcodes/' + os.path.basename(local)
        if len(f'upload {remote}'.encode()) > LINE_MAX:
            raise OSError(f'remote path longer than the machine takes ({LINE_MAX - 7} bytes)')
        with open(local, 'rb') as f:
            con.enqueue(Upload(f.read(), remote))
    elif words[0] == '/reconnect':
        if con.connected:
            con.drop('reconnect asked for')
        else:
            con.show('not connected, already retrying')
    elif words[0] in ('/quit', '/exit'):
        raise Quit
    else:
        con.show('\n'.join(f'{usage:28}{text}' for usage, text in LOCAL.items()))


def enter(con, text):
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            if line.startswith('/'):
                local_command(con, line)
            elif line in REALTIME:
                what = con.realtime(REALTIME[line])
                if what:
                    con.show(what)
            elif line.split()[0] in ('upload', 'download'):
                raise OSError('it starts a transfer this console does not drive; use /upload')
            elif len(line.encode()) > LINE_MAX:
                raise OSError(f'longer than the machine takes ({LINE_MAX} bytes), not sent')
            else:
                con.enqueue(line)
        except OSError as e:
            con.show(f'failed: "{line}": {e}')


def port_number(text):
    port = int(text)
    if not 0 < port < 65536:
        raise argparse.ArgumentTypeError(f'{text} is not a port')
    return port


def main():
    p = argparse.ArgumentParser(description='Console to the machine (Makera frame protocol).')
    p.add_argument('host')
    p.add_argument('port', type=port_number, nargs='?', default=DEFAULT_PORT)
    args = p.parse_args()
    try:
        args.host.encode('idna')
    except UnicodeError:
        p.error(f'{args.host} is not a host name')
    ui = Ui()
    con = Console(args.host, args.port, show=ui.say)
    ui.attach(con)
    ui.app.run()
    con.close()
    # the full screen is gone with the app: leave the last of the log in the terminal
    for line in list(ui.log)[-20:]:
        print(''.join(t for _, t in line))


if __name__ == '__main__':
    main()
