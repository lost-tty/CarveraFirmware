#!/usr/bin/env python3
# Console to the machine over TCP using the Makera frame protocol. Needs prompt_toolkit.
# Replies scroll above the prompt, the machine status sits below it. Pasted text is sent line by line.
# Ctrl-X aborts, Ctrl-P holds, Ctrl-O resumes: sent the moment the key is pressed, no Enter needed.
# "?" "!" "~" and ^X typed as text also go out as realtime bytes.
# /upload <local> [<remote>] sends a file, default /sd/gcodes/<name>; "play <remote>" then runs it.
# Tab completes commands, local paths after /upload and paths on the machine (listed on first Tab).
# History lives in ~/.carvera_cli_history; Up/Down, Ctrl-R and the usual line editing come from prompt_toolkit.
import glob
import hashlib
import os
import socket
import struct
import sys
import threading
import time

from prompt_toolkit import PromptSession
from prompt_toolkit.completion import Completer, Completion
from prompt_toolkit.history import FileHistory
from prompt_toolkit.key_binding import KeyBindings
from prompt_toolkit.patch_stdout import patch_stdout

from upload import frame, FrameReader, INFO, CTRL_MULTI, FILE_START, MD5, VIEW, DATA, END, CAN, PACKET_SIZE

CTRL_SINGLE = 0xA1
STATUS = 0x81
LOAD_INFO, LOAD_FINISH, LOAD_ERROR = 0x83, 0x84, 0x85
NAMES = {0x82: 'diag', LOAD_INFO: 'load', LOAD_FINISH: 'load-end', LOAD_ERROR: 'load-err', INFO: ''}
SHELL = ('ls cd pwd cat echo rm mv mkdir upload download reset dfu break help ftype version model mem task get '
         'set_temp switch net ap wlan diagnose sleep power remount calc_thermistor thermistors time test '
         'play progress abort suspend resume goto list').split()
REMOTE_PATH = 'ls cd cat rm mv mkdir upload download play'.split()  # commands taking a path on the machine
POLL_S = 0.5  # status poll, also keeps the machine from dropping an idle connection (wifi.tcp_timeout_s)
MODAL_S = 2.0  # $G poll for the modal state, which the status frame does not carry
HISTORY_FILE = os.path.expanduser('~/.carvera_cli_history')


class Console:
    def __init__(self, sock):
        self.sock = sock
        self.lock = threading.Lock()
        self.status = ''
        self.modal = ''  # [G0 G54 G17 G21 G90 G94 M0 M5 M9 T0 F0. S0.] from $G
        self.quiet_status = 0  # replies to our own polls are not shown
        self.quiet_modal = 0  # $G replies to our own polls, each followed by an ok, are not shown
        self.swallow_ok = False
        self.closed = False
        self.upload = None  # {data, md5, total, sent} while a transfer runs
        self.listing = {}  # remote directory -> entries, filled by list_remote for completion
        self.listing_dir = None  # directory an ls for completion is running for
        threading.Thread(target=self.reader, daemon=True).start()
        threading.Thread(target=self.poller, daemon=True).start()

    def send(self, data):
        with self.lock:
            self.sock.sendall(data)

    def show(self, text):
        print(text)

    def reader(self):
        r = FrameReader(self.sock)
        try:
            while True:
                ftype, payload = r.next(3600)
                if ftype is None:
                    continue
                if self.upload is not None and self.transfer(ftype, payload):
                    continue
                if self.listing_dir is not None and ftype in (LOAD_INFO, LOAD_FINISH, LOAD_ERROR):
                    if ftype == LOAD_INFO:
                        self.listing[self.listing_dir] += [e.split()[0] for e in payload.decode(errors='replace').split('\n') if e.strip()]
                    else:
                        self.listing_dir = None
                    continue
                text = payload.decode(errors='replace').rstrip()
                if ftype == INFO and text.startswith('[G'):
                    self.modal = text
                    if self.quiet_modal > 0:
                        self.quiet_modal -= 1
                        self.swallow_ok = True
                        continue
                if ftype == INFO and text == 'ok' and self.swallow_ok:
                    self.swallow_ok = False
                    continue
                if ftype == STATUS:
                    self.status = text
                    if self.quiet_status > 0:
                        self.quiet_status -= 1
                        continue
                tag = NAMES.get(ftype, 'status' if ftype == STATUS else '%02x' % ftype)
                self.show(f'[{tag}] {text}' if tag else text)
        except (ConnectionError, OSError):
            self.closed = True
            self.show('connection closed')

    def poller(self):
        last_modal = 0.0
        while not self.closed:
            time.sleep(POLL_S)
            if self.upload is None:
                try:
                    self.poll_status()
                    if time.monotonic() - last_modal >= MODAL_S:
                        last_modal = time.monotonic()
                        self.poll_modal()
                except OSError:
                    return

    def command(self, line):
        if line in ('?', '!', '~'):
            self.send(frame(CTRL_SINGLE, line.encode()))
        elif line.upper() == '^X':
            self.send(frame(CTRL_SINGLE, b'\x18'))
        else:
            self.send(frame(CTRL_MULTI, line.encode()))
            if line != '$G':
                self.poll_modal()  # answered after the line has been processed, so the block shows its effect
                self.poll_status()

    def poll_modal(self):
        self.quiet_modal += 1
        self.send(frame(CTRL_MULTI, b'$G'))

    def poll_status(self):
        self.quiet_status += 1
        self.send(frame(CTRL_SINGLE, b'?'))

    def list_remote(self, directory):
        self.listing[directory] = []
        self.listing_dir = directory
        self.send(frame(CTRL_MULTI, f'ls {directory}'.encode()))

    def start_upload(self, local, remote):
        with open(local, 'rb') as f:
            data = f.read()
        md5 = hashlib.md5(data).hexdigest().encode()
        self.upload = {'data': data, 'md5': md5, 'total': (len(data) + PACKET_SIZE - 1) // PACKET_SIZE, 'sent': 0}
        self.show(f"uploading {local} -> {remote}: {len(data)} bytes, {self.upload['total']} packets")
        self.send(frame(FILE_START, f'upload {remote}'.encode()))
        self.send(frame(MD5, md5))

    # answers the machine's requests; returns False for frames that are not part of the transfer
    def transfer(self, ftype, payload):
        u = self.upload
        if ftype == MD5:
            self.send(frame(MD5, u['md5']))
        elif ftype == VIEW:
            self.send(frame(VIEW, struct.pack('>IH', u['total'], PACKET_SIZE)))
        elif ftype == DATA:
            seq = struct.unpack('>I', payload[:4])[0]
            self.send(frame(DATA, struct.pack('>I', seq) + u['data'][(seq - 1) * PACKET_SIZE: seq * PACKET_SIZE]))
            u['sent'] = max(u['sent'], seq)
        elif ftype == END:
            self.show(f"upload complete, {u['total']} packets")
            self.upload = None
            self.listing.clear()  # the card changed
        elif ftype == CAN:
            self.show('upload cancelled by machine')
            self.upload = None
        else:
            return False
        return True


def status_block(con):
    # <Idle|MPos:x,y,z,a|WPos:x,y,z,a|F:cur,set,ovr|S:cur,set,ovr,..|T:n,tlo|H:reason> -> lines
    status = con.status
    if not status.startswith('<'):
        lines = [status or 'no status yet']
    else:
        parts = status.strip('<>').split('|')
        fields = dict(p.split(':', 1) for p in parts[1:] if ':' in p)
        state = parts[0]
        if 'H' in fields:
            state += f'   halt reason {fields["H"]}'
        lines = [state]
        axes = fields.get('MPos', '').split(',')
        if axes and axes[0]:
            lines.append('      ' + ''.join(f'{a:>11}' for a in 'XYZAB'[:len(axes)]))
            for key in ('MPos', 'WPos'):
                if key in fields:
                    lines.append(f'{key:5} ' + ''.join(f'{float(v):11.3f}' for v in fields[key].split(',')))
        info = []
        if 'F' in fields:
            f = fields['F'].split(',')
            info.append(f'F {float(f[0]):.0f} / {float(f[1]):.0f} mm/min' + (f'  {float(f[2]):.0f}%' if len(f) > 2 else ''))
        if 'S' in fields:
            sp = fields['S'].split(',')
            info.append(f'S {float(sp[0]):.0f} / {float(sp[1]):.0f} rpm' + (f'  {float(sp[2]):.0f}%' if len(sp) > 2 else ''))
        if 'T' in fields:
            t = fields['T'].split(',')
            info.append(f'T{t[0]}' + (f'  TLO {float(t[1]):.3f}' if len(t) > 1 else ''))
        if info:
            lines.append('    '.join(info))
    if con.modal:
        lines.append(con.modal.strip('[]'))
    if con.upload is not None:
        lines.append(f"upload {con.upload['sent']}/{con.upload['total']} packets")
    return '\n'.join(lines)


class CarveraCompleter(Completer):
    def __init__(self, con):
        self.con = con

    def get_completions(self, document, complete_event):
        words = document.text_before_cursor.split(' ')
        word = words[-1]
        if len(words) == 1:
            options = [c + ' ' for c in SHELL + ['/upload'] if c.startswith(word)]
        elif words[0] == '/upload' and len(words) == 2:
            options = [p + '/' if os.path.isdir(p) else p for p in glob.glob(os.path.expanduser(word) + '*')]
        elif words[0] in REMOTE_PATH and word.startswith('/'):
            directory, _, name = word.rpartition('/')
            directory = directory or '/'
            if directory not in self.con.listing or self.con.listing_dir == directory:
                if self.con.listing_dir is None:
                    self.con.list_remote(directory)  # ready on the next Tab
                return
            options = [directory.rstrip('/') + '/' + e for e in self.con.listing[directory] if e.startswith(name)]
        else:
            return
        for o in sorted(options):
            yield Completion(o, start_position=-len(word), display=os.path.basename(o.rstrip('/ ')) + ('/' if o.endswith('/') else ''))


# realtime keys: they must reach the machine on the keypress, not after Enter
def realtime_keys(con):
    keys = KeyBindings()

    def send(byte, what):
        try:
            con.send(frame(CTRL_SINGLE, byte))
            print(what)
        except OSError as e:
            print(f'failed: {e}')

    @keys.add('c-x', eager=True)  # prompt_toolkit uses c-x as a prefix, eager stops it waiting for a second key
    def _(event):
        send(b'\x18', '^X abort sent')

    @keys.add('c-p')
    def _(event):
        send(b'!', 'feed hold sent')

    @keys.add('c-o')
    def _(event):
        send(b'~', 'resume sent')

    return keys


def local_command(con, line):
    words = line.split()
    if words[0] == '/upload' and len(words) in (2, 3):
        if con.upload is not None:
            print('upload already running')
            return
        remote = words[2] if len(words) == 3 else '/sd/gcodes/' + os.path.basename(words[1])
        con.start_upload(words[1], remote)
    else:
        print('commands: /upload <local> [<remote>]')


def main():
    if len(sys.argv) != 3:
        sys.exit('usage: cli.py <host> <port>')
    sock = socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=10)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    con = Console(sock)
    session = PromptSession('> ', history=FileHistory(HISTORY_FILE), completer=CarveraCompleter(con),
                            complete_while_typing=False, bottom_toolbar=lambda: status_block(con), refresh_interval=POLL_S,
                            key_bindings=realtime_keys(con))
    try:
        with patch_stdout():
            while not con.closed:
                for line in session.prompt().splitlines():
                    line = line.strip()
                    if not line:
                        continue
                    if line == '?':
                        con.quiet_status = 0  # show this one
                    try:
                        if line.startswith('/'):
                            local_command(con, line)
                        else:
                            con.command(line)
                    except OSError as e:
                        print(f'failed: {e}')
    except (EOFError, KeyboardInterrupt):
        pass
    sock.close()


if __name__ == '__main__':
    main()
