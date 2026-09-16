#!/usr/bin/env python3
# Console to the machine over TCP using the Makera frame protocol.
# Header: live status block. Middle: sent lines and replies. Bottom: input; pasted text is sent line by line.
# "?" "!" "~" and ^X (typed as the two characters) go out as realtime bytes.
import curses
import queue
import socket
import sys
import threading
import time

from upload import frame, FrameReader, INFO, CTRL_MULTI

CTRL_SINGLE = 0xA1
STATUS = 0x81
NAMES = {0x82: 'diag', 0x83: 'load', 0x84: 'load-end', 0x85: 'load-err', INFO: ''}
POLL_S = 0.5  # status poll, also keeps the machine from dropping an idle connection (wifi.tcp_timeout_s)


class Console:
    def __init__(self, sock):
        self.sock = sock
        self.lock = threading.Lock()
        self.events = queue.Queue()
        self.status = ''
        self.quiet_status = 0  # replies to our own polls are not logged
        self.closed = False
        threading.Thread(target=self.reader, daemon=True).start()

    def send(self, data):
        with self.lock:
            self.sock.sendall(data)

    def reader(self):
        r = FrameReader(self.sock)
        try:
            while True:
                ftype, payload = r.next(3600)
                if ftype is None:
                    continue
                text = payload.decode(errors='replace').rstrip()
                if ftype == STATUS:
                    self.status = text
                    if self.quiet_status > 0:
                        self.quiet_status -= 1
                        continue
                tag = NAMES.get(ftype, 'status' if ftype == STATUS else '%02x' % ftype)
                self.events.put(f'[{tag}] {text}' if tag else text)
        except (ConnectionError, OSError):
            self.events.put('connection closed')
            self.closed = True

    def command(self, line):
        if line in ('?', '!', '~'):
            self.send(frame(CTRL_SINGLE, line.encode()))
        elif line.upper() == '^X':
            self.send(frame(CTRL_SINGLE, b'\x18'))
        else:
            self.send(frame(CTRL_MULTI, line.encode()))

    def poll(self):
        self.quiet_status += 1
        self.send(frame(CTRL_SINGLE, b'?'))


def header(status):
    # <Idle|MPos:x,y,z,a|WPos:x,y,z,a|F:cur,set,ovr|S:cur,set,ovr,..|T:n,tlo|H:reason> -> lines
    if not status.startswith('<'):
        return [status]
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
    return lines


def run(stdscr, con):
    curses.use_default_colors()
    stdscr.nodelay(True)
    stdscr.timeout(50)
    log, buf, last_poll = [], '', 0.0
    while True:
        if time.monotonic() - last_poll >= POLL_S and not con.closed:
            con.poll()
            last_poll = time.monotonic()
        while True:
            try:
                log.append(con.events.get_nowait())
            except queue.Empty:
                break
        del log[:-2000]

        while True:
            ch = stdscr.getch()
            if ch == -1:
                break
            if ch in (curses.KEY_ENTER, 10, 13):
                line = buf.strip()
                buf = ''
                if line:
                    log.append('> ' + line)
                    if line == '?':
                        con.quiet_status = 0  # show this one
                    try:
                        con.command(line)
                    except OSError:
                        log.append('send failed')
            elif ch in (curses.KEY_BACKSPACE, 127, 8):
                buf = buf[:-1]
            elif ch == curses.KEY_RESIZE:
                pass
            elif 32 <= ch < 127:
                buf += chr(ch)

        h, w = stdscr.getmaxyx()
        stdscr.erase()
        head = header(con.status) or ['']
        for i, text in enumerate(head):
            stdscr.addnstr(i, 0, text.ljust(w - 1), w - 1, curses.A_REVERSE)
        rows = h - 1 - len(head)
        for i, text in enumerate(log[-rows:]):
            stdscr.addnstr(len(head) + i, 0, text, w - 1)
        prompt = '> ' + buf
        stdscr.addnstr(h - 1, 0, prompt[-(w - 1):], w - 1)
        stdscr.refresh()


def main():
    if len(sys.argv) != 3:
        sys.exit('usage: cli.py <host> <port>')
    sock = socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=10)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    con = Console(sock)
    try:
        curses.wrapper(run, con)
    except KeyboardInterrupt:
        pass
    sock.close()


if __name__ == '__main__':
    main()
