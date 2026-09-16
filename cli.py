#!/usr/bin/env python3
# Interactive console to the machine over TCP using the Makera frame protocol.
# Lines are sent as commands; "?" "!" "~" and ctrl-x (type ^X) as realtime bytes.
import socket
import sys
import threading

from upload import frame, FrameReader, INFO, CTRL_MULTI

CTRL_SINGLE = 0xA1
NAMES = {0x81: 'status', 0x82: 'diag', 0x83: 'load', 0x84: 'load-end', 0x85: 'load-err', INFO: ''}


def reader(sock):
    r = FrameReader(sock)
    try:
        while True:
            ftype, payload = r.next(3600)
            if ftype is None:
                continue
            text = payload.decode(errors='replace').rstrip()
            tag = NAMES.get(ftype, '%02x' % ftype)
            print(f'[{tag}] {text}' if tag else text, flush=True)
    except (ConnectionError, OSError):
        print('connection closed', flush=True)


def main():
    if len(sys.argv) != 3:
        sys.exit('usage: cli.py <host> <port>')
    sock = socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=10)
    threading.Thread(target=reader, args=(sock,), daemon=True).start()
    try:
        while True:
            line = input('> ').strip()
            if not line:
                continue
            if line in ('?', '!', '~'):
                sock.sendall(frame(CTRL_SINGLE, line.encode()))
            elif line.upper() == '^X':
                sock.sendall(frame(CTRL_SINGLE, b'\x18'))
            else:
                sock.sendall(frame(CTRL_MULTI, line.encode()))
    except (EOFError, KeyboardInterrupt):
        pass
    sock.close()


if __name__ == '__main__':
    main()
