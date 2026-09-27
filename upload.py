#!/usr/bin/env python3
# Upload a file to the machine over TCP using the Makera frame protocol (src/libs/Frame.h).
import argparse
import collections
import hashlib
import socket
import struct
import sys
import time

HEADER = b'\x86\x68'
FOOTER = b'\x55\xaa'
INFO, CTRL_MULTI, FILE_START = 0x90, 0xA2, 0xB0
MD5, VIEW, DATA, END, CAN, RETRY = 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6
PACKET_SIZE = 8192
TIMEOUT = 10
RESET_SETTLE_S = 0.5
RATE_WINDOW_S = 2.0


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(ftype, payload=b''):
    body = struct.pack('>HB', len(payload) + 3, ftype) + payload
    return HEADER + body + struct.pack('>H', crc16(body)) + FOOTER


def format_rate(bytes_per_s):
    return f'{bytes_per_s / 1024:.1f} KiB/s'


class Progress:
    # update() and line() may run on different threads: only update() touches the deque
    def __init__(self, size):
        self.size = size
        self.done = 0
        self.start = time.monotonic()
        self.samples = collections.deque([(self.start, 0)])

    def update(self, done):
        self.done = max(self.done, min(done, self.size))
        now = time.monotonic()
        self.samples.append((now, self.done))
        # keep the newest sample at or before the window start as the baseline
        while len(self.samples) > 1 and self.samples[1][0] <= now - RATE_WINDOW_S:
            self.samples.popleft()

    def line(self):
        # measured from the baseline to now, so a stall decays toward 0 instead of freezing the last rate
        base_t, base_done = self.samples[0]
        done = self.done
        elapsed = time.monotonic() - base_t
        rate = (done - base_done) / elapsed if elapsed > 0 else 0.0
        line = f'{done}/{self.size} bytes  {100 * done / max(self.size, 1):.0f}%  {format_rate(rate)}'
        if rate > 0:
            line += f'  {(self.size - done) / rate:.0f} s left'
        return line

    def summary(self):
        elapsed = time.monotonic() - self.start
        return f'{self.size} bytes in {elapsed:.1f} s, {format_rate(self.size / elapsed) if elapsed > 0 else "-"}'


class FrameReader:
    def __init__(self, sock):
        self.sock = sock
        self.buf = b''

    def next(self, timeout):
        deadline = time.time() + timeout
        while True:
            i = self.buf.find(HEADER)
            if i >= 0 and len(self.buf) >= i + 5:
                flen = struct.unpack('>H', self.buf[i + 2:i + 4])[0]
                total = 2 + flen + 4  # header, (len+type+payload), crc+footer
                if len(self.buf) >= i + total:
                    f = self.buf[i:i + total]
                    self.buf = self.buf[i + total:]
                    if f[-2:] == FOOTER and struct.unpack('>H', f[-4:-2])[0] == crc16(f[2:-4]):
                        return f[4], f[5:-4]
                    self.buf = f[1:] + self.buf  # bad frame: resync after this header byte
                    continue
            remaining = deadline - time.time()
            if remaining <= 0:
                return None, None
            self.sock.settimeout(remaining)
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                return None, None
            if not chunk:
                raise ConnectionError('connection closed')
            self.buf += chunk


def main():
    p = argparse.ArgumentParser(description='Upload a file to the machine (Makera frame protocol).')
    p.add_argument('host')
    p.add_argument('port', type=int)
    p.add_argument('source_file')
    p.add_argument('destination_path')
    p.add_argument('-r', '--reset', action='store_true', help='send "reset" after the upload')
    args = p.parse_args()

    with open(args.source_file, 'rb') as f:
        data = f.read()
    md5 = hashlib.md5(data).hexdigest().encode()
    total = (len(data) + PACKET_SIZE - 1) // PACKET_SIZE
    print(f'{args.source_file}: {len(data)} bytes, {total} packets, md5 {md5.decode()}')

    sock = socket.create_connection((args.host, args.port), timeout=TIMEOUT)
    reader = FrameReader(sock)
    sock.sendall(frame(FILE_START, f'upload {args.destination_path}'.encode()))
    sock.sendall(frame(MD5, md5))

    progress = Progress(len(data))
    timeouts = 0
    while True:
        ftype, payload = reader.next(TIMEOUT)
        if ftype is None:
            timeouts += 1
            if timeouts > 3:
                sys.exit('no request from machine')
            continue
        timeouts = 0
        if ftype == INFO:
            print(payload.decode(errors='replace'), end='')
        elif ftype == MD5:
            sock.sendall(frame(MD5, md5))
        elif ftype == VIEW:
            sock.sendall(frame(VIEW, struct.pack('>IH', total, PACKET_SIZE)))
        elif ftype == DATA:
            seq = struct.unpack('>I', payload[:4])[0]
            chunk = data[(seq - 1) * PACKET_SIZE: seq * PACKET_SIZE]
            sock.sendall(frame(DATA, struct.pack('>I', seq) + chunk))
            progress.update(seq * PACKET_SIZE)
            print(f'\r{progress.line()}\x1b[K', end='', flush=True)
        elif ftype == END:
            print(f'\nupload complete, {progress.summary()}')
            break
        elif ftype == CAN:
            sys.exit('\nupload cancelled by machine')

    if args.reset:
        sock.sendall(frame(CTRL_MULTI, b'reset'))
        time.sleep(RESET_SETTLE_S)  # closing right away can lose the command
    sock.close()


if __name__ == '__main__':
    main()
