#!/usr/bin/env python3
# Upload a file to the machine over TCP using the Makera frame protocol (src/libs/Frame.h).
import argparse
import binascii
import collections
import hashlib
import select
import socket
import struct
import sys
import time

HEADER = b'\x86\x68'
FOOTER = b'\x55\xaa'
INFO, CTRL_MULTI, FILE_START = 0x90, 0xA2, 0xB0
MD5, VIEW, DATA, END, CAN, RETRY = 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6
PACKET_SIZE = 8192
SUCCESS, FAILURE = 'Info: upload success', 'Upload failed for file'  # FileTransfer::finish, the last word
TIMEOUT = 10
RESET_SETTLE_S = 0.5
RATE_WINDOW_S = 2.0


def crc16(data):
    return binascii.crc_hqx(data, 0)  # CRC-16/XMODEM, poly 0x1021 from 0


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


class Closed(ConnectionError):
    pass


# answers the machine's requests for one file
class Upload:
    def __init__(self, data, remote):
        self.data = data
        self.remote = remote
        self.md5 = hashlib.md5(data).hexdigest().encode()
        self.total = (len(data) + PACKET_SIZE - 1) // PACKET_SIZE
        self.progress = Progress(len(data))
        self.began = None  # when FILE_START went out
        self.started = False  # the machine has asked for something
        self.ended = False  # FILE_END seen: all data is in, a .lz still unpacks and ignores a cancel
        # 'done', 'failed', 'cancelled' or 'refused'; FILE_END is not the end, a .lz still unpacks after it
        self.result = None

    def start(self):
        self.began = time.monotonic()
        self.progress = Progress(len(self.data))  # timed from here, not from when it was queued
        return [frame(FILE_START, f'upload {self.remote}'.encode()), frame(MD5, self.md5)]

    # the reply frame, b'' when the frame ends the transfer, None when it is not part of it
    def answer(self, ftype, payload):
        if ftype == INFO:
            text = payload.decode(errors='replace')
            if text.startswith(SUCCESS):
                self.result = 'done'
            elif text.startswith(FAILURE):
                self.result = 'failed'
            elif not self.started and text.startswith('error:'):
                self.result = 'refused'  # "another transfer", "is being played": no FILE_CAN follows these
            return None
        if ftype in (MD5, VIEW, DATA):
            self.started = True
        if ftype == MD5:
            return frame(MD5, self.md5)
        if ftype == VIEW:
            return frame(VIEW, struct.pack('>IH', self.total, PACKET_SIZE))
        if ftype == DATA and len(payload) >= 4:
            seq = struct.unpack('>I', payload[:4])[0]
            self.progress.update((seq - 1) * PACKET_SIZE)  # a request for seq confirms the ones before it
            return frame(DATA, payload[:4] + self.data[(seq - 1) * PACKET_SIZE: seq * PACKET_SIZE])
        if ftype == END:
            self.ended = True
            self.progress.update(len(self.data))
            return b''
        if ftype == CAN:
            self.result = 'cancelled'
            return b''
        return None


class FrameReader:
    def __init__(self, sock):
        self.sock = sock
        self.buf = b''

    def next(self, timeout):
        deadline = time.monotonic() + timeout
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
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None, None
            # select, not settimeout: other threads send on this socket and must keep their own timeout
            if not select.select([self.sock], [], [], remaining)[0]:
                return None, None
            chunk = self.sock.recv(4096)
            if not chunk:
                raise Closed('connection closed')
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
        up = Upload(f.read(), args.destination_path)
    print(f'{args.source_file}: {len(up.data)} bytes, {up.total} packets, md5 {up.md5.decode()}')

    sock = socket.create_connection((args.host, args.port), timeout=TIMEOUT)
    reader = FrameReader(sock)
    for f in up.start():
        sock.sendall(f)

    timeouts = 0
    on_progress_line = False
    while up.result is None:
        ftype, payload = reader.next(TIMEOUT)
        if ftype is None:
            timeouts += 1
            if timeouts > 3:
                sys.exit('no request from machine')
            continue
        timeouts = 0
        reply = up.answer(ftype, payload)
        if reply:
            sock.sendall(reply)
        if ftype == INFO:
            print(('\n' if on_progress_line else '') + payload.decode(errors='replace'), end='')
            on_progress_line = False
        elif ftype == DATA:
            print(f'\r{up.progress.line()}\x1b[K', end='', flush=True)
            on_progress_line = True
    if up.result == 'cancelled':  # the reason follows the FILE_CAN
        ftype, payload = reader.next(1)
        if ftype == INFO:
            print(payload.decode(errors='replace'), end='')
    if up.result != 'done':
        sys.exit(f'upload {up.result}')
    print(f'upload complete, {up.progress.summary()}')

    if args.reset:
        sock.sendall(frame(CTRL_MULTI, b'reset'))
        time.sleep(RESET_SETTLE_S)  # closing right away can lose the command
    sock.close()


if __name__ == '__main__':
    main()
