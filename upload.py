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


# QuickLZ level 3, one block: the inverse of the machine's qlz_compress (src/libs/quicklz.c)
def qlz_block(src):
    n = 4 if src[0] & 2 else 1
    size = int.from_bytes(src[1 + n:1 + 2 * n], 'little')
    pos = 1 + 2 * n
    if not src[0] & 1:
        return bytes(src[pos:pos + size])
    out = bytearray()
    cword = 1
    while True:
        if pos >= len(src):
            raise ValueError('block ends early')
        if cword == 1:
            cword = int.from_bytes(src[pos:pos + 4], 'little')
            pos += 4
        fetch = int.from_bytes(src[pos:pos + 4].ljust(4, b'\0'), 'little')
        if cword & 1:
            cword >>= 1
            if fetch & 3 == 0:
                offset, length, pos = (fetch & 0xff) >> 2, 3, pos + 1
            elif fetch & 2 == 0:
                offset, length, pos = (fetch & 0xffff) >> 2, 3, pos + 2
            elif fetch & 1 == 0:
                offset, length, pos = (fetch & 0xffff) >> 6, ((fetch >> 2) & 15) + 3, pos + 2
            elif fetch & 127 != 3:
                offset, length, pos = (fetch >> 7) & 0x1ffff, ((fetch >> 2) & 0x1f) + 2, pos + 3
            else:
                offset, length, pos = fetch >> 15, ((fetch >> 7) & 255) + 3, pos + 4
            start = len(out) - offset
            if offset < 3 or start < 0:
                raise ValueError('bad match')
            for i in range(length):  # the copy may overlap what it writes
                out.append(out[start + i])
        elif len(out) < size - 11:
            count = (4, 1, 2, 1, 3, 1, 2, 1)[(cword & 0xf) >> 1]
            out += src[pos:pos + count]
            cword >>= count
            pos += count
        else:
            while len(out) < size:
                if cword == 1:
                    pos += 4
                    cword = 1 << 31
                out.append(src[pos])
                pos += 1
                cword >>= 1
            return bytes(out)


# a .lz file: blocks of a 4-byte size and a QuickLZ block, then the 16-bit sum of the bytes they hold
def unpack_lz(data):
    out = bytearray()
    pos = 0
    while pos < len(data) - 2:
        size = int.from_bytes(data[pos:pos + 4], 'big')
        out += qlz_block(data[pos + 4:pos + 4 + size])
        pos += 4 + size
    if sum(out) & 0xffff != int.from_bytes(data[-2:], 'big'):
        raise ValueError('bad sum')
    return bytes(out)


# asks the machine for one file; a file uploaded packed comes back packed and is unpacked here
class Download:
    def __init__(self, remote):
        self.remote = remote
        self.md5 = None
        self.total = None
        self.chunks = []
        self.started = False
        self.result = None  # 'done', 'failed', 'cancelled' or 'refused'
        self.data = None

    def start(self):
        return [frame(CTRL_MULTI, f'download {self.remote}'.encode())]

    # the reply frame, b'' when the frame ends the transfer, None when it is not part of it
    def answer(self, ftype, payload):
        if ftype == INFO:
            if not self.started and payload.decode(errors='replace').lower().startswith('error'):
                self.result = 'refused'
            return None
        if ftype == MD5:
            self.started = True
            self.md5 = payload[:32].decode(errors='replace').lower()
            return frame(VIEW)
        if ftype == VIEW and len(payload) >= 6:
            self.total = struct.unpack('>IH', payload[:6])[0]
            return frame(DATA, struct.pack('>I', 1)) if self.total else frame(END)
        if ftype == DATA and len(payload) >= 4:
            seq = struct.unpack('>I', payload[:4])[0]
            if seq != len(self.chunks) + 1:
                return frame(DATA, struct.pack('>I', len(self.chunks) + 1))
            self.chunks.append(payload[4:])
            return frame(DATA, struct.pack('>I', seq + 1)) if seq < self.total else frame(END)
        if ftype == END:
            self.finish()
            return b''
        if ftype == CAN:
            self.result = self.result or 'cancelled'
            return b''
        return None

    def finish(self):
        data = b''.join(self.chunks)
        if hashlib.md5(data).hexdigest() != self.md5:
            try:
                data = unpack_lz(data)
            except (ValueError, IndexError):
                data = None
        if data is None or hashlib.md5(data).hexdigest() != self.md5:
            self.result = 'failed'
            return
        self.data = data
        self.result = 'done'


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


# one file on a connection of its own, so the console's stays free
def fetch(host, port, remote, timeout=TIMEOUT):
    sock = socket.create_connection((host, port), timeout=timeout)
    try:
        reader = FrameReader(sock)
        dl = Download(remote)
        for f in dl.start():
            sock.sendall(f)
        while dl.result is None:
            ftype, payload = reader.next(timeout)
            if ftype is None:
                raise TimeoutError(f'no reply to the download of {remote}')
            reply = dl.answer(ftype, payload)
            if reply:
                sock.sendall(reply)
        if dl.result != 'done':
            raise OSError(f'download of {remote} {dl.result}')
        return dl.data
    finally:
        sock.close()


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
