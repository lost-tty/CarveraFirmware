#!/usr/bin/env python3
# Upload a file to the machine over TCP using the Makera frame protocol (src/libs/Frame.h).
import argparse
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

    sent = 0
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
            sent = max(sent, seq)
            print(f'\r{sent}/{total}', end='', flush=True)
        elif ftype == END:
            print('\nupload complete')
            break
        elif ftype == CAN:
            sys.exit('\nupload cancelled by machine')

    if args.reset:
        sock.sendall(frame(CTRL_MULTI, b'reset'))
        end = time.time() + 4
        while time.time() < end:
            ftype, payload = reader.next(1)
            if ftype == INFO:
                print(payload.decode(errors='replace'), end='')
    sock.close()


if __name__ == '__main__':
    main()
