#!/bin/sh
# Pack the machine scripts into a cpio "newc" archive.
cd "$(dirname "$1")" && for f in "$@"; do basename "$f"; done | cpio -o -H newc --quiet
