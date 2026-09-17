#!/bin/sh
# Concatenate the machine scripts into one blob: a "(file: name)" marker per file, ordinary comments
# blanked (their line numbers must stay, trace and errors refer to the source files), (MSG/DEBUG/PRINT,..) kept.
for f in "$@"; do
    echo "(file: $(basename "$f"))"
    awk '{
        if ($0 ~ /^[[:space:]]*\([[:space:]]*(MSG|DEBUG|PRINT|msg|debug|print)[[:space:]]*,/) { print; next }
        gsub(/\([^)]*\)/, "")
        sub(/[[:space:]]*;.*$/, "")
        sub(/[[:space:]]+$/, "")
        print
    }' "$f"
    echo
done
