# Machine scripts

One O-word sub per file, compiled into the firmware. Copy a file to `/sd/macros/` to replace that sub, add a
file to add a sub. `macro check` reloads and validates, `macro list` shows the subs, `macro params` the `#<_name>` values, `macro run <sub> [args]`
runs one, `macro trace on` echoes every executed line, `list [n]` shows the lines around the one running. `(MSG, text)` prints text, `(DEBUG, text)` prints it with
`#n` and `#<name>` replaced by their values. The firmware calls `tool_change` for M6, `calibrate` for M491, `auto_work` for M495, `goto` for M496,
`g28` for G28, `laser_on`/`laser_off` for M321/M322; the words of the block arrive as `#<t>`, `#<x>`, ... and the
subcode as `#<subcode>`, the triggering code as `#<code>`. `call` and `macro run` pass `#1..#30`. A sub that is not defined leaves the code to its
C++ handler. Numbers are mm, machine coordinates unless G90 is used; G21 G90 are forced before a sub starts. `o<name> abort [n]` ends the script and halts the machine with reason n; `return [n]` just returns a value.
