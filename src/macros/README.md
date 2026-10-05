# Machine scripts

One O-word sub per file, compiled into the firmware, named as the file. Copy a file to `/sd/macros/` to replace
that sub, add a file to add a sub. A sub is read and checked when it is first called, and read again in the next
program. `macro list` shows the subs, `macro params` the `#<_name>` values, `macro run <sub> [args]`
runs one, `trace on` echoes every line with its file and number, `list [n]` shows the lines around the one running. `(MSG, text)` prints text, `(DEBUG, text)` prints it with
`#n` and `#<name>` replaced by their values. A sub named after a G or M code takes that code over: `m6` runs for M6,
`m496.3` for M496.3, `g28` for G28 but not for G28.2. The words of the block arrive as `#<t>`, `#<x>`, ... A code
without a sub stays with its C++ handler. `call` and `macro run` pass `#1..#30`. Numbers are mm, machine
coordinates unless G90 is used.

A sub sets the modal state it needs and puts it back before `endsub`: a `G91` retract that ends without a
`G90` carries on into the caller's next absolute move. Started on its own (`macro run`, boot) there is
nothing to inherit, so it opens with `G21 G90 G91.1 G17`. `#<_motion_mode>` (81: G81) can be set back,
`#<_metric>`/`#<_imperial>` tell the units.

`o<name> abort [n]` ends the script and halts the machine with reason n; `return [n]` just returns a value.
