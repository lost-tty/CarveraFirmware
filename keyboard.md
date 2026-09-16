# USB keyboard pendant

Plug a USB keyboard or numpad into the host port (`usb_host.enable true`). Shift = NumLock held or Shift.

| Numpad    | Keyboard     | Plain                      | Shifted                             |
|-----------|--------------|----------------------------|-------------------------------------|
| 4 / 6     | Left / Right | jog X - / +                | zero X in current WCS               |
| 2 / 8     | Down / Up    | jog Y - / +                | zero Y                              |
| 3 / 9     | PgDn / PgUp  | jog Z - / +                | zero Z                              |
| 1 / 7     |              | jog A - / +                |                                     |
| + / -     | = / -        | mode up / down             | feed override +10 / -10 % (10..200) |
| 5         | Space        | feed hold, again to resume |                                     |
| 0         |              |                            | home                                |
| *         |              |                            | spindle off, or on at last speed    |
| /         |              | vacuum toggle              | light toggle                        |
| Enter     |              | resume job                 |                                     |
| Backspace | Escape       | abort                      | unlock after halt                   |

Modes: 0.01, 0.1, 1, 10 mm per press, then continuous at 10 % and 50 % of the axis max rate while the key is held.

While halted only unlock and home work. Zero, home, unlock, resume, feed override and probing go through the
normal command path, so the not-homed guard applies.

LEDs. NumLock (the only LED on a numpad): off idle, on running, slow blink feed hold, fast blink alarm, short
pulse not homed. CapsLock: continuous mode. ScrollLock: alarm.

Not mapped yet (need scripts): probe, go to zero, tool change. Key table: `src/modules/communication/usb/Pendant.cpp`.
