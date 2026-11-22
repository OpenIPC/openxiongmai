# xm_v200_motor

`xm_v200_motor` drives the V200 series' pan/tilt step motors (XM530V200, XM533, XM535D1/D2/W1). It provides `/dev/motor`, builds as `open_motor.ko`, and replaces the vendor `kmotor.ko`. It has the same ioctls, the same text commands, and the same module parameters.

## Hardware

There are two unipolar motors, X (pan) and Y (tilt). Each is half-stepped through eight coil patterns: A, AB, B, BC, C, CD, D, DA. A turn is 4096 half steps, and `MAX_DEGREE_X/Y` set the travel. An SP804 pair at 0x100D0000 paces them: X runs on its first timer and Y on its second, both on IRQ 37. Each tick moves its motor one half step.

The coils are driven one of two ways:

- **GPIOs.** `gpio_pin=` takes 12 values: A, B, C, D of X; then D, C, B, A of Y; then the limit switches for X right, X left, Y up and Y down. A value of -1 means none. The limit switches are active low and only read with `limit=1`.
  - Each pin has its own register at 0x10020000 + 4 × pin: 0xc80 sets it high, 0x480 low and 0x1080 makes it an input; bit 13 is its level.
  - A pin above 1000 is MIPI pad (pin − 1000) on the XM533. It is set through `open_mipi_io`'s `mipi_io_set_output`, which is looked up at load time, so the module has no hard dependency on it.
- **A 10-bit shift register** (`iic_drive=1`; the default with `MOTOR_CHIP=xm535d2`), on the XM535D2 X9 boards. `IIC_SDA` and `IIC_SCL` clock the bits in LSB first, and `IIC_EN` latches them. Bits 0..3 are X's coils (bit 3 is A), bits 4..7 are Y's, and bits 8 and 9 move the IR-cut filter (ioctl `0x40044d04`). `IIC_RST` is held low. A thread sends the bits, because one transfer takes about 0.6 ms.

`pin_antitone` reverses a motor's direction: bit 0 is X, bit 1 is Y. The XM535D2's `pin_antitone1` is XORed with it.

## Speeds

A tick is 33750000 / (128 + 16i) counts of the 12 MHz timer for i ≤ 52, and 33750000 / (960 + 80(i − 52)) above that. Index i runs from 0 (about 45 half steps/s) to 60 (about 570/s). The tick is multiplied by the timer-clock multiple, which `kmotor.ko` reads from 0x20000F2C / 0x20000FD0 and `timer_mul=` overrides, and divided by `speed_up`.

A user speed of 0..17 maps to index 7, 12, … 52, 53, … 60. The four plain text moves give the index directly instead, plus 8 for X or plus 5 for Y (ioctl `a5 = 0x5867`).

## Commands

Commands come in through ioctl `0x40184d01` as `{cmd, a1..a5}`, or are written as text (64 bytes at most):

| cmd | text | |
|-----|------|-|
| 1, 2, 4, 8 | `left N`, `right N`, `up N`, `down N` | move until stopped or at the end; a1 is the X speed, a2 the Y speed |
| 5, 6, 9, 10 | `leftup N`, `rightup N`, `leftdown N`, `rightdown N` | move both motors. Sending the same command again only changes the speed |
| 0x10 | `lscan`, `rscan`, `scan [N]` | `lscan` and `rscan` mark the ends at the current position, and `scan` sweeps X between a2 and a4 (Y at a3) until stopped. The text speed defaults to 25 |
| 0x11 | `setpreset`, `preset` | go to X a1, Y a2 at speeds a3, a4. `setpreset` marks the current position and `preset` goes back to it |
| 0x12 | `stop` | |
| 0x13 | | the end-to-end test, then the middle (`MID_DEGREE_X/Y`, or half the travel) |
| 0x14 | | set the current position to X a1, Y a2 (only when stopped) |

ioctl `0x80184d02` reads `{action, X position, Y position, X flags, Y flags, 0}`, and `0x80184d03` reads `{action, X max steps, Y max steps, MAX_DEGREE_X << 16 | MAX_DEGREE_Y, 0, 0}`.

- Action 0xffff means idle.
- Flags: 2 means a waiter, 4 running, 0x10 the end-to-end test done (on X).

Commands fail until the end-to-end test has run (`auto_test=1`, the default, runs it at load) or has been skipped (`auto_test=0`, which the vendor's `loadpublic` passes unless `motor.init` says otherwise). `/proc/motor` shows the state. `kmotor.ko` creates no proc entry.

## Changes from `kmotor.ko`

- The timers run periodic. `kmotor.ko` runs them free and reloads them every tick, which is the same on the chip, but in QEMU a free-running SP804 never counts from its load value.
- User input is range-checked: preset and scan targets are clamped to the travel, unused (-1) pins are skipped, and the commands are queued to one thread.
- A scan that starts at its left end goes right first.
- A plain move cancels a scan that is waiting between legs.
