# xm_v200_adc

`xm_v200_adc` is the V200 series' ADC, at `/dev/adc`, and builds as `open_adc.ko`. It replaces the vendor `adc.ko`.

One ioctl, `0xc0080001 {channel, value}`: the channel is converted (control at 0x10020CC0) and its value read back (+4). On the XM535s the converter is also clocked from the system controller and takes control bit 14 (`ADC_CHIP=xm535`).

## XM232 (`ADC_CHIP=xm232`)

The XM232's `adc.ko` calibrates the converter: at load, and on ioctl `0x40040002`, it converts the ten reference inputs 5..14 (control `(in << 4) | 0x40db03`, 500 us each). `0xc0080001` then converts the channel (control `(chn + 1) << 4 | 0x40db03`, 500 us when the channel changed) and returns the reading mapped to 0..255 through the references (nominally 0, 28.033, 56.066, ... 224.264, 255 steps), piecewise linear in thousandths and rounded on the hundreds digit, with readings outside the references as 0. This driver does the same in the same integer steps; where two references read the same it returns 0 rather than dividing by zero.
