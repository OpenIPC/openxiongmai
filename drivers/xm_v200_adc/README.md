# xm_v200_adc

`xm_v200_adc` is the V200 series' ADC, at `/dev/adc`, and builds as `open_adc.ko`. It replaces the vendor `adc.ko`.

One ioctl, `0xc0080001 {channel, value}`: the channel is converted (control at 0x10020CC0) and its value read back (+4). On the XM535s the converter is also clocked from the system controller and takes control bit 14 (`ADC_CHIP=xm535`).
