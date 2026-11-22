# xm533_mipi_io

`xm533_mipi_io` puts the XM533's MIPI lanes 6..9 to use as GPIOs, at `/dev/mipi_io`, and builds as `open_mipi_io.ko`. It replaces the vendor `mipi_io.ko`.

`0x40084d00 {pin, level}` makes a pin an output at that level, and `0xc0084d01 {pin, level}` makes it an input and returns its level. `mipi_io_set_output` and `mipi_io_get_input` do the same for other modules.
