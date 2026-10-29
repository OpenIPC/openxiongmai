# xm-isp-tune

Calibration tools for `libxmisp` sensors (MIT). They measure the camera's
picture and fit tuning from the measurements. Nothing is taken from the
vendor's tuning except as a baseline to compare with.

- `ispreg` reads and writes XM510 ISP registers through `/dev/isp` from the
  shell, alongside a running ISP daemon. It's static, so copy it to
  `/tmp/ispreg` on the camera.
- `colorchecker.py` fits a sensor's colour matrix against a 24-patch colour
  checker (ColorChecker Classic layout).

## Colour matrix

Put the chart in front of the camera in daylight, with `xmispd` running
under majestic. AWB must have the chart's greys neutral; the script checks
that and prints the ratios. Then:

```sh
make CROSS_COMPILE=arm-openipc-linux-musleabi-
cat ispreg | ssh root@CAMERA 'cat > /tmp/ispreg; chmod +x /tmp/ispreg'
./colorchecker.py CAMERA --grid 533,404,848,583
```

`--grid` is the centres of the first patch (top left) and the last (bottom
right) in picture pixels. The script:
- reads the camera's native response through an identity matrix at Cb/Cr
  gain 1;
- decodes `/video.mp4` full-range BT.601 and undoes the XM510 RGB gamma;
- fits the matrix (rows summing to 256, so grey stays grey) for the least
  CIELAB error of the picture as an sRGB display shows it;
- then measures the picture with the fitted matrix and with the vendor's,
  for comparison.

It needs numpy, scipy and Pillow, and ffmpeg in docker
(`linuxserver/ffmpeg`).

On the lab XM510 (JX-H62, daylight) it fits a matrix scoring a mean dE of
10.7 on the chart, against 17.7 for the vendor's matrix with its 2.3× Cb/Cr
gain, which clips three patches. A second run reproduces the matrix to
within a few units in 256.

These findings let the fit model the pipeline:
- **Cb/Cr unity:** the Cb/Cr gains (`0x40076004`/`08`) are 1 at `0x100`.
  Through an identity matrix, a 30% blue gain change came back as +30.5% in
  B with R and G unchanged, and at `0x200` it leaked into them.
- **Matrix registers:** they read back as 13-bit two's complement.
- **Stream range:** the stream is full-range: the XM510's RGB to YCbCr has
  Y coefficients summing to 255 and no offset, and it doesn't flag the
  range, so decoders assume limited range unless told.
