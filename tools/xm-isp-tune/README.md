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
SSHPASS=... ./colorchecker.py CAMERA --grid 533,404,848,583
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

It needs numpy and scipy, and ffmpeg: the local one, or else a docker
image pinned by digest. The camera's root password comes from `SSHPASS`
(`sshpass -e`), and its host key is checked as ssh checks it. Check a new
camera's key fingerprint on its console, then pass `--accept-new-host-key`
once.

On the lab XM510 (JX-H62, daylight) it fits matrices scoring a mean dE of
10.4–10.8 on the chart over three runs. The vendor daemon's matrix and Cb/Cr
gain, as traced in the same light, score 16.9 and clip three patches. The
vendor moves its matrix with the light (80–150 distinct matrices a trace)
and its Cb/Cr gain too (`0x20e`–`0x250`), so its baseline is light-specific.
The fit also moves a little as the daylight changes: rows within about 40
in 256 over an afternoon.

These findings let the fit model the pipeline:
- **Cb/Cr unity:** the Cb/Cr gains (`0x40076004`/`08`) are 1 at `0x100`.
  Through an identity matrix, a 30% blue gain change came back as +30.5% in
  B with R and G unchanged, and at `0x200` it leaked into them.
- **Matrix registers:** they read back as 13-bit two's complement.
- **Stream range:** the stream is full-range: the XM510's RGB to YCbCr has
  Y coefficients summing to 255 and no offset, and it doesn't flag the
  range, so decoders assume limited range unless told.
