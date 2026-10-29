# xmispd

An open ISP daemon for the XM510 (MIT), a drop-in for the vendor
`/usr/bin/isp` that majestic starts. It works on `libxmisp`:
- brings up VI, the sensor and the ISP;
- prints the vendor daemon's last start-up line, `ispMsgReceive start!`,
  after which majestic starts video;
- runs auto-exposure and auto white balance at each frame signal from
  `/dev/isp` (with `XMISPD_VERBOSE` in its environment it prints every
  change; majestic logs each line, so not by default);
- on SIGTERM or SIGINT, does what the vendor daemon does on its way out:
  3D noise reduction off, then exit.

Its output uses the vendor daemon's format (`\033[0;32;32mDEBUG: ...`,
`\033[0;32;31mERR: ...`), which majestic forwards to its log. It holds
`/tmp/Isplock` like the vendor daemon, so two can't run.

## Build and try

```sh
make CROSS_COMPILE=arm-openipc-linux-musleabi-

# camera, non-persistent: umount and restart majestic to go back
cp xmispd /tmp/xmispd
mount --bind /tmp/xmispd /usr/bin/isp
/etc/init.d/S95majestic restart
logread | grep 'open ISP'
```

Unmount only with majestic stopped: the running daemon keeps the mount busy.

Tested on the lab XM510 with a JX-H62 under majestic:
- **Video:** it streams, and three stop/start cycles each came back with
  3D-NR off between them.
- **AE from the fixed state:** it settles in about 16 frames, near the
  vendor daemon's brightness in the same light, and then makes no change
  for a minute.
- **Knocked off:** exposure forced to 20 lines, or to 746 lines at 8× gain,
  comes back within about 20 frames.
- **AWB, daylight, colour checker in view:** from the script's fixed state
  it settles in about 22 frames at R 1.47, B 2.31. The chart's greys come
  out R/G 0.92, B/G 1.02; the vendor daemon's are R/G 0.35, B/G 1.30 (cyan).
  It then makes no change for a minute.
- **Colour:** the H62's calibrated matrix gives a mean dE of 10.4 on the
  colour checker, against the vendor daemon's 16.9 in the same light.
- **Footprint:** about 200 KB of RAM (the vendor's: 430 KB) and almost no
  CPU.
