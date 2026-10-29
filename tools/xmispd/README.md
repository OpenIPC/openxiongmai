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

Anti-flicker follows majestic's `isp.antiFlicker` (`disabled`, `50` or `60`,
as `/api/v1/config.schema.json` describes it). The daemon reads it from
`/etc/majestic.yaml` at start and again on SIGHUP; `XMISPD_ANTIFLICKER`
(0, 50, 60) overrides it. Changing it through majestic needs nothing more:
majestic logs `isp.antiFlicker (changed)` and, on this platform, restarts
itself, and with it the daemon.

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
- **AWB under controlled light:** the room dark, the only light three
  ceiling RGBCCT bulbs (5 W each) set to one colour temperature. For each,
  xmispd starts fresh and AWB settles; then, with xmispd stopped, a closed
  loop on the picture finds the gains that make the chart's greys neutral.
  The vendor daemon gets the same measurement.

  | Bulbs | xmispd: chart greys R/G, B/G | xmispd gains vs neutral, R / B | vendor daemon: R/G, B/G |
  |---|---|---|---|
  | 3000 K | 1.02, 1.06 | +1% / +5% | 2.56, 0.46 (orange) |
  | 4000 K | 1.02, 1.07 | +2% / +8% | 0.28, 2.11 (cyan) |
  | 5000 K | 1.00, 1.07 | +1% / +9% | 0.25, 2.07 (cyan) |
  | 6000 K | 1.00, 1.10 | +1% / +10% | 0.24, 2.00 (cyan) |

  The red gain follows the light to within 2%. The blue gain sits 5–10% high,
  so greys come out slightly cool; the bluish-white surfaces in the room are
  the likely pull. Under these bulbs the daylight colour matrix scores mean
  dE 15.5 (3000 K) and 14.6 (6000 K). A matrix fitted at 3000 K reaches only
  14.4, and is stronger, so the bulbs' LED spectrum, not the colour
  temperature, limits accuracy there.
- **Anti-flicker, under a 200 W LED lamp** (switched by the lab's
  controllable plug): with `isp.antiFlicker` disabled, AE settled at 87
  lines (1.6 ms), and the rows varied by 8.6% in rolling bands 186.4 rows
  apart. That's the 10 ms period at 53.33 µs a line (185.9 rows after VI's
  726 to 720 scaling), so it confirms the line time. With `50` AE holds one
  period (188 lines), the bands are gone (rows vary by 0.15%), and the
  picture is overexposed 1.45×.
- **Anti-flicker, dim light:** at `50` AE settles at 563 lines (3 periods),
  at `60` at 625 (4), and `disabled` returns it to 746.
- **Colour:** the H62's calibrated matrix gives a mean dE of 10.4 on the
  colour checker, against the vendor daemon's 16.9 in the same light.
- **Footprint:** about 200 KB of RAM (the vendor's: 430 KB) and almost no
  CPU.
