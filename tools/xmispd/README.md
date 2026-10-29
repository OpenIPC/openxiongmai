# xmispd

An open ISP daemon for the XM510 (MIT), a drop-in for the vendor
`/usr/bin/isp` that majestic starts. It works on `libxmisp`:
- brings up VI, the sensor and the ISP;
- prints the vendor daemon's last start-up line, `ispMsgReceive start!`,
  after which majestic starts video;
- counts frames from the `/dev/isp` frame signal;
- on SIGTERM or SIGINT, does what the vendor daemon does on its way out:
  3D noise reduction off, then exit.

Its output uses the vendor daemon's format (`\033[0;32;32mDEBUG: ...`,
`\033[0;32;31mERR: ...`), which majestic forwards to its log. It holds
`/tmp/Isplock` like the vendor daemon, so two can't run.

M1 has no AE or AWB yet: exposure and white balance stay at the sensor
script's fixed state.

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

Tested on the lab XM510 with a JX-H62 under majestic. Video streams with a
picture close to the vendor daemon's in the same light. Three stop/start
cycles each came back, with 3D-NR off between them. The daemon uses about
200 KB of RAM (the vendor's: 430 KB) and almost no CPU.
