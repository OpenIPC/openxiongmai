# xm-rgn-test

`xm-rgn-test` draws XM530/XM550 regions (OSD overlays and covers) with raw `/dev/rgn` ioctls, beside a running streamer. It works with the vendor `xm530_rgn.ko` and with `drivers/xm530_rgn` (`open_rgn.ko`); `drivers/xm530_rgn/README.md` describes the calls. MIT.

```sh
make -C tools/xm-rgn-test CROSS_COMPILE=arm-openipc-linux-musleabi-
```

On a camera with majestic running, channel 1 is the stream `/image.jpg` comes from:

```sh
T=./xm-rgn-test
$T ocreate 1 0 128 32 0            # overlay 0 of channel 1: 128 x 32, 1 bit a pixel
$T oattr 1 0 32 32 0xf0 0          # at 32,32; foreground palette 15 (white)
$T obmp 1 0 128 32 0 3             # a 2-pixel border
$T oen 1 0 1
$T ccreate 1 0 100 50 200 32 0 0
$T cattr 1 0 100 50 200 32 0x515af0 1   # a solid red cover
$T cen 1 0 1
wget -O /tmp/osd.jpg http://127.0.0.1/image.jpg
$T reset                           # remove everything
```

One operation a run; the source's header lists them all. Each prints `ok` or the error.
