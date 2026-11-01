# libisp_XM530

An open `libisp_XM530.so` for the XM530/XM550: the ISP library majestic
loads next to `libispapp_XM530.so` and the sensor library, in place of the
vendor's (issue #76). It is laid out as openhisilicon's `libraries/isp`:
`src/main` for what any chip shares, `arch/xm530` for this chip's tables. It
builds against the SDK headers in `libraries/isp/include`, as `libispapp`
does.

```sh
make CROSS_COMPILE=arm-openipc-linux-musleabi-
```

## What it does (M1, first part)

- **Exports** every function majestic, `libispapp_XM530` and the sensor
  libraries import from the vendor library: the `XM_MPI_ISP_*` calls, the
  `XM_MPI_VDA_*` calls they use, `Read_IspReg`/`Write_IspReg`,
  `ispMemnCpy`, `SysDelay_ms`, `IspSetSlowFrameRate`, `Mipi_check_*`.
- **Registers** go through `/dev/isp` (`_IOW('I', 1/2, 12)`), the system
  controller through `/dev/mem`.
- **3A:** AE and AWB are majestic's. `MemInit` takes their functions from
  `XM_MPI_ISP_AELibRegCallBack`/`AWBLibRegCallBack` and gives each its handle
  (`ctrl` command 1). `Init` runs their inits and lays the AWB statistics out
  over the window `awb_init` returns. `Run` starts a thread that, every
  frame, runs AWB (it reads the statistics itself, through `Read_IspReg`, and
  its gains and colour matrix go to `AWB_BASE` and the CSM), makes the luma
  grids AE takes from what AWB read, and runs AE (it sets the sensor itself;
  its ISP gain goes to `DIGITAL_GAIN`). `tools/xm-isp-trace/README.md` has
  the interface as traced.
- **Start-up** replays what the vendor library wrote for the sensor and mode
  (`arch/xm530/xm530_imx335.c`, generated from a trace by
  `tools/xm-isp-trace/extract530.py`), with the 3D noise reduction's buffers
  in an "Isp" MMZ block of its own, and starts with the attributes the vendor
  library's `Init` set.
- **Attributes** are kept and returned. The gamma curve takes effect at once,
  and a saturation of 0 makes the picture black and white (Cb/Cr gain 0), as
  with the vendor library.
- **Tuning by gain** (`src/main/isp_tune.c`): every 7 frames or so, as the
  vendor library's loop, from the attributes' auto tables (one entry an ISO
  octave from 100, interpolated linearly in ISO): 2D noise reduction,
  sharpening, black level, Cb/Cr gain, dynamic defect pixels and DRC (its
  strength from the defog table, its level the largest local tone mapping
  statistic), and the luma curve once. Each was fitted against the vendor
  library at eleven light levels (ISO 100 to 12800).
- **Motion detection** (`XM_MPI_VDA_*`, `src/main/mpi_vda.c`) on the AE
  library's 7 × 7 grid, as the vendor library behaves: every `MoDframe + 1`
  grids the grid is compared with the last analysed one, a zone moved when it
  differs by more than its 1/`MoRatio`; `MoVdaBmark` marks every moved zone,
  the share of moved zones among `MoWinset`'s is returned (and sets
  `MoReflag` from `u8RatioThresh`), and results gather until read. On the lab
  XM550 it answers bulb changes and a blinking LED's zone as the vendor
  library does. Cover detection reports nothing yet.
- **Restarts:** majestic's AE and AWB keep their state, and AE its last
  statistics, in the block `/dev/isp` shares, at the vendor library's places,
  so a restarted majestic starts AE where it was, as with the vendor library.

## Not yet

- Only the IMX335 at 2600×1954 has start-up tables; `Init` fails for any
  other size.
- 3D noise reduction keeps its start-up values (the vendor library moves it
  with gain and motion). The chroma and anti-false-colour settings are kept
  but not applied.
- After `Init`, the ISP application's MIPI start leaves `0x300f0010` at 0
  with this library (1 with the vendor's), and no frame comes; `Run` restarts
  the receiver and sets it. Why is not known yet.

## Testing on a camera

Majestic stopped, the library bind-mounted over the vendor one:

```sh
mount --bind /tmp/libisp_XM530.so /usr/lib/libisp_XM530.so
LIBISP_DEBUG=1 majestic    # every 50 frames: grids, AWB gains, AE result
```

On the lab XM550 (IMX335, 4000 K bulbs), restarted after the vendor library,
it settles on the same sensor gain and frame rate, and the picture's mean
R/G/B is within 2 of the vendor library's at 100%, 30% and 2% light. Both
libraries start AE from the last run's state: compare one restart against
another, or two cold boots, since from a cold boot AE starts from nothing
and in dim light may drop to a slower frame rate.
