#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Calibrates an XM510 sensor's colour matrix against a 24-patch colour checker.

    colorchecker.py HOST --grid X0,Y0,X1,Y1 [--accept-new-host-key]

The camera runs xmispd (AWB on, the chart's greys neutral) under majestic,
with /tmp/ispreg on it. The script:
1. sets an identity colour matrix and Cb/Cr gain 1;
2. takes ten frames of /video.mp4, decodes them full-range BT.601 (the
   XM510's RGB to YCbCr has no offset), and averages each patch;
3. undoes the camera's RGB gamma (the XM510 GAMMA60 table, 61 points), which
   gives the native, white-balanced camera RGB;
4. fits a 3x3 matrix, each row summing to 1 so grey stays grey, for the least
   CIELAB error of the picture as an sRGB display would show it, through the
   camera's own gamma, against the ColorChecker Classic reference;
5. measures the picture with the fitted matrix, and with the vendor daemon's
   matrix and Cb/Cr gain as traced in the lab's daylight, and prints the
   matrix for the sensor driver.

--grid gives the centres of the first patch (dark skin, top left) and the
last (black, bottom right), in picture pixels.

The camera's root password comes from SSHPASS in the environment, for
sshpass -e; the host key is checked as the user's ssh configuration says
(by default, an unknown host is refused). For a camera not yet in
known_hosts, check its key's fingerprint on the console
(dropbearkey -y -f /etc/dropbear/dropbear_ed25519_host_key) and pass
--accept-new-host-key once, which adds it.

Requires numpy and scipy, and ffmpeg: the local one if on PATH, else the
docker image pinned in FFMPEG_IMAGE (to update it: docker pull
linuxserver/ffmpeg, then docker image inspect --format
'{{index .RepoDigests 0}}' linuxserver/ffmpeg).
"""
import argparse
import os
import shutil
import subprocess
import sys

import numpy as np
from scipy.optimize import least_squares

# ColorChecker Classic, sRGB (D65), 8-bit: the published BabelColor averages.
REF = np.array([
    [115, 82, 68], [194, 150, 130], [98, 122, 157], [87, 108, 67], [133, 128, 177], [103, 189, 170],
    [214, 126, 44], [80, 91, 166], [193, 90, 99], [94, 60, 108], [157, 188, 64], [224, 163, 46],
    [56, 61, 150], [70, 148, 73], [175, 54, 60], [231, 199, 31], [187, 86, 149], [8, 133, 161],
    [243, 243, 242], [200, 200, 200], [160, 160, 160], [122, 122, 121], [85, 85, 85], [52, 52, 52]], float)

# The XM510 RGB gamma as the JX-H62 script sets it (GAMMA60, 0x40062040..):
# outputs at inputs 0..512 in steps of 16, ..1024 by 32, ..2048 by 128,
# ..4096 by 512.
GAMMA60 = [0, 54, 106, 158, 209, 259, 308, 356, 403, 450, 495, 540, 584, 628, 670, 713, 754, 795,
           835, 874, 913, 951, 989, 1026, 1062, 1098, 1133, 1168, 1203, 1236, 1270, 1303, 1335, 1398,
           1460, 1520, 1578, 1635, 1690, 1744, 1796, 1848, 1897, 1946, 1993, 2040, 2085, 2129, 2172,
           2335, 2483, 2620, 2746, 2862, 2970, 3070, 3164, 3482, 3732, 3933, 4095]
KNOTS = (list(range(0, 513, 16)) + list(range(544, 1025, 32)) + list(range(1152, 2049, 128))
         + list(range(2560, 4097, 512)))

FFMPEG_IMAGE = "linuxserver/ffmpeg@sha256:6547359fac23a2747e606d380e10267b2923a38f68485d9cacefd039e74d3aa8"

CSM_BANKS = (0x40061040, 0x40061080)
CSM_SWITCH = 0x40061000
CB_GAIN, CR_GAIN = 0x40076004, 0x40076008
# The vendor daemon's matrix and Cb/Cr gain with the lab colour checker in
# view in daylight, where its AWB had converged (last values in the trace
# h62-720p-colorchecker). Its matrix moves with the light (80-150 distinct
# ones a trace), so this is a baseline for that light only.
VENDOR = [[197, 98, -39], [-90, 360, -14], [-62, -78, 396]]
VENDOR_CBCR = 0x20e
SRGB_XYZ = np.array([[0.4124, 0.3576, 0.1805], [0.2126, 0.7152, 0.0722], [0.0193, 0.1192, 0.9505]])


def gamma(x):
    return np.interp(np.asarray(x) * 4096, KNOTS, GAMMA60) / 4095


def inv_gamma(y):
    return np.interp(np.asarray(y) * 4095, GAMMA60, KNOTS) / 4096


def srgb_lin(c):
    c = np.asarray(c) / 255
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def lab(lin):
    xyz = lin @ SRGB_XYZ.T / np.array([0.9505, 1.0, 1.089])
    f = np.where(xyz > 0.008856, np.cbrt(xyz), 7.787 * xyz + 16 / 116)
    return np.stack([116 * f[..., 1] - 16, 500 * (f[..., 0] - f[..., 1]), 200 * (f[..., 1] - f[..., 2])], -1)


class Camera:
    def __init__(self, host, accept_new):
        if "SSHPASS" not in os.environ:
            sys.exit("set SSHPASS to the camera's root password")
        # Host keys as the user's ssh configuration checks them (by default,
        # an unknown host is refused without a terminal to ask on).
        self.ssh = ["sshpass", "-e", "ssh", "-o", "LogLevel=ERROR"]
        if accept_new:
            self.ssh += ["-o", "StrictHostKeyChecking=accept-new"]
        self.ssh.append(f"root@{host}")

    def run(self, cmd, **kw):
        return subprocess.run(self.ssh + [cmd], capture_output=True, **kw)

    def ispreg(self, args):
        r = self.run("/tmp/ispreg " + args, text=True)
        if r.returncode:
            sys.exit(f"ispreg {args.split()[0]}: {r.stderr.strip() or 'failed'} (exit {r.returncode})")
        return r.stdout

    def write(self, pairs):
        self.ispreg("w " + " ".join(f"{a:#x} {v & 0xffffffff:#x}" for a, v in pairs))

    def read(self, addr, n):
        return [int(line.split()[1], 16) for line in self.ispreg(f"r {addr:#x} {n}").splitlines()]

    def matrix(self, m256, cbcr):
        pairs = []
        for bank in CSM_BANKS:
            for row in range(3):
                pairs.append((bank + row * 0x10, 0))
                pairs += [(bank + row * 0x10 + 4 + col * 4, int(m256[row][col])) for col in range(3)]
        self.write(pairs + [(CSM_SWITCH, 0xffffffff), (CB_GAIN, cbcr), (CR_GAIN, cbcr)])
        # The coefficients read back as 13-bit two's complement.
        for bank in CSM_BANKS:
            got = [self.read(bank + row * 0x10 + 4, 3) for row in range(3)]
            got = [[v - 0x2000 if v & 0x1000 else v for v in row] for row in got]
            if got != [[int(v) for v in row] for row in m256]:
                sys.exit(f"colour matrix at {bank:#x} reads back {got}, not {m256}")
        if self.read(CB_GAIN, 2) != [cbcr, cbcr]:
            sys.exit("Cb/Cr gain did not take")

    def frames(self, n=10):
        """The last n frames of a clip, full-range BT.601 decoded, averaged: HxWx3 RGB."""
        fsz = 1280 * 720 * 3
        for _ in range(6):
            mp4 = self.run("timeout 3 wget -q -O /tmp/cc.mp4 http://127.0.0.1/video.mp4 2>/dev/null;"
                           " cat /tmp/cc.mp4; rm -f /tmp/cc.mp4").stdout
            raw = decode(mp4, n)
            k = len(raw) // fsz
            if k:
                break
            subprocess.run(["sleep", "2"])  # majestic allows one viewer on this board
        else:
            sys.exit("no frames decoded")
        yuv = np.frombuffer(raw[:k * fsz], np.uint8).reshape(k, 3, 720, 1280).astype(float).mean(0)
        y, cb, cr = yuv[0], yuv[1] - 128, yuv[2] - 128
        return np.stack([y + 1.402 * cr, y - 0.344136 * cb - 0.714136 * cr, y + 1.772 * cb], -1).clip(0, 255)


def decode(mp4, n):
    """The last n frames of an MP4 as raw YUV 4:4:4, through ffmpeg."""
    args = f"-loglevel error -sseof -1 -i {{}} -frames:v {n} -f rawvideo -pix_fmt yuv444p -"
    if shutil.which("ffmpeg"):
        import tempfile
        with tempfile.NamedTemporaryFile(suffix=".mp4") as f:
            f.write(mp4)
            f.flush()
            return subprocess.run(["ffmpeg"] + args.format(f.name).split(), capture_output=True).stdout
    return subprocess.run(["docker", "run", "--rm", "-i", "--entrypoint", "sh", FFMPEG_IMAGE, "-c",
                           "cat > /tmp/in.mp4; ffmpeg " + args.format("/tmp/in.mp4")],
                          input=mp4, capture_output=True).stdout


def patches(rgb, grid):
    x0, y0, x1, y1 = grid
    out = []
    for row in range(4):
        for col in range(6):
            x, y = round(x0 + (x1 - x0) * col / 5), round(y0 + (y1 - y0) * row / 3)
            out.append(rgb[y - 10:y + 10, x - 10:x + 10].reshape(-1, 3).mean(0))
    return np.array(out)


def score(p8):
    e = lab(srgb_lin(p8)) - lab(srgb_lin(REF))
    clipped = int(((p8 >= 254) | (p8 <= 1)).any(1).sum())
    return np.linalg.norm(e, axis=1), np.linalg.norm(e[:, 1:], axis=1), clipped


def fit(cam):
    ref = lab(srgb_lin(REF))

    def unpack(x):
        return np.array([[x[0], x[1], 1 - x[0] - x[1]], [x[2], x[3], 1 - x[2] - x[3]],
                         [x[4], x[5], 1 - x[4] - x[5]]]), x[6]

    def res(x):
        m, k = unpack(x)
        return (lab(srgb_lin(gamma(((cam * k) @ m.T).clip(0, 1)) * 255)) - ref).ravel()

    m, _ = unpack(least_squares(res, [1, 0, 0, 1, 0, 0, 1]).x)
    m256 = np.round(m * 256).astype(int)
    for row in range(3):
        m256[row, row] += 256 - m256[row].sum()  # grey stays grey, exactly
    return m256


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("host")
    ap.add_argument("--accept-new-host-key", action="store_true",
                    help="add the camera's host key to known_hosts if it isn't there (check it first)")
    ap.add_argument("--grid", required=True, help="X0,Y0,X1,Y1: first and last patch centres")
    a = ap.parse_args()
    grid = [float(v) for v in a.grid.split(",")]
    cam = Camera(a.host, a.accept_new_host_key)

    cam.matrix([[256, 0, 0], [0, 256, 0], [0, 0, 256]], 0x100)
    native = patches(cam.frames(), grid)
    lin = inv_gamma(native / 255)
    greys = lin[19:23]
    print(f"greys R/G {np.mean(greys[:, 0] / greys[:, 1]):.3f} B/G {np.mean(greys[:, 2] / greys[:, 1]):.3f}"
          " (white balance; near 1 wanted)")
    m256 = fit(lin * (srgb_lin(REF)[19:23, 1].sum() / greys[:, 1].sum()))

    for name, m, cbcr in (("vendor's (traced), Cb/Cr 2.05x", VENDOR, VENDOR_CBCR),
                          ("fitted matrix, Cb/Cr 1", m256, 0x100)):
        cam.matrix(m, cbcr)
        de, dc, clipped = score(patches(cam.frames(), grid))
        print(f"{name:28} dE mean {de.mean():5.1f} max {de.max():5.1f}, chroma {dc.mean():5.1f}, "
              f"clipped patches {clipped}")
    print("ccm (rows sum 256):")
    for row, ch in zip(m256, "RGB"):
        print(f"    {row[0]}, {row[1]}, {row[2]}, /* {ch} */")


if __name__ == "__main__":
    main()
