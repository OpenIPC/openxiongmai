// SPDX-License-Identifier: MIT
/*
 * xm-rgn-test: draw XM530/XM550 regions (OSD) with raw /dev/rgn ioctls, for
 * the vendor xm530_rgn.ko or drivers/xm530_rgn (open_rgn.ko), alongside a
 * running streamer: the regions are drawn in VI, over every stream of the
 * channel (on majestic, VI channel 0 is the main stream and 1 the stream
 * snapshots come from).
 *
 * One operation a run, its numbers as the ioctl records hold them:
 *   reset                               remove every region (ioctl 1)
 *   pal chn c0 .. c15                   palette, 0x00YYUUVV each
 *   ocreate chn h w hh fmt              overlay; fmt 0, 1, 2: 1, 2, 4 bits a pixel
 *   oattr chn h x y colormap effect     place it
 *   obmp chn h w hh fmt pattern         load a test bitmap of the overlay's
 *                                       size and format: 0 solid, 1 a pixel at
 *                                       x 0, 2 a value ramp (8-pixel columns),
 *                                       3 a 2-pixel border
 *   oen chn h on | odestroy chn h
 *   ccreate|cattr chn h w hh x y color effect   cover; effect 1 solid
 *   cen chn h on | cdestroy chn h
 * Numbers may be hex (0x...).
 */
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

struct osd_create {
	uint32_t chn, handle, w, h, format;
};

struct bitmap {
	uint32_t chn, handle;
	uint32_t data; /* a pointer, 32-bit ARM */
	uint32_t size;
};

static int fd;

/* How many numbers each operation takes */
static const struct {
	const char *op;
	int n;
} ops[] = {
	{ "reset", 0 },    { "pal", 17 }, { "ocreate", 5 }, { "odestroy", 2 },
	{ "oattr", 6 },    { "obmp", 6 }, { "oen", 3 },     { "ccreate", 8 },
	{ "cdestroy", 2 }, { "cattr", 8 }, { "cen", 3 },
};

static int op_args(const char *op)
{
	size_t i;

	for (i = 0; i < sizeof(ops) / sizeof(ops[0]); i++)
		if (!strcmp(op, ops[i].op))
			return ops[i].n;
	return -1;
}

/* The operation's numbers into a; 0 if they are all there and numbers */
static int parse(const char *op, int argc, char **argv, uint32_t *a)
{
	int i, n = op_args(op);
	unsigned long v;
	char *end;

	if (n < 0) {
		fprintf(stderr, "unknown operation %s\n", op);
		return 2;
	}
	if (argc != n) {
		fprintf(stderr, "%s takes %d numbers\n", op, n);
		return 2;
	}
	for (i = 0; i < n; i++) {
		errno = 0;
		v = strtoul(argv[i], &end, 0);
		if (errno || end == argv[i] || *end || argv[i][0] == '-' ||
		    v > UINT32_MAX) {
			fprintf(stderr, "%s: not a number: %s\n", op, argv[i]);
			return 2;
		}
		a[i] = v;
	}
	return 0;
}

static int run(unsigned long cmd, void *arg, const char *what)
{
	int r = ioctl(fd, cmd, arg);

	if (r)
		printf("%s: %s\n", what, strerror(errno));
	else
		printf("%s: ok\n", what);
	return r ? 1 : 0;
}

/* The bitmap of an overlay w x hh at bpp bits a pixel, MSB first, rows not
 * padded */
static int load_bitmap(const uint32_t *a)
{
	uint32_t w = a[2], hh = a[3], bpp = 1u << a[4], pat = a[5];
	uint32_t max = (1u << bpp) - 1, x, y, v, bit;
	size_t n = ((size_t)w * hh * bpp + 7) / 8;
	unsigned char *b = calloc(1, n ? n : 1);
	struct bitmap r;

	if (!b)
		return 1;
	for (y = 0; y < hh; y++)
		for (x = 0; x < w; x++) {
			switch (pat) {
			case 0:
				v = max;
				break;
			case 1:
				v = x == 0 ? max : 0;
				break;
			case 2:
				v = (x / 8) & max;
				break;
			default:
				v = x < 2 || y < 2 || x + 2 >= w || y + 2 >= hh
					    ? max
					    : 0;
				break;
			}
			bit = (y * w + x) * bpp;
			b[bit / 8] |= v << (8 - bpp - bit % 8);
		}
	r.chn = a[0];
	r.handle = a[1];
	r.data = (uint32_t)(uintptr_t)b;
	r.size = n;
	return run(_IOW('R', 5, struct bitmap), &r, "obmp");
}

int main(int argc, char **argv)
{
	uint32_t a[17] = { 0 };
	const char *op;

	if (argc < 2) {
		fprintf(stderr,
			"usage: %s reset | pal chn c0..c15 |\n"
			"  ocreate chn h w hh fmt | oattr chn h x y map effect |\n"
			"  obmp chn h w hh fmt pattern | oen chn h on | odestroy chn h |\n"
			"  ccreate|cattr chn h w hh x y color effect |\n"
			"  cen chn h on | cdestroy chn h\n",
			argv[0]);
		return 2;
	}
	op = argv[1];
	if (parse(op, argc - 2, argv + 2, a))
		return 2;
	fd = open("/dev/rgn", O_RDWR);
	if (fd < 0) {
		perror("/dev/rgn");
		return 1;
	}

	if (!strcmp(op, "reset"))
		return run(_IO('R', 1), NULL, op);
	if (!strcmp(op, "ocreate"))
		return run(_IOW('R', 2, uint32_t[5]), a, op);
	if (!strcmp(op, "odestroy"))
		return run(_IOW('R', 3, uint32_t[5]), a, op);
	if (!strcmp(op, "oattr"))
		return run(_IOW('R', 4, uint32_t[6]), a, op);
	if (!strcmp(op, "obmp"))
		return load_bitmap(a);
	if (!strcmp(op, "oen"))
		return run(_IOW('R', 6, uint32_t[3]), a, op);
	if (!strcmp(op, "ccreate"))
		return run(_IOW('R', 7, uint32_t[8]), a, op);
	if (!strcmp(op, "cdestroy"))
		return run(_IOW('R', 8, uint32_t[8]), a, op);
	if (!strcmp(op, "cattr"))
		return run(_IOW('R', 9, uint32_t[8]), a, op);
	if (!strcmp(op, "cen"))
		return run(_IOW('R', 10, uint32_t[3]), a, op);
	if (!strcmp(op, "pal"))
		return run(_IOW('R', 11, uint32_t[17]), a, op);
	fprintf(stderr, "unknown operation %s\n", op);
	return 2;
}
