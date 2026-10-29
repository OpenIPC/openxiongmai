// SPDX-License-Identifier: MIT
/*
 * ispreg: XM510 ISP registers from the shell, through /dev/isp (raw
 * commands 0x101 write and 0x103 read on {addr, val}), alongside a running
 * ISP daemon.
 *
 *   ispreg r ADDR [COUNT]        prints "addr value" lines
 *   ispreg w ADDR VAL [ADDR VAL]...
 */
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

struct isp_rec {
  uint32_t addr;
  uint32_t val;
};

int main(int argc, char **argv) {
  const int fd = open("/dev/isp", O_RDWR);
  if (fd < 0) {
    perror("/dev/isp");
    return 1;
  }
  if (argc >= 3 && !strcmp(argv[1], "r")) {
    const uint32_t a = strtoul(argv[2], NULL, 0);
    const int n = argc > 3 ? atoi(argv[3]) : 1;
    for (int i = 0; i < n; i++) {
      struct isp_rec r = {a + 4 * i, 0};
      if (ioctl(fd, 0x103, &r) < 0) {
        perror("read");
        return 1;
      }
      printf("%08x %08x\n", r.addr, r.val);
    }
    return 0;
  }
  if (argc >= 4 && !strcmp(argv[1], "w") && argc % 2 == 0) {
    for (int i = 2; i + 1 < argc; i += 2) {
      struct isp_rec r = {strtoul(argv[i], NULL, 0),
                          strtoul(argv[i + 1], NULL, 0)};
      if (ioctl(fd, 0x101, &r) < 0) {
        perror("write");
        return 1;
      }
    }
    return 0;
  }
  fprintf(stderr, "usage: ispreg r ADDR [COUNT] | ispreg w ADDR VAL...\n");
  return 2;
}
