// SPDX-License-Identifier: GPL-2.0
/*
 * xm_v200_i2c: the V200 series' I2C masters (sensor control), a drop-in
 * replacement for the vendor's i2c.ko (built as open_i2c.ko).
 *
 * Three buses, each a misc device: /dev/i2c0..2, or on the XM533
 * /dev/xm_i2c, /dev/xm_i2c1, /dev/xm_i2c2. A call takes a 20-byte record
 * {dev, reg, reg_bytes, val, val_bytes}: ioctl 1 writes reg then val, ioctl
 * 3 writes reg and reads val_bytes into val (written back), ioctl 4 sets the
 * bus up again. Register and value go most significant byte first.
 *
 * The controller, bus n at 0x10000000 + n * 0x1000:
 *   +0x00 control: 0x40 / 0x80 reset the TX / RX FIFO; 0xb sends what the
 *         TX FIFO holds (address, then the bytes); 0x803 then reads; 1 stops
 *   +0x0c the device address   +0x10 TX FIFO   +0x14 RX FIFO
 *   +0x18 bytes to send        +0x1c bytes to receive
 *   +0x20..0x38 timing (SCL high/low, setup and hold counts)
 *   +0x44 interrupt clear      +0x48 status: 0x10 sent, 0x20 received
 * The vendor polls the status (every 10 us, about 40 ms in all) and so
 * does this. Opening a bus sets its pins to I2C (the pin mux at 0x10020000);
 * the XM530V200 and XM535 also reset each bus in the clock block
 * (0x20000090 + 4n) when they set it up.
 *
 * The chips differ in the timing (their I2C clock), the pins, the names
 * and the resets: I2C_CHIP=xm530v200 (default), xm533 or xm535 at build
 * time.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>

#define I2C_BASE 0x10000000
#define PINMUX_BASE 0x10020000
#define RESET_BASE 0x20000090 /* in the clock block, a register per bus */
#define N_BUS 3

#define I2C_CTRL 0x00
#define I2C_ADDR 0x0c
#define I2C_TX 0x10
#define I2C_RX 0x14
#define I2C_TX_COUNT 0x18
#define I2C_RX_COUNT 0x1c
#define I2C_TIMING 0x20
#define I2C_INT_CLEAR 0x44
#define I2C_STATUS 0x48
#define ST_SENT 0x10
#define ST_RECEIVED 0x20

#define I2C_IOC_WRITE 1
#define I2C_IOC_READ 3
#define I2C_IOC_INIT 4

struct i2c_rec {
	u8 dev;
	u8 pad[3];
	u32 reg, reg_bytes, val, val_bytes;
};

struct pin {
	u16 off;
	u8 val;
};

struct chip {
	const char *names[N_BUS];
	u32 timing[7];
	bool reset;	    /* the clock block resets the bus at set-up */
	bool write_fifo_reset; /* a write resets the TX FIFO first */
	struct pin pins[N_BUS][2];
};

#if defined(I2C_CHIP_XM533)
static const struct chip chip = {
	{"xm_i2c", "xm_i2c1", "xm_i2c2"},
	{0xa8, 0xa8, 0xa8, 0xa8, 0xa8, 0x50, 0xfc},
	false,
	false,
	{{{0x80, 0x82}, {0x88, 0x82}},
	 {{0x28, 0x84}, {0x30, 0x84}},
	 {{0x2c, 0x84}, {0x34, 0x84}}},
};
#elif defined(I2C_CHIP_XM535)
static const struct chip chip = {
	{"i2c0", "i2c1", "i2c2"},
	{0xa8, 0xa8, 0xa8, 0xa8, 0xa8, 0x50, 0xfc},
	true,
	true,
	{{{0x80, 0x72}, {0x88, 0x72}},
	 {{0x28, 0x74}, {0x2c, 0x74}},
	 {{0x178, 0x72}, {0x17c, 0x72}}},
};
#else /* XM530V200 */
static const struct chip chip = {
	{"i2c0", "i2c1", "i2c2"},
	{0x23, 0x23, 0x23, 0x23, 0x23, 0x13, 0x34},
	true,
	true,
	{{{0x80, 0x82}, {0x88, 0x82}},
	 {{0x28, 0x84}, {0x30, 0x84}},
	 {{0x2c, 0x84}, {0x34, 0x84}}},
};
#endif

static void __iomem *bus_regs[N_BUS], *pinmux, *resets;
static DEFINE_MUTEX(i2c_lock);
static struct miscdevice devs[N_BUS];

static u32 rd(int bus, unsigned int off)
{
	return readl(bus_regs[bus] + off);
}

static void wr(int bus, unsigned int off, u32 v)
{
	writel(v, bus_regs[bus] + off);
}

/* caller holds i2c_lock */
static void bus_setup(int bus)
{
	int i;

	if (chip.reset) {
		writel(0xa9b10000 | bus, resets + bus * 4);
		udelay(100);
		writel(1, resets + bus * 4);
	}
	udelay(100);
	for (i = 0; i < 7; i++)
		wr(bus, I2C_TIMING + i * 4, chip.timing[i]);
}

static void fifo_reset(int bus, u32 bit)
{
	u32 c = rd(bus, I2C_CTRL);

	wr(bus, I2C_CTRL, c | bit);
	udelay(1);
	wr(bus, I2C_CTRL, c & ~bit);
	udelay(1);
}

static void push(int bus, u32 v, int n)
{
	while (n-- > 0)
		wr(bus, I2C_TX, (v >> (n * 8)) & 0xff);
}

static bool wait_status(int bus, u32 bit)
{
	int i;

	for (i = 0; i < 0x1001; i++) {
		if (rd(bus, I2C_STATUS) & bit)
			return true;
		udelay(10);
	}
	return false;
}

/* a transfer that did not finish: stop, clear, and set the bus up again
 * unless it still has its timing (as the vendor's); caller holds the lock */
static int fail(int bus, const char *what, u8 dev, u32 reg)
{
	pr_err("i2c%d %s dev %#x reg %#x timeout %#x\n", bus, what, dev, reg,
	       rd(bus, I2C_STATUS));
	wr(bus, I2C_CTRL, 1);
	wr(bus, I2C_INT_CLEAR, 0xffff);
	if (rd(bus, I2C_TIMING) != chip.timing[0])
		bus_setup(bus);
	return -1;
}

static int i2c_xfer_write(int bus, u8 dev, u32 reg, int rb, u32 val, int vb)
{
	int ret = 0;

	mutex_lock(&i2c_lock);
	wr(bus, I2C_ADDR, dev);
	wr(bus, I2C_TX_COUNT, rb + vb);
	if (chip.write_fifo_reset)
		fifo_reset(bus, 0x40);
	push(bus, reg, rb);
	push(bus, val, vb);
	wr(bus, I2C_TX, 0);
	wr(bus, I2C_CTRL, 0xb);
	if (wait_status(bus, ST_SENT))
		wr(bus, I2C_INT_CLEAR, ST_SENT);
	else
		ret = fail(bus, "write", dev, reg);
	mutex_unlock(&i2c_lock);
	return ret;
}

static int i2c_xfer_read(int bus, u8 dev, u32 reg, int rb, int vb, u32 *val)
{
	u32 v = 0;
	int i, ret = 0;

	mutex_lock(&i2c_lock);
	wr(bus, I2C_ADDR, dev);
	wr(bus, I2C_TX_COUNT, rb);
	fifo_reset(bus, 0x40);
	push(bus, reg, rb);
	wr(bus, I2C_TX, 0);
	wr(bus, I2C_CTRL, 0xb);
	if (!wait_status(bus, ST_SENT)) {
		ret = fail(bus, "read", dev, reg);
		goto out;
	}
	wr(bus, I2C_INT_CLEAR, ST_SENT);
	wr(bus, I2C_RX_COUNT, vb);
	fifo_reset(bus, 0x80);
	wr(bus, I2C_CTRL, 0x803);
	if (!wait_status(bus, ST_RECEIVED)) {
		ret = fail(bus, "read", dev, reg);
		goto out;
	}
	for (i = 0; i < vb; i++)
		v = v << 8 | (rd(bus, I2C_RX) & 0xff);
	wr(bus, I2C_INT_CLEAR, ST_RECEIVED);
	*val = v;
out:
	mutex_unlock(&i2c_lock);
	return ret;
}

static int bus_of(struct file *file)
{
	int minor = iminor(file_inode(file)), i;

	for (i = 0; i < N_BUS; i++)
		if (devs[i].minor == minor)
			return i;
	return -1;
}

static int i2c_open(struct inode *inode, struct file *file)
{
	int bus = bus_of(file), i;

	if (bus < 0)
		return -ENODEV;
	/* the bus's pins to I2C */
	for (i = 0; i < 2; i++)
		writel(chip.pins[bus][i].val, pinmux + chip.pins[bus][i].off);
	return 0;
}

static int i2c_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long i2c_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_rec r;
	int bus = bus_of(file), ret;

	if (bus < 0)
		return -1;
	/* the bus reset takes no record */
	if (cmd == I2C_IOC_INIT) {
		mutex_lock(&i2c_lock);
		bus_setup(bus);
		mutex_unlock(&i2c_lock);
		return 0;
	}
	if (copy_from_user(&r, (void __user *)arg, sizeof(r))) {
		pr_err("i2c: copy_from_user error\n");
		return -1;
	}
	if (r.reg_bytes > 4 || r.val_bytes > 4)
		return -EINVAL;
	switch (cmd) {
	case I2C_IOC_WRITE:
		return i2c_xfer_write(bus, r.dev, r.reg, r.reg_bytes, r.val,
				      r.val_bytes);
	case I2C_IOC_READ:
		ret = i2c_xfer_read(bus, r.dev, r.reg, r.reg_bytes,
				    r.val_bytes, &r.val);
		if (ret)
			return ret;
		if (copy_to_user((void __user *)arg, &r, sizeof(r))) {
			pr_err("i2c: copy_to_user error\n");
			return -1;
		}
		return 0;
	}
	pr_err("i2c: command error %#x\n", cmd);
	return -1;
}

static const struct file_operations i2c_fops = {
	.owner = THIS_MODULE,
	.open = i2c_open,
	.release = i2c_release,
	.unlocked_ioctl = i2c_ioctl,
};

static void i2c_unmap(void)
{
	int i;

	for (i = 0; i < N_BUS; i++)
		if (bus_regs[i])
			iounmap(bus_regs[i]);
	if (pinmux)
		iounmap(pinmux);
	if (resets)
		iounmap(resets);
}

static int __init xm_v200_i2c_init(void)
{
	int i, ret;

	for (i = 0; i < N_BUS; i++)
		bus_regs[i] = ioremap(I2C_BASE + i * 0x1000, 0x1000);
	pinmux = ioremap(PINMUX_BASE, 0x200);
	resets = ioremap(RESET_BASE, 4 * N_BUS);
	if (!bus_regs[0] || !bus_regs[1] || !bus_regs[2] || !pinmux ||
	    !resets) {
		i2c_unmap();
		return -ENOMEM;
	}
	for (i = 0; i < N_BUS; i++) {
		devs[i].minor = MISC_DYNAMIC_MINOR;
		devs[i].name = chip.names[i];
		devs[i].fops = &i2c_fops;
		ret = misc_register(&devs[i]);
		if (ret) {
			pr_err("i2c: %s init error!\n", chip.names[i]);
			while (i-- > 0)
				misc_deregister(&devs[i]);
			i2c_unmap();
			return ret;
		}
	}
	mutex_lock(&i2c_lock);
	for (i = 0; i < N_BUS; i++)
		bus_setup(i);
	mutex_unlock(&i2c_lock);
	pr_info("i2c: openxiongmai xm_v200_i2c init ok (%s)\n", chip.names[0]);
	return 0;
}

static void __exit xm_v200_i2c_exit(void)
{
	int i;

	for (i = 0; i < N_BUS; i++)
		misc_deregister(&devs[i]);
	i2c_unmap();
}

module_init(xm_v200_i2c_init);
module_exit(xm_v200_i2c_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("V200-series I2C masters, compatible with i2c.ko");
