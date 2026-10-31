// SPDX-License-Identifier: GPL-2.0
/*
 * The V200 series' pan/tilt step motors (/dev/motor): a drop-in for the
 * vendor kmotor.ko (XM530V200, XM533, XM535D1/D2/W1).
 *
 * Two unipolar step motors, X (pan) and Y (tilt), half-stepped through
 * eight coil patterns (A, AB, B, BC, C, CD, D, DA). One SP804 pair at
 * 0x100D0000 paces them, X on its first timer and Y on its second, both on
 * IRQ 37; each tick moves the motor one half step. 4096 half steps are a
 * turn: MAX_DEGREE_X/Y give the travel (max_steps = degree * 4096 / 360).
 *
 * The coils are driven one of two ways:
 *  - GPIOs (the usual): gpio_pin= A, B, C, D of X, then D, C, B, A of Y,
 *    then the limit switches X right, X left, Y up, Y down (active low,
 *    with limit=1; -1 none). A pin's register is 0x10020000 + 4 * pin
 *    (0xc80 high, 0x480 low, 0x1080 input, bit 13 its level); a pin above
 *    1000 is the XM533's MIPI pad pin - 1000 (open_mipi_io's
 *    mipi_io_set_output).
 *  - a 10-bit shift register (iic_drive=1, the XM535D2's X9 boards):
 *    IIC_SDA / IIC_SCL clock the bits in LSB first, IIC_EN latches them.
 *    Bits 0..3 are X's coils (A = bit 3), 4..7 Y's, 8 and 9 the IR-cut
 *    filter's two directions (ioctl 0x40044d04). IIC_RST is held low.
 * pin_antitone (and the D2's pin_antitone1, XORed with it) turns a motor
 * round: bit 0 X, bit 1 Y.
 *
 * Speeds: a motor's tick is tick(i) 12 MHz counts (times the timer clock
 * multiple, divided by speed_up), i from 0 (slowest, ~45 half steps/s) to
 * 60 (~570/s). A user speed 0..17 is index 7, 12, .. 52, 53, .. 60; the
 * text commands give an index + 8 (X) or + 5 (Y) instead.
 *
 * Commands, ioctl 0x40184d01 {cmd, a1..a5} or written as text:
 *   1 left, 2 right, 4 up, 8 down: a1 the X speed, a2 the Y speed, until
 *     stopped; 5 left-up, 6 right-up, 9 left-down, 10 right-down both
 *     ("left N", "right N", "up N", "down N", "leftup N", "rightup N",
 *     "leftdown N", "rightdown N"); a5 = 0x5867 marks the speeds as text
 *     indices. A command repeated changes only the speed.
 *   0x10 scan: speed a1 between X a2 and a4 (Y a3), to and fro until
 *     stopped ("lscan", "rscan" mark the ends where the camera is, "scan
 *     [N]" runs it, speed 25 by default)
 *   0x11 go to X a1, Y a2 at speeds a3, a4 ("setpreset" marks one,
 *     "preset" goes there)
 *   0x12 stop ("stop");  0x13 the end-to-end test, then the middle
 *   0x14 the position is X a1, Y a2 (when stopped)
 * ioctl 0x80184d02 reads {action, X position, Y position, X flags, Y
 * flags, 0}; 0x80184d03 {action, X max steps, Y max steps, MAX_DEGREE_X <<
 * 16 | MAX_DEGREE_Y, 0, 0}. Flags: 2 a waiter, 4 running, 0x10 the
 * end-to-end test done (X). Action 0xffff is idle. Until the test is done
 * (auto_test=1 runs it at load) or skipped (auto_test=0), commands fail.
 * /proc/motor shows the state.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

int mipi_io_set_output(u32 pin, u32 level);
int mipi_io_get_input(u32 pin);

#define TIMER_BASE	0x100d0000
#define TIMER_IRQ	37
#define T_LOAD		0x00
#define T_CTRL		0x08
#define T_INTCLR	0x0c
#define T_MIS		0x14
#define T_BGLOAD	0x18
#define T_ENABLE	0x80
/* periodic, 32-bit, interrupting (kmotor.ko runs it free and reloads it
 * every tick: the same on the chip) */
#define T_MODE		0x62
#define T_IDLE_LOAD	12000000

#define GPIO_BASE	0x10020000
#define GPIO_PINS	1000
#define MIPI_PIN	1000
#define PIN_HIGH	0xc80
#define PIN_LOW		0x480
#define PIN_INPUT	0x1080
#define PIN_LEVEL	BIT(13)
/* the shift register's pins: no 0x80 */
#define SPIN_HIGH	0xc00
#define SPIN_LOW	0x400

#define SYS_BASE	0x20000000

#define MOTOR_CMD	0x40184d01
#define MOTOR_STATUS	0x80184d02
#define MOTOR_INFO	0x80184d03
#define MOTOR_IRCUT	0x40044d04

#define DIR_LEFT	1
#define DIR_RIGHT	2
#define DIR_UP		4
#define DIR_DOWN	8

#define ACT_SCAN	0x10
#define ACT_PRESET	0x11
#define ACT_STOP	0x12
#define ACT_TEST	0x13
#define ACT_SETPOS	0x14
#define ACT_IDLE	0xffff

#define F_WAIT		BIT(1)
#define F_RUN		BIT(2)
#define F_AUTO		BIT(3)
#define F_TESTED	BIT(4)
#define F_STOPPING	BIT(5)

#define RAW_SPEED	0x5867

static int IIC_EN = 8, IIC_SDA = 13, IIC_SCL = 11, IIC_RST = 276;
static int pin_antitone, pin_antitone1, AUTOTEST_SPEED = 8;
static int MID_DEGREE_X, MID_DEGREE_Y, MAX_DEGREE_X = 345, MAX_DEGREE_Y = 135;
static int gpio_pin[12] = { 3, 70, 75, 77, 74, 76, 69, 71, -1, -1, -1, -1 };
static int pin_num = 12;
static int gpio_mux_type = 1;
static int soft_limit, limit, auto_test = 1, speed_up = 1;
#ifdef MOTOR_IIC
static int iic_drive = 1;
#else
static int iic_drive;
#endif
static int timer_mul;

module_param(IIC_EN, int, 0444);
module_param(IIC_SDA, int, 0444);
module_param(IIC_SCL, int, 0444);
module_param(IIC_RST, int, 0444);
module_param(pin_antitone, int, 0444);
module_param(pin_antitone1, int, 0444);
module_param(AUTOTEST_SPEED, int, 0444);
module_param(MID_DEGREE_Y, int, 0444);
module_param(MID_DEGREE_X, int, 0444);
module_param(MAX_DEGREE_Y, int, 0444);
module_param(MAX_DEGREE_X, int, 0444);
module_param_array(gpio_pin, int, &pin_num, 0444);
module_param(gpio_mux_type, int, 0444);
module_param(soft_limit, int, 0644);
module_param(limit, int, 0444);
module_param(auto_test, int, 0444);
module_param(speed_up, int, 0444);
MODULE_PARM_DESC(iic_drive, "coils through the shift register (XM535D2 X9)");
module_param(iic_drive, int, 0444);
MODULE_PARM_DESC(timer_mul, "timer clock in 12 MHz units, 0: from the chip");
module_param(timer_mul, int, 0444);

/* half-step coil patterns: A = 8, B = 4, C = 2, D = 1 */
static const u8 phases[8] = { 0x8, 0xc, 0x4, 0x6, 0x2, 0x3, 0x1, 0x9 };

struct axis {
	u32 steps, pos, speed, cur, dir, flags, phase, max;
	u32 plus, minus;	/* directions */
	int coil[4];		/* A..D */
	int lim_plus, lim_minus;
	bool reverse;
	void __iomem *t;
	const char *name;
};

static struct axis ax[2];
#define AX (&ax[0])
#define AY (&ax[1])
static u32 action = ACT_IDLE;
static DEFINE_SPINLOCK(lock);
static DECLARE_WAIT_QUEUE_HEAD(idle_wq);

static void __iomem *gpio, *timers;
static u32 mul = 1;
static int (*mipi_out)(u32, u32);
static int (*mipi_in)(u32);
static int s_max_steps_x = -1, s_max_steps_y = -1;

/* the shift register */
static DEFINE_MUTEX(iic_lock);
static u32 iic_data, iic_sent = ~0u;
static DECLARE_WAIT_QUEUE_HEAD(iic_wq);
static struct task_struct *iic_task;

/* commands for the helper thread */
struct cmd {
	u32 c, a[5];
};
static struct cmd pending;
static bool pending_valid;
static DECLARE_WAIT_QUEUE_HEAD(helper_wq);
static struct task_struct *helper;
static int test_req, test_done;

static struct {
	u32 state;	/* 0 off, 1 last leg right, 2 left, 3 to the start */
	u32 tick, left, right, delay;
} scan;
static struct {
	bool set;
	u32 x, y;
} my_preset;
static struct {
	bool set;
	u32 lx, ly, rx, ry;
} my_scan;

static u32 tick(u32 i)
{
	i = min(i, 60u);
	return 33750000 / (i <= 52 ? 128 + 16 * i : 960 + 80 * (i - 52));
}

static u32 reflect(u32 s)
{
	s = min(s, 17u);
	return s < 10 ? 7 + 5 * s : 43 + s;
}

static void pin_write(int pin, u32 v)
{
	if (pin < 0)
		return;
	if (pin > MIPI_PIN) {
		if (mipi_out)
			mipi_out(pin - MIPI_PIN, v == PIN_HIGH);
		return;
	}
	if (pin < GPIO_PINS)
		writel(v, gpio + 4 * pin);
}

/* a limit switch, active low; never hit without one */
static bool limit_hit(int pin)
{
	if (limit != 1 || pin < 0)
		return false;
	if (pin > MIPI_PIN)
		return mipi_in && mipi_in(pin - MIPI_PIN) == 0;
	return pin < GPIO_PINS && !(readl(gpio + 4 * pin) & PIN_LEVEL);
}

static u32 pattern(struct axis *a, u32 phase)
{
	u32 p = phases[phase & 7];

	/* turned round: A <-> D, B <-> C */
	if (a->reverse)
		p = (p & 8) >> 3 | (p & 4) >> 1 | (p & 2) << 1 | (p & 1) << 3;
	return p;
}

/* the coils, under lock */
static void coils(struct axis *a, u32 p)
{
	int i;

	if (iic_drive) {
		if (a == AX)
			iic_data = (iic_data & 0xf0) | p;
		else
			iic_data = (iic_data & 0x0f) | p << 4;
		wake_up(&iic_wq);
		return;
	}
	for (i = 0; i < 4; i++)
		pin_write(a->coil[i], p & (8 >> i) ? PIN_HIGH : PIN_LOW);
}

static void iic_pin(int pin, bool high)
{
	if (pin >= 0 && pin < GPIO_PINS)
		writel(high ? SPIN_HIGH : SPIN_LOW, gpio + 4 * pin);
}

/* ten bits in, LSB first, then latched; under iic_lock */
static void iic_send(u32 v)
{
	int i;

	iic_pin(IIC_EN, false);
	for (i = 0; i < 10; i++, v >>= 1) {
		iic_pin(IIC_SDA, v & 1);
		udelay(25);
		iic_pin(IIC_SCL, true);
		udelay(25);
		iic_pin(IIC_SCL, false);
	}
	iic_pin(IIC_EN, true);
	udelay(75);
	iic_pin(IIC_EN, false);
}

static int iic_thread(void *data)
{
	unsigned long flags;
	u32 v;

	while (!kthread_should_stop()) {
		wait_event_interruptible_timeout(iic_wq, iic_data != iic_sent ||
						 kthread_should_stop(), HZ);
		spin_lock_irqsave(&lock, flags);
		v = iic_data;
		spin_unlock_irqrestore(&lock, flags);
		if (v == iic_sent)
			continue;
		mutex_lock(&iic_lock);
		iic_send(v);
		iic_sent = v;
		mutex_unlock(&iic_lock);
	}
	return 0;
}

/* the IR-cut filter, 1 or 2: its coil pulsed for 200 ms */
static int ircut(u32 dir)
{
	if (!iic_drive || (dir != 1 && dir != 2))
		return -EINVAL;
	mutex_lock(&iic_lock);
	iic_send(dir << 8);
	iic_pin(IIC_EN, true);
	msleep(200);
	iic_pin(IIC_EN, false);
	iic_sent = ~0u; /* the motors' bits again on the next step */
	mutex_unlock(&iic_lock);
	wake_up(&iic_wq);
	return 0;
}

static u32 load_of(struct axis *a)
{
	return tick(a->cur) * mul / (speed_up > 0 ? speed_up : 1);
}

/* under lock */
static void axis_stop(struct axis *a)
{
	struct axis *o = a == AX ? AY : AX;

	writel(readl(a->t + T_CTRL) & ~T_ENABLE, a->t + T_CTRL);
	writel(T_IDLE_LOAD * mul, a->t + T_LOAD);
	coils(a, 0);
	a->flags &= ~(F_RUN | F_AUTO);
	if (!(o->flags & F_RUN))
		action = ACT_IDLE;
	a->dir = 0;
	a->speed = 0;
	a->cur = 0;
	if (a->flags & F_WAIT)
		a->flags &= ~F_WAIT;
	wake_up(&idle_wq);
}

/* under lock */
static void motor_start(void)
{
	int i;

	for (i = 0; i < 2; i++) {
		struct axis *a = &ax[i];
		u32 v;

		if (!(a->flags & F_RUN))
			continue;
		v = load_of(a);
		writel(v, a->t + T_LOAD);
		writel(v, a->t + T_BGLOAD);
		writel(1, a->t + T_INTCLR);
		writel(T_MODE | T_ENABLE, a->t + T_CTRL);
	}
}

static irqreturn_t motor_irq(int irq, void *dev)
{
	struct axis *a = dev;
	unsigned long flags;

	if (!(readl(a->t + T_MIS) & 1))
		return IRQ_NONE;
	writel(1, a->t + T_INTCLR);
	spin_lock_irqsave(&lock, flags);
	if (!(a->flags & F_RUN))
		goto out;
	writel(load_of(a), a->t + T_BGLOAD);
	if (a->dir == a->minus) {
		if (limit_hit(a->lim_minus)) {
			pr_info("motor: %s at the %s limit (%u)\n", a->name,
				a == AX ? "left" : "down", a->pos);
			a->pos = 0;
			axis_stop(a);
			goto out;
		}
		if (soft_limit == 1 && a->pos == 0) {
			axis_stop(a);
			goto out;
		}
		a->phase = (a->phase + 1) & 7;
		if (a->pos > 0)
			a->pos--;
	} else {
		if (limit_hit(a->lim_plus)) {
			int *s = a == AX ? &s_max_steps_x : &s_max_steps_y;

			pr_info("motor: %s at the %s limit (%u)\n", a->name,
				a == AX ? "right" : "up", a->pos);
			if (*s < 100)
				*s = a->pos;
			a->pos = a->max;
			axis_stop(a);
			goto out;
		}
		if (soft_limit == 1 && a->pos == a->max) {
			axis_stop(a);
			goto out;
		}
		a->phase = (a->phase - 1) & 7;
		if (a->pos < a->max)
			a->pos++;
	}
	coils(a, pattern(a, a->phase));
	if (!a->steps || !--a->steps || action == ACT_STOP)
		axis_stop(a);
out:
	spin_unlock_irqrestore(&lock, flags);
	return IRQ_HANDLED;
}

static bool is_idle(void)
{
	unsigned long flags;
	bool r;

	spin_lock_irqsave(&lock, flags);
	r = action == ACT_IDLE;
	spin_unlock_irqrestore(&lock, flags);
	return r;
}

/* until idle (or the module goes); helper context */
static void wait_idle(void)
{
	while (!kthread_should_stop() &&
	       !wait_event_timeout(idle_wq, is_idle(), HZ / 10))
		;
}

/* whatever runs, stopped at its next step */
static void cntl_stop(void)
{
	unsigned long flags;
	int i, n = 0;

	spin_lock_irqsave(&lock, flags);
	if (action == ACT_IDLE) {
		spin_unlock_irqrestore(&lock, flags);
		return;
	}
	action = ACT_STOP;
	for (i = 0; i < 2; i++) {
		if (ax[i].flags & F_RUN) {
			ax[i].speed = 0;
			ax[i].flags |= F_WAIT;
			n++;
		}
	}
	if (!n)
		action = ACT_IDLE;
	spin_unlock_irqrestore(&lock, flags);
	if (n && !wait_event_timeout(idle_wq, is_idle(), 2 * HZ)) {
		spin_lock_irqsave(&lock, flags);
		for (i = 0; i < 2; i++)
			if (ax[i].flags & F_RUN)
				axis_stop(&ax[i]);
		spin_unlock_irqrestore(&lock, flags);
	}
	spin_lock_irqsave(&lock, flags);
	for (i = 0; i < 2; i++)
		ax[i].flags &= ~(F_WAIT | F_RUN);
	spin_unlock_irqrestore(&lock, flags);
}

/* towards target, under lock */
static void aim(struct axis *a, u32 target)
{
	target = min(target, a->max);
	if (a->pos < target) {
		a->dir = a->plus;
		a->steps = target - a->pos;
	} else {
		a->dir = a->minus;
		a->steps = a->pos - target;
	}
	if (a->steps)
		a->flags |= F_RUN;
}

static void set_speeds(const struct cmd *c)
{
	if (c->a[4] != RAW_SPEED) {
		AX->speed = AX->cur = reflect(c->a[0]);
		AY->speed = AY->cur = reflect(c->a[1]);
		return;
	}
	if (AX->flags & F_RUN)
		AX->speed = AX->cur = c->a[0] < 8 ? 0 :
				      c->a[0] < 61 ? c->a[0] - 8 : 52;
	if (AY->flags & F_RUN)
		AY->speed = AY->cur = c->a[1] < 5 ? 0 :
				      c->a[1] < 61 ? c->a[1] - 5 : 55;
}

/* a command, in the helper thread; true when a motor is to start */
static bool cmd_run(const struct cmd *c)
{
	unsigned long flags;
	bool tested;

	spin_lock_irqsave(&lock, flags);
	tested = AX->flags & F_TESTED;
	spin_unlock_irqrestore(&lock, flags);
	if (c->c == ACT_TEST) {
		spin_lock_irqsave(&lock, flags);
		action = ACT_TEST;
		AX->dir = DIR_LEFT;
		AY->dir = DIR_DOWN;
		AX->steps = AX->max + 170;
		AY->steps = AY->max + 170;
		AX->flags |= F_RUN;
		AY->flags |= F_RUN;
		set_speeds(c);
		spin_unlock_irqrestore(&lock, flags);
		return true;
	}
	if (!tested) {
		pr_err("motor: the end-to-end test is not done\n");
		return false;
	}
	switch (c->c) {
	case DIR_LEFT:
	case DIR_RIGHT:
	case DIR_UP:
	case DIR_DOWN:
	case DIR_LEFT | DIR_UP:
	case DIR_RIGHT | DIR_UP:
	case DIR_LEFT | DIR_DOWN:
	case DIR_RIGHT | DIR_DOWN:
		spin_lock_irqsave(&lock, flags);
		if (c->c == action) {
			/* the same way: only the speed */
			set_speeds(c);
			spin_unlock_irqrestore(&lock, flags);
			return false;
		}
		spin_unlock_irqrestore(&lock, flags);
		cntl_stop();
		spin_lock_irqsave(&lock, flags);
		scan.state = 0;
		action = c->c;
		if (c->c & (DIR_LEFT | DIR_RIGHT)) {
			AX->dir = c->c & (DIR_LEFT | DIR_RIGHT);
			AX->steps = AX->max;
			AX->flags |= F_RUN;
		}
		if (c->c & (DIR_UP | DIR_DOWN)) {
			AY->dir = c->c & (DIR_UP | DIR_DOWN);
			AY->steps = AY->max;
			AY->flags |= F_RUN;
		}
		break;
	case ACT_SCAN:
		spin_lock_irqsave(&lock, flags);
		if (action != ACT_IDLE || c->a[1] == c->a[3]) {
			spin_unlock_irqrestore(&lock, flags);
			return false;
		}
		scan.left = min(c->a[1], AX->max);
		scan.right = min(c->a[3], AX->max);
		pr_info("motor: scan x %u..%u\n", scan.left, scan.right);
		action = ACT_SCAN;
		aim(AX, scan.left);
		aim(AY, c->a[2]);
		scan.state = AX->steps || AY->steps ? 3 : 2;
		if (scan.state == 2)
			action = ACT_IDLE; /* there already: the helper goes on */
		scan.delay = 0;
		set_speeds(c);
		scan.tick = AX->cur;
		spin_unlock_irqrestore(&lock, flags);
		return true;
	case ACT_PRESET:
		spin_lock_irqsave(&lock, flags);
		if (action != ACT_PRESET && action != ACT_IDLE) {
			spin_unlock_irqrestore(&lock, flags);
			return false;
		}
		if (c->a[0] == AX->pos && c->a[1] == AY->pos) {
			spin_unlock_irqrestore(&lock, flags);
			return false;
		}
		scan.state = 0;
		action = ACT_PRESET;
		aim(AX, c->a[0]);
		aim(AY, c->a[1]);
		if (!(AX->flags & F_RUN) && !(AY->flags & F_RUN))
			action = ACT_IDLE;
		AX->speed = AX->cur = reflect(c->a[2]);
		AY->speed = AY->cur = reflect(c->a[3]);
		spin_unlock_irqrestore(&lock, flags);
		return true;
	case ACT_STOP:
		cntl_stop();
		spin_lock_irqsave(&lock, flags);
		scan.state = 0;
		spin_unlock_irqrestore(&lock, flags);
		return false;
	default:
		pr_err("motor: unknown command %u\n", c->c);
		return false;
	}
	set_speeds(c);
	spin_unlock_irqrestore(&lock, flags);
	return true;
}

static void run_and_wait(const struct cmd *c)
{
	unsigned long flags;

	if (!cmd_run(c))
		return;
	spin_lock_irqsave(&lock, flags);
	AX->flags |= F_WAIT;
	AY->flags |= F_WAIT;
	motor_start();
	spin_unlock_irqrestore(&lock, flags);
	wait_idle();
}

static u32 mid(int degree, u32 max)
{
	return degree ? degree * 4096 / 360 : max / 2;
}

/* to both ends (no soft limit), then the middle */
static void end_to_end(void)
{
	struct cmd t = { ACT_TEST, { AUTOTEST_SPEED, AUTOTEST_SPEED } };
	struct cmd m = { ACT_PRESET, { mid(MID_DEGREE_X, AX->max),
				       mid(MID_DEGREE_Y, AY->max),
				       AUTOTEST_SPEED, AUTOTEST_SPEED } };
	unsigned long flags;
	int soft = soft_limit;

	soft_limit = 0;
	spin_lock_irqsave(&lock, flags);
	AX->flags |= F_TESTED;
	spin_unlock_irqrestore(&lock, flags);
	run_and_wait(&t);
	soft_limit = soft;
	run_and_wait(&m);
	pr_info("motor auto test over\n");
}

static void scan_step(void)
{
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);
	if (!scan.state || action != ACT_IDLE || ++scan.delay < 10) {
		spin_unlock_irqrestore(&lock, flags);
		return;
	}
	scan.delay = 0;
	scan.state = scan.state == 1 ? 2 : 1;
	AX->dir = scan.state == 1 ? DIR_RIGHT : DIR_LEFT;
	AX->steps = scan.right - scan.left;
	AX->speed = AX->cur = scan.tick;
	AX->flags |= F_RUN;
	action = ACT_SCAN;
	motor_start();
	spin_unlock_irqrestore(&lock, flags);
}

static int helper_thread(void *data)
{
	unsigned long flags;
	struct cmd c;
	bool have;

	if (auto_test)
		end_to_end();
	spin_lock_irqsave(&lock, flags);
	AX->flags |= F_TESTED;
	spin_unlock_irqrestore(&lock, flags);
	while (!kthread_should_stop()) {
		wait_event_interruptible_timeout(helper_wq, pending_valid ||
						 test_req != test_done ||
						 kthread_should_stop(), 1);
		if (test_req != test_done) {
			int req = test_req;

			end_to_end();
			test_done = req;
			wake_up(&helper_wq);
			continue;
		}
		scan_step();
		spin_lock_irqsave(&lock, flags);
		have = pending_valid;
		c = pending;
		pending_valid = false;
		spin_unlock_irqrestore(&lock, flags);
		if (have && cmd_run(&c)) {
			spin_lock_irqsave(&lock, flags);
			motor_start();
			spin_unlock_irqrestore(&lock, flags);
		}
	}
	return 0;
}

/* a command from user space: checked, then queued for the helper */
static int usr_cmd(struct cmd *c)
{
	unsigned long flags;
	int ret = 0;
	u32 t;

	spin_lock_irqsave(&lock, flags);
	if (!(AX->flags & F_TESTED)) {
		spin_unlock_irqrestore(&lock, flags);
		pr_err("motor: auto test not done\n");
		return -EBUSY;
	}
	switch (c->c) {
	case DIR_LEFT:
	case DIR_RIGHT:
	case DIR_UP:
	case DIR_DOWN:
	case DIR_LEFT | DIR_UP:
	case DIR_RIGHT | DIR_UP:
	case DIR_LEFT | DIR_DOWN:
	case DIR_RIGHT | DIR_DOWN:
		if (action == ACT_SCAN)
			ret = -EBUSY;
		break;
	case ACT_SCAN:
		if (action != ACT_IDLE) {
			ret = -EBUSY;
			break;
		}
		if (c->a[1] > c->a[3]) {
			t = c->a[1];
			c->a[1] = c->a[3];
			c->a[3] = t;
		}
		break;
	case ACT_PRESET:
		if (action != ACT_PRESET && action != ACT_IDLE)
			ret = -EBUSY;
		break;
	case ACT_STOP:
		break;
	case ACT_TEST:
		spin_unlock_irqrestore(&lock, flags);
		t = ++test_req;
		wake_up(&helper_wq);
		return wait_event_interruptible(helper_wq,
						(int)(test_done - t) >= 0);
	case ACT_SETPOS:
		if (action != ACT_IDLE) {
			ret = -EBUSY;
		} else if (c->a[0] > AX->max || c->a[1] > AY->max) {
			pr_err("motor: set position [%u, %u] err!\n",
			       c->a[0], c->a[1]);
			ret = -EINVAL;
		} else {
			AX->pos = c->a[0];
			AY->pos = c->a[1];
		}
		spin_unlock_irqrestore(&lock, flags);
		return ret;
	default:
		ret = -EINVAL;
	}
	if (ret) {
		spin_unlock_irqrestore(&lock, flags);
		pr_err("motor: could not run motor cntl cmd: %u\n", c->c);
		return ret;
	}
	pending = *c;
	pending_valid = true;
	spin_unlock_irqrestore(&lock, flags);
	wake_up(&helper_wq);
	return 0;
}

static long motor_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	void __user *u = (void __user *)arg;
	unsigned long flags;
	struct cmd c;
	u32 r[6] = { 0 }, v;

	switch (cmd) {
	case MOTOR_CMD:
		if (copy_from_user(&c, u, sizeof(c)))
			return -EFAULT;
		return usr_cmd(&c);
	case MOTOR_STATUS:
	case MOTOR_INFO:
		spin_lock_irqsave(&lock, flags);
		r[0] = action;
		if (cmd == MOTOR_STATUS) {
			r[1] = AX->pos;
			r[2] = AY->pos;
			r[3] = AX->flags;
			r[4] = AY->flags;
		} else {
			r[1] = AX->max;
			r[2] = AY->max;
			r[3] = MAX_DEGREE_X << 16 | MAX_DEGREE_Y;
		}
		spin_unlock_irqrestore(&lock, flags);
		return copy_to_user(u, r, sizeof(r)) ? -EFAULT : 0;
	case MOTOR_IRCUT:
		if (get_user(v, (u32 __user *)u))
			return -EFAULT;
		return ircut(v);
	}
	pr_err("motor: unknown cmd:%x\n", cmd);
	return -ENOTTY;
}

/* "word" then a number (spaces before it skipped), or -1 */
static long match(const char *s, const char *word)
{
	size_t n = strlen(word);

	if (strncmp(s, word, n))
		return -1;
	s += n;
	while (*s == ' ')
		s++;
	return simple_strtoul(s, NULL, 10);
}

static void text_cmd(const char *s)
{
	static const struct {
		const char *word;
		u32 c;
	} moves[] = {
		{ "left ", DIR_LEFT }, { "right ", DIR_RIGHT },
		{ "up ", DIR_UP }, { "down ", DIR_DOWN },
		{ "leftup", DIR_LEFT | DIR_UP },
		{ "rightup", DIR_RIGHT | DIR_UP },
		{ "leftdown", DIR_LEFT | DIR_DOWN },
		{ "rightdown", DIR_RIGHT | DIR_DOWN },
	};
	struct cmd c = { 0 };
	unsigned long flags;
	long n;
	int i;

	if (match(s, "stop") >= 0) {
		c.c = ACT_STOP;
		usr_cmd(&c);
		return;
	}
	for (i = 0; i < ARRAY_SIZE(moves); i++) {
		n = match(s, moves[i].word);
		if (n < 0)
			continue;
		c.c = moves[i].c;
		c.a[0] = c.a[1] = n;
		/* the four plain ways take the speed as an index */
		if (i < 4)
			c.a[4] = RAW_SPEED;
		usr_cmd(&c);
		return;
	}
	spin_lock_irqsave(&lock, flags);
	if (match(s, "setpreset") >= 0) {
		my_preset.set = true;
		my_preset.x = AX->pos;
		my_preset.y = AY->pos;
	} else if (match(s, "lscan") >= 0) {
		my_scan.set = true;
		my_scan.lx = AX->pos;
		my_scan.ly = AY->pos;
		pr_info("motor: left_limit: x=%u,y=%u\n", AX->pos, AY->pos);
	} else if (match(s, "rscan") >= 0) {
		my_scan.set = true;
		my_scan.rx = AX->pos;
		my_scan.ry = AY->pos;
		pr_info("motor: right_limit: x=%u,y=%u\n", AX->pos, AY->pos);
	} else if (match(s, "preset") >= 0) {
		if (!my_preset.set) {
			spin_unlock_irqrestore(&lock, flags);
			pr_err("motor: no preset, set it first\n");
			return;
		}
		c.c = ACT_PRESET;
		c.a[0] = my_preset.x;
		c.a[1] = my_preset.y;
	} else if ((n = match(s, "scan")) >= 0) {
		if (!my_scan.set || my_scan.lx == my_scan.rx) {
			spin_unlock_irqrestore(&lock, flags);
			pr_err("motor: no scan, set it first\n");
			return;
		}
		c.c = ACT_SCAN;
		c.a[0] = n ? n : 25;
		c.a[1] = my_scan.lx;
		c.a[2] = my_scan.ly;
		c.a[3] = my_scan.rx;
		c.a[4] = RAW_SPEED;
	}
	spin_unlock_irqrestore(&lock, flags);
	if (c.c)
		usr_cmd(&c);
}

static ssize_t motor_write(struct file *file, const char __user *buf,
			   size_t len, loff_t *off)
{
	char s[65];

	if (len > 64) {
		pr_err("motor write len=%zu >64, error\n", len);
		return len;
	}
	if (copy_from_user(s, buf, len))
		return -EFAULT;
	s[len] = 0;
	text_cmd(s);
	return len;
}

static ssize_t motor_read(struct file *file, char __user *buf, size_t len,
			  loff_t *off)
{
	return 0;
}

static const struct file_operations motor_fops = {
	.owner = THIS_MODULE,
	.read = motor_read,
	.write = motor_write,
	.unlocked_ioctl = motor_ioctl,
};

static struct miscdevice motor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "motor",
	.fops = &motor_fops,
};

static const char *action_name(u32 a)
{
	switch (a) {
	case ACT_IDLE: return "idle";
	case DIR_LEFT: return "left";
	case DIR_RIGHT: return "right";
	case DIR_UP: return "up";
	case DIR_DOWN: return "down";
	case DIR_LEFT | DIR_UP: return "left-up";
	case DIR_RIGHT | DIR_UP: return "right-up";
	case DIR_LEFT | DIR_DOWN: return "left-down";
	case DIR_RIGHT | DIR_DOWN: return "right-down";
	case ACT_SCAN: return "scan";
	case ACT_PRESET: return "preset";
	case ACT_STOP: return "stop";
	case ACT_TEST: return "auto-test";
	}
	return "";
}

static const char *dir_name(u32 d)
{
	switch (d) {
	case DIR_LEFT: return "|left";
	case DIR_RIGHT: return "|right";
	case DIR_UP: return "|up";
	case DIR_DOWN: return "|down";
	}
	return "";
}

static int motor_show(struct seq_file *s, void *v)
{
	static const char *const fl[] = { "", "|wait-cmplt", "|start-run",
		"|run-auto", "|auto-test-cmplt", "|stopping" };
	unsigned long flags;
	struct axis snap[2];
	u32 act;
	int i, b;

	spin_lock_irqsave(&lock, flags);
	memcpy(snap, ax, sizeof(snap));
	act = action;
	spin_unlock_irqrestore(&lock, flags);
	for (i = 0; i < 2; i++) {
		seq_printf(s, "####%s: %s####\n", "open_motor", snap[i].name);
		if (!i)
			seq_printf(s, "action=%s\n", action_name(act));
		seq_printf(s, "dir=%s\n", dir_name(snap[i].dir));
		seq_puts(s, "flag=");
		for (b = 1; b < ARRAY_SIZE(fl); b++)
			if (snap[i].flags & BIT(b))
				seq_puts(s, fl[b]);
		seq_printf(s, "\n steps=%u , curr_steps=%u , speed=%u , curr_speed=%u \n",
			   snap[i].steps, snap[i].pos, snap[i].speed,
			   snap[i].cur);
	}
	return 0;
}

static int motor_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, motor_show, NULL);
}

static const struct file_operations motor_proc_fops = {
	.owner = THIS_MODULE,
	.open = motor_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* the timers' clock, in 12 MHz units, as kmotor.ko reads it */
static u32 clock_mul(void)
{
	void __iomem *sys;
	u32 m = 1, a, b;

	if (timer_mul > 0)
		return timer_mul;
	sys = ioremap(SYS_BASE + 0xf00, 0x100);
	if (!sys)
		return m;
	a = readl(sys + 0x2c);
	b = readl(sys + 0xd0);
	iounmap(sys);
	if (a == 0x24242424)
		m = 2;
	if (a == 0x48484848)
		m = 4;
	if (b > 10)
		m = 2;
	return m;
}

static void axis_init(struct axis *a, int i)
{
	int anti = (pin_antitone ^ pin_antitone1) >> i & 1;
	int k;

	a->name = i ? "MOTOR_Y" : "MOTOR_X";
	a->max = (i ? MAX_DEGREE_Y : MAX_DEGREE_X) * 4096 / 360;
	a->plus = i ? DIR_UP : DIR_RIGHT;
	a->minus = i ? DIR_DOWN : DIR_LEFT;
	/* X: pins 0..3 are A..D; Y: pins 7..4 */
	for (k = 0; k < 4; k++)
		a->coil[k] = gpio_pin[i ? 7 - k : k];
	a->reverse = anti;
	a->lim_plus = gpio_pin[8 + 2 * i];
	a->lim_minus = gpio_pin[9 + 2 * i];
	a->t = timers + 0x20 * i;
	a->phase = 0;
}

static int __init xm_v200_motor_init(void)
{
	void __iomem *pad;
	int i, k, ret;

	for (i = pin_num; i < ARRAY_SIZE(gpio_pin); i++)
		gpio_pin[i] = -1;
	gpio = ioremap(GPIO_BASE, 4 * GPIO_PINS);
	timers = ioremap(TIMER_BASE, 0x40);
	if (!gpio || !timers) {
		ret = -ENOMEM;
		goto unmap;
	}
	for (i = 0; i < ARRAY_SIZE(gpio_pin); i++)
		if (gpio_pin[i] > MIPI_PIN)
			break;
	if (i < ARRAY_SIZE(gpio_pin)) {
		mipi_out = symbol_get(mipi_io_set_output);
		mipi_in = symbol_get(mipi_io_get_input);
		if (!mipi_out)
			pr_warn("motor: pins above %d need open_mipi_io\n",
				MIPI_PIN);
	}
	axis_init(AX, 0);
	axis_init(AY, 1);
	/* no end-to-end test: commands at once (not after the helper starts) */
	if (!auto_test)
		AX->flags |= F_TESTED;
	pr_info("max_degree: %d, %d; max_steps: %d, %d\n", MAX_DEGREE_X,
		MAX_DEGREE_Y, AX->max, AY->max);
	mul = clock_mul();
	/* as kmotor.ko: the pad register at 0x100203C0 to 2 */
	pad = ioremap(GPIO_BASE + 0x3c0, 4);
	if (pad) {
		writel(2, pad);
		iounmap(pad);
	}
	if (iic_drive) {
		iic_pin(IIC_RST, false);
		iic_pin(IIC_EN, false);
		iic_pin(IIC_SCL, false);
	} else {
		for (i = 0; i < 2; i++)
			for (k = 0; k < 4; k++)
				pin_write(ax[i].coil[k], PIN_LOW);
	}
	if (limit)
		for (i = 8; i < 12; i++)
			if (gpio_pin[i] >= 0 && gpio_pin[i] < GPIO_PINS)
				writel(PIN_INPUT, gpio + 4 * gpio_pin[i]);
	for (i = 0; i < 2; i++) {
		writel(1, ax[i].t + T_INTCLR);
		writel(T_MODE, ax[i].t + T_CTRL);
		writel(T_IDLE_LOAD * mul, ax[i].t + T_LOAD);
		writel(T_IDLE_LOAD * mul, ax[i].t + T_BGLOAD);
	}
	ret = request_irq(TIMER_IRQ, motor_irq, IRQF_SHARED, "motor-x", AX);
	if (ret) {
		pr_err("request motor-x irq %d failed\n", TIMER_IRQ);
		goto unmap;
	}
	ret = request_irq(TIMER_IRQ, motor_irq, IRQF_SHARED, "motor-y", AY);
	if (ret) {
		pr_err("request motor-y irq %d failed\n", TIMER_IRQ);
		goto irq_x;
	}
	if (iic_drive) {
		iic_task = kthread_run(iic_thread, NULL, "motor_iic");
		if (IS_ERR(iic_task)) {
			ret = PTR_ERR(iic_task);
			goto irq_y;
		}
	}
	ret = misc_register(&motor_misc);
	if (ret)
		goto iic;
	helper = kthread_run(helper_thread, NULL, "motor_helper");
	if (IS_ERR(helper)) {
		ret = PTR_ERR(helper);
		goto misc;
	}
	proc_create("motor", 0, NULL, &motor_proc_fops);
	return 0;
misc:
	misc_deregister(&motor_misc);
iic:
	if (iic_drive)
		kthread_stop(iic_task);
irq_y:
	free_irq(TIMER_IRQ, AY);
irq_x:
	free_irq(TIMER_IRQ, AX);
unmap:
	if (mipi_out)
		symbol_put(mipi_io_set_output);
	if (mipi_in)
		symbol_put(mipi_io_get_input);
	if (timers)
		iounmap(timers);
	if (gpio)
		iounmap(gpio);
	return ret;
}

static void __exit xm_v200_motor_exit(void)
{
	unsigned long flags;
	int i;

	remove_proc_entry("motor", NULL);
	misc_deregister(&motor_misc);
	spin_lock_irqsave(&lock, flags);
	for (i = 0; i < 2; i++)
		axis_stop(&ax[i]);
	spin_unlock_irqrestore(&lock, flags);
	kthread_stop(helper);
	if (iic_drive) {
		kthread_stop(iic_task);
		mutex_lock(&iic_lock);
		iic_send(0);
		mutex_unlock(&iic_lock);
	}
	free_irq(TIMER_IRQ, AY);
	free_irq(TIMER_IRQ, AX);
	if (mipi_out)
		symbol_put(mipi_io_set_output);
	if (mipi_in)
		symbol_put(mipi_io_get_input);
	iounmap(timers);
	iounmap(gpio);
}

module_init(xm_v200_motor_init);
module_exit(xm_v200_motor_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("V200-series pan/tilt step motors, compatible with kmotor.ko");
