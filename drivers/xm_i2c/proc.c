#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/list.h>

#define I2C_WAIT_TIME_OUT 0x1000
#define I2C_ADRESS_BASE 0xFE000000

#define CMD_I2C_WRITE 0x01
#define CMD_I2C_READ 0x03

#define I2C_WRITE_REG(Addr, Value)                                             \
	((*(volatile unsigned int *)(Addr)) = (Value))
#define I2C_READ_REG(Addr) (*(volatile unsigned int *)(Addr))

typedef struct I2C_DATA_S {
	unsigned char dev_addr;
	unsigned int reg_addr;
	unsigned int addr_byte_num;
	unsigned int data;
	unsigned int data_byte_num;
} I2C_DATA_S;

static int i2c_open(struct inode *node, struct file *file)
{
	return 0;
}

static int i2c_release(struct inode *node, struct file *file)
{
	return 0;
}

#define I2C_RECEIVE_INTR 0x10

static int I2C_DRV_WaitRead(unsigned int I2cNum)
{
	u32 I2cSrReg, reg;
	unsigned int i = 0;

	I2cSrReg = I2C_READ_REG(I2C_ADRESS_BASE + 0x12);
	while (!(I2cSrReg & I2C_RECEIVE_INTR)) {
		__const_udelay(0x10624C); // udelay(50); ?
		I2cSrReg = I2C_READ_REG(I2C_ADRESS_BASE + 0x12);

		if (i > I2C_WAIT_TIME_OUT) {
			reg = I2C_READ_REG(I2C_ADRESS_BASE + 0x48);
			printk("i2c_read timeout %#x\n", reg);
			return -1;
		}
		i++;
	}

	return 0;
}

const char *LC0 asm("LC0") = "<3>i2c_write timeout %#x\n";
const char *LC1 asm("LC1") = "<3>i2c_read timeout %#x\n";

unsigned int i2c_read(unsigned int I2cNum, u8 I2cDevAddr,
			     unsigned int I2cRegAddr,
			     unsigned int I2cRegAddrByteNum,
			     unsigned int DataLen)
;
#if 0
{
	asm volatile(
		"L_184:	mov	ip, sp\n"
		"L_188:	push	{r4, r5, r6, r7, fp, ip, lr, pc}\n"
		"L_18c:	sub	fp, ip, #4\n"
		"L_190:	ldr	r6, [fp, #4]\n"
		"L_194:	dsb	sy\n"
		"L_198:	lsl	r0, r0, #16\n"
		"L_19c:	add	r5, r0, #-33554432	@ 0xfe000000\n"
		"L_1a0:	str	r1, [r5, #12]\n"
		"L_1a4:	dsb	sy\n"
		"L_1a8:	str	r3, [r5, #24]\n"
		"L_1ac:	ldr	r1, [r5]\n"
		"L_1b0:	dsb	sy\n"
		"L_1b4:	orr	r0, r1, #64	@ 0x40\n"
		"L_1b8:	dsb	sy\n"
		"L_1bc:	str	r0, [r5]\n"
		"L_1c0:	bic	r1, r1, #64	@ 0x40\n"
		"L_1c4:	dsb	sy\n"
		"L_1c8:	cmp	r3, #0\n"
		"L_1cc:	str	r1, [r5]\n"
		"L_1d0:	addeq	lr, r5, #16\n"
		"L_1d4:	beq	L_208\n"
		"L_1d8:	sub	ip, r3, #-536870911	@ 0xe0000001\n"
		"L_1dc:	add	lr, r5, #16\n"
		"L_1e0:	mov	r0, #0\n"
		"L_1e4:	lsl	ip, ip, #3\n"
		"L_1e8:	lsr	r1, r2, ip\n"
		"L_1ec:	uxtb	r1, r1\n"
		"L_1f0:	dsb	sy\n"
		"L_1f4:	add	r0, r0, #1\n"
		"L_1f8:	str	r1, [lr]\n"
		"L_1fc:	cmp	r0, r3\n"
		"L_200:	sub	ip, ip, #8\n"
		"L_204:	bne	L_1e8\n"
		"L_208:	dsb	sy\n"
		"L_20c:	mov	r3, #0\n"
		"L_210:	str	r3, [lr]\n"
		"L_214:	dsb	sy\n"
		"L_218:	mov	r7, r5\n"
		"L_21c:	mov	r3, #11\n"
		"L_220:	str	r3, [r7], #72	@ 0x48\n"
		"L_224:	ldr	r3, [r5, #72]	@ 0x48\n"
		"L_228:	dsb	sy\n"
		"L_22c:	tst	r3, #16\n"
		"L_230:	movweq	r4, #4097	@ 0x1001\n"
		"L_234:	beq	L_244\n"
		"L_238:	b	L_260\n"
		"L_23c:	subs	r4, r4, #1\n"
		"L_240:	beq	L_320\n"
		"L_244:	movw	r0, #25164	@ 0x624c\n"
		"L_248:	movt	r0, #16\n"
		"L_24c:	bl	__const_udelay	@R_ARM_CALL	__const_udelay\n"
		"L_250:	ldr	r3, [r7]\n"
		"L_254:	dsb	sy\n"
		"L_258:	tst	r3, #16\n"
		"L_25c:	beq	L_23c\n"
		"L_260:	dsb	sy\n"
		"L_264:	mov	r3, #16\n"
		"L_268:	str	r3, [r5, #68]	@ 0x44\n"
		"L_26c:	dsb	sy\n"
		"L_270:	str	r6, [r5, #28]\n"
		"L_274:	ldr	r3, [r5]\n"
		"L_278:	dsb	sy\n"
		"L_27c:	orr	r2, r3, #128	@ 0x80\n"
		"L_280:	dsb	sy\n"
		"L_284:	str	r2, [r5]\n"
		"L_288:	bic	r3, r3, #128	@ 0x80\n"
		"L_28c:	dsb	sy\n"
		"L_290:	str	r3, [r5]\n"
		"L_294:	dsb	sy\n"
		"L_298:	movw	r3, #2051	@ 0x803\n"
		"L_29c:	str	r3, [r5]\n"
		"L_2a0:	ldr	r3, [r5, #72]	@ 0x48\n"
		"L_2a4:	dsb	sy\n"
		"L_2a8:	tst	r3, #32\n"
		"L_2ac:	movweq	r4, #4097	@ 0x1001\n"
		"L_2b0:	beq	L_2c0\n"
		"L_2b4:	b	L_2dc\n"
		"L_2b8:	subs	r4, r4, #1\n"
		"L_2bc:	beq	L_354\n"
		"L_2c0:	movw	r0, #25164	@ 0x624c\n"
		"L_2c4:	movt	r0, #16\n"
		"L_2c8:	bl	__const_udelay	@R_ARM_CALL	__const_udelay\n"
		"L_2cc:	ldr	r3, [r7]\n"
		"L_2d0:	dsb	sy\n"
		"L_2d4:	tst	r3, #32\n"
		"L_2d8:	beq	L_2b8\n"
		"L_2dc:	cmp	r6, #0\n"
		"L_2e0:	moveq	r0, r6\n"
		"L_2e4:	beq	L_310\n"
		"L_2e8:	mov	r0, #0\n"
		"L_2ec:	add	r1, r5, #20\n"
		"L_2f0:	mov	r3, r0\n"
		"L_2f4:	lsl	r0, r0, #8\n"
		"L_2f8:	ldr	r2, [r1]\n"
		"L_2fc:	dsb	sy\n"
		"L_300:	add	r3, r3, #1\n"
		"L_304:	orr	r0, r2, r0\n"
		"L_308:	cmp	r3, r6\n"
		"L_30c:	bne	L_2f4\n"
		"L_310:	dsb	sy\n"
		"L_314:	mov	r3, #32\n"
		"L_318:	str	r3, [r5, #68]	@ 0x44\n"
		"L_31c:	ldm	sp, {r4, r5, r6, r7, fp, sp, pc}\n"
		"L_320:	ldr	r1, [r5, #72]	@ 0x48\n"
		"L_324:	dsb	sy\n"
		//"L_328:	movw	r0, #0	@R_ARM_MOVW_ABS_NC	.LC0\n"
		//"L_32c:	movt	r0, #0	@R_ARM_MOVT_ABS	.LC0\n"
		"       ldr     r0, =LC0\n"
		"L_330:	bl	printk	@R_ARM_CALL	printk\n"
		"L_334:	dsb	sy\n"
		"L_338:	mov	r3, #1\n"
		"L_33c:	str	r3, [r5]\n"
		"L_340:	dsb	sy\n"
		"L_344:	movw	r3, #65535	@ 0xffff\n"
		"L_348:	str	r3, [r5, #68]	@ 0x44\n"
		"L_34c:	mvn	r0, #0\n"
		"L_350:	ldm	sp, {r4, r5, r6, r7, fp, sp, pc}\n"
		"L_354:	ldr	r1, [r5, #72]	@ 0x48\n"
		"L_358:	dsb	sy\n"
		//"L_35c:	movw	r0, #0	@R_ARM_MOVW_ABS_NC	.LC1\n"
		//"L_360:	movt	r0, #0	@R_ARM_MOVT_ABS	.LC1\n"
		"       ldr     r0, =LC1\n"
		"L_364:	b	L_330\n"
		:
		:
		: "memory");
#if 0
	I2C_WRITE_REG(I2C_ADRESS_BASE + 0xC, buffer);
	I2C_WRITE_REG(I2C_ADRESS_BASE + 0x18, offset);

#endif
}
#endif

static ssize_t i2c_write(struct file *filp, const char *buff, size_t len,
			 loff_t *off)
{
	printk("<1>Sorry, this operation isn't supported.\n");
	return 0;
}

static long i2c_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	I2C_DATA_S __user *argp = (I2C_DATA_S __user *)arg;
	int ret = 0;
	char buf[32] = { 0 };
	const char *dev_name;
	unsigned char devAdd;
	unsigned int RegAddr;
	unsigned int Reg_Len;
	unsigned int DataLen;
	unsigned int Wdata;

	dev_name = d_path(&filp->f_path, buf, sizeof(buf));
	if (strcmp(dev_name, "/dev/xm_i2c") &&
	    strcmp(dev_name, "/dev/xm_i2c1")) {
		printk("<3>xm_i2c dev error!\n");
		return -1;
	}

	switch (cmd) {
	case CMD_I2C_WRITE: {
	printk("<1>Sorry, this operation isn't supported.\n");
	return 0;
#if 0
		devAdd = argp->dev_addr;
		RegAddr = argp->reg_addr;
		Reg_Len = argp->addr_byte_num;
		Wdata = argp->data;
		DataLen = argp->data_byte_num;
		//Ret = I2C_DRV_Write(I2cData.I2cNum, I2cData.I2cDevAddr, I2cData.I2cRegAddr, I2cData.I2cRegCount, pData, I2cData.DataLen);
		ret = I2C_DRV_Write(1, devAdd, RegAddr, Reg_Len, Wdata,
				    DataLen);
#endif
		break;
	}

	case CMD_I2C_READ: {
#if 1
		devAdd = argp->dev_addr;
		RegAddr = argp->reg_addr;
		Reg_Len = argp->addr_byte_num;
		//Wdata  = argp->RWData    ;
		DataLen = argp->data_byte_num;

		printk("i2c_read(%d, %#x, %#x, %#x, %#x) invoke\n", 0, devAdd, RegAddr, Reg_Len, DataLen);
		ret = i2c_read(0, devAdd, RegAddr, Reg_Len, DataLen);
		argp->data = ret;
#endif

		break;
	}

	default: {
		printk("command error!\n");
	}
	}

	return 0;
}

static struct file_operations xm_i2c_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = i2c_ioctl,
	.open = i2c_open,
	.release = i2c_release,
};

static struct miscdevice xm_i2c_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "xm_i2c",
	.fops = &xm_i2c_fops,
};

static struct miscdevice xm_i2c1_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "xm_i2c1",
	.fops = &xm_i2c_fops,
};

static int __init xm_i2c_init(void)
{
	int ret;
	u32 reg;

	ret = misc_register(&xm_i2c_dev);
	if (0 != ret) {
		printk("hi_i2c init error!\n");
		return ret;
	}

	ret = misc_register(&xm_i2c1_dev);
	if (0 != ret) {
		printk("hi_i2c1 init error!\n");
		return ret;
	}

#if 0
	asm volatile("L_58:	mov	r2, #0\n"
		     "L_5c:	movt	r2, #65026	@ 0xfe02\n"
		     "L_60:	ldr	r3, [r2, #4]\n"
		     "L_64:	dsb	sy\n"
		     "L_68:	bic	r3, r3, #12288	@ 0x3000\n"
		     "L_6c:	bic	r3, r3, #53	@ 0x35\n"
		     "L_70:	orr	r3, r3, #3145728	@ 0x300000\n"
		     "L_74:	orr	r3, r3, #1280	@ 0x500\n"
		     "L_78:	dsb	sy\n"
		     "L_7c:	str	r3, [r2, #4]\n"
		     "L_80:	dsb	sy\n"
		     "L_84:	mov	r2, #-33554432	@ 0xfe000000\n"
		     "L_88:	mov	r3, #168	@ 0xa8\n"
		     "L_8c:	str	r3, [r2, #32]\n"
		     "L_90:	dsb	sy\n"
		     "L_94:	str	r3, [r2, #36]	@ 0x24\n"
		     "L_98:	dsb	sy\n"
		     "L_9c:	str	r3, [r2, #40]	@ 0x28\n"
		     "L_a0:	dsb	sy\n"
		     "L_a4:	str	r3, [r2, #44]	@ 0x2c\n"
		     "L_a8:	dsb	sy\n"
		     "L_ac:	str	r3, [r2, #48]	@ 0x30\n"
		     "L_b0:	dsb	sy\n"
		     "L_b4:	mov	r0, #80	@ 0x50\n"
		     "L_b8:	str	r0, [r2, #52]	@ 0x34\n"
		     "L_bc:	dsb	sy\n"
		     "L_c0:	mov	r1, #252	@ 0xfc\n"
		     "L_c4:	str	r1, [r2, #56]	@ 0x38\n"
		     "L_c8:	dsb	sy\n"
		     "L_cc:	mov	r2, #0\n"
		     "L_d0:	movt	r2, #65025	@ 0xfe01\n"
		     "L_d4:	str	r3, [r2, #32]\n"
		     "L_d8:	dsb	sy\n"
		     "L_dc:	str	r3, [r2, #36]	@ 0x24\n"
		     "L_e0:	dsb	sy\n"
		     "L_e4:	str	r3, [r2, #40]	@ 0x28\n"
		     "L_e8:	dsb	sy\n"
		     "L_ec:	str	r3, [r2, #44]	@ 0x2c\n"
		     "L_f0:	dsb	sy\n"
		     "L_f4:	str	r3, [r2, #48]	@ 0x30\n"
		     "L_f8:	dsb	sy\n"
		     "L_fc:	str	r0, [r2, #52]	@ 0x34\n"
		     "L_100:	dsb	sy\n");
#else
	reg = readl(0xFE020004);
	// TODO: check corretness of parenthesis
	writel(reg & (0xFFFFCFCA | 0x300500), 0xFE020004);

	writel(0xA8, 0xFE000020);
	writel(0xA8, 0xFE000024);
	writel(0xA8, 0xFE000028);
	writel(0xA8, 0xFE00002C);
	writel(0xA8, 0xFE000030);
	writel(0x50, 0xFE000034);
	writel(0xFC, 0xFE000038);

	writel(0xA8, 0xFE000020);
	writel(0xA8, 0xFE000024);
	writel(0xA8, 0xFE000028);
	writel(0xA8, 0xFE00002C);
	writel(0xA8, 0xFE000030);
	writel(0x50, 0xFE000034);
	writel(0xFC, 0xFE000038);
#endif

	return 0;
}
module_init(xm_i2c_init);

static void __exit xm_i2c_exit(void)
{
	misc_deregister(&xm_i2c_dev);
	misc_deregister(&xm_i2c1_dev);
}
module_exit(xm_i2c_exit);

/* module description */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dmitry Ilyin");
