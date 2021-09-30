#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/io.h>

static int i2c_open(struct inode *node, struct file *file)
{
	return 0;
}

static int i2c_release(struct inode *node, struct file *file)
{
	return 0;
}

static ssize_t i2c_read(struct file *filp,
		       char *buffer, /* The buffer to fill with data */
		       size_t length, /* The length of the buffer     */
		       loff_t *offset) /* Our offset in the file       */
{
	printk("<1>Sorry, this operation isn't supported.\n");
	return 0 /* bytes_read */;
}

static ssize_t i2c_write(struct file *filp, const char *buff, size_t len,
			 loff_t *off)
{
	printk("<1>Sorry, this operation isn't supported.\n");
	return 0;
}

static long i2c_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	printk("<1>Sorry, this operation isn't supported.\n");
	return 0;
}

static struct file_operations xm_i2c_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = i2c_ioctl,
	.open = i2c_open,
	.release = i2c_release,
	.read = i2c_read,
	.write = i2c_write,
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

#define I2C_BASE 0xFE000000

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
