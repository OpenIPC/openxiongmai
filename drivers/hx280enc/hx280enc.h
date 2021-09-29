/*
 * Encoder device driver (kernel module)
 *
--------------------------------------------------------------------------------
--
--  Abstract : 6280/7280/8270/8290 Encoder device driver (kernel module)
--
------------------------------------------------------------------------------*/

#ifndef _HX280ENC_H_
#define _HX280ENC_H_
#include <linux/cdev.h>     /* character device definitions */
#include <linux/ioctl.h>    /* needed for the _IOW etc stuff used later */

/*
 * Macros to help debugging
 */

#undef PDEBUG   /* undef it, just in case */
#ifdef HX280ENC_DEBUG
#ifdef __KERNEL__
    /* This one if debugging is on, and kernel space */
#define PDEBUG(fmt, args...) printk(KERN_INFO "hmp4e: " fmt, ## args)
#else
    /* This one for user space */
#define PDEBUG(fmt, args...) printf(__FILE__ ":%d: " fmt, __LINE__ , ## args)
#endif
#else
#define PDEBUG(fmt, args...)  /* not debugging: nothing */
#endif

/*
 * Ioctl definitions
 */

/* Use 'k' as magic number */
#define HX280ENC_IOC_MAGIC  'k'
/*
 * S means "Set" through a ptr,
 * T means "Tell" directly with the argument value
 * G means "Get": reply by setting through a pointer
 * Q means "Query": response is on the return value
 * X means "eXchange": G and S atomically
 * H means "sHift": T and Q atomically
 */

#define HX280ENC_IOCGHWOFFSET	_IOR(HX280ENC_IOC_MAGIC,  3, unsigned long *)
#define HX280ENC_IOCGHWIOSIZE	_IOR(HX280ENC_IOC_MAGIC,  4, unsigned int *)
#define HX280ENC_IOC_CLI	_IO(HX280ENC_IOC_MAGIC,  5)
#define HX280ENC_IOC_STI	_IO(HX280ENC_IOC_MAGIC,  6)
#define HX280ENC_IOCXVIRT2BUS	_IOWR(HX280ENC_IOC_MAGIC,  7, unsigned long *)

#define HX280ENC_IOCHARDRESET	_IO(HX280ENC_IOC_MAGIC, 8) /* debugging tool */

#define HX280ENC_IOC_MAXNR 8

struct hx280enc_dev {
	struct cdev cdev;
	struct class *hx280enc_class;
};

#endif /* !_HX280ENC_H_ */
