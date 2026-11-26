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

/*
 * Xiongmai extensions, as implemented by the vendor xm530_h264.ko. The sizes
 * are part of the ioctl numbers userspace already uses, so they are spelled
 * out instead of taken from sizeof.
 */
/*
 * The XM510's records are smaller than the XM530's: 8 bytes less before the
 * prediction flags and the rest of the H.264 one, no ErrCnt word in the VENC
 * one. With the XM530's sizes its ioctls matched nothing and were ignored.
 */
#ifdef CONFIG_ARCH_XM510
#define H264PROC_SIZE 2528
#define JPEGPROC_SIZE 1544
#define VENCINFO_SIZE 72
#define H264PROC_TAIL 8
#define HX280ENC_VENC_ERRCNT 0
#define JPEGPROC_CODING_TYPE 0x550
#else
#define H264PROC_SIZE 2572
#define JPEGPROC_SIZE 1588
#define VENCINFO_SIZE 76
#define H264PROC_TAIL 0
#define HX280ENC_VENC_ERRCNT 1
#define JPEGPROC_CODING_TYPE 0x57c
#endif
/* Statistics shown in /proc/umap/h264, one record per channel (0-2) */
#define HX280ENC_IOCSH264PROC \
	_IOC(_IOC_WRITE, HX280ENC_IOC_MAGIC, 9, H264PROC_SIZE)
#define HX280ENC_IOCSJPEGPROC \
	_IOC(_IOC_WRITE, HX280ENC_IOC_MAGIC, 10, JPEGPROC_SIZE)
/* Per-channel stream state, indexed by its first word (0-3) */
#define HX280ENC_IOCSVENCINFO \
	_IOC(_IOC_WRITE, HX280ENC_IOC_MAGIC, 11, VENCINFO_SIZE)
/* Sleep until the encoder raises its frame interrupt */
#define HX280ENC_IOCWAITDONE	_IO(HX280ENC_IOC_MAGIC, 12)

#define HX280ENC_IOC_MAXNR 12

struct hx280enc_dev {
	struct cdev cdev;
	struct class *hx280enc_class;
};

#endif /* !_HX280ENC_H_ */
