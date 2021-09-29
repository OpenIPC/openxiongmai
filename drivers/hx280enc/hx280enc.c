/*
 * Encoder device driver (kernel module)
 *
--------------------------------------------------------------------------------
--
--  Abstract : 6280/7280/8270/8290 Encoder device driver (kernel module)
--
------------------------------------------------------------------------------*/

#include <asm/irq.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/printk.h>

/* our own stuff */
#include "hx280enc.h"

/* module description */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dmitry Ilyin");
MODULE_DESCRIPTION("Hantro 6280/7280/8270/8290 Encoder driver");

/* Encoder interrupt register */
#define X280_INTERRUPT_REGISTER_ENC (0x04)
#define X280_REGISTER_DISABLE_ENC (0x38)

#define ENC_HW_ID1 0x62800000
#define ENC_HW_ID2 0x72800000
#define ENC_HW_ID3 0x82700000
#define ENC_HW_ID4 0x82900000

#define VIDEO_ENC_CHRDEV_NAME "h264"

char *encirq = "plat";

/* module_param(name, type, perm) */
module_param(encirq, charp, 0);

/* and this is our MAJOR; use 0 for dynamic allocation (recommended)*/
static int hx280enc_major;

/* here's all the must remember stuff */
struct hx280enc_t {
	char *buffer;
	unsigned int buffsize;
	unsigned long iobaseaddr;
	unsigned int iosize;
	volatile u8 *hwregs;
	unsigned int irq;
	struct fasync_struct *async_queue;
};

typedef union {
	u32 nal;
	u32 vp;
	u32 gob;
} TblBase_u;

struct regValues_s {
	u32 irqDisable;
	u32 irqInterval;
	u32 mbsInCol;
	u32 mbsInRow;
	u32 qp;
	u32 qpMin;
	u32 qpMax;
	u32 constrainedIntraPrediction;
	u32 roundingCtrl;
	u32 frameCodingType;
	u32 codingType;
	u32 pixelsOnRow;
	u32 xFill;
	u32 yFill;
	u32 ppsId;
	u32 idrPicId;
	u32 frameNum;
	u32 picInitQp;
	u32 sliceAlphaOffset;
	u32 sliceBetaOffset;
	u32 filterDisable;
	u32 transform8x8Mode;
	u32 enableCabac;
	u32 cabacInitIdc;
	u32 chromaQpIndexOffset;
	u32 sliceSizeMbRows;
	u32 inputImageFormat;
	u32 inputImageRotation;
	u32 outputStrmBase;
	u32 outputStrmSize;
	u32 MADPrecisionSelect;
	u32 firstFreeBit;
	u32 strmStartMSB;
	u32 strmStartLSB;
	u32 rlcBase;
	u32 rlcLimitSpace;
	TblBase_u sizeTblBase;
	u32 sliceReadyInterrupt;
	u32 recWriteDisable;
	u32 reconImageId;
	u32 internalImageLumBaseW;
	u32 internalImageChrBaseW;
	u32 internalImageLumBaseR;
	u32 internalImageChrBaseR;
	u32 inputLumBase;
	u32 inputCbBase;
	u32 inputCrBase;
	u32 cpDistanceMbs;
	u32 cpTargetResults;
	u32 cpTarget;
	u32 targetError;
	u32 deltaQp;
	u32 rlcCount;
	u32 qpSum;
	u32 h264StrmMode;
	u32 sizeTblPresent;
	u32 gobHeaderMask;
	u32 gobFrameId;
	u32 quantTable[128];
	u32 jpegMode;
	u32 jpegSliceEnable;
	u32 jpegRestartInterval;
	u32 jpegRestartMarker;
	u32 regMirror[64];
	u32 inputLumaBaseOffset;
	u32 inputLumaBaseOffsetVert;
	u32 inputChromaBaseOffset;
	u32 h264Inter4x4Disabled;
	u32 disableQuarterPixelMv;
	u32 vsNextLumaBase;
	u32 vsMode;
	u32 vpSize;
	u32 vpMbBits;
	u32 hec;
	u32 moduloTimeBase;
	u32 intraDcVlcThr;
	u32 vopFcode;
	u32 timeInc;
	u32 timeIncBits;
	u32 asicCfgReg;
	u32 asicHwId;
	u32 intra16Favor;
	u32 interFavor;
	u32 FaverDertaFlag;
	u32 Faver16DertaFlag;
	u32 prevModeFavorcml;
	u32 diffMvPenaltyFlag;
	u32 diffMvPenalty4pFlag;
	u32 skipPenaltyFlag;
	u8 Inter8x4Enable;
	u8 Inter4x8Enable;
	u8 Inter16x16Preden;
	u8 Inter16x8Preden;
	u8 Inter8x16Preden;
	u8 Inter8x8Preden;
	u8 gap1;
	u8 gap2;
	u32 skipPenalty;
	u32 diffMvPenalty;
	u32 diffMvPenalty4p;
	u32 madQpDelta;
	u32 madThreshold;
	u32 madCount;
	u32 mvcAnchorPicFlag;
	u32 mvcPriorityId;
	u32 mvcViewId;
	u32 mvcTemporalId;
	u32 mvcInterViewFlag;
	u32 cirStart;
	u32 cirInterval;
	u32 intraSliceMap1;
	u32 intraSliceMap2;
	u32 intraSliceMap3;
	u32 intraAreaTop;
	u32 intraAreaLeft;
	u32 intraAreaBottom;
	u32 intraAreaRight;
	u32 roi1Top;
	u32 roi1Left;
	u32 roi1Bottom;
	u32 roi1Right;
	u32 roi2Top;
	u32 roi2Left;
	u32 roi2Bottom;
	u32 roi2Right;
	u32 roi1DeltaQp;
	u32 roi2DeltaQp;
	u32 mvOutputBase;
	u32 cabacCtxBase;
	u32 colorConversionCoeffA;
	u32 colorConversionCoeffB;
	u32 colorConversionCoeffC;
	u32 colorConversionCoeffE;
	u32 colorConversionCoeffF;
	u32 rMaskMsb;
	u32 gMaskMsb;
	u32 bMaskMsb;
};

struct stream_s {
	u32 stream;
	u32 size;
	u32 byteCnt;
	u32 bitCnt;
	u32 byteBuffer;
	u32 bufferedBits;
	u32 zeroBytes;
	u32 overflow;
	u32 emulCnt;
	u32 table;
	u32 tableSize;
	u32 tableCnt;
};

struct preProcess_s {
	u32 lumWidthSrc;
	u32 lumHeightSrc;
	u32 lumWidth;
	u32 lumHeight;
	u32 horOffsetSrc;
	u32 verOffsetSrc;
	u32 inputFormat;
	u32 rotation;
	u32 videoStab;
	u32 colorConversionType;
	u32 colorConversionCoeffA;
	u32 colorConversionCoeffB;
	u32 colorConversionCoeffC;
	u32 colorConversionCoeffE;
	u32 colorConversionCoeffF;
};

struct JpegEncRestart {
	u32 _Lr;
	u32 Ri;
};

struct JpegEncCommentHeader {
	u32 comEnable;
	u32 Lc;
	u32 comLen;
	u32 pComment;
};

struct JpegEncFrameHeader {
	u32 header;
	u32 Lf;
	u32 P;
	u32 Y;
	u32 X;
	u32 _Nf;
	u32 Ci[3];
	u32 Hi[3];
	u32 Vi[3];
	u32 Tqi[3];
};

struct JpegEncAppn_t_0 {
	u32 Lp;
	u32 ident1;
	u32 ident2;
	u32 ident3;
	u32 version;
	u32 units;
	u32 Xdensity;
	u32 Ydensity;
	u32 thumbEnable;
};

struct JpegEncQuantTables {
	u32 pQlumi;
	u32 pQchromi;
};

struct JpegEncThumb {
	u32 format;
	u8 width;
	u8 height;
	u8 gap1;
	u8 gap2;
	u32 data;
	u16 dataLength;
	u8 gap3;
	u8 gap4;
};

struct jpegData_s {
	bool header;
	struct JpegEncRestart restart;
	u32 rstCount;
	struct JpegEncFrameHeader frame;
	struct JpegEncCommentHeader com;
	struct JpegEncAppn_t_0 appn;
	struct JpegEncQuantTables qTable;
	u32 markerType;
	u32 codingType;
	u32 codingMode;
	u32 sliceNum;
	u32 sliceRows;
	u32 width;
	u32 height;
	u32 mbNum;
	u32 mbPerFrame;
	u32 row;
	u8 qTableLuma[64];
	u8 qTableChroma[64];
	u32 streamStartAddress;
	struct JpegEncThumb thumbnail;
};

struct VenChnAttr {
	u32 width;
	u32 height;
	u32 type;
	u32 Field;
	u32 viField;
	u32 strmType;
	u32 sequence;
	u32 freeBytes;
	u32 busyBytes;
};

struct VencStrmState {
	u32 freeCnt;
	u32 busyCnt;
	u32 userGet;
	u32 userRls;
	u32 getTimes;
	u32 interval;
	u32 frameRate;
	u32 frameCntOk;
	u32 start;
	u32 getFrameErr;
	u32 bufferFull;
};

struct VencInfo {
	u32 chnNo;
	struct VenChnAttr chnAttr;
	struct VencStrmState strmState;
};

struct vui_t {
	u32 timeScale;
	u32 numUnitsInTick;
	u32 bitStreamRestrictionFlag;
	u32 videoFullRange;
	u32 sarWidth;
	u32 sarHeight;
	u32 nalHrdParametersPresentFlag;
	u32 vclHrdParametersPresentFlag;
	u32 pictStructPresentFlag;
	u32 initialCpbRemovalDelayLength;
	u32 cpbRemovalDelayLength;
	u32 dpbOutputDelayLength;
	u32 timeOffsetLength;
	u32 bitRate;
	u32 cpbSize;
};

struct pps_s {
	u32 byteStream;
	u32 picParameterSetId;
	u32 seqParameterSetId;
	bool entropyCodingMode;
	bool picOrderPresent;
	u32 numSliceGroupsMinus1;
	u32 numRefIdxL0ActiveMinus1;
	u32 numRefIdxL1ActiveMinus1;
	bool weightedPred;
	u32 weightedBipredIdc;
	u32 picInitQpMinus26;
	u32 picInitQsMinus26;
	u32 chromaQpIndexOffset;
	bool deblockingFilterControlPresent;
	bool constIntraPred;
	bool redundantPicCntPresent;
	bool transform8x8Mode;
	u32 enableCabac;
};

struct linReg_s {
	u32 a1;
	u32 a2;
	u32 qp_prev;
	u32 qs[11];
	u32 bits[11];
	u32 pos;
	u32 len;
	u32 zero_div;
};

struct timeStamp_s {
	u32 fts;
	u32 timeScale;
	u32 nuit;
	u32 time;
	u32 secf;
	u32 sec;
	u32 minf;
	u32 min;
	u32 hrf;
	u32 hr;
};

struct sei_s {
	u32 nalUnitSize;
	u32 enabled;
	bool byteStream;
	u32 hrd;
	u32 seqId;
	u32 icrd;
	u32 icrdLen;
	u32 icrdo;
	u32 icrdoLen;
	u32 crd;
	u32 crdLen;
	u32 dod;
	u32 dodLen;
	u32 _psp;
	u32 ps;
	u32 cts;
	u32 cntType;
	u32 cdf;
	u32 nframes;
	u32 toffs;
	u32 toffsLen;
	u32 userDataEnabled;
	u32 pUserData;
	u32 userDataSize;
};

struct h264QpCtrl_s {
	u32 wordError[7];
	u32 qpChange[7];
	u32 wordCntTarget[10];
	u32 wordCntPrev[10];
	u32 checkPointDistance;
	u32 checkPoints;
};

struct h264VirtualBuffer_s {
	u32 bufferSize;
	u32 bitRate;
	u32 bitPerPic;
	u32 picTimeInc;
	u32 timeScale;
	u32 unitsInTic;
	u32 virtualBitCnt;
	u32 realBitCnt;
	u32 bufferOccupancy;
	u32 skipFrameTarget;
	u32 skippedFrames;
	u32 nonZeroTarget;
	u32 bucketFullness;
	u32 windowRem;
	u32 rcMode;
};

struct h264RateControl_s {
	bool picRc;
	bool mbRc;
	bool picSkip;
	bool hrd;
	u32 fillerIdx;
	u32 mbPerPic;
	u32 mbRows;
	u32 coeffCntMax;
	u32 nonZeroCnt;
	u32 srcPrm;
	u32 qpSum;
	u32 sliceTypeCur;
	u32 sliceTypePrev;
	bool frameCoded;
	u32 fixedQp;
	u32 qpHdr;
	u32 qpMin;
	u32 qpMax;
	u32 qpHdrPrev;
	u32 qpLastCoded;
	u32 qpTarget;
	u32 estTimeInc;
	u32 outRateNum;
	u32 outRateDenom;
	u32 gDelaySum;
	u32 gInitialDelay;
	u32 gInitialDoffs;
	struct h264QpCtrl_s qpCtrl;
	struct h264VirtualBuffer_s virtualBuffer;
	struct h264VirtualBuffer_s sei;
	u32 gBufferMin;
	u32 gBufferMax;
	struct linReg_s linReg;
	struct linReg_s rError;
	struct linReg_s intra;
	struct linReg_s intraError;
	struct linReg_s gop;
	u32 targetPicSize;
	u32 frameBitCnt;
	u32 gopQpSum;
	u32 gopQpDiv;
	u32 gopBitCnt;
	u32 gopAvgBitCnt;
	u32 frameCnt;
	u32 gopLen;
	u32 windowLen;
	u32 intraInterval;
	u32 intraIntervalCtr;
	u32 intraQpDelta;
	u32 fixedIntraQp;
	u32 mbQpAdjustment;
};

struct sps_s {
	bool byteStream;
	u32 profileIdc;
	bool constraintSet0;
	bool constraintSet1;
	bool constraintSet2;
	bool constraintSet3;
	u32 levelIdc;
	u32 levelIdx;
	u32 seqParameterSetId;
	u32 log2MaxFrameNumMinus4;
	u32 picOrderCntType;
	u32 numRefFrames;
	bool gapsInFrameNumValueAllowed;
	u32 picWidthInMbsMinus1;
	u32 picHeightInMapUnitsMinus1;
	bool frameMbsOnly;
	bool direct8x8Inference;
	bool frameCropping;
	bool vuiParametersPresent;
	struct vui_t vui;
	u32 frameCropLeftOffset;
	u32 frameCropRightOffset;
	u32 frameCropTopOffset;
	u32 frameCropBottomOffset;
};

struct H264ProcType {
	struct stream_s stream;
	struct preProcess_s preProcess;
	struct sps_s seqParameterSet;
	struct pps_s picParameterSet;
	struct h264RateControl_s rateControl;
	struct regValues_s regs;
	u32 ChannelNum;
	u32 overFlowCnt;
};

static unsigned int counter_22192;
static struct hx280enc_t hx280enc_data;
static struct H264ProcType H264Proc[3];
static struct JpegProcType JpegProc;
static struct VencInfo VencProc[4];
static int hx280enc_major;

static int ReserveIO(void);
static void ReleaseIO(void);
static void ResetAsic(struct hx280enc_t *dev);

/* IRQ handler */
static irqreturn_t hx280enc_isr(int irq, void *dev_id);

/* VM operations */
static int hx280enc_vm_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	PDEBUG("hx280enc_vm_fault: problem with mem access\n");
	return VM_FAULT_SIGBUS; /* send a SIGBUS */
}

static void hx280enc_vm_open(struct vm_area_struct *vma)
{
	PDEBUG("hx280enc_vm_open:\n");
}

static void hx280enc_vm_close(struct vm_area_struct *vma)
{
	PDEBUG("hx280enc_vm_close:\n");
}

static struct vm_operations_struct hx280enc_vm_ops = {
	.open = hx280enc_vm_open,
	.close = hx280enc_vm_close,
	.fault = hx280enc_vm_fault,
};

/* the device's mmap method. The VFS has kindly prepared the process's
 * vm_area_struct for us, so we examine this to see what was requested.
 */

static int hx280enc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int result = -EINVAL;

	result = -EINVAL;

	vma->vm_ops = &hx280enc_vm_ops;

	return result;
}

static long hx280enc_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long arg)
{
	int err = 0;

	PDEBUG("ioctl cmd 0x%08ux\n", cmd);
	/*
	 * extract the type and number bitfields, and don't encode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != HX280ENC_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > HX280ENC_IOC_MAXNR)
		return -ENOTTY;

	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	switch (cmd) {
	case HX280ENC_IOCGHWOFFSET:
		__put_user(hx280enc_data.iobaseaddr, (unsigned long *)arg);
		break;

	case HX280ENC_IOCGHWIOSIZE:
		__put_user(hx280enc_data.iosize, (unsigned int *)arg);
		break;
	}
	return 0;
}

static int hx280enc_open(struct inode *inode, struct file *filp)
{
	int result = 0;
	struct hx280enc_t *dev = &hx280enc_data;

	filp->private_data = (void *)dev;

	PDEBUG("dev opened\n");
	return result;
}

static int hx280enc_fasync(int fd, struct file *filp, int mode)
{
	struct hx280enc_t *dev = (struct hx280enc_t *)filp->private_data;

	PDEBUG("fasync called\n");

	return fasync_helper(fd, filp, mode, &dev->async_queue);
}

static int hx280enc_release(struct inode *inode, struct file *filp)
{
	/* remove this filp from the asynchronusly notified filp's */
	hx280enc_fasync(-1, filp, 0);

	PDEBUG("dev closed\n");
	return 0;
}

/* VFS methods */
static const struct file_operations hx280enc_fops = {
	.owner = THIS_MODULE,
	.mmap = hx280enc_mmap,
	.open = hx280enc_open,
	.release = hx280enc_release,
	.unlocked_ioctl = hx280enc_ioctl,
	.fasync = hx280enc_fasync,
};

static struct miscdevice xm_h264_dev = {
	0xFF,
	VIDEO_ENC_CHRDEV_NAME,
	&hx280enc_fops,
};

static int __init hx280_video_enc_init(void)
{
	int result;

#if 0
	struct proc_dir_entry *pvenc, *ph264;

	pvenc = create_proc_entry("umap/venc", S_IFDIR | S_IRUGO | S_IXUGO, NULL);
	if (pvenc == NULL)
		return -ENOMEM;

	ph264 = create_proc_entry("umap/h264", S_IFDIR | S_IRUGO | S_IXUGO, NULL);
	if (ph264 == NULL)
		return -ENOMEM;
#endif

	hx280enc_data.iosize = 0x1C4;
	hx280enc_data.async_queue = 0;
	hx280enc_data.hwregs = 0;
	hx280enc_data.iobaseaddr = 0x20070000;
	// XM510 specific
	hx280enc_data.irq = 0x1A;

	if (misc_register(&xm_h264_dev) < 0) {
		pr_err("Unable to register driver\n");
		return -1;
	}

	if (ReserveIO() < 0) {
		pr_err("Unable reserve IO\n");
		return -1;
	}

	ResetAsic(&hx280enc_data); /* reset hardware */
	/* get the IRQ line */
	if (hx280enc_data.irq > 0) {
		result = request_irq(hx280enc_data.irq, hx280enc_isr,
				     IRQF_DISABLED | IRQF_SHARED, "hx280enc",
				     (void *)&hx280enc_data);
		if (result != 0) {
			if (result == -EINVAL) {
				pr_err("Bad irq number or handler\n");
			} else if (result == -EBUSY) {
				pr_err("IRQ <%d> busy, change your config\n",
				       hx280enc_data.irq);
			}

			ReleaseIO();
			return -1;
		}
	} else {
		pr_info("IRQ not in use!\n");
	}

	pr_info("Video encoder initialized. Major = %d\n", hx280enc_major);

	return 0;
}
module_init(hx280_video_enc_init);

static void __exit hx280_video_enc_cleanup(void)
{
	/* disable HW */
	writel(0, hx280enc_data.hwregs + X280_REGISTER_DISABLE_ENC);
	/* clear enc IRQ */
	writel(0, hx280enc_data.hwregs + X280_INTERRUPT_REGISTER_ENC);

	/* free the encoder IRQ */
	if (hx280enc_data.irq != -1)
		free_irq(hx280enc_data.irq, (void *)&hx280enc_data);

	ReleaseIO();
#if 0
	remove_proc_entry("umap/venc", 0);
	remove_proc_entry("umap/h264", 0);
#endif
	misc_deregister(&xm_h264_dev);
	printk("<6>hx280enc: module removed\n");
}
module_exit(hx280_video_enc_cleanup);

static int ReserveIO(void)
{
	long int hwid;

	hx280enc_data.hwregs = (volatile u8 *)ioremap_nocache(
		hx280enc_data.iobaseaddr, hx280enc_data.iosize);

	if (hx280enc_data.hwregs == NULL) {
		printk(KERN_INFO "hx280enc: failed to ioremap HW regs\n");
		ReleaseIO();
		return -EBUSY;
	}

	hwid = readl(hx280enc_data.hwregs);

	/* check for encoder HW ID */
	if ((((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID1 >> 16) & 0xFFFF)) &&
	    (((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID2 >> 16) & 0xFFFF)) &&
	    (((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID3 >> 16) & 0xFFFF)) &&
	    (((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID4 >> 16) & 0xFFFF))) {
		printk(KERN_INFO
		       "hx280enc: HW not found at 0x%08lx (hwid: 0x%08lx)\n",
		       hx280enc_data.iobaseaddr, hwid);
#ifdef CONFIG_VIDEO_SPEAR_VIDEOENC_DEBUG
		dump_regs((unsigned long)&hx280enc_data);
#endif
		ReleaseIO();
		return -EBUSY;
	}

	printk(KERN_INFO "hx280enc: HW at base <0x%08lx> with ID <0x%08lx>\n",
	       hx280enc_data.iobaseaddr, hwid);

	return 0;
}

static void ReleaseIO(void)
{
	if (hx280enc_data.hwregs)
		iounmap((void *)hx280enc_data.hwregs);
}

irqreturn_t hx280enc_isr(int irq, void *dev_id)
{
	struct hx280enc_t *dev = (struct hx280enc_t *)dev_id;
	u32 irq_status;

	irq_status = readl(dev->hwregs + 0x04);

	if (irq_status & 0x01) {
		/* clear enc IRQ and slice ready interrupt bit */
		writel(irq_status & (~0x101), dev->hwregs + 0x04);

		/* Handle slice ready interrupts. The reference implementation
		 * doesn't signal slice ready interrupts to EWL.
		 * The EWL will poll the slices ready register value. */
		if ((irq_status & 0x1FE) == 0x100) {
			PDEBUG("Slice ready IRQ handled!\n");
			return IRQ_HANDLED;
		}

		/* All other interrupts will be signaled to EWL. */
		if (dev->async_queue)
			kill_fasync(&dev->async_queue, SIGIO, POLL_IN);
		else {
			printk(KERN_WARNING
			       "hx280enc: IRQ received w/o anybody waiting for it!\n");
		}

		PDEBUG("IRQ handled!\n");
		return IRQ_HANDLED;
	} else {
		PDEBUG("IRQ received, but NOT handled!\n");
		return IRQ_NONE;
	}
}

void ResetAsic(struct hx280enc_t *dev)
{
	int i;

	writel(0, dev->hwregs + 0x38);

	for (i = 4; i < dev->iosize; i += 4)
		writel(0, dev->hwregs + i);
}
