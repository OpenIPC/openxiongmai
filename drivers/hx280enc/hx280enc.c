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
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/string.h>
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

/* Same parameter names as the vendor xm530_h264.ko, which defaults to 77 */
#ifdef H264_V200
/* the XM533's and XM535s' h264.ko, the same encoder moved */
static unsigned long base_port = 0x21100000;
static int irq = 78;
#else
static unsigned long base_port = 0x20070000;
#ifdef CONFIG_ARCH_XM530
static int irq = 77;
#else
static int irq = 0x1A; /* XM510 */
#endif
#endif

/* module_param(name, type, perm) */
module_param(base_port, ulong, 0);
module_param(irq, int, 0);

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

static struct hx280enc_t hx280enc_data;
static DECLARE_WAIT_QUEUE_HEAD(hx280enc_wait);
/* A finished frame not yet claimed by HX280ENC_IOCWAITDONE: at most one is
 * kept, as in the vendor module, and each is handed to a single waiter. */
static atomic_t hx280enc_done = ATOMIC_INIT(0);
static atomic_t hx280enc_users = ATOMIC_INIT(0);
static bool hx280enc_irq_requested;
/*
 * The statistics records user space pushes, kept raw: the structs above are
 * close to but not the records the vendor modules take. /proc/umap shows the
 * fields the vendor modules print (see venc_show and h264_show).
 */
/* the last two words: the channel, its overflow count */
#define H264PROC_CHANNEL_OFFSET (H264PROC_SIZE - 8)
static u8 H264Proc[3][_IOC_SIZE(HX280ENC_IOCSH264PROC)];
static u8 JpegProc[_IOC_SIZE(HX280ENC_IOCSJPEGPROC)];
static u8 VencProc[4][_IOC_SIZE(HX280ENC_IOCSVENCINFO)];
static DEFINE_MUTEX(proc_lock); /* the three above */
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

static u32 rec32(const u8 *rec, unsigned int off)
{
	u32 v;

	memcpy(&v, rec + off, sizeof(v));
	return v;
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

	case HX280ENC_IOCSH264PROC: {
		u8 *rec = memdup_user((void __user *)arg, sizeof(H264Proc[0]));
		u32 ch;

		if (IS_ERR(rec))
			return PTR_ERR(rec);
		ch = rec32(rec, H264PROC_CHANNEL_OFFSET);
		/* the vendor module ignores out-of-range channels */
		if (ch < ARRAY_SIZE(H264Proc)) {
			mutex_lock(&proc_lock);
			memcpy(H264Proc[ch], rec, sizeof(H264Proc[ch]));
			mutex_unlock(&proc_lock);
		}
		kfree(rec);
		break;
	}

	case HX280ENC_IOCSJPEGPROC: {
		u8 *rec = memdup_user((void __user *)arg, sizeof(JpegProc));

		if (IS_ERR(rec))
			return PTR_ERR(rec);
		mutex_lock(&proc_lock);
		memcpy(JpegProc, rec, sizeof(JpegProc));
		mutex_unlock(&proc_lock);
		kfree(rec);
		break;
	}

	case HX280ENC_IOCSVENCINFO: {
		u8 rec[sizeof(VencProc[0])];
		u32 idx;

		if (copy_from_user(rec, (void __user *)arg, sizeof(rec)))
			return -EFAULT;
		idx = rec32(rec, 0);
		if (idx < ARRAY_SIZE(VencProc)) {
			mutex_lock(&proc_lock);
			memcpy(VencProc[idx], rec, sizeof(rec));
			mutex_unlock(&proc_lock);
		}
		break;
	}

	case HX280ENC_IOCWAITDONE:
		/* -EPERM on a signal is what xm530_h264.ko returns */
		if (wait_event_interruptible(hx280enc_wait,
					     atomic_xchg(&hx280enc_done, 0)))
			return -EPERM;
		break;
	}
	return 0;
}

static int hx280enc_open(struct inode *inode, struct file *filp)
{
	int result = 0;
	struct hx280enc_t *dev = &hx280enc_data;

	filp->private_data = (void *)dev;

	/* The first user must not inherit a frame that finished for an
	 * earlier one. Later opens keep it: it may be another user's. */
	if (atomic_inc_return(&hx280enc_users) == 1)
		atomic_set(&hx280enc_done, 0);

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
	atomic_dec(&hx280enc_users);

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

/*
 * /proc/umap/venc and /proc/umap/h264, in the vendor modules' layout, from
 * what user space last pushed. The vendor modules open these files with
 * seq_open() and never release them, so each open leaks a 4 KiB buffer that
 * not even rmmod returns: majestic's /metrics reads venc, and a scrape every
 * 30 s took 18 MB of an XM550 in 20 h (OpenIPC/firmware#2956).
 *
 * The V200 series' encoder has no such tables here: their umap/venc is
 * drivers/xm_v200_venc's.
 */
#ifndef H264_V200
#define VENC_CNT (HX280ENC_VENC_ERRCNT ? 4 : 0) /* words after ErrCnt move */

static int venc_show(struct seq_file *s, void *unused)
{
	unsigned int i;

	mutex_lock(&proc_lock);
	seq_puts(s, "\n\n-------VENC CHN ATTR--------------------------------"
		    "------------------------------\n");
	seq_puts(s, "NO. Width  Height  Type  Field  VIField  StreamType  "
		    "Sequence  freeBytes  busyBytes\n");
	for (i = 0; i < ARRAY_SIZE(VencProc); i++) {
		const u8 *r = VencProc[i];

		seq_printf(s,
			   "%-2d  %-5d  %-6d  %-4d  %-5d  %-7d  %-10d  %-8d  "
			   "%-9d  %-9d\t\n",
			   i, rec32(r, 4), rec32(r, 8), r[12], r[13], r[14],
			   r[15], rec32(r, 16), rec32(r, 20), rec32(r, 24));
	}
	seq_puts(s, "-------VENC STREAM STATE------------------------------"
		    "--------------------------------\n");
	seq_puts(s, "NO. Start  SucCnt  ");
	if (HX280ENC_VENC_ERRCNT)
		seq_puts(s, "ErrCnt  ");
	seq_puts(s, "GetFrErr  BufErr  FreeCnt  BusyCnt  UserGet  UserRls  "
		    "GetTimes  Interval(us)  FramRate\n");
	for (i = 0; i < ARRAY_SIZE(VencProc); i++) {
		const u8 *r = VencProc[i];

		seq_printf(s, "%-2d  %-5s  %-6d  ", i,
			   rec32(r, 60 + VENC_CNT) ? "Y" : "N", rec32(r, 56));
		if (HX280ENC_VENC_ERRCNT)
			seq_printf(s, "%-6d  ", rec32(r, 60));
		seq_printf(s,
			   "%-8d  %-6d  %-7d  %-7d  %-7d  %-7d  %-8d  %-8d"
			   "      %-8d \n",
			   rec32(r, 64 + VENC_CNT), rec32(r, 68 + VENC_CNT),
			   rec32(r, 28), rec32(r, 32), rec32(r, 36),
			   rec32(r, 40), rec32(r, 44), rec32(r, 48),
			   rec32(r, 52));
	}
	mutex_unlock(&proc_lock);
	return 0;
}

/* a "not used" flag as the prediction table shows it */
static const char *used(u32 disabled)
{
	return disabled ? "NO" : "YES";
}

static int h264_show(struct seq_file *s, void *unused)
{
	const unsigned int n = ARRAY_SIZE(H264Proc);
	const unsigned int t = H264PROC_TAIL;
	const u8 *r;
	unsigned int i;
	u32 v;

	mutex_lock(&proc_lock);
	seq_puts(s, "\n\n[H264] openxiongmai hx280enc\n");
	seq_puts(s, "----------------------------------CHN ATTR------------"
		    "------------------------\n");
	seq_puts(s, "ID  Width  Height  InLBase     InCbBase    InCrBase    "
		    "MainStr   H264+  \n");
	for (i = 0; i < n; i++) {
		r = H264Proc[i];
		seq_printf(s,
			   "%-2d  %-5d  %-6d  0x%-8x  0x%-8x  0x%-8x  %-7s  "
			   "%-7s\n",
			   rec32(r, H264PROC_CHANNEL_OFFSET), rec32(r, 56),
			   rec32(r, 60), rec32(r, 1596), rec32(r, 1600),
			   rec32(r, 1604), i ? "NO" : "YES",
			   r[1404] ? "enable" : "disable");
	}
	seq_puts(s, "-------------------------------PPS INFO --------------"
		    "--------------------\n");
	seq_puts(s, "ID  Type   ppsID  spsID  ChrQpOffset  constIntraPred  "
		    "8x8Mod    CABAC  \n");
	for (i = 0; i < n; i++) {
		r = H264Proc[i];
		v = rec32(r, 328);
		seq_printf(s,
			   "%-2d  %-5s  %-5d  %-5d  %-11d  %-14s  %-8s  "
			   "%-17s \n",
			   rec32(r, H264PROC_CHANNEL_OFFSET),
			   rec32(r, 260) == 1 ? "ANNEX" : "PLAIN",
			   rec32(r, 264), rec32(r, 268), rec32(r, 308),
			   rec32(r, 316) ? "YES" : "NO",
			   rec32(r, 324) == 2 ? "enable" :
			   rec32(r, 324) == 1 ? "adaptive" : "disable",
			   !v ? "cavlc" : v == 1 ? "cabac" :
						   "cavlc(I)+cabac(P)");
	}
	seq_puts(s, "-------------------------------STREAM BUFFER----------"
		    "------------------------\n");
	seq_puts(s, "ID  OutBase     Size      OverFlow  streamType\n");
	for (i = 0; i < n; i++) {
		r = H264Proc[i];
		seq_printf(s, "%-2d  0x%-8x  %-8d  %-6d %-10s\n",
			   rec32(r, H264PROC_CHANNEL_OFFSET), rec32(r, 1532),
			   rec32(r, 1536),
			   rec32(r, H264PROC_CHANNEL_OFFSET + 4),
			   rec32(r, 260) == 1 ? " ANNEX" : " PLAIN");
	}
	seq_puts(s, "-------------------------------ROI INFO --------------"
		    "------------------------\n");
	seq_puts(s, "ID  index  QP  Left  Top   Right  Bottom\n");
	for (i = 0; i < n; i++) {
		r = H264Proc[i];
		seq_printf(s, "%-2d  %-5d  %-2d  %-4d  %-4d  %-5d  %-6d\n",
			   rec32(r, H264PROC_CHANNEL_OFFSET), 1,
			   rec32(r, 2460 - t), rec32(r, 2432 - t),
			   rec32(r, 2428 - t), rec32(r, 2440 - t),
			   rec32(r, 2436 - t));
		seq_printf(s, "    %-5d  %-2d  %-4d  %-4d  %-5d  %-6d\n", 2,
			   rec32(r, 2464 - t), rec32(r, 2448 - t),
			   rec32(r, 2444 - t), rec32(r, 2456 - t),
			   rec32(r, 2452 - t));
	}
	seq_puts(s, "---------------------Inter & Intra prediction INFO----"
		    "------------------------\n");
	seq_puts(s, "ID  profile  P16x16  P16x8  P8x16  P8x8  P8x4  P4x8  "
		    "P4x4  1/4PixelMv\n");
	for (i = 0; i < n; i++) {
		r = H264Proc[i];
		v = rec32(r, 112);
		seq_printf(s,
			   "%-2d  %-7s  %-6s  %-5s  %-5s  %-4s  %-4s  %-4s  "
			   "%-4s  %-s\n",
			   rec32(r, H264PROC_CHANNEL_OFFSET),
			   v == 66 ? "BP" : v == 77 ? "MP" : "HP",
			   used(r[2342 - t]), used(r[2343 - t]),
			   used(r[2344 - t]), used(r[2345 - t]),
			   used(r[2340 - t]), used(r[2341 - t]),
			   used(rec32(r, 2252 - t)), used(rec32(r, 2256 - t)));
	}
	seq_puts(s, "-------------------------------RC  INFO --------------"
		    "---------------------\n");
	seq_puts(s, "ID  QP  QPMin  QPMax  GOPLen  frameCnt  QpDelta  "
		    "gopBitCnt  gopAvgBitCnt  rcMod  bitRate  frameRate\n");
	for (i = 0; i < n; i++) {
		r = H264Proc[i];
		seq_printf(s,
			   "%-2d  %-2d  %-5d  %-5d  %-6d  %-8d  %-7d  %-8d\t"
			   "%-8d  %-5s  %-8d  %-8d \n",
			   rec32(r, H264PROC_CHANNEL_OFFSET), rec32(r, 392),
			   rec32(r, 396), rec32(r, 400), rec32(r, 1376),
			   rec32(r, 1372), rec32(r, 1392), rec32(r, 1364),
			   rec32(r, 1368), rec32(r, 640) == 1 ? "CBR" : "VBR",
			   rec32(r, 588), rec32(r, 420));
	}
	v = rec32(JpegProc, 0x4c);
	seq_puts(s, "\n\n[JPEG] openxiongmai hx280enc\n");
	seq_puts(s, "-------------------------------JPEG INFO -------------"
		    "----------------------\n");
	seq_puts(s, "Width  Height  rotation\tcodingType  CodedSize  \n");
	seq_printf(s, "%-5d  %-6d  %-8s  %-10s  %-9d               \n",
		   rec32(JpegProc, 0x38), rec32(JpegProc, 0x3c),
		   !v ? "Disable" : v == 1 ? "R-90" : "L-90",
		   rec32(JpegProc, JPEGPROC_CODING_TYPE) == 1 ? "SLICE" :
								"WHOLE",
		   rec32(JpegProc, 0xe0));
	mutex_unlock(&proc_lock);
	return 0;
}

static int venc_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, venc_show, NULL);
}

static int h264_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, h264_show, NULL);
}

static const struct file_operations venc_proc_fops = {
	.owner = THIS_MODULE,
	.open = venc_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations h264_proc_fops = {
	.owner = THIS_MODULE,
	.open = h264_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct proc_dir_entry *venc_proc, *h264_proc;

/* /proc/umap is the MMZ module's; encoding does not need the tables */
static void proc_start(void)
{
	venc_proc = proc_create("umap/venc", 0444, NULL, &venc_proc_fops);
	h264_proc = proc_create("umap/h264", 0444, NULL, &h264_proc_fops);
	if (!venc_proc || !h264_proc)
		pr_warn("hx280enc: no /proc/umap/venc or h264, "
			"MMZ not loaded?\n");
}

static void proc_stop(void)
{
	if (venc_proc)
		remove_proc_entry("umap/venc", NULL);
	if (h264_proc)
		remove_proc_entry("umap/h264", NULL);
}
#else
static void proc_start(void)
{
}

static void proc_stop(void)
{
}
#endif

static struct miscdevice xm_h264_dev = {
	0xFF,
	VIDEO_ENC_CHRDEV_NAME,
	&hx280enc_fops,
};

static int __init hx280_video_enc_init(void)
{
	int result;

	hx280enc_data.iosize = 0x1C4;
	hx280enc_data.async_queue = 0;
	hx280enc_data.hwregs = 0;
	hx280enc_data.iobaseaddr = base_port;
	hx280enc_data.irq = irq;

#ifdef H264_V200
	{
		/* the encoder's clock, which the V200 series' h264.ko turns on */
		void __iomem *crg = ioremap(0x20000120, 4);

		if (crg) {
			writel(readl(crg) | 3, crg);
			iounmap(crg);
		}
	}
#endif
	if (ReserveIO() < 0) {
		pr_err("Unable reserve IO\n");
		return -1;
	}

	if (misc_register(&xm_h264_dev) < 0) {
		pr_err("Unable to register driver\n");
		ReleaseIO();
		return -1;
	}

	ResetAsic(&hx280enc_data); /* reset hardware */
	/* get the IRQ line */
	if (irq > 0) {
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

			misc_deregister(&xm_h264_dev);
			ReleaseIO();
			return -1;
		}
		hx280enc_irq_requested = true;
	} else {
		pr_info("IRQ not in use!\n");
	}

	proc_start();
	pr_info("Video encoder initialized. Major = %d\n", hx280enc_major);

	return 0;
}
module_init(hx280_video_enc_init);

static void __exit hx280_video_enc_cleanup(void)
{
	proc_stop();
	/* disable HW */
	writel(0, hx280enc_data.hwregs + X280_REGISTER_DISABLE_ENC);
	/* clear enc IRQ */
	writel(0, hx280enc_data.hwregs + X280_INTERRUPT_REGISTER_ENC);

	/* free the encoder IRQ */
	if (hx280enc_irq_requested)
		free_irq(hx280enc_data.irq, (void *)&hx280enc_data);

	ReleaseIO();
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

		/* A reset is not a finished frame: clear it and wake nobody,
		 * as xm530_h264.ko does. */
		if (irq_status & 0x10) {
			writel(0, dev->hwregs + 0x04);
			return IRQ_HANDLED;
		}

		/* All other interrupts complete the frame: wake
		 * HX280ENC_IOCWAITDONE and signal the reference EWL. */
		atomic_set(&hx280enc_done, 1);
		wake_up_interruptible(&hx280enc_wait);
		if (dev->async_queue)
			kill_fasync(&dev->async_queue, SIGIO, POLL_IN);

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
