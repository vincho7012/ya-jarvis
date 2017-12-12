
//VIV ==========================================================================
#if 0

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Do-nothing test driver");
MODULE_VERSION("0.1");

static int __init virt_snd_drv_init(void){
   printk(KERN_INFO "Hello, world.\n");
   return 0;
}

static void __exit virt_snd_drv_exit(void){
   printk(KERN_INFO "Goodbye, world.\n");
}

module_init(virt_snd_drv_init);
module_exit(virt_snd_drv_exit);

#endif
//VIV ==========================================================================



/*
 *  Minimal virtual oscillator (minivosc) soundcard
 *
 *  Based on Loopback soundcard (aloop-kernel.c):
 *  Original code:
 *  Copyright (c) by Jaroslav Kysela <perex@perex.cz>
 *
 *  More accurate positioning and full-duplex support:
 *  Copyright (c) Ahmet İnan <ainan at mathematik.uni-freiburg.de>
 *
 *  Major (almost complete) rewrite:
 *  Copyright (c) by Takashi Iwai <tiwai@suse.de>
 *
 *  with snippets from Ben Collins: Writing an ALSA driver
 *  http://ben-collins.blogspot.com/2010/04/writing-alsa-driver.html
 *
 *  minivosc specific parts:
 *  Copyright (c) by Smilen Dimitrov <sd at imi.aau.dk>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

static int debug = 1;
/* Use our own dbg macro http://www.n1ywb.com/projects/darts/darts-usb/darts-usb.c*/
#undef dbg
#define dbg(format, arg...) do { if (debug) printk(KERN_DEBUG __FILE__ ": " format "\n" , ## arg); } while (0)
#define dbg2(format, arg...) do { if (debug) printk( ": " format "\n" , ## arg); } while (0)


/* Here is our user defined breakpoint to */
/* initiate communication with remote (k)gdb */
/* don't use if not actually using kgdb */
#define BREAKPOINT() asm("   int $3");

#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/netdevice.h>
#include <linux/in.h>

// copy from aloop-kernel.c:
#include <linux/init.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <linux/version.h>

MODULE_AUTHOR("sdaau");
MODULE_DESCRIPTION("minivosc soundcard");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA,minivosc soundcard}}");


#define MODULE_NAME "ksocket"

#define DEFAULT_PORT (27772) //(49153) //27772

#define INADDR_RECV "192.168.1.101" //VIV "192.168.1.104"
//#define INADDR_RECV ((unsigned long int)0xC0A80168) /* 192.168.1.104 */

#define SLEEP_MILLI_SEC(nMilliSec)\
do { \
long timeout = (nMilliSec) * HZ / 1000; \
while(timeout > 0) \
{ \
timeout = schedule_timeout(timeout); \
} \
}while(0);

struct media_bus_t
{
	struct
	{
		struct task_struct *thread_id;
		int running;
	} sender;

	struct
	{
		struct task_struct *thread_id;
		int running;
	} receiver;

	//VIV struct task_struct *thread;
	struct socket *sock;
	struct sockaddr_in addr;
	//VIV int running;
};

struct media_bus_t *media_bus = NULL;

struct mbus_cfg_t
{
	char* my_ip_addr;
	unsigned short my_listening_port;
	char* sendto_ip_addr;
	unsigned short sendto_port;
} mbus_cfg = {
	.my_ip_addr = "192.168.1.101",
	.my_listening_port = 27772,
	.sendto_ip_addr = "192.168.1.102",
	.sendto_port = 37773,
};


static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;	/* Index 0-MAX */
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;	/* ID for this card */
static int enable[SNDRV_CARDS] = {1, [1 ... (SNDRV_CARDS - 1)] = 0};

static struct platform_device *devices[SNDRV_CARDS];

#define byte_pos(x)	((x) / HZ)
#define frac_pos(x)	((x) * HZ)

#define MAX_BUFFER (32 * 48)
static struct snd_pcm_hardware minivosc_pcm_hw =
{
	.info = (SNDRV_PCM_INFO_MMAP |
	SNDRV_PCM_INFO_INTERLEAVED |
	SNDRV_PCM_INFO_BLOCK_TRANSFER |
	SNDRV_PCM_INFO_MMAP_VALID),
	.formats          = SNDRV_PCM_FMTBIT_U8,
	.rates            = SNDRV_PCM_RATE_8000,
	.rate_min         = 8000,
	.rate_max         = 8000,
	.channels_min     = 1,
	.channels_max     = 1,
	.buffer_bytes_max = MAX_BUFFER, //(32 * 48) = 1536,
	.period_bytes_min = 48,
	.period_bytes_max = 48,
	.periods_min      = 1,
	.periods_max      = 32,
};


struct minivosc_device
{
	struct snd_card *card;
	struct snd_pcm *pcm;
	const struct minivosc_pcm_ops *timer_ops;
	/*
	* we have only one substream, so all data in this struct
	*/
	/* copied from struct loopback: */
	struct mutex cable_lock;
	/* copied from struct loopback_cable: */
	/* PCM parameters */
	unsigned int pcm_period_size;
	unsigned int pcm_bps;		/* bytes per second */
	/* flags */
	unsigned int valid;
	unsigned int running;
	unsigned int period_update_pending :1;
	/* timer stuff */
	unsigned int irq_pos;		/* fractional IRQ position */
	unsigned int period_size_frac;
	unsigned long last_jiffies;
	struct timer_list timer;
	/* copied from struct loopback_pcm: */
	struct snd_pcm_substream *substream;
	unsigned int pcm_buffer_size;
	unsigned int buf_pos;	/* position in buffer */
	unsigned int silent_size;
	/* added for waveform: */
	unsigned int wvf_pos;	/* position in waveform array */
	unsigned int wvf_lift;	/* lift of waveform array */
};

// waveform
#if defined(COPYALG_V1) || defined(COPYALG_V2) || defined(COPYALG_V3)
static char wvfdat[]={	20, 22, 24, 25, 24, 22, 21,
			19, 17, 15, 14, 15, 17, 19,
			20, 127, 22, 19, 17, 15, 19};
#endif
#if defined(COPYALG_V1) || defined(COPYALG_V2)
static char wvfdat2[]={	20, 22, 24, 25, 24, 22, 21,
			19, 17, 15, 14, 15, 17, 19,
			20, 127, 22, 19, 17, 15, 19};
#endif
#if defined(COPYALG_V1) || defined(COPYALG_V2) || defined(COPYALG_V3)
static unsigned int wvfsz=sizeof(wvfdat);//*sizeof(float) is included already
#endif
// * functions for driver/kernel module initialization
static void minivosc_unregister_all(void);
static int __init virt_snd_drv_init(void);
static void __exit virt_snd_drv_exit(void);

// * declare functions for this struct describing the driver (to be defined later):
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
static int __devinit minivosc_probe(struct platform_device *devptr);
static int __devexit minivosc_remove(struct platform_device *devptr);
#else
static int minivosc_probe(struct platform_device *devptr);
static int minivosc_remove(struct platform_device *devptr);
#endif


// * here declaration of functions that will need to be in _ops, before they are defined
static int minivosc_hw_params(struct snd_pcm_substream *ss,
                        struct snd_pcm_hw_params *hw_params);
static int minivosc_hw_free(struct snd_pcm_substream *ss);
static int minivosc_pcm_open(struct snd_pcm_substream *ss);
static int minivosc_pcm_close(struct snd_pcm_substream *ss);
static int minivosc_pcm_prepare(struct snd_pcm_substream *ss);
static int minivosc_pcm_trigger(struct snd_pcm_substream *ss,
                          int cmd);
static snd_pcm_uframes_t minivosc_pcm_pointer(struct snd_pcm_substream *ss);

static int minivosc_pcm_dev_free(struct snd_device *device);
static int minivosc_pcm_free(struct minivosc_device *chip);

// * declare timer functions - copied from aloop-kernel.c
static void minivosc_timer_start(struct minivosc_device *mydev);
static void minivosc_timer_stop(struct minivosc_device *mydev);
static void minivosc_pos_update(struct minivosc_device *mydev);
static void minivosc_timer_function(unsigned long data);
static void minivosc_xfer_buf(struct minivosc_device *mydev, unsigned int count);
static void minivosc_fill_capture_buf(struct minivosc_device *mydev, unsigned int bytes);


// note snd_pcm_ops can usually be separate _playback_ops and _capture_ops
static struct snd_pcm_ops minivosc_pcm_ops =
{
	.open      = minivosc_pcm_open,
	.close     = minivosc_pcm_close,
	.ioctl     = snd_pcm_lib_ioctl,
	.hw_params = minivosc_hw_params,
	.hw_free   = minivosc_hw_free,
	.prepare   = minivosc_pcm_prepare,
	.trigger   = minivosc_pcm_trigger,
	.pointer   = minivosc_pcm_pointer,
};

// specifies what func is called @ snd_card_free
// used in snd_device_new
static struct snd_device_ops dev_ops =
{
	.dev_free = minivosc_pcm_dev_free,
};


#define SND_MINIVOSC_DRIVER	"snd_minivosc"

// * we need a struct describing the driver:
static struct platform_driver minivosc_driver =
{
	.probe		= minivosc_probe,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
	.remove		= __devexit_p(minivosc_remove),
#else
	.remove		= minivosc_remove,
#endif
//~ #ifdef CONFIG_PM
	//~ .suspend	= minivosc_suspend,
	//~ .resume	= minivosc_resume,
//~ #endif
	.driver		= {
		.name	= SND_MINIVOSC_DRIVER,
		.owner = THIS_MODULE
	},
};


/*
 *
 * Probe/remove functions
 *
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
static int __devinit minivosc_probe(struct platform_device *devptr)
#else
static int minivosc_probe(struct platform_device *devptr)
#endif
{

	struct snd_card *card;
	struct minivosc_device *mydev;
	int ret;

	int nr_subdevs; // how many capture substreams we want
	struct snd_pcm *pcm;

	int dev = devptr->id; // from aloop-kernel.c

	dbg("%s: probe", __func__);


	// no need to kzalloc minivosc_device separately, if the sizeof is included here
	ret = snd_card_new(NULL, index[dev], id[dev],
	                      THIS_MODULE, sizeof(struct minivosc_device), &card);

	if (ret < 0)
		goto __nodev;

	mydev = card->private_data;
	mydev->card = card;
	// MUST have mutex_init here - else crash on mutex_lock!!
	mutex_init(&mydev->cable_lock);

	dbg2("-- mydev %p", mydev);

	sprintf(card->driver, "my_driver-%s", SND_MINIVOSC_DRIVER);
	sprintf(card->shortname, "MySoundCard Audio %s", SND_MINIVOSC_DRIVER);
	sprintf(card->longname, "%s", card->shortname);


	snd_card_set_dev(card, &devptr->dev); // present in dummy, not in aloop though


	ret = snd_device_new(card, SNDRV_DEV_LOWLEVEL, mydev, &dev_ops);

	if (ret < 0)
		goto __nodev;


	nr_subdevs = 1; // how many capture substreams we want
	// * we want 0 playback, and 1 capture substreams (4th and 5th arg) ..
	ret = snd_pcm_new(card, card->driver, 0, 0, nr_subdevs, &pcm);

	if (ret < 0)
		goto __nodev;


	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &minivosc_pcm_ops); // in both aloop-kernel.c and dummy.c, after snd_pcm_new...
	pcm->private_data = mydev; //here it should be dev/card struct (the one containing struct snd_card *card) - this DOES NOT end up in substream->private_data

	pcm->info_flags = 0;
	strcpy(pcm->name, card->shortname);

	/*
	trid to add this - but it crashes here:
	//mydev->substream->private_data = mydev;
	Well, first time real substream comes in, is in _open - so
	that has to be handled there.. That is: at this point, mydev->substream is null,
	and we first have a chance to set it ... in _open!
	*/

	ret = snd_pcm_lib_preallocate_pages_for_all(pcm,
	        SNDRV_DMA_TYPE_CONTINUOUS,
	        snd_dma_continuous_data(GFP_KERNEL),
	        MAX_BUFFER, MAX_BUFFER); // in both aloop-kernel.c and dummy.c, after snd_pcm_set_ops...

	if (ret < 0)
		goto __nodev;

	// * will use the snd_card_register form from aloop-kernel.c/dummy.c here..
	ret = snd_card_register(card);

	if (ret == 0)   // or... (!ret)
	{
		platform_set_drvdata(devptr, card);
		return 0; // success
	}

__nodev: // as in aloop/dummy...
	dbg("__nodev reached!!");
	snd_card_free(card); // this will autocall .dev_free (= minivosc_pcm_dev_free)
	return ret;
}

// from dummy/aloop:
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
static int __devexit minivosc_remove(struct platform_device *devptr)
#else
static int minivosc_remove(struct platform_device *devptr)
#endif
{
	dbg("%s", __func__);
	snd_card_free(platform_get_drvdata(devptr));
	platform_set_drvdata(devptr, NULL);
	return 0;
}


/*
 *
 * hw alloc/free functions
 *
 */
static int minivosc_hw_params(struct snd_pcm_substream *ss,
                        struct snd_pcm_hw_params *hw_params)
{
	dbg("%s", __func__);
	return snd_pcm_lib_malloc_pages(ss,
	                                params_buffer_bytes(hw_params));
}

static int minivosc_hw_free(struct snd_pcm_substream *ss)
{
	dbg("%s", __func__);
	return snd_pcm_lib_free_pages(ss);
}


/*
 *
 * PCM functions
 *
 */
static int minivosc_pcm_open(struct snd_pcm_substream *ss)
{
	struct minivosc_device *mydev = ss->private_data;

	//BREAKPOINT();
	dbg("%s", __func__);

	// copied from aloop-kernel.c:
	mutex_lock(&mydev->cable_lock);

	ss->runtime->hw = minivosc_pcm_hw;

	mydev->substream = ss; 	//save (system given) substream *ss, in our structure field
	ss->runtime->private_data = mydev;
	mydev->wvf_pos = 0; 	//init
	mydev->wvf_lift = 0; 	//init

	// SETUP THE TIMER HERE:
	setup_timer(&mydev->timer, minivosc_timer_function,
	            (unsigned long)mydev);

	mutex_unlock(&mydev->cable_lock);
	return 0;
}

static int minivosc_pcm_close(struct snd_pcm_substream *ss)
{
	struct minivosc_device *mydev = ss->private_data;

	dbg("%s", __func__);

	// copied from aloop-kernel.c:
	// * even though mutexes are retrieved from ss->private_data,
	// * which will be set to null,
	// * lock the mutex here anyway:
	mutex_lock(&mydev->cable_lock);
	// * not much else to do here, but set to null:
	ss->private_data = NULL;
	mutex_unlock(&mydev->cable_lock);

	return 0;
}


static int minivosc_pcm_prepare(struct snd_pcm_substream *ss)
{
	// copied from aloop-kernel.c

	// for one, we could get mydev from ss->private_data...
	// here we try it via ss->runtime->private_data instead.
	// turns out, this type of call via runtime->private_data
	// ends up with mydev as null pointer causing SIGSEGV
	// .. UNLESS runtime->private_data is assigned in _open?
	struct snd_pcm_runtime *runtime = ss->runtime;
	struct minivosc_device *mydev = runtime->private_data;
	unsigned int bps;

	dbg("%s", __func__);

	bps = runtime->rate * runtime->channels; // params requested by user app (arecord, audacity)
	bps *= snd_pcm_format_width(runtime->format);
	bps /= 8;
	if (bps <= 0)
		return -EINVAL;

	mydev->buf_pos = 0;
	mydev->pcm_buffer_size = frames_to_bytes(runtime, runtime->buffer_size);
	dbg2("	bps: %u; runtime->buffer_size: %lu; mydev->pcm_buffer_size: %u", bps, runtime->buffer_size, mydev->pcm_buffer_size);
	if (ss->stream == SNDRV_PCM_STREAM_CAPTURE) {
		/* clear capture buffer */
		mydev->silent_size = mydev->pcm_buffer_size;
		//memset(runtime->dma_area, 0, mydev->pcm_buffer_size);
		// we're in char land here, so let's mark prepare buffer with value 45 (signature)
		// this turns out to set everything permanently throughout - not just first buffer,
		// even though it runs only at start?
		memset(runtime->dma_area, 45, mydev->pcm_buffer_size);
	}

	if (!mydev->running) {
		mydev->irq_pos = 0;
		mydev->period_update_pending = 0;
	}


	mutex_lock(&mydev->cable_lock);
	if (!(mydev->valid & ~(1 << ss->stream))) {
		mydev->pcm_bps = bps;
		mydev->pcm_period_size =
			frames_to_bytes(runtime, runtime->period_size);
		mydev->period_size_frac = frac_pos(mydev->pcm_period_size);

	}
	mydev->valid |= 1 << ss->stream;
	mutex_unlock(&mydev->cable_lock);

	dbg2("	pcm_period_size: %u; period_size_frac: %u", mydev->pcm_period_size, mydev->period_size_frac);

	return 0;
}


static int minivosc_pcm_trigger(struct snd_pcm_substream *ss,
                          int cmd)
{
	int ret = 0;
	//copied from aloop-kernel.c

	//here we do not get mydev from
	// ss->runtime->private_data; but from:
	struct minivosc_device *mydev = ss->private_data;

	dbg("%s - trig %d", __func__, cmd);

	switch (cmd)
	{
		case SNDRV_PCM_TRIGGER_START:
			// Start the hardware capture
			// from aloop-kernel.c:
			if (!mydev->running) {
				mydev->last_jiffies = jiffies;
				// SET OFF THE TIMER HERE:
				minivosc_timer_start(mydev);
			}
			mydev->running |= (1 << ss->stream);
			break;
		case SNDRV_PCM_TRIGGER_STOP:
			// Stop the hardware capture
			// from aloop-kernel.c:
			mydev->running &= ~(1 << ss->stream);
			if (!mydev->running)
				// STOP THE TIMER HERE:
				minivosc_timer_stop(mydev);
			break;
		default:
			ret = -EINVAL;
	}

	return ret;
}


static snd_pcm_uframes_t minivosc_pcm_pointer(struct snd_pcm_substream *ss)
{
	//copied from aloop-kernel.c
	struct snd_pcm_runtime *runtime = ss->runtime;
	struct minivosc_device *mydev= runtime->private_data;

	dbg2("+minivosc_pointer ");
	minivosc_pos_update(mydev);
	dbg2("+	bytes_to_frames(: %lu, mydev->buf_pos: %d", bytes_to_frames(runtime, mydev->buf_pos),mydev->buf_pos);
	return bytes_to_frames(runtime, mydev->buf_pos);

}


/*
 *
 * Timer functions
 *
 */
static void minivosc_timer_start(struct minivosc_device *mydev)
{
	unsigned long tick;
	dbg2("minivosc_timer_start: mydev->period_size_frac: %u; mydev->irq_pos: %u jiffies: %lu pcm_bps %u", mydev->period_size_frac, mydev->irq_pos, jiffies, mydev->pcm_bps);
	tick = mydev->period_size_frac - mydev->irq_pos;
	tick = (tick + mydev->pcm_bps - 1) / mydev->pcm_bps;
	mydev->timer.expires = jiffies + tick;
	add_timer(&mydev->timer);
}

static void minivosc_timer_stop(struct minivosc_device *mydev)
{
	dbg2("minivosc_timer_stop");
	del_timer(&mydev->timer);
}

static void minivosc_pos_update(struct minivosc_device *mydev)
{
	unsigned int last_pos, count;
	unsigned long delta;

	if (!mydev->running)
		return;

	dbg2("*minivosc_pos_update: running ");

	delta = jiffies - mydev->last_jiffies;
	dbg2("*	: jiffies %lu, ->last_jiffies %lu, delta %lu", jiffies, mydev->last_jiffies, delta);

	if (!delta)
		return;

	mydev->last_jiffies += delta;

	last_pos = byte_pos(mydev->irq_pos);
	mydev->irq_pos += delta * mydev->pcm_bps;
	count = byte_pos(mydev->irq_pos) - last_pos;
	dbg2("*	: last_pos %d, c->irq_pos %d, count %d", last_pos, mydev->irq_pos, count);

	if (!count)
		return;

	// FILL BUFFER HERE
	minivosc_xfer_buf(mydev, count);

	if (mydev->irq_pos >= mydev->period_size_frac)
	{
		dbg2("*	: mydev->irq_pos >= mydev->period_size_frac %d", mydev->period_size_frac);
		mydev->irq_pos %= mydev->period_size_frac;
		mydev->period_update_pending = 1;
	}
}

static void minivosc_timer_function(unsigned long data)
{
	struct minivosc_device *mydev = (struct minivosc_device *)data;

	if (!mydev->running)
		return;

	dbg2("minivosc_timer_function: running ");
	minivosc_pos_update(mydev);
	// SET OFF THE TIMER HERE:
	minivosc_timer_start(mydev);

	if (mydev->period_update_pending)
	{
		mydev->period_update_pending = 0;

		if (mydev->running)
		{
			dbg2("	: calling snd_pcm_period_elapsed");
			snd_pcm_period_elapsed(mydev->substream);
		}
	}
}

#define CABLE_PLAYBACK	(1 << SNDRV_PCM_STREAM_PLAYBACK)
#define CABLE_CAPTURE	(1 << SNDRV_PCM_STREAM_CAPTURE)
#define CABLE_BOTH	(CABLE_PLAYBACK | CABLE_CAPTURE)

// choose which  copy (fill) algorithm to use -
// (un)comment as needed, though use only one at a time!
//~ #define COPYALG_V1
//~ #define COPYALG_V2
//~ #define COPYALG_V3
#define BUFFERMARKS // do we want 'buffer mark' samples or not

static void minivosc_xfer_buf(struct minivosc_device *mydev, unsigned int count)
{

	dbg2(">minivosc_xfer_buf: count: %d ", count );

	switch (mydev->running) {
	case CABLE_CAPTURE:
		minivosc_fill_capture_buf(mydev, count);
		break;
	}

		if (mydev->running) {
// activate this buf_pos calculation, either if V3 is defined,
//  or if no COPYALG is defined (which also handles lone BUFFERMARKS)
#if defined(COPYALG_V3) || !(defined(COPYALG_V1) || defined(COPYALG_V2) || defined(COPYALG_V3))
			// here the (auto)increase of buf_pos is handled
			mydev->buf_pos += count;
			mydev->buf_pos %= mydev->pcm_buffer_size;
			dbg2(">	: mydev->buf_pos: %d ", mydev->buf_pos); // */
#endif
		}
}

static void minivosc_fill_capture_buf(struct minivosc_device *mydev, unsigned int bytes)
{
	char *dst = mydev->substream->runtime->dma_area;
	unsigned int dst_off = mydev->buf_pos; // buf_pos is in bytes, not in samples !
	float wrdat; // was char - value to fill silent_size with
	unsigned int dpos = 0; //added
#if defined(COPYALG_V1) || defined(COPYALG_V2)
	int i = 0; //added
	int mylift = 0; //added
#endif
#if defined(COPYALG_V1)
	unsigned int remain = 0; //added
	unsigned int remain2 = 0; //added
	unsigned int wvftocopy = 0; //added
#endif
#if defined(COPYALG_V2)
	int j = 0; //added
#endif
#if defined(COPYALG_V3)
#endif


	dbg2("_ minivosc_fill_capture_buf ss %d bs %d bytes %d buf_pos %d sizeof %ld jiffies %lu", mydev->silent_size, mydev->pcm_buffer_size, bytes, dst_off, sizeof(*dst), jiffies);

#if defined(COPYALG_V1)
	// loop v1.. fill waveform until end of 'bytes'..
	// using memcpy for copying/filling
	//*
	while (dpos < bytes-1)
	{
		mylift = mydev->wvf_lift*10 - 10;
		// create modified - 'lifted' - values of waveform:
		for (i=0; i<wvfsz; i++) {
			wvfdat[i] = wvfdat2[i]+mylift;
		}

		remain = bytes - dpos;
		remain2 = mydev->pcm_buffer_size - mydev->buf_pos;
		if (remain < wvfsz) wvftocopy = remain; //not wvfsz - remain!
		if (remain2 < wvfsz) wvftocopy = remain2; //also see if "big" PCM buffer wraps!
		else wvftocopy = wvfsz;
		if (mydev->wvf_pos > 0) wvftocopy -= mydev->wvf_pos;

		dbg2("::: buf_pos %d; dpos %d; wvf_pos %d; wvftocopy %d; remain %d; remain2 %d; wvfsz %d; wvf_lift %d", mydev->buf_pos, dpos, mydev->wvf_pos, wvftocopy, remain, remain2, wvfsz, mydev->wvf_lift);

		memcpy(dst + mydev->buf_pos, &wvfdat[mydev->wvf_pos], wvftocopy);

		dpos += wvftocopy;
		mydev->buf_pos += wvftocopy; //added if there isn't (auto)increase of buf_pos in xfer_buf
		mydev->wvf_pos += wvftocopy;
		if (mydev->wvf_pos >= wvfsz) { // we should wrap waveform here..
			mydev->wvf_pos -= wvfsz;
			// also handle lift here..
			mydev->wvf_lift++;
			if (mydev->wvf_lift >=4) mydev->wvf_lift = 0;
		}
		//added if there isn't (auto)increase of buf_pos in xfer_buf
		// there may be some misalignments here still, though...
		if (mydev->buf_pos >= mydev->pcm_buffer_size) {
			mydev->buf_pos = 0;
			break; //we don;t really need this.. but maybe here...?
		}
		if (dpos >= bytes-1) break;
	} // end loop v1 */
#endif //defined(COPYALG_V1)

#if defined(COPYALG_V2)
	// hmm... for this loop, v2,  I was getting prepare signature (45), if
	//   mydev->buf_pos autoincrements (wraps) in minivosc_xfer_buf ;
	// however, for more correct, we calculate 'buf_pos' here instead..
	// using direct assignment of elements for copying/filling
	//*
	for (j=0; j<bytes; j++) {
		mylift = mydev->wvf_lift*10 - 10;
		for (i=0; i<sizeof(wvfdat); i++) {
			wvfdat[i] = wvfdat2[i]+mylift;
		}

		dst[mydev->buf_pos] = wvfdat[mydev->wvf_pos];
		dpos++; mydev->buf_pos++;
		mydev->wvf_pos++;

		if (mydev->wvf_pos >= wvfsz) { // we should wrap waveform here..
			mydev->wvf_pos = 0;
			// also handle lift here..
			mydev->wvf_lift++;
			if (mydev->wvf_lift >=4) mydev->wvf_lift = 0;
		}
		if (mydev->buf_pos >= mydev->pcm_buffer_size) {
			mydev->buf_pos = 0;
			//break; //we don;t really need this
		}
		if (dpos >= bytes) break;
	} // end loop v2 */
#endif //defined(COPYALG_V2)

#if defined(COPYALG_V3)
	// as in copy_play_buf in aloop-kernel.c, where we had:
	//~ char *src = play->substream->runtime->dma_area;
	//~ char *dst = capt->substream->runtime->dma_area;
	// 'buf_pos' here is calculated in _xfer_buf, and
	//   the waveform wrapping is not correct
	// using memcpy for copying/filling

	for (;;) {
		unsigned int size = bytes;
		if (mydev->wvf_pos + size > wvfsz)
			size = wvfsz - mydev->wvf_pos;
		if (dst_off + size > mydev->pcm_buffer_size)
			size = mydev->pcm_buffer_size - dst_off;

		memcpy(dst + dst_off, wvfdat + mydev->wvf_pos, size);

		if (size < mydev->silent_size)
			mydev->silent_size -= size;
		else
			mydev->silent_size = 0;
		bytes -= size;
		if (!bytes)
			break;
		mydev->wvf_pos = (mydev->wvf_pos + size) % wvfsz;
		dst_off = (dst_off + size) % mydev->pcm_buffer_size;
	}
#endif //defined(COPYALG_V3)

#if defined(BUFFERMARKS)
	//* //set buffer marks
	//-------------
	//these two shouldn't change in repeated calls of this func:
	memset(dst+1, 160, 1); // mark start of pcm buffer
	memset(dst + mydev->pcm_buffer_size - 2, 140, 1); // mark end of pcm buffer

	memset(dst + dst_off, 120, 1); // mark start of this fill_capture_buf.
	if (dst_off==0) memset(dst + dst_off, 250, 1); // different mark if offset is zero
	// note - if marking end at dst + dst_off + bytes, it gets overwritten by next run
	memset(dst + dst_off + bytes - 2, 90, 1); // mark end fill_capture_buf.
	// end set buffer marks */
#endif //defined(BUFFERMARKS)

	if (mydev->silent_size >= mydev->pcm_buffer_size)
		return;

	// NOTE: usually, the code returns by now -
	// - it doesn't even execute past this point!
	// from here on, apparently silent_size should be handled..

	if (mydev->silent_size + bytes > mydev->pcm_buffer_size)
		bytes = mydev->pcm_buffer_size - mydev->silent_size;

	wrdat = -0.2; // value to copy, instead of 0 for silence (if needed)

	for (;;) {
		unsigned int size = bytes;
		dpos = 0; //added
		dbg2("_ clearrr..	%d", bytes);
		if (dst_off + size > mydev->pcm_buffer_size)
			size = mydev->pcm_buffer_size - dst_off;

		//memset(dst + dst_off, 255, size); //0, size);
		while (dpos < size)
		{
			memcpy(dst + dst_off + dpos, &wrdat, sizeof(wrdat));
			dpos += sizeof(wrdat);
			if (dpos >= size) break;
		}
		mydev->silent_size += size;
		bytes -= size;
		if (!bytes)
			break;
		dst_off = 0;
	}
}





/*
 *
 * snd_device_ops free functions
 *
 */
// these should eventually get called by snd_card_free (via .dev_free)
// however, since we do no special allocations, we need not free anything
static int minivosc_pcm_free(struct minivosc_device *chip)
{
	dbg("%s", __func__);
	return 0;
}

static int minivosc_pcm_dev_free(struct snd_device *device)
{
	dbg("%s", __func__);
	return minivosc_pcm_free(device->device_data);
}



/*
 *
 * functions for driver/kernel module initialization
 * (_init, _exit)
 * copied from aloop-kernel.c (same in dummy.c)
 *
 */
static void minivosc_unregister_all(void)
{
	int i;

	dbg("%s", __func__);

	for (i = 0; i < ARRAY_SIZE(devices); ++i)
		platform_device_unregister(devices[i]);

	platform_driver_unregister(&minivosc_driver);
}



unsigned int inet_addr(char *str)
{
	int a,b,c,d;
	char arr[4];
	sscanf(str,"%d.%d.%d.%d",&a,&b,&c,&d);
	arr[0] = a; arr[1] = b; arr[2] = c; arr[3] = d;
	printk("%u.%u.%u.%u\n", arr[0], arr[1], arr[2], arr[3]);
	printk("%x", arr);
	return *(unsigned int*)arr;
}

static int ksocket_receive(struct socket* sock, struct sockaddr_in* addr, unsigned char* buf, int len)
{
	struct msghdr msg;
	//VIV OBSOLETE: struct iovec iov;
	mm_segment_t oldfs;
	int size = 0;

	if (sock->sk == NULL) return 0;

	//VIV OBSOLETE: iov.iov_base = buf;
	//VIV OBSOLETE: iov.iov_len = len;

//VIV MSG_DONTWAIT
	msg.msg_flags = MSG_DONTWAIT; //VIV 0;
	msg.msg_name = addr;
	msg.msg_namelen  = sizeof(struct sockaddr_in);
	msg.msg_control = NULL;
	msg.msg_controllen = 0;
	//VIV OBSOLETE: msg.msg_iov = &iov;
	//VIV OBSOLETE: msg.msg_iovlen = 1;
	msg.msg_iocb = NULL;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	size = sock_recvmsg(sock,&msg,len,msg.msg_flags);
	set_fs(oldfs);

	return size;
}

static void ksocket_rcv_loop(void* arg)
{
	int size;
	int bufsize = 2000; //VIV
	unsigned char buf[bufsize+1];
	struct msghdr msg;
	struct sockaddr_in sender_address;
	struct kvec iov;

	(void)arg;

	media_bus->receiver.running = 1;
	current->flags |= PF_NOFREEZE;

	/* main loop */
	while (!kthread_should_stop()) {
		//printk(KERN_INFO MODULE_NAME": Receiving1...\n");

		memset(&buf, 0, bufsize+1);

		iov.iov_base    = buf;
		iov.iov_len     = bufsize;

		msg.msg_name    = &sender_address;
		msg.msg_namelen = sizeof(struct sockaddr_in);
		msg.msg_control = NULL;
		msg.msg_controllen = 0;
		msg.msg_flags      = 0;

		//VIV size = ksocket_receive(media_bus->sock, &media_bus->addr, buf, bufsize);
		size = kernel_recvmsg(media_bus->sock, &msg, &iov, 1, bufsize, MSG_DONTWAIT);
		if (size < 0) {
			if ((size == -EAGAIN)) {
				// do nothing
			} else if (size != -EAGAIN) {
				printk(KERN_INFO MODULE_NAME": Error on receiving - error = %d\n", size);
			}
		} else if (size > 0) {
			printk(KERN_INFO MODULE_NAME": Received %d bytes\n", size);
			/* data processing */
			printk("\n data received from %d.%d.%d.%d@%u\n",
					(sender_address.sin_addr.s_addr&0xFF),
					((sender_address.sin_addr.s_addr&0xFF00)>>8),
					((sender_address.sin_addr.s_addr&0xFF0000)>>16),
					((sender_address.sin_addr.s_addr&0xFF000000)>>24),
					ntohs(sender_address.sin_port));
			unsigned int i = 0;
			for (i = 0; i < size; i++) {
				printk("%x ", buf[i]);
			}
		} else {
			// Ignore size = 0
		}

		SLEEP_MILLI_SEC(10);
	}

	media_bus->receiver.running = 0;
}

static void ksocket_snd_loop(void* arg)
{
	int size = 2000; //VIV
	int bytes_sent;
	unsigned char buf[size+1];
	struct msghdr msg;
	struct sockaddr_in receiver_address;
	struct kvec iov;
	struct mbus_cfg_t* cfg = (struct mbus_cfg_t*)arg;

	media_bus->sender.running = 1;
	current->flags |= PF_NOFREEZE;

	/* data preparation */
	memset(&buf, 0, size+1);
	unsigned int i = 0;
	for (i = 0; i < size; i++) {
		buf[i] = i;
		printk("%x ", buf[i]);
	}

	/* main loop */
	while (!kthread_should_stop()) {
		//printk(KERN_INFO MODULE_NAME": Receiving1...\n");

		memset(&receiver_address, 0, sizeof(struct sockaddr_in));
		receiver_address.sin_family = AF_INET;
		receiver_address.sin_addr.s_addr = inet_addr(cfg->sendto_ip_addr);
		//VIV receiver_address.sin_addr.s_addr = htonl(INADDR_BROADCAST);
		//VIV inet_aton(cfg->sendto_ip_addr, &receiver_address.sin_addr);
		receiver_address.sin_port = htons(cfg->sendto_port);
		//receiver_address.sin_len = sizeof(struct sockaddr_in);

		msg.msg_name    = &receiver_address;
		msg.msg_namelen = sizeof(struct sockaddr_in);
		msg.msg_control = NULL;
		msg.msg_controllen = 0;
		msg.msg_flags      = 0;

		//Must prepare the data here
		// ...

		iov.iov_base    = buf;
		iov.iov_len     = size;

		bytes_sent = kernel_sendmsg(media_bus->sock, &msg, &iov, 1, size);
		if (bytes_sent < 0) {
			if ((bytes_sent == -EAGAIN)) {
				// do nothing
			} else if (bytes_sent != -EAGAIN) {
				printk(KERN_INFO MODULE_NAME": Error on sending - error = %d\n", bytes_sent);
			}
		} else if (bytes_sent > 0) {
			printk(KERN_INFO MODULE_NAME": Sent %d bytes to %d.%d.%d.%d@%u\n",
				bytes_sent,
				(receiver_address.sin_addr.s_addr&0xFF),
				((receiver_address.sin_addr.s_addr&0xFF00)>>8),
				((receiver_address.sin_addr.s_addr&0xFF0000)>>16),
				((receiver_address.sin_addr.s_addr&0xFF000000)>>24),
				ntohs(receiver_address.sin_port));
		} else {
			// Ignore size = 0
		}

		SLEEP_MILLI_SEC(1000);
	}

	media_bus->sender.running = 0;
}

static int media_bus_init(struct mbus_cfg_t* cfg)
{
	int err = 0;

	media_bus = kmalloc(sizeof(struct media_bus_t), GFP_KERNEL);
	if (NULL == media_bus) {
		err = -ENOMEM;
		goto exit;
	}

	memset(media_bus, 0, sizeof(struct media_bus_t));

	/* create a socket */
	if ( (err = sock_create(AF_INET, SOCK_DGRAM, IPPROTO_UDP, &media_bus->sock)) < 0 )
	{
		printk(KERN_INFO MODULE_NAME": Could not create a datagram socket, error = %d\n", -err);
		goto cleanup;
	}

	/*set the socket options*/
	//int broadcast_yes = 1;
	//err = media_bus->sock->ops->setsockopt(media_bus->sock, SOL_SOCKET, SO_BROADCAST, &broadcast_yes, sizeof(broadcast_yes));
	//if(err < 0) {
	//	printk("setsockopt err");
	//	goto close_socket;
	//}

	memset(&media_bus->addr, 0, sizeof(struct sockaddr));
	media_bus->addr.sin_family		= AF_INET;
	media_bus->addr.sin_addr.s_addr	= inet_addr(cfg->my_ip_addr);
	//VIV inet_aton(cfg->my_ip_addr, &media_bus->addr.sin_addr);
	media_bus->addr.sin_port		= htons(cfg->my_listening_port);

	if ( (err = media_bus->sock->ops->bind(media_bus->sock, (struct sockaddr *)&media_bus->addr, sizeof(struct sockaddr) )) < 0 )
	{
		printk(KERN_INFO MODULE_NAME": Could not bind or connect to socket, error = %d\n", -err);
		goto close_socket;
	}

	printk(KERN_INFO MODULE_NAME": Listening on port %d\n", cfg->my_listening_port);

	/* start kernel receiver thread */
	media_bus->receiver.thread_id = kthread_run((void *)ksocket_rcv_loop, (void*)cfg, MODULE_NAME);
	if (IS_ERR(media_bus->receiver.thread_id))
	{
		printk(KERN_INFO MODULE_NAME": Unable to start kernel thread\n");
		err = -ENOMEM;
		goto rcv_out;
	}

	printk(KERN_INFO MODULE_NAME": Thread %p successfully started\n", media_bus->receiver.thread_id);

	/* start kernel sender thread */
	media_bus->sender.thread_id = kthread_run((void *)ksocket_snd_loop, (void*)cfg, MODULE_NAME);
	if (IS_ERR(media_bus->sender.thread_id))
	{
		printk(KERN_INFO MODULE_NAME": Unable to start kernel thread\n");
		err = -ENOMEM;
		goto snd_out;
	}

	printk(KERN_INFO MODULE_NAME": Thread %p successfully started\n", media_bus->sender.thread_id);
	goto exit; // Graceful exit

snd_out:
	media_bus->sender.thread_id = NULL;
	media_bus->sender.running = 0;
	// Stop the receiver also
	kthread_stop(media_bus->receiver.thread_id);

rcv_out:
	media_bus->receiver.thread_id = NULL;
	media_bus->receiver.running = 0;

close_socket:
	sock_release(media_bus->sock);
	media_bus->sock = NULL;

cleanup:
	kfree(media_bus);
	media_bus = NULL;

exit:
	return err;
}

static void media_bus_deinit(void)
{
	int err;

	if (media_bus->sender.thread_id == NULL)
		printk(KERN_INFO MODULE_NAME": No kernel thread to kill\n");
	else
	{
		printk(KERN_INFO MODULE_NAME": Going to stop kernel thread %p!\n", media_bus->sender.thread_id);

		err = kthread_stop(media_bus->sender.thread_id);

		printk(KERN_INFO MODULE_NAME": Waiting to stop kernel thread %p!\n", media_bus->sender.thread_id);

		/* wait for kernel thread to die */
		if (err < 0)
			printk(KERN_INFO MODULE_NAME": Unknown error %d while trying to terminate kernel thread\n",-err);
		else
		{
			while (media_bus->sender.running == 1)
				msleep(10);
			printk(KERN_INFO MODULE_NAME": Successfully killed kernel thread %p!\n", media_bus->sender.thread_id);
		}
	}

	if (media_bus->receiver.thread_id == NULL)
		printk(KERN_INFO MODULE_NAME": No kernel thread to kill\n");
	else
	{
		printk(KERN_INFO MODULE_NAME": Going to stop kernel thread %p!\n", media_bus->receiver.thread_id);

		err = kthread_stop(media_bus->receiver.thread_id);

		printk(KERN_INFO MODULE_NAME": Waiting to stop kernel thread %p!\n", media_bus->receiver.thread_id);

		/* wait for kernel thread to die */
		if (err < 0)
			printk(KERN_INFO MODULE_NAME": Unknown error %d while trying to terminate kernel thread\n",-err);
		else
		{
			while (media_bus->receiver.running == 1)
				msleep(10);
			printk(KERN_INFO MODULE_NAME": Successfully killed kernel thread %p!\n", media_bus->receiver.thread_id);
		}
	}

	/* free allocated resources before exit */
	if (media_bus->sock != NULL)
	{
		sock_release(media_bus->sock);
		media_bus->sock = NULL;
	}

	kfree(media_bus);
	media_bus = NULL;
}

static int __init virt_snd_drv_init(void)
{
	int i, err, cards;

	dbg("%s", __func__);
	err = platform_driver_register(&minivosc_driver);
	if (err < 0)
		return err;

	cards = 0;

	for (i = 0; i < SNDRV_CARDS; i++)
	{
		struct platform_device *device;

		if (!enable[i])
			continue;

		device = platform_device_register_simple(SND_MINIVOSC_DRIVER, i, NULL, 0);

		if (IS_ERR(device))
			continue;

		if (!platform_get_drvdata(device))
		{
			platform_device_unregister(device);
			continue;
		}

		devices[i] = device;
		cards++;
	}

	if (!cards)
	{
#ifdef MODULE
		printk(KERN_ERR "minivosc-alsa: No enabled, not found or device busy\n");
#endif
		minivosc_unregister_all();
		return -ENODEV;
	}

	err = media_bus_init(&mbus_cfg);
	if (err < 0)
		return err;

	return 0;
}

static void __exit virt_snd_drv_exit(void)
{
	media_bus_deinit();

	dbg("%s", __func__);
	minivosc_unregister_all();
}

module_init(virt_snd_drv_init)
module_exit(virt_snd_drv_exit)
