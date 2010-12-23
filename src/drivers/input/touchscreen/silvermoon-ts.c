// rmmod silvermoon-ts.ko; rm silvermoon-ts.ko ; wget http://192.168.0.200/silvermoon-ts.ko; insmod silvermoon-ts.ko
/* linux/drivers/input/touchscreen/silvermoon-ts.c
 *
 * $Id$
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (c) 2004 Arnaud Patard <arnaud.patard@rtp-net.org>
 * iPAQ H1940 touchscreen support
 *
 * ChangeLog
 *
 * 2004-09-05: Herbert Potzl <herbert@13thfloor.at>
 *	- added clock (de-)allocation code
 *
 * 2005-03-06: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - h1940_ -> s3c24xx (this driver is now also used on the n30
 *        machines :P)
 *      - Debug messages are now enabled with the config option
 *        TOUCHSCREEN_S3C_DEBUG
 *      - Changed the way the value are read
 *      - Input subsystem should now work
 *      - Use ioremap and readl/writel
 *
 * 2005-03-23: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - Make use of some undocumented features of the touchscreen
 *        controller
 *
 * 2006-09-05: Ryu Euiyoul <ryu.real@gmail.com>
 *      - added power management suspend and resume code
 *
 * 2008-10-10: Greg Hutchins <ghutchins@gmail.com>
 *      - left the change log here, however basically started with the
 *        s3c_tsb.c touchscreen driver and completely rewrote it for
 *        Chumby Industries needs
 *
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/proc_fs.h>
#include <linux/irq.h>

#include <asm/io.h>
#include <asm/irq.h>
//#include <asm/hardware.h>

//#include <asm/arch/regs-s3c6400-clock.h>
//#include <asm/arch/regs-adc.h>
//#include <asm/arch/regs-spi.h>
//#include <asm/arch/regs-gpio.h>
//#include <asm/arch/irqs.h>
// hgroover mods for 2.6.25 kernel
//#include <asm/arch/ttc-regs.h>
//#include <asm/arch/pxa-regs.h>
//#include <asm-arm/arch-pxa/regs-ssp.h>
#include <mach/irqs.h>
// Previously, we used 49, which is now used by GPIOX (GPIO multiplexed IRQ)
// This magic number (which is 52) is configured explicitly in silvermoon-ts.c
//#define IRQ_GPIO_AP	IRQ_PXA168CS_TOUCHSCREEN

// Get gpio API
//#include <asm-arm/arch-pxa/hardware.h>
//#include <asm-arm/arch-pxa/pxa-regs.h>
//#include <asm-arm/arch-pxa/gpio.h>
#include <linux/gpio.h>

// hgroover - copied from 2.6.24.7 mach-ttc
#if !defined(IO_ADDRESS)

//#include <asm-arm/arch-pxa/platform_pxa168.h>

//#ifdef CONFIG_MMU
/* macro to get at IO space when running virtually */
//#define IO_ADDRESS(x) ((x) | VIRT_IO_BASE)
//#else
#define IO_ADDRESS(x) (x)
//#endif

#endif

//#include <asm/arch/timex.h>
#include <mach/timex.h>


//#include <asm/arch/regs-ssp.h>
//#include <asm/arch/ssp.h>
#include <plat/regs-ssp.h>
#include <plat/ssp.h>

#define TS_DRIVER_VER "1.06-$Rev$"

static struct ssp_dev sspdev;


#define MY_MFP_52 0x03C
#define MY_MFP_50 0x034

#define MY_GPIO52_OFFSET  (1 << (52 - 32))  // bit mask offset for GPIO52
#define MY_GPIO52_BASE    0x4        // extra offset added to basic base to select the right bank register
#define MY_GPIO50_OFFSET  (1 << (50 - 32))
#define MY_GPIO50_BASE    0x4

#define MY_GPIO_SDR  0x0054  // bitwise set of direction register -- makes it an output
#define MY_GPIO_CDR  0x0060  // bitwise clr of direction register -- makes it an input
#define MY_GPIO_SRER 0x006C  // bitwise set of GPIO rising edge detect enable
#define MY_GPIO_CRER 0x0078  // bitwise clr of GPIO rising edge detect enable
#define MY_GPIO_SFER 0x0084  // falling set
#define MY_GPIO_CFER 0x0090  // falling clr

#define MY_GPIO_PSR  0x0018  // bitwise GPIO setting
#define MY_GPIO_PCR  0x0024  // bitwise GPIO clearing

#define MY_GPIO_PLR  0x0000  // pin level readback
#define MY_GPIO_PDR  0x000C  // pin direction register

#define MY_GPIO_RER  0x0030  // rising edge det enable register
#define MY_GPIO_FER  0x003C  // rising edge det enable register

#define MY_GPIO_APMASK 0x009C // SHEEVA masking
#define MY_GPIO_CPMASK 0x00A8 // mask of gpio edge detect

#define MY_GPIO_EDR    0x0048 // edge detect status register

#define FAST_MODE 1
#define WORKAROUND_SSP_DIVIDER 1

// Define this to use GPIO api whenever possible
#define USE_GPIO_API

static char *silvermoon_ts_name = "Silvermoon TouchScreen";

static struct my_gpio {
  void __iomem  *mmio_base;
  unsigned long phys_base;
  int irq;
  unsigned long size;
} my_gpio;

static struct my_gpioedge {
  void __iomem  *mmio_base;
  unsigned long phys_base;
  unsigned long size;
} my_gpioedge;

static struct my_mfp {
  void __iomem  *mmio_base;
  unsigned long phys_base;
  unsigned long size;
} my_mfp;


#define DBG(fmt, args...) do {} while (0)

// Define to use polling rather than IRQ
//#define CONFIG_SILVERMOON_TS_POLLING
// Uncomment to get verbose spew on timer
//#define SILVERMOON_TS_POLLING_DBGSPEW

// version # of the touchscreen driver
//
#define SILVERMOON_TSVERSION 0x0101

// have a problem with the SPI interface where X or Y coordinate is repeated
// resulting in an incorrect (X,Y) reading -- this #define checks for that
// and simply throws the sample away, leaving an X==Y dead spot on the screen
//
#define SPI_COMM_BUG
//#undef SPI_COMM_BUG

// Timer intervals, slow timer goes off every 100ms and checks to make sure
// a rising edge touchscreen interrupt was not lost.
//
#define SLOW_TIMER_INTERVAL  msecs_to_jiffies(2) // was 100
#define TOUCH_TIMER_INTERVAL msecs_to_jiffies(2)  // stylus down interval
#define TOUCH_TIMER_ACTIVATE msecs_to_jiffies(2)   // activate after interrupt

// # of bytes in RX FIFO before RX_FIFO_RDY interrupt -- This corresponds to
// dont-care, X-coordinate, Y-coordinate (2 bytes each).
//
#define RX_FIFO_RDY_BYTES 6    // dont-care, X, Y
//#define RX_FIFO_RDY_BYTES 8  // dont-care, X, dont-care, Y

// We collect SAMPLES samples every 10ms, every 50ms we report the position of
// the stylus based on an average of the samples that are the most similar.
// SAMPLES*3 bytes will be allocated on the stack for variance and averaging
// purposes.
//
#define SAMPLES	15
#define CHECKSAMPLES 0   // some cheesy code by bunnie to try and limit noise, delete if not used

// Number of 10ms timer periods to elapse before we smooth points
#define TIMER_PERIOD_COUNT	2

// OUTLIER_DELTA is the maximum allowed difference between the previous
// coordinate sample and this coordinate sample.  If the difference is greater
// than OUTLIER_DELTA the sampling is terminated -- this is to catch the problem
// where the coordinate sample is starting to degrade because the touch pressure
// is diminishing.
//
#define OUTLIER_DELTA 200


#define MAXX        0x1000    // max X value -- *MUST* be a power of two
#define MAXY        0x1000    // max Y value -- *MUST* be a power of two

#define REQUEST_X   0x8100ul  // bunnie's interface, get X coordinate
#define REQUEST_Y   0x8200ul  // bunnie's interface, get Y coordinate


// henry@chumby.com - looks like the values are already in a good range
// for calibration, just need to be swapped...
//#define XY_TRANSFORM_DEBUG
#define TS_X_TRANSFORM(v)   (((MAXX - v->yp) / 5) + 130)
#define	TS_Y_TRANSFORM(v)   (((MAXY - v->xp) / 5) + 130)
//#define TS_X_TRANSFORM(v)   (v->yp)
//#define	TS_Y_TRANSFORM(v)   (v->xp)


// Per-touchscreen data.
//
struct silvermoon_ts {
    struct input_dev *dev;
    long xp;
    long yp;
    char phys[32];

    int timer_count;
    long xp_timer[5];
    long yp_timer[5];

    int sample_count;
    long xp_sample[SAMPLES];
    long yp_sample[SAMPLES];

  long xhist;
  long yhist;
  int lastStylus;

    long dummy;
};

int stylus_cleanup = 0;
int stylus_down_state = 0;
#define NOISE_THRESH 100

static void timer_fire(unsigned long data);
static void slow_timer_fire(unsigned long data);

static struct silvermoon_ts  *ts = NULL;
static void __iomem     *spi_regs = NULL;
static struct clk       *spi_clock = NULL;
static int              timer_enabled = 0;
static struct timer_list touch_timer = TIMER_INITIALIZER(timer_fire, 1, 0);
static struct timer_list slow_timer = TIMER_INITIALIZER(slow_timer_fire, 1, 0);

static int iter_request = 0;

static inline int stylus_down(void)
{
    // read the touchscreen interrupt line, high denotes stylus down
  // TS-touched is GPIO52
    return (__raw_readl(my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_PLR) & MY_GPIO52_OFFSET) != 0;
}


static inline void spi_stop(void)
{
    // simply disable SPI interrupt
  //    writel(0x00, spi_regs + S3C_SPI_INT_EN);
  ssp_disable(&sspdev);
}


static inline void spi_request_xy(void)
{
    // take advantage of the SPI FIFO and blast out everything we want to do
    // in a sort of batch job -- works as long as the data does not exceed
    // the size of the fifo (64 bytes)
    //
    // note, we issue two Y requests because of the way SPI works, the data
    // returned during the same cycle as the X request is trash, the data
    // returned by the next two cycles are the X and Y coordinates we are
    // interested in
    //
  //    writel(0xff, spi_regs + S3C_PENDING_CLR);
  //    writel(SPI_INT_RX_FIFORDY_EN, spi_regs + S3C_SPI_INT_EN);
  ssp_enable(&sspdev);
#if RX_FIFO_RDY_BYTES == 6
#if !WORKAROUND_SSP_DIVIDER
  if( sample_count % 3 != 2 ) {
    ssp_write_word(&sspdev, REQUEST_X); // just fill with x requests to reset the stream
    ssp_write_word(&sspdev, REQUEST_X);
    ssp_write_word(&sspdev, REQUEST_X);
  } else {
  ssp_write_word(&sspdev, REQUEST_X);
  ssp_write_word(&sspdev, REQUEST_Y);
  if( ssp_write_word(&sspdev, REQUEST_Y) != 0 ) {
    printk( "timed out!\n");
    printk( "ISCCR0: %08x\n", readl(IO_ADDRESS(0xD4050040)) );
    printk( "ISCCR1: %08x\n", readl(IO_ADDRESS(0xD4050044)) );
    // 5820130B  0101 1000 0010 0000 0001 0011 0000 1011
    //
    printk( "APCR: %08x\n", readl(IO_ADDRESS(0xD4051000)) );
    printk( "ACGR: %08x\n", readl(IO_ADDRESS(0xD4051024)) );
    printk( "ASSPDR: %08x\n", readl(IO_ADDRESS(0xD4051054)) );

    //    writel(0x03,IO_ADDRESS(0xD4015020));
    //    writel(0x03,IO_ADDRESS(0xD4015820));
    //    writel(0x03,IO_ADDRESS(0xD401581C)); /* SSP1 clock enable*/

// This should be handled in arch/arm/mach-mmp/silvermoon.c
#if 0
    // 5820130B  1011 1000 0010 0000 0001 0011 0000 1011
    writel(0x63,IO_ADDRESS(0xD4015020));
    writel(0x63,IO_ADDRESS(0xD4015820));
    writel(0x63,IO_ADDRESS(0xD401581C)); /* SSP1 clock enable*/
#endif

    writel(0xB820130B , IO_ADDRESS(0xD4050040));
    writel(0xB820130B , IO_ADDRESS(0xD4050044));

// See previous comment
#if 0
    writel(0x63,IO_ADDRESS(0xD4015020));
    writel(0x63,IO_ADDRESS(0xD4015820));
    writel(0x63,IO_ADDRESS(0xD401581C)); /* SSP1 clock enable*/
#endif

  }
  }
#else

#if 0
  if( iter_request == 0 ) {
    printk( "re-init'ing clocks for workaround.\n" );
    writel(0x63,IO_ADDRESS(0xD4015020));
    writel(0x63,IO_ADDRESS(0xD4015820));
    writel(0x63,IO_ADDRESS(0xD401581C));
  }
#endif

  switch( iter_request % 3 ) {
  case 0:
	#ifdef SILVERMOON_TS_POLLING_DBGSPEW
	printk( "%s() iter_request=0\n", __FUNCTION__ );
	#endif
    //    printk( "." );
    ssp_write_word(&sspdev, REQUEST_X);
    break;
  case 1:
    //    printk( "+" );
    ssp_write_word(&sspdev, REQUEST_Y);
    break;
  case 2:
    //    printk( "*" );
    ssp_write_word(&sspdev, REQUEST_Y);
    ts->sample_count++;
    break;
  }
  iter_request++;
#endif
#elif RX_FIFO_RDY_BYTES == 8
  ssp_write_word(&sspdev, REQUEST_X);
  ssp_write_word(&sspdev, REQUEST_X);
  ssp_write_word(&sspdev, REQUEST_Y);
  ssp_write_word(&sspdev, REQUEST_Y);
#else
#error "RX_FIFO_RDY_BYTES invalid, must be 6 or 8!"
#endif
}


static inline void spi_read_xy(unsigned long status)
{
  u32 X, x;
  u32 Y, y;
  int count;
  int another_sample;
//  int checkSample = 0;
//  int xabs, yabs;

  if (status & (1 << 6)) {  // weird -- this is for "exceeding" the level, not "at level"...could be problematic vs. s3c6410
#if RX_FIFO_RDY_BYTES == 6
      ssp_read_word(&sspdev, &X);
      ssp_read_word(&sspdev, &x);
      ssp_read_word(&sspdev, &y);
      Y = 0xdead;
#else
      ssp_read_word(&sspdev, &X);
      ssp_read_word(&sspdev, &x);
      ssp_read_word(&sspdev, &Y);
      ssp_read_word(&sspdev, &y);
#endif
	x &= 0xFFFF;
	X &= 0xFFFF;
	y &= 0xFFFF;
	Y &= 0xFFFF;
#if FAST_MODE
	//	printk( "x:%04d X:%04d y:%04d Y:%04d\n", x, X, y, Y );
	/////////////////
	//	if( x == y ) {  // I don't see this problem anymore?
	  //	    printk(".DOWN(%d,%d) [(%d,%d)]\n",
	  //	    TS_X_TRANSFORM(ts), TS_Y_TRANSFORM(ts), ts->xp, ts->yp);
	//	  return;
	//	}
	ts->xp = x;
	ts->yp = y;
	if( ts->sample_count > 2 ) {
#if defined(XY_TRANSFORM_DEBUG)
	  	    printk("+DOWN(%d,%d) [(%d,%d)]\n",
				TS_X_TRANSFORM(ts), TS_Y_TRANSFORM(ts), ts->xp, ts->yp);
#endif
	  if( (ts->sample_count % 3 == 0) ) {
#if CHECKSAMPLES
	    checkSample = 0;
	    if( ts->lastStylus == 0 && stylus_down_state == 1 ) {
	      ts->xhist = x; // first time into state
	      ts->yhist = y;
	      ts->lastStylus = 1;
	      checkSample = 0;
	    } else if( ts->lastStylus == 1 && stylus_down_state == 1 ) {
	      ts->lastStylus = 1;
	      checkSample = 1;
	    } else {
	      ts->lastStylus = 0;
	      checkSample = 0;
	    }
	    if( checkSample ) {
	      xabs = ts->xhist - x;
	      if( xabs < 0 )
		xabs = -xabs;
	      yabs = ts->yhist - y;
	      if( yabs < 0 )
		yabs = -yabs;

	      if( xabs > NOISE_THRESH || yabs > NOISE_THRESH )
		return;

	      // important to keep this below
	      // so that we don't initialize history with a discarded sample...
	      ts->xhist = x;
	      ts->yhist = y;

	    }
#endif
	    //	    printk( "        ************ X: %4d Y: %4d *************\n", ts->xp, ts->yp );
	    input_report_abs(ts->dev, ABS_X, TS_X_TRANSFORM(ts));
	    input_report_abs(ts->dev, ABS_Y, TS_Y_TRANSFORM(ts));
	    input_report_key(ts->dev, BTN_TOUCH, 1);
	    input_report_abs(ts->dev, ABS_PRESSURE, 1);
	    input_sync(ts->dev);
	  }
	}
	return;
    ////////////
#endif

        count = ts->sample_count;
	//	printk( "count: %d\n", count );
        another_sample = count < SAMPLES;

        #ifdef SPI_COMM_BUG
        // note, don't accept x==y, this is a bug in the SPI communications
        // (so we have an X==Y dead spot), not sure that is a big deal...
        //
        if (x == y)
            DBG("x==y (%d) PITCHING!\n", x);
        #endif

        #ifdef SPI_COMM_BUG
        if (another_sample && x != y) {
        #else
        if (another_sample) {
        #endif
            ts->xp_sample[count] = (unsigned short) x;
            ts->yp_sample[count] = (unsigned short) y;
            ts->dummy = (unsigned short) X + Y; // shut the compiler up!

            // If beyond first sample, look for outliers.  Note, first sample is
            // sometimes quite a bit different than the second and subsequent
            // samples, don't use it when looking for outliers.  First sample
            // will be pitched when taking the average of the 60% samples with
            // the least variance.  I know, does it really need to be this
            // complicated?  Afterall, it is just a touchscreen...
            //
            if (count > 1) {
//                long delta_x = abs(ts->xp_sample[count-1] - ts->xp_sample[count]);
//                long delta_y = abs(ts->yp_sample[count-1] - ts->yp_sample[count]);
		//		int outlier = delta_x >= OUTLIER_DELTA || delta_y >= OUTLIER_DELTA;
		int outlier = 0; /// STANDIN CODE WHILE DEBUGGING OTHER SHIT

                #ifdef SPI_COMM_BUG
                // if this is an outlier and X==Y, possibly a misread
                // on the SPI bus, just throw this sample out!
                if (outlier && x == y) {
                    // this code will actually *NEVER* execute because of the
                    // "if" above, left in for future reference...
                    outlier = 0;
                    count -= 1;
                }
                #endif

                if (outlier)
                    printk("count=%d, x/y=%d/%d OUTLIER!!!\n", count, x, y);
                another_sample = !outlier;
            }

            if (another_sample) {
                ts->sample_count = count + 1;
		//		printk( "sc: %d\n", ts->sample_count);
                another_sample = ts->sample_count < SAMPLES;
                #if 0
                DBG("count=%d, x/y=%d/%d\n", count, x, y);
                #endif
            }
        }

        if (another_sample) {
	  //	  printk( "a" );
            spi_request_xy();
	}
        else
            spi_stop();
    }
}

static irqreturn_t spi0_isr(int irq, void *dev_id)
  {
    struct ssp_dev *dev = (struct ssp_dev*) dev_id;
    struct ssp_device *ssp = dev->ssp;
    unsigned int status;
    int rx_fifo_rdy_bytes;

    status = __raw_readl(ssp->mmio_base + SSSR);
    __raw_writel(status, ssp->mmio_base + SSSR);

    rx_fifo_rdy_bytes = (status>>12) & 0xf;

#ifdef SILVERMOON_TS_POLLING_DBGSPEW
	printk( "%s(%d) stat=0x%x rdy=%d mmio=%08lx\n",
		__FUNCTION__, irq, status, rx_fifo_rdy_bytes, (unsigned long)(ssp->mmio_base+SSSR) );
#endif

    if (status & SSSR_ROR)
      printk(KERN_WARNING "SSP(%d): receiver overrun\n", dev->port);

    if (status & SSSR_TUR)
      printk(KERN_WARNING "SSP(%d): transmitter underrun\n", dev->port);

    if (status & SSSR_BCE)
      printk(KERN_WARNING "SSP(%d): bit count error\n", dev->port);

    if (rx_fifo_rdy_bytes != RX_FIFO_RDY_BYTES) {
      //        printk(KERN_ERR "ts_spi_isr: RX_FIFO_RDY_BYTES %d (should be %d)\n",
      //               rx_fifo_rdy_bytes, RX_FIFO_RDY_BYTES);
        printk("ERROR! RX_FIFO_RDY_BYTES %d (should be %d)\n",
            rx_fifo_rdy_bytes, RX_FIFO_RDY_BYTES);
        spi_read_xy(status);
        return IRQ_HANDLED;
    }

    spi_read_xy(status);

    return IRQ_HANDLED;
  }

#if 0
static irqreturn_t spi0_isr(int irqno, void *param)
{
    unsigned long status = readl(spi_regs + S3C_SPI_STATUS);
    int rx_fifo_rdy_bytes = (status>>13) & 0x3f;

    if (status & (SPI_STUS_RX_OVERRUN_ERR|
                  SPI_STUS_TX_OVERRUN_ERR|
                  SPI_STUS_RX_UNDERRUN_ERR|
                  SPI_STUS_TX_UNDERRUN_ERR)) {
        DBG("ERROR! status=%x\n", status);
        if (status & SPI_STUS_RX_OVERRUN_ERR)
            printk(KERN_ERR "ts_spi_isr: RX overrun error detected\n");
        if (status & SPI_STUS_RX_UNDERRUN_ERR)
            printk(KERN_ERR "ts_spi_isr: RX underrun error detected\n");
        if (status & SPI_STUS_TX_OVERRUN_ERR)
            printk(KERN_ERR "ts_spi_isr: TX overrun error detected\n");
        if (status & SPI_STUS_TX_UNDERRUN_ERR)
            printk(KERN_ERR "ts_spi_isr: TX underrun error detected\n");
    }

    if (rx_fifo_rdy_bytes != RX_FIFO_RDY_BYTES) {
        printk(KERN_ERR "ts_spi_isr: RX_FIFO_RDY_BYTES %d (should be %d)\n",
               rx_fifo_rdy_bytes, RX_FIFO_RDY_BYTES);
        DBG("ERROR! RX_FIFO_RDY_BYTES %d (should be %d)\n",
            rx_fifo_rdy_bytes, RX_FIFO_RDY_BYTES);
        spi_request_xy();
        return IRQ_HANDLED;
    }

    spi_read_xy(status);
    return IRQ_HANDLED;
}
#endif

static void smooth_coords( int count,
                           long* xp_data, long *yp_data,
                           long* xp_avg, long *yp_avg )
{
    int  i, j, swapped, samples;
    short delta[SAMPLES];
   	unsigned char index[SAMPLES];
    long xp, yp;

    // build the variance table
    for (i=0; i<count; ++i) {
        index[i] = i;
        delta[i] = 0;
        for (j=0; j<count; ++j) {
            delta[i] += abs(xp_data[j] - xp_data[i]);
            delta[i] += abs(yp_data[j] - yp_data[i]);
        }
    }

    // bubble sort the variance table indices
    do {
        swapped = SAMPLES;
        for (i=0; i<count-1; ++i) {
            if (delta[index[i]] > delta[index[i+1]]) {
                swapped = index[i];
                index[i] = index[i+1];
                index[i+1] = swapped;
            }
        }
    } while (swapped < SAMPLES);

#if TIMER_PERIOD_COUNT < 5
	samples = count;
#else
    // average first 60% of samples -- throw out worst 40%
    samples = (count*6)/10;
#endif
    xp = 0;
    yp = 0;
    for (i=0; i<samples; ++i) {
        xp += xp_data[index[i]];
        yp += yp_data[index[i]];
    }
    *xp_avg = (xp + (samples/2)) / samples;
    *yp_avg = (yp + (samples/2)) / samples;

    #if 0
    // debugging code
    if (count>3) {
        int pitched = count - samples;

        if ( pitched >= 3 )
            printk("PITCHED %d samples [%d(%d,%d), %d(%d,%d), %d(%d,%d)] [(%d,%d)]\n",
                pitched,
                index[count-3], yp_data[index[count-3]], xp_data[index[count-3]],
                index[count-2], yp_data[index[count-2]], xp_data[index[count-2]],
                index[count-1], yp_data[index[count-1]], xp_data[index[count-1]],
                *yp_avg, *xp_avg);
        else if ( pitched >= 2 )
            printk("PITCHED %d samples [%d(%d,%d), %d(%d,%d)] [(%d,%d)]\n",
                pitched,
                index[count-2], yp_data[index[count-2]], xp_data[index[count-2]],
                index[count-1], yp_data[index[count-1]], xp_data[index[count-1]],
                *yp_avg, *xp_avg);
        else if ( pitched >= 1 )
            printk("PITCHED 1 sample [ %d(%d,%d)] [(%d,%d)]\n",
                index[count-1], yp_data[index[count-1]], xp_data[index[count-1]],
                *yp_avg, *xp_avg);
    }
    #endif
}


static void timer_fire(unsigned long data)
{
    DBG("timer_count=%d\n", ts->timer_count);

    if (!stylus_down()) {
      stylus_down_state = 0;
      stylus_cleanup = 1; // we might be dirty
#if WORKAROUND_SSP_DIVIDER
	if( iter_request % 3 != 1 ) {
	  //	  printk( "cleaning up...\n" );
	  spi_request_xy();
	  timer_enabled = 1;
	  mod_timer(&touch_timer, jiffies + TOUCH_TIMER_INTERVAL);
	  return;
	}
#endif
        timer_enabled = 0;
        ts->timer_count = 0;

	//	printk("UP()\n", 0);
        input_report_key(ts->dev, BTN_TOUCH, 0);
        input_report_abs(ts->dev, ABS_PRESSURE, 0);
        input_sync(ts->dev);
	stylus_cleanup = 0; // we're all cleaned up

#if FAST_MODE
	ts->sample_count = 0;
#endif
        return;
    }

    stylus_down_state = 1;
    if( stylus_cleanup != 0 ) {
      printk ( "re-entered without cleanup finished...\n" );
    }
#if FAST_MODE
    //////////////
    //    printk( "t" );
    spi_request_xy();
#if !WORKAROUND_SSP_DIVIDER
    ts->sample_count++;
#endif
    timer_enabled = 1;
    mod_timer(&touch_timer, jiffies + TOUCH_TIMER_INTERVAL);
    return;
    ///////////////
#endif

    if (ts->timer_count < TIMER_PERIOD_COUNT) {
        int count = ts->timer_count;
        if (ts->sample_count == SAMPLES)
            smooth_coords( ts->sample_count,
                           ts->xp_sample, ts->yp_sample,
                           ts->xp_timer + count, ts->yp_timer + count );
        else {
            if (!count)
                ts->timer_count = -1;
            else {
                ts->xp_timer[count] = ts->xp_timer[count-1];
                ts->yp_timer[count] = ts->yp_timer[count-1];
            }
        }
    }

    ts->timer_count++;
    if (ts->timer_count >= TIMER_PERIOD_COUNT) {
        smooth_coords( ts->timer_count,
                       ts->xp_timer, ts->yp_timer,
                       &ts->xp, &ts->yp);

        printk("DOWN(%ld,%ld) [(%ld,%ld)]\n",
            TS_X_TRANSFORM(ts), TS_Y_TRANSFORM(ts), ts->xp, ts->yp);
	//        printk("DOWN(%d,%d) [(%d,%d)]\n",
	//            TS_X_TRANSFORM(ts), TS_Y_TRANSFORM(ts), ts->xp, ts->yp);
        input_report_abs(ts->dev, ABS_X, TS_X_TRANSFORM(ts));
        input_report_abs(ts->dev, ABS_Y, TS_Y_TRANSFORM(ts));
        input_report_key(ts->dev, BTN_TOUCH, 1);
        input_report_abs(ts->dev, ABS_PRESSURE, 1);
        input_sync(ts->dev);

        ts->timer_count = 0;
    }

    ts->sample_count = 0;
    timer_enabled = 1;
    mod_timer(&touch_timer, jiffies + TOUCH_TIMER_INTERVAL);
    //    printk( "t" );
    spi_request_xy();
}


#ifdef CONFIG_SILVERMOON_TS_POLLING
static void touch_poll( int is_stylus_down )
{

    if (!is_stylus_down) {
      stylus_down_state = 0;
      stylus_cleanup = 1; // we might be dirty
#if WORKAROUND_SSP_DIVIDER
	if( iter_request % 3 != 1 ) {
	  //	  printk( "cleaning up...\n" );
	  #ifdef SILVERMOON_TS_POLLING_DBGSPEW
		printk( "Requesting xy via spi\n" );
	  #endif
	  spi_request_xy();
	  //timer_enabled = 1;
	  //mod_timer(&touch_timer, jiffies + TOUCH_TIMER_INTERVAL);
	  return;
	}
#endif
        //timer_enabled = 0;
        ts->timer_count = 0;

	//	printk("UP()\n", 0);
        input_report_key(ts->dev, BTN_TOUCH, 0);
        input_report_abs(ts->dev, ABS_PRESSURE, 0);
        input_sync(ts->dev);
	stylus_cleanup = 0; // we're all cleaned up

#if FAST_MODE
	ts->sample_count = 0;
#endif
        return;
    }

	// Stylus is down
    stylus_down_state = 1;
    if( stylus_cleanup != 0 ) {
      printk ( "re-entered without cleanup finished...\n" );
    }
#if FAST_MODE
    //////////////
    //    printk( "t" );
    #ifdef SILVERMOON_TS_POLLING_DBGSPEW
    printk( "%s() FAST_MODE requesting xy via spi\n", __FUNCTION__ );
    #endif
    spi_request_xy();
#if !WORKAROUND_SSP_DIVIDER
    ts->sample_count++;
#endif
    //timer_enabled = 1;
    //mod_timer(&touch_timer, jiffies + TOUCH_TIMER_INTERVAL);
    return;
    ///////////////
#endif

    if (ts->timer_count < TIMER_PERIOD_COUNT) {
        int count = ts->timer_count;
        if (ts->sample_count == SAMPLES)
            smooth_coords( ts->sample_count,
                           ts->xp_sample, ts->yp_sample,
                           ts->xp_timer + count, ts->yp_timer + count );
        else {
            if (!count)
                ts->timer_count = -1;
            else {
                ts->xp_timer[count] = ts->xp_timer[count-1];
                ts->yp_timer[count] = ts->yp_timer[count-1];
            }
        }
    }

    ts->timer_count++;
    if (ts->timer_count >= TIMER_PERIOD_COUNT) {
        smooth_coords( ts->timer_count,
                       ts->xp_timer, ts->yp_timer,
                       &ts->xp, &ts->yp);

        printk("DOWN(%d,%d) [(%d,%d)]\n",
            TS_X_TRANSFORM(ts), TS_Y_TRANSFORM(ts), ts->xp, ts->yp);
	//        printk("DOWN(%d,%d) [(%d,%d)]\n",
	//            TS_X_TRANSFORM(ts), TS_Y_TRANSFORM(ts), ts->xp, ts->yp);
        input_report_abs(ts->dev, ABS_X, TS_X_TRANSFORM(ts));
        input_report_abs(ts->dev, ABS_Y, TS_Y_TRANSFORM(ts));
        input_report_key(ts->dev, BTN_TOUCH, 1);
        input_report_abs(ts->dev, ABS_PRESSURE, 1);
        input_sync(ts->dev);

        ts->timer_count = 0;
    }

    ts->sample_count = 0;
    //timer_enabled = 1;
    //mod_timer(&touch_timer, jiffies + TOUCH_TIMER_INTERVAL);
    //    printk( "t" );
    #ifdef SILVERMOON_TS_POLLING_DBGSPEW
    printk( "%s() not FAST_MODE, requesting XY via spi\n", __FUNCTION__ );
    #endif
    spi_request_xy();
}
#endif

static inline void touch_timer_activate(void)
{
    if (!timer_enabled) {
        timer_enabled = 1;
        mod_timer(&touch_timer, jiffies + TOUCH_TIMER_ACTIVATE);
    }
}

static inline void touch_timer_activate2(void)
{
    if (!timer_enabled) {
        timer_enabled = 1;
        mod_timer(&touch_timer, jiffies + 5);
    }
}


static void slow_timer_fire(unsigned long data)
{
#ifdef CONFIG_SILVERMOON_TS_POLLING
	// This is the only place where we check
	static int slow_timer_count = 0;
	static int last_stylus_state = -1;
	int stylus_state = stylus_down();
	slow_timer_count++;
#ifdef SILVERMOON_TS_POLLING_DBGSPEW
	if (slow_timer_count % 256 == 0 || stylus_state != last_stylus_state)
	{
		last_stylus_state = stylus_state;
		printk( "%s(%lx) count=%u stylus=%d\n", __FUNCTION__, data, slow_timer_count, stylus_state );
		if (stylus_state || stylus_state != last_stylus_state)
		{
			touch_poll( stylus_state );
		}
	}
#else
	last_stylus_state = stylus_state;
	touch_poll( stylus_state );
#endif
#else
    // this timer catches any lost rising edge interrupts
    //
    if (stylus_down() && !timer_enabled) {
        printk(KERN_ERR "%s(): lost ts int\n", __FUNCTION__);
        touch_timer_activate2();
    }
#endif
    mod_timer(&slow_timer, jiffies + SLOW_TIMER_INTERVAL);
}


static irqreturn_t touchscreen_isr(int irqno, void *param)
{
  //  set_irq_type(my_gpio.irq, IRQF_TRIGGER_NONE); // clear irqs
  //  __raw_writel(MY_GPIO52_OFFSET, my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_CRER);
  //  __raw_writel(MY_GPIO52_OFFSET, my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_CFER);
  DBG("stylus_down=%d\n", stylus_down());
  #ifdef SILVERMOON_TS_POLLING_DBGSPEW
  printk("%s() stylus_down=%d\n", __FUNCTION__, stylus_down());
  #endif

  touch_timer_activate();

  //  __raw_writel(MY_GPIO52_OFFSET, my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_SRER);
  //  __raw_writel(MY_GPIO52_OFFSET, my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_SFER);
  //  set_irq_type(my_gpio.irq, IRQF_TRIGGER_RISING); // set irqs

    return IRQ_HANDLED;
}


static int silvermoon_ts_remove(struct platform_device *dev)
{
  //int i;
  //  printk( "slow_timer\n" );
  del_timer(&slow_timer);

  //  printk( "touch_timer\n" );
  del_timer(&touch_timer);


	if (sspdev.irq)
	{
		printk( "ssp_exit\n" );
		ssp_exit(&sspdev);
	}
	else
	{
		printk( "Skipping ssp_exit - IRQ allocation appears to have failed\n" );
	}

    set_irq_type(my_gpio.irq, IRQF_TRIGGER_NONE); // mask it effectively
    //  printk( "ssp_exit done\n" );
    if (ts) {
      //      printk( "disable irq\n" );
      disable_irq(IRQ_PXA168CS_TOUCHSCREEN);
      //      printk( "enter ts\n" );
        if (ts->dev) {
	  //	  printk( "free irq\n" );
      free_irq(IRQ_PXA168CS_TOUCHSCREEN, ts->dev);
      //      printk( "enter dev\n" );
            input_unregister_device(ts->dev);
	    //      printk( "unregistered\n" );
            input_free_device(ts->dev);
	    //      printk( "freed\n" );
            ts->dev = NULL;
        }
        kfree(ts);
	//      printk( "kfreed\n" );
        ts = NULL;
    }

    //    printk( "unmapping ios\n" );
  iounmap(my_gpio.mmio_base);
  release_mem_region(my_gpio.phys_base, my_gpio.size);
  iounmap(my_gpioedge.mmio_base);
  release_mem_region(my_gpioedge.phys_base, my_gpioedge.size);
  iounmap(my_mfp.mmio_base);
  release_mem_region(my_mfp.phys_base, my_mfp.size);

  //  printk( "trying physical addrs\n" );
  my_gpioedge.phys_base = 0xD4019800;
  my_gpio.phys_base = 0xD4019000;
  my_mfp.phys_base = 0xD401E000;
  release_mem_region(my_gpio.phys_base, my_gpio.size);
  release_mem_region(my_gpioedge.phys_base, my_gpioedge.size);
  release_mem_region(my_mfp.phys_base, my_mfp.size);

    printk( "done\n" );

	// henry@chumby.com - we have no mapping for these registers.
#if 0
    printk( "SSC registers: \n" );
    printk( "SSP1" );
    for( i = 0; i < 0x11; i ++ ) {
      if( (i % 4) == 0)
	printk( "\n%03x: ", i * 4 );
      printk( "%08lx ", readl(IO_ADDRESS(0xD401B000 + i * 4)) );
    }
    printk( "\nSSP2" );
    for( i = 0; i < 0x11; i ++ ) {
      if( (i % 4) == 0)
	printk( "\n%03x: ", i * 4 );
      //      printk( "%08lx ", readl(IO_ADDRESS(0xD42A0C00 + i * 4)) );
      printk( "%08lx ", readl(IO_ADDRESS(0xD401C000 + i * 4)) );
    }
    printk( "\nSSP3" );
    for( i = 0; i < 0x11; i ++ ) {
      if( (i % 4) == 0)
	printk( "\n%03x: ", i * 4 );
      //      printk( "%08lx ", readl(IO_ADDRESS(0xD401C000 + i * 4)) );
      printk( "%08lx ", readl(IO_ADDRESS(0xD401F000 + i * 4)) );
    }
#endif

    //    if (spi_regs) {
    //        iounmap(spi_regs);
    //        spi_regs = NULL;
    //    }
    //    if (spi_clock) {
    //        clk_disable(spi_clock);
    //        clk_put(spi_clock);
    //        spi_clock = NULL;
    //    }
#ifdef GRH_DBUG_PRINTK
    dbug_proc_exit();
#endif
    return 0;
}


static int __init init_gpios(void)
{
  struct resource *res;
  int ret;

	printk( "%s() in %s:%d - entering\n", __FUNCTION__, __FILE__, __LINE__ );

#if 0 // henry@chumby.com - this has been taken care of in arch/arm/mach-pxa/aspenite.c
  ////////////// HERE! --> check for clocks to gpio unit, maybe they are off....
  //  printk( "attempt to turn on GPIO clocks\n" );
  writel(0x03,IO_ADDRESS(0xD4015808));
  writel(0x07,IO_ADDRESS(0xD4015808));
  writel(0x03,IO_ADDRESS(0xD4015808));

  //  printk( "attempt to turn on GPIO clocks, perspec addy\n" );
  writel(0x03,IO_ADDRESS(0xD4015008));
  writel(0x07,IO_ADDRESS(0xD4015008));
  writel(0x03,IO_ADDRESS(0xD4015008));
  // gpio's are curretly configured in u-boot, of all places...
  // for now, I have to invent the whole world of GPIO management around me because this code
  // does not exist! This *will* break once the kernel gets native support for these features.
#endif

  //////////// MFP
  my_mfp.phys_base = 0xD401E000;
  my_mfp.size = 0x208;

  res = request_mem_region(my_mfp.phys_base, my_mfp.size, "mfp" );
  if( res == NULL ) {
    printk( "failed to get memory region for MFP.\n" );
    return -ENODEV;
  }

  my_mfp.phys_base = res->start;

  my_mfp.mmio_base = ioremap(res->start, res->end - res->start + 1);
  if( my_mfp.mmio_base == NULL ) {
    printk( "failed to ioremap MFP area\n" );
    goto err1;
  }

	printk( "%s() - set mfp vm addr\n", __FUNCTION__ );

  //////// gpio edge registers
  my_gpioedge.phys_base = 0xD4019800;
  my_gpioedge.size = 0xC;
  res = request_mem_region(my_gpioedge.phys_base, my_gpioedge.size, "gpioedge" );
  if( res == NULL ) {
    printk( "failed to get memory region for GPIO edge registers.\n" );
    goto err2;
  }

  my_gpioedge.phys_base = res->start;

  my_gpioedge.mmio_base = ioremap(res->start, res->end - res->start + 1);
  if( my_gpioedge.mmio_base == NULL ) {
    printk( "failed to ioremap GPIO edge area\n" );
    goto err3;
  }

	printk( "%s() - set gpio edge base\n", __FUNCTION__ );

  /////////// gpio
  my_gpio.phys_base = 0xD4019000;
  my_gpio.size = 0x1A8;
  res = request_mem_region(my_gpio.phys_base, my_gpio.size, "gpio" );
  if( res == NULL ) {
    printk( "failed to get memory region for GPIO.\n" );
    goto err4;
  }

  my_gpio.phys_base = res->start;

  my_gpio.mmio_base = ioremap(res->start, res->end - res->start + 1);
  if( my_gpio.mmio_base == NULL ) {
    printk( "failed to ioremap GPIO area\n" );
    goto err5;
  }

#ifndef CONFIG_SILVERMOON_TS_POLLING
	printk( "Setting IRQ %d\n", IRQ_PXA168CS_TOUCHSCREEN );

  my_gpio.irq = IRQ_PXA168CS_TOUCHSCREEN;
  set_irq_type(my_gpio.irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
  //  set_irq_type(my_gpio.irq, IRQF_TRIGGER_NONE);
  ret = request_irq(my_gpio.irq, touchscreen_isr, IRQF_DISABLED | IRQF_SAMPLE_RANDOM, "Aspen-TS", ts->dev);
  if( ret ) {
    printk( "Can't allocate GPIO IRQ %d/%d (%s:%d) ret=%d (%s) ts->dev=%08lx\n",
		my_gpio.irq, NR_IRQS, __FILE__, __LINE__, ret, ret == -EINVAL ? "EINVAL" : (ret == -ENOMEM ? "ENOMEM" : "other"),
		(unsigned long)ts->dev );
    goto err6;
  }
  //  set_irq_type(my_gpio.irq, IRQF_TRIGGER_NONE);
  set_irq_type(my_gpio.irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
#endif

	printk( "%s() setting MFP\n", __FUNCTION__ );

  // ok, now we can finally touch some GPIO stuff...
  // gpio_52
  __raw_writel(0xC090, my_mfp.mmio_base + MY_MFP_52); // set up MFP52, detect rising edge, GPIO function (AF0)
  // for falling edge detect, use 0xC0A0

  __raw_writel(0xC000, my_mfp.mmio_base + MY_MFP_50); // for testing setup GPIO50 as an output


#ifdef USE_GPIO_API
	if (!gpio_request( 52, "slvrmn_ts1" ))
	{
		gpio_direction_output( 52, 0 ); // Make GPIO52 an input
		gpio_direction_input( 52 ); // Make GPIO52 an input
	}
	else
	{
		printk( "%s() ERROR - failed gpio request for gpio52\n", __FUNCTION__ );
	}
	if (!gpio_request( 50, "slvrmn_ts2" ))
	{
		gpio_direction_output( 50, 1 ); // Make GPIO50 an output
		gpio_free( 50 );
	}
	else
	{
		printk( "%s() ERROR - failed gpio request for gpio50\n", __FUNCTION__ );
	}
#else
  __raw_writel(MY_GPIO52_OFFSET, my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_CDR); // make GPIO52 an input
  __raw_writel(MY_GPIO50_OFFSET, my_gpio.mmio_base + MY_GPIO50_BASE + MY_GPIO_SDR); // make GPIO50 an output
#endif

#ifndef CONFIG_SILVERMOON_TS_POLLING
	// No API equivalent for this
  __raw_writel(MY_GPIO52_OFFSET, my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_SRER); // set gpio52 rising det
#endif

  printk( "Pin readback of bank 2: *0x%08lx = %08x\n", (unsigned long)my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_PLR, __raw_readl(my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_PLR) );
  printk( "Dir readback of bank 2: *0x%08lx = %08x\n", (unsigned long)my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_PDR, __raw_readl(my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_PDR) );

  __raw_writel(0xA8C0, my_mfp.mmio_base + 0x1A0); // gpio104

#ifdef USE_GPIO_API
	gpio_free( 52 );
	if (!gpio_request( 104, "slvrmn_ts3" ))
	{
		gpio_direction_output( 104, 1 ); // Make GPIO104 an output
	}
	else
	{
		printk( "%s() ERROR - failed gpio request for gpio104\n", __FUNCTION__ );
		goto err6;
	}
#else
  __raw_writel((1 << (104-96)), my_gpio.mmio_base + 0x100 + MY_GPIO_SDR); // make it an output
#endif
  __raw_writel((1 << (104-96)), my_gpio.mmio_base + 0x100 + MY_GPIO_PCR); // clear the bit to 0
#ifdef USE_GPIO_API
	gpio_free( 104 );
	printk( "%s() completed, gpio base = 0x%08lx\n", __FUNCTION__, (unsigned long)my_gpio.mmio_base );
#endif
  return 0;

 err6:
  iounmap(my_gpio.mmio_base);
 err5:
  release_mem_region(my_gpio.phys_base, my_gpio.size);
 err4:
  iounmap(my_gpioedge.mmio_base);
 err3:
  release_mem_region(my_gpioedge.phys_base, my_gpioedge.size);
 err2:
  iounmap(my_mfp.mmio_base);
 err1:
  release_mem_region(my_mfp.phys_base, my_mfp.size);

  return -ENODEV;
}


static int __init init_spi(void)
{
  u32 mode;
  u32 flags;
  u32 psp_flags;
  u32 speed;
  int rc;

  sspdev.ssp = NULL;
  sspdev.port = 1; //?2;
  sspdev.mode = 0x0;
  sspdev.flags = 0x0;
  sspdev.psp_flags = 0x0;
  sspdev.speed = 0x0;
  sspdev.irq = 0;

  // freq = 624 * ((den/num)/2)
  //  writel( (1 & 0xFFF) | (2000 << 16),IO_ADDRESS(0xD4051054)); // dither hither
  //  writel( (2000 & 0xFFF) | (1 << 16),IO_ADDRESS(0xD4051054)); // dither hither

  //  printk( "attempt to turn on SSP2 clocks\n" );
  //  printk( "APBC_SSP2_CLK_RST: %08x\n", readl(IO_ADDRESS(0xD4015820)) );
#if 1
	printk( "SSP2 clock should already be enabled\n" );
#else
  printk("SSP2 clock enable\n");
  writel(0x63,IO_ADDRESS(0xD4015820));
  writel(0x67,IO_ADDRESS(0xD4015820));
  writel(0x63,IO_ADDRESS(0xD4015820));
  //  printk( "APBC_SSP2_CLK_RST (2): %08x\n", readl(IO_ADDRESS(0xD4015820)) );

  //  printk( "attempt to turn on SSP2 clocks\n" );
  //  printk( "APBC_SSP2_CLK_RST: %08x\n", readl(IO_ADDRESS(0xD4015020)) );
  writel(0x63,IO_ADDRESS(0xD4015020));
  writel(0x67,IO_ADDRESS(0xD4015020));
  writel(0x63,IO_ADDRESS(0xD4015020));
  //  printk( "APBC_SSP2_CLK_RST (2): %08x\n", readl(IO_ADDRESS(0xD4015020)) );
#endif

  // freq = 624 * ((den/num)/2)
  //  writel( (1 & 0xFFF) | (2000 << 16),IO_ADDRESS(0xD4051054)); // dither hither
  //  writel( (2000 & 0xFFF) | (1 << 16),IO_ADDRESS(0xD4051054)); // dither hither

  // hgroover isr fn not included in 2.6.25 kernel
  // bunnie isr has to be patched in -- aspen SSP implementation is anemic
  // hgroover: note that devices are set up and accessed origin:0 in platform init
  // but origin:1 here.
  if( 0 != (rc = ssp_init(&sspdev, 2, 0)) ) { // flags indicate to allocate an IRQ
    printk( "silvermoon-ts: could not allocate SSP port 2, error %d\n", rc );
    sspdev.irq = 0;
    return rc;
  }
  ssp_set_handler(&sspdev, spi0_isr);
  printk( "allocated IRQ number: %d\n", sspdev.irq );


    // clock defaults to low, samples on falling edge, generates on rising edge.
    // sscr0
    // 0   normal ssp mode
    // 0   clock selection from network and external clock fields
    // 0   no fifo packing
    // 0   rsvd
    // 0   13 mbps mode
    // 000 frame rate for network mode
    // 1   TUR masked
    // 1   ROR masked
    // 0   clock by external clock field
    // 0   extended data is 0
    // 0000 0000 1111   serial clock rate (SSPx clock / x + 1)  divide by 16 for now, measure & fix
    // 0   SSPx port not enabled
    // 0   ECS selects on-chip clock
    // 00  frame format is motoral SPI
    // 1111 16-bit data size
    // 0000 0000 1100 0000 0000 1111 1000 1111
    // 0x00C00F8F
  mode = 0x00C0003F;  // speed is broken out below
  //  speed = (0x3F & 0xFFF) << 8;
  speed = (0 & 0xFFF) << 8; // for now, set to zero because we use upstream clock divider to set 500khz clock

  // 1  three state on the edge that ends the LSB
  // 0  don't use three-stating
  // 0  disable bitcount error
  // 1  slave clock is active only during transfers
  // 0  other SSPx clock request disabled
  // 0  ditto
  // 0  master mode
  // 0  transmit/receive mode
  // 0  trailing bytes handled by cpu
  // 0  no DMA
  // 0  no DMA
  // 0  no receiver timeout interrupt
  // 0  no periph trailing byte interrupt
  // 0  rsvd
  // 0  frame signal determined by PSP bits
  // 0  not test mode
  // 0  not test mode
  // xxxx  == RX_FIFO_RDY_BYTES - 1
  // 0000  tx fifo trigger level
  // 1  sph = 1
  // 0  spo, inactive state of clock is low
  // 0  no loopback mode
  // 0  no transmit fifo interrupt
  // 1  receive fifo interrupt
  //1 0 0 1 00000000000000000000010001
  flags = 0x90000011 | ((RX_FIFO_RDY_BYTES & 0xF) << 10);

  // now i need to compute the PSP flags  //////////////////////////
  // 0    rsvd
  // 000  edmystp not used
  // 00   edmystart not used
  // 0    frame sync at end of dmtstop timing
  // 00   dmystop is 0, so no extra clocks
  // 0    rsvd
  // 010000 16 cycles in a frame
  // 0000000    no serial frame delay
  // 00     no dummy start
  // 111    no start delay
  // 0    end of data transfer, line goes to 0 (and not last bit value)
  // 0    serial frame polarity is active low
  // 00   data driven (rising) sampled (falling), idle state low
  // 0 000 00 0 00 0 010000 0000000 00 111 0 0 00
  // 0000 0000 0001 0000 0000 0000 0111 0000
  // 0x00100070
  //  psp_flags = 0x7FF0FFF0;
  psp_flags = 0x00170071;

  ssp_disable(&sspdev);
  ssp_config(&sspdev, mode, flags, psp_flags, speed); // this function configures the SSP
  ssp_enable(&sspdev);

  // and that should be it!

#if 0
    spi_clock = clk_get(NULL, "spi0");
    if (spi_clock == NULL) {
        printk(KERN_ERR "%s(): failed to get spi clock source\n", __FUNCTION__);
        return -ENOENT;
    }
    clk_enable(spi_clock);

    printk("SSP2 clock enable\n");
    writel(0x43,IO_ADDRESS(0xD4015820)); /* SSP2 clock enable*/  // note weirdness of | 0x800 into offset?
    writel(0x47,IO_ADDRESS(0xD4015820)); /* SSP2 clock enable*/ // reset
    writel(0x43,IO_ADDRESS(0xD4015820)); /* SSP2 clock enable*/

    spi_regs = ioremap(S3C24XX_PA_SPI, 0x40);
    if (spi_regs == NULL) {
        printk(KERN_ERR "%s(): Failed to remap register block\n", __FUNCTION__);
        return -ENOMEM;
    }

    //    write(0x00C00F8F, spi_regs ... );

    // MSB first -- cpol 0 , cpha 1
    // 00 0100
    writel(0x04, spi_regs + S3C_CH_CFG);  // rx, tx off until config is done

    // halfword widths, trigger rx-fifo-rdy with 6 bytes
    // 0010 0000 0001 0010 0000 0000 0000 0000
    writel(0x20120000 | (RX_FIFO_RDY_BYTES<<11), spi_regs + S3C_MODE_CFG);

    // some amount of CS turnaround time...
    // 00 0000 0010
    writel(0x302, spi_regs + S3C_SLAVE_SEL);

    // enable packet counter
    writel(0x10000, spi_regs + S3C_PACKET_CNT);

    // SPI interrupts disabled for now -- enabled by spi_request_xy()
    writel(0x0, spi_regs + S3C_SPI_INT_EN);

    // now turn on the SPI interface, Rx/Tx channel on, SPI format B
    writel(0x07, spi_regs + S3C_CH_CFG);

#endif
    return 0;
}


static int __init init_touchscreen(void)
{
    ts = kzalloc(sizeof(struct silvermoon_ts), GFP_KERNEL);
    if (ts == NULL) {
        printk(KERN_ERR "%s(): no memory!\n", __FUNCTION__);
        return -ENOMEM;
    }
    ts->dev = input_allocate_device();
    if (ts->dev == NULL) {
        printk(KERN_ERR "%s(): no memory!\n", __FUNCTION__);
        return -ENOMEM;
    }

	// Tell the linux event interface we support absolute X/Y, and key events (down / up)
    ts->dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);  // hail mary, i don't know what this does
    ts->dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	//    ts->dev->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
	//    ts->dev->keybit[LONG(BTN_TOUCH)] = BIT(BTN_TOUCH);

    //    input_set_abs_params(ts->dev, ABS_X, 0, MAXX-1, 0, 0);
    //    input_set_abs_params(ts->dev, ABS_Y, 0, MAXY-1, 0, 0);
    //    input_set_abs_params(ts->dev, ABS_PRESSURE, 0, 1, 0, 0);
    input_set_abs_params(ts->dev, ABS_X, 0, 0x3FF, 0, 0);
    input_set_abs_params(ts->dev, ABS_Y, 0, 0x3FF, 0, 0);
    input_set_abs_params(ts->dev, ABS_PRESSURE, 0, 1, 0, 0);


    sprintf(ts->phys, "ts0");

	// there is no private in struct dev_input - moved to struct input_handle
    //ts->dev->grab->private = ts;
    ts->dev->name = silvermoon_ts_name;
    ts->dev->phys = ts->phys;
    ts->dev->id.bustype = BUS_RS232;
    ts->dev->id.vendor = 0xDEAD;
    ts->dev->id.product = 0xBEEF;
    ts->dev->id.version = SILVERMOON_TSVERSION;

    ts->lastStylus = 0;

    // register touchscreen to the input system
    if( input_register_device(ts->dev) ) {
      printk("silvermoon-ts.c: failed to register input device!\n");
    }
    return 0;
}


static int __init init_irqs(void)
{
#if 0 // this is now handled by ssp_init
    // setup the SPI IRQ
    if (request_irq(IRQ_SPI0, spi0_isr, SA_SAMPLE_RANDOM,
                    "spi0_isr", ts->dev)) {
        printk(KERN_ERR "%s(): IRQ_SPI0 not available!\n", __FUNCTION__);
        return -EIO;
    }
#endif

    // actually this is handled in the GPIO initializations...
#if 0
    // setup the touchscreen IRQ
    set_irq_type(IRQ_EINT11, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING);
    if (request_irq(IRQ_EINT11, touchscreen_isr, SA_SAMPLE_RANDOM,
                    "silvermoon_ts_isr", ts->dev)) {
        printk(KERN_ERR "%s(): IRQ_EINT11 not available!\n", __FUNCTION__);
        return -EIO;
    }
    set_irq_type(IRQ_EINT11, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING);
#endif
    return 0;
}


static int __devinit silvermoon_ts_probe(struct platform_device *pdev)
{
    int rc;
    //int i;

    printk("init_spi\n" );
    rc = init_spi();
    if (rc) {
        silvermoon_ts_remove(pdev);
        return rc;
    }

    printk("init_touchscreen\n" );
    rc = init_touchscreen();
    if (rc) {
        silvermoon_ts_remove(pdev);
        return rc;
    }

    printk("init_gpio\n" );
    rc = init_gpios();
    if( rc ) {
      printk( "init_gpio failed" );
      del_timer(&slow_timer);
      del_timer(&touch_timer);
      return rc;
    }

    printk("init_irqs\n" );
    rc = init_irqs();
    if (rc) {
        silvermoon_ts_remove(pdev);
        return rc;
    }

    //    printk("init dbug\n" );
#ifdef GRH_DBUG_PRINTK
    dbug_proc_init();
#endif


#ifdef CONFIG_SILVERMOON_TS_POLLING
    printk("init mod_timer\n" );
    mod_timer(&slow_timer, jiffies + SLOW_TIMER_INTERVAL);
#endif

    printk(KERN_INFO "%s loaded successfully.\n", silvermoon_ts_name);

    // hgroover: in 2.6.28 we don't have the addresses/resources mapped to do this dump
#if 0
    printk( "SSC registers: \n" );
    printk( "SSP1" );
    for( i = 0; i < 0x11; i ++ ) {
      if( (i % 4) == 0)
	printk( "\n%03x: ", i * 4 );
      printk( "%08lx ", readl(IO_ADDRESS(0xD401B000 + i * 4)) );
    }
    printk( "\nSSP2" );
    for( i = 0; i < 0x11; i ++ ) {
      if( (i % 4) == 0)
	printk( "\n%03x: ", i * 4 );
      //      printk( "%08lx ", readl(IO_ADDRESS(0xD42A0C00 + i * 4)) );
      printk( "%08lx ", readl(IO_ADDRESS(0xD401C000 + i * 4)) );
    }
    printk( "\nSSP3" );
    for( i = 0; i < 0x11; i ++ ) {
      if( (i % 4) == 0)
	printk( "\n%03x: ", i * 4 );
      //      printk( "%08lx ", readl(IO_ADDRESS(0xD401C000 + i * 4)) );
      printk( "%08lx ", readl(IO_ADDRESS(0xD401F000 + i * 4)) );
    }

    printk( "\n" );

    writel(0x63,IO_ADDRESS(0xD4015820));
    writel(0x63,IO_ADDRESS(0xD4015020));
    printk( "clock: %08X, %08X\n", readl(IO_ADDRESS(0xD4015020)), readl(IO_ADDRESS(0xD4015820)) );
    printk( "MFP\n" );
    for( i = 0; i < 0x82; i ++ ) {
      if( (i % 4) == 0)
	printk( "\n%03x: ", i * 4 );
      //      printk( "%08lx ", readl(IO_ADDRESS(0xD401C000 + i * 4)) );
      printk( "%08lx ", readl(IO_ADDRESS(0xD401E000 + i * 4)) );
    }
    // 118 - 121 : 1d8, 1dc, 1e0, 1e4
    // d8c1, d8c1, d8c1, d8c1   1101 1000 1100 0001

    printk( "\n" );
#endif

    return 0;
}


static struct platform_driver silvermoon_ts_driver = {
    .probe          = silvermoon_ts_probe,
    .remove         = silvermoon_ts_remove,
    .suspend        = NULL,
    .resume         = NULL,
    .driver     = {
    //        .owner  = THIS_MODULE,
    //        .name   = "s3c2410-adc", // platform_driver_register() keys off name!
    .name   = "silvermoon-ts", // platform_driver_register() keys off name!
    },
};


static char banner[] __initdata =
        KERN_INFO "Silvermoon Touchscreen driver v" TS_DRIVER_VER ", (c) 2008-9 Chumby Industries\n";


static int __init silvermoon_ts_init(void)
{
    printk(banner);
    silvermoon_ts_probe(&silvermoon_ts_driver);
    return platform_driver_register(&silvermoon_ts_driver);
}


static void __exit silvermoon_ts_exit(void)
{
   silvermoon_ts_remove(&silvermoon_ts_driver);
    platform_driver_unregister(&silvermoon_ts_driver);
}


module_init(silvermoon_ts_init);
module_exit(silvermoon_ts_exit);

MODULE_AUTHOR("Chumby Industries -- Greg Hutchins (ghutchins@gmail.com)");
MODULE_DESCRIPTION("Silvermoon Touchscreen Driver");
MODULE_LICENSE("GPL");
