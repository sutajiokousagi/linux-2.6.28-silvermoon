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
#include <linux/moduleparam.h>
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

#include <mach/irqs.h>


#include <linux/gpio.h>

#define CHLOG(format, arg...)            \
	printk("silvermoon-tsb.c - %s():%d - " format, __func__, __LINE__, ## arg)


// hgroover - copied from 2.6.24.7 mach-ttc
#if !defined(IO_ADDRESS)
#define IO_ADDRESS(x) (x)
#endif

#include <mach/timex.h>

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

#define WORKAROUND_SSP_DIVIDER 1

// Define this to use GPIO api whenever possible
#define USE_GPIO_API

static char *silvermoon_ts_name = "Silvermoon Touchscreen";

static int scaled_touchscreen = 0;
static int debug_enabled = 0;

#define X_MIN 440
#define X_MAX 3650
#define Y_MIN 350
#define Y_MAX 3800 
#define SCREEN_W 800 
#define SCREEN_H 600
static inline int axis_to_screen(int raw, int scale, int min, int max) {
    int flip = 0;
    if(min > max) {
        int tmp;
        tmp  = max;
        max  = min;
        min  = tmp;
        flip = 1;
    }

    raw -= min;
    raw = raw / (max / scale);

    if(!flip)
        raw = scale - raw;

    if( scale == 800 )
    {
        if( debug_enabled )
        {
            printk( "X: %d\n", raw);
        }
    }
    else
    {
        // y is flipped
        raw = scale - raw;
        if( debug_enabled )
        {
            printk( "Y: %d\n", raw );
        }
    }

    return raw;
}


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

// version # of the touchscreen driver
#define SILVERMOON_TSVERSION 0x0102


// Timer intervals,
//
#define TOUCH_TIMER_INTERVAL msecs_to_jiffies(1)  // stylus down interval
#define TOUCH_TIMER_ACTIVATE msecs_to_jiffies(5)   // activate after interrupt
#define TOUCH_TIMER_BKP_INTERVAL msecs_to_jiffies(10)  // failed packet timeout

#define RX_FIFO_RDY_BYTES 2

// This driver gathers data directly from the cryptoprocessor

// We collect SAMPLES samples every 10ms, every 50ms we report the position of
// the stylus based on an average of the samples that are the most similar.
// SAMPLES*3 bytes will be allocated on the stack for variance and averaging
// purposes.
//
#define SAMPLES	15

#define MAXX        0x1000    // max X value -- *MUST* be a power of two
#define MAXY        0x1000    // max Y value -- *MUST* be a power of two

#define MIN_RAW_SAMPLE_VALUE 50

//------------------------------------------------------------
// from the CP code (commonCrypto.h)

// SPI address space
// These are the virtual registers that can be addressed
//   with input messages to the controller
#define SPI_REGADR_NOOP             0   // wr only, used only for shunting out data
#define SPI_REGADR_LOOP             1   // not implemented
#define SPI_REGADR_DUMP_REQ         2   // wr only, will trigger the controller to send a
                                       //    series of readings, capped by an EOS
#define SPI_REGADR_COOLDOWN_DELAY   3   // wr only
#define SPI_REGADR_NUMSAMPS         4   // wr only
#define SPI_REGADR_PENUP_THRESH_HI  5   // wr only
#define SPI_REGADR_PENUP_THRESH_LO  6   // wr only
#define SPI_REGADR_SETTLE_DELAY     7  // wr only
#define SPI_REGADR_INVALID_MAX_HI   8  // wr only
#define SPI_REGADR_INVALID_MAX_LO   9  // wr only
#define SPI_REGADR_INVALID_MIN_HI   10  // wr only
#define SPI_REGADR_INVALID_MIN_LO   11  // wr only
#define SPI_REGADR_LIGHTSENSOR_REQ  12  // wr only, response is sent back with next xfer

// These are messages that are transfered out to the master.  They may be in response to
//   any previous command.
#define SPI_OUTPUT_NOOP  0
#define SPI_OUTPUT_X     1
#define SPI_OUTPUT_Y     2
#define SPI_OUTPUT_Z1    3
#define SPI_OUTPUT_Z2    4
#define SPI_OUTPUT_EOS   5 // signals the end of a reading dump
#define SPI_OUTPUT_LIGHT 6
#define SPI_OUTPUT_PRESSURE 7

//------------------------------------------------------------------
#define WRITE_TS_REG(reg, val) ( (reg << 8) | (val & 0xFF))
#define TS_OUTPUT_TYPE(x) (x>>12)
#define TS_OUTPUT_VALUE(x) (x & 0x0FFF)

#define MODULE_NAME "ts"  //don't want the full path from __FILE__ in debug statements

// Number of samples... ?
static unsigned int param_numsamps = 10;
module_param_named(numsamps, param_numsamps, uint, 0644);
MODULE_PARM_DESC(numsamps, "numsamps (10=default)");

static unsigned int param_cooldown_delay = 3;
module_param_named(cooldown_delay, param_cooldown_delay, uint, 0644);
MODULE_PARM_DESC(cooldown_delay, "cooldown_delay (3=default)");

static unsigned int param_penup_thresh = 3980;
module_param_named(penup_thresh, param_penup_thresh, uint, 0644);
MODULE_PARM_DESC(penup_thresh, "penup_thresh (3980=default)");

static unsigned int param_settle_delay = 500;
module_param_named(settle_delay, param_settle_delay, uint, 0644);
MODULE_PARM_DESC(settle_delay, "settle_delay (500=default)");

static unsigned int param_min_raw_pressure = 500;
module_param_named(min_raw_pressure, param_min_raw_pressure, uint, 0644);
MODULE_PARM_DESC(min_raw_pressure, "min_raw_pressure (500=default)");

static unsigned int param_debug = 0;
module_param_named(debug, param_debug, uint, 0644);
MODULE_PARM_DESC(param_debug, "debug (0=default)");

// henry@chumby.com - looks like the values are already in a good range
// for calibration, just need to be swapped...
#define TS_X_TRANSFORM(v)   (((MAXX - v->yp) / 5) + 130)
#define	TS_Y_TRANSFORM(v)   (((MAXY - v->xp) / 5) + 130)




// Per-touchscreen data.
//
struct silvermoon_ts {
	struct input_dev *dev;
	long xp;
	long yp;
	char phys[32];

	long xp_timer[5];
	long yp_timer[5];

	long xp_sample[SAMPLES];
	long yp_sample[SAMPLES];

	long xhist;
	long yhist;
	int lastStylus;

	long dummy;

	long packet_count;
};

#define NOISE_THRESH 100
#define NOOPS_THRESHOLD 10

static void timer_fire(unsigned long data);
static void timer_bkp_fire(unsigned long data);

static struct silvermoon_ts  *ts = NULL;
static int packet_to_send = SPI_REGADR_DUMP_REQ;
static int noops_received;
static u32 x_val, y_val, z1_val, z2_val, pressure_val, mouse_down;
static struct timer_list touch_timer = TIMER_INITIALIZER(timer_fire, 1, 0);
static struct timer_list touch_timer_bkp = TIMER_INITIALIZER(timer_bkp_fire, 0, 0);

static int lightSensorValue = 0;




static int sensitivity_read(char *buf, char **start, off_t offset,
							int count, int *eof, void *data) {
	*eof = 1;
	return sprintf(buf, "%d\n", param_min_raw_pressure);
}

static int sensitivity_write(struct file *file, const char *buf,
							 unsigned long count, void *data) {
	unsigned long new_pressure = simple_strtoul(buf, NULL, 0);
	if(new_pressure > 0) {
		param_min_raw_pressure = new_pressure;
		return count;
	}
	return -EINVAL;
}





static inline int stylus_down(void)
{
	// read the touchscreen interrupt line, high denotes stylus down
	// TS-touched is GPIO52
	return (__raw_readl(my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_PLR) & MY_GPIO52_OFFSET) != 0;
}


static inline void spi_stop(void)
{
	ssp_disable(&sspdev);
}



static inline void spi_request_ts_data(void) {
	// Initiate a request to dump all of the registers.
	// They will come back as "X, Y, Pressure", in that order.
	// In order to get a value, we must request the full dump.
	ssp_write_word(&sspdev, WRITE_TS_REG(packet_to_send, 0));
	mod_timer(&touch_timer_bkp, jiffies + TOUCH_TIMER_BKP_INTERVAL);
}


static inline void spi_process_fifo(int fifosize) {
	u32 data;
	int i;

	del_timer(&touch_timer_bkp);

	// go through each pair of bytes
	for(i=0; i<fifosize; i+=2) {

		ssp_read_word(&sspdev, &data);

		packet_to_send = -1;

		switch(TS_OUTPUT_TYPE(data)) {
			case SPI_OUTPUT_X:
				noops_received = 0;
				x_val  = TS_OUTPUT_VALUE(data);
				x_val &= 0xfffF;
				ts->xp = x_val;
				break;

			case SPI_OUTPUT_Y:
				noops_received = 0;
				y_val  = TS_OUTPUT_VALUE(data);
				y_val &= 0xfffF;
				ts->yp = y_val;
				break;

			case SPI_OUTPUT_PRESSURE:
				noops_received = 0;
				pressure_val = TS_OUTPUT_VALUE(data);
				if (x_val >= MIN_RAW_SAMPLE_VALUE
				 && y_val >= MIN_RAW_SAMPLE_VALUE
				 && pressure_val > 0
				 && pressure_val > param_min_raw_pressure
				 && ts->packet_count) {
        				if(scaled_touchscreen) {
            					input_report_abs(ts->dev, ABS_X,
                   					axis_to_screen(ts->yp, SCREEN_W, X_MIN, X_MAX));
            					input_report_abs(ts->dev, ABS_Y,
                    					axis_to_screen(ts->xp, SCREEN_H, Y_MIN, Y_MAX));
        				}
        				else {
						input_report_abs(ts->dev, ABS_X, TS_X_TRANSFORM(ts));
						input_report_abs(ts->dev, ABS_Y, TS_Y_TRANSFORM(ts));
						if( debug_enabled )
						{
							printk( "x: %d, abs_x: %d, y: %d, abs_y: %d\n", ts->xp, ABS_X, ts->yp, ABS_Y );
						}
					}
					input_report_key(ts->dev, BTN_TOUCH, 1);
					input_report_abs(ts->dev, ABS_PRESSURE, pressure_val);
					input_sync(ts->dev);
					mouse_down = 1;
				}

				// Specify that the next packet we send should request a
				// dump of all registers, to re-fetch all values.
				packet_to_send = SPI_REGADR_DUMP_REQ;
				ts->packet_count++;
				break;

			case SPI_OUTPUT_Z1:
				z1_val = TS_OUTPUT_VALUE(data);
				break;

			case SPI_OUTPUT_Z2:
				z2_val = TS_OUTPUT_VALUE(data);
				break;

			case SPI_OUTPUT_LIGHT:
				lightSensorValue = TS_OUTPUT_VALUE(data);
				break;

			case SPI_OUTPUT_NOOP:
			case SPI_OUTPUT_EOS:
			default:

				break;
		}
	}

	// Reuqest another packet for unknown, garbage packets.
	if(packet_to_send == -1 && stylus_down()) {
		// If we receive a NOOP, and we've already received a lot
		// of NOOPS, send another register dump.  Otherwise, send another
		// NOOP to try and pump out data.
		noops_received++;
		if(stylus_down() && noops_received >= NOOPS_THRESHOLD) {
			noops_received = 0;
			packet_to_send = SPI_REGADR_DUMP_REQ;
			CHLOG("Got a lot of NOOP packets.  Sending another dump request.\n");
		}
		else
			packet_to_send = SPI_REGADR_NOOP;
	}

	// If the packet to send is a valid one, queue up the timer and send it.
	if(packet_to_send != -1)
		mod_timer(&touch_timer, jiffies + TOUCH_TIMER_INTERVAL);
	else
		x_val = y_val = z1_val = z2_val = pressure_val = 0;

	return;
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

	if (status & SSSR_ROR)
		printk(KERN_WARNING "SSP(%d): receiver overrun\n", dev->port);

	if (status & SSSR_TUR)
		printk(KERN_WARNING "SSP(%d): transmitter underrun\n", dev->port);

	if (status & SSSR_BCE)
		printk(KERN_WARNING "SSP(%d): bit count error\n", dev->port);

	//printk( "rx_fifo_rdy_bytes = %d\n", rx_fifo_rdy_bytes);
	if (status & SSSR_RFS && rx_fifo_rdy_bytes > 0)
		spi_process_fifo(rx_fifo_rdy_bytes);

	return IRQ_HANDLED;
}



/* Called when the SSP fails to issue an interrupt.  It means that a packet
 * was lost between the CP and the CPU.  We'll re-issue the request and
 * hope things work out for the best.
 */
static void timer_bkp_fire(unsigned long data) {
	CHLOG("SPI response timed out.  Re-issuing SPI dump register request...\n");
	packet_to_send = SPI_REGADR_DUMP_REQ;
	spi_request_ts_data();
}



static void timer_fire(unsigned long data) {
	// If we got a mouse-up event (and if we've already reported the X/Y
	// coordinates), report the last mouse position.
	if (!stylus_down() && mouse_down) {
		if (param_debug)
			printk("%s: ---------------pen up\n", MODULE_NAME);

		// Only send the mouse-up event if we at one point send a
		// mouse-down event.
		input_report_key(ts->dev, BTN_TOUCH, 0);
		input_report_abs(ts->dev, ABS_PRESSURE, 0);
		input_sync(ts->dev);
		x_val = y_val = z1_val = z2_val = 0;
		ts->packet_count = pressure_val = mouse_down = 0;
	}
	else  //stylus is down
		spi_request_ts_data();
}



static inline void touch_timer_activate(void)
{
	packet_to_send = SPI_REGADR_DUMP_REQ;
	// Clear out past X and Y values to prevent the next tap
	// from appearing like a drag.
	mod_timer(&touch_timer, jiffies + TOUCH_TIMER_ACTIVATE);
}


static irqreturn_t touchscreen_isr(int irqno, void *param)
{
	touch_timer_activate();
	return IRQ_HANDLED;
}


static int silvermoon_ts_remove(struct platform_device *dev)
{
	remove_proc_entry("driver/touchscreen/sensitivity", NULL);
	remove_proc_entry("driver/touchscreen", NULL);

	del_timer(&touch_timer);
	del_timer(&touch_timer_bkp);

	if (sspdev.irq)
		ssp_exit(&sspdev);
	else
		printk( "Skipping ssp_exit - IRQ allocation appears to have failed\n" );

	set_irq_type(my_gpio.irq, IRQF_TRIGGER_NONE); // mask it effectively
	//printk( "ssp_exit done\n" );
	if (ts)
	{
		disable_irq(IRQ_PXA168CS_TOUCHSCREEN);
		if (ts->dev) {
			free_irq(IRQ_PXA168CS_TOUCHSCREEN, ts->dev);
			input_unregister_device(ts->dev);
			input_free_device(ts->dev);
			ts->dev = NULL;
		}
		kfree(ts);
		ts = NULL;
	}

	//printk( "unmapping ios\n" );
	iounmap(my_gpio.mmio_base);
	release_mem_region(my_gpio.phys_base, my_gpio.size);
	iounmap(my_gpioedge.mmio_base);
	release_mem_region(my_gpioedge.phys_base, my_gpioedge.size);
	iounmap(my_mfp.mmio_base);
	release_mem_region(my_mfp.phys_base, my_mfp.size);

	/*
	printk( "trying physical addrs\n" );
	my_gpioedge.phys_base = 0xD4019800;
	my_gpio.phys_base = 0xD4019000;
	my_mfp.phys_base = 0xD401E000;
	release_mem_region(my_gpio.phys_base, my_gpio.size);
	release_mem_region(my_gpioedge.phys_base, my_gpioedge.size);
	release_mem_region(my_mfp.phys_base, my_mfp.size);
	*/

	printk( "done\n" );

#ifdef GRH_DBUG_PRINTK
	dbug_proc_exit();
#endif
	return 0;
}


static int __init init_gpios(void)
{
	struct resource *res;
	int ret;

	CHLOG("Initting GPIOs...\n");


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

	CHLOG( "Setting IRQ %d\n", IRQ_PXA168CS_TOUCHSCREEN );

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
	set_irq_type(my_gpio.irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);


	CHLOG("Setting MFP\n");

	// ok, now we can finally touch some GPIO stuff...
	// gpio_52
	__raw_writel(0xC090, my_mfp.mmio_base + MY_MFP_52); // set up MFP52, detect rising edge, GPIO function (AF0)
	// for falling edge detect, use 0xC0A0

	__raw_writel(0xC000, my_mfp.mmio_base + MY_MFP_50); // for testing setup GPIO50 as an output


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

		// No API equivalent for this
	__raw_writel(MY_GPIO52_OFFSET, my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_SRER); // set gpio52 rising det

	CHLOG( "Pin readback of bank 2: *0x%08lx = %08x\n", (unsigned long)my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_PLR, __raw_readl(my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_PLR) );
	CHLOG( "Dir readback of bank 2: *0x%08lx = %08x\n", (unsigned long)my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_PDR, __raw_readl(my_gpio.mmio_base + MY_GPIO52_BASE + MY_GPIO_PDR) );

	__raw_writel(0xA8C0, my_mfp.mmio_base + 0x1A0); // gpio104

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
	__raw_writel((1 << (104-96)), my_gpio.mmio_base + 0x100 + MY_GPIO_PCR); // clear the bit to 0
	gpio_free( 104 );
	CHLOG( "%s() completed, gpio base = 0x%08lx\n", __FUNCTION__, (unsigned long)my_gpio.mmio_base );
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

	// hgroover isr fn not included in 2.6.25 kernel
	// bunnie isr has to be patched in -- aspen SSP implementation is anemic
	// hgroover: note that devices are set up and accessed origin:0 in platform init
	// but origin:1 here.
	if( 0 != (rc = ssp_init(&sspdev, 2, 0)) )
	{
		// flags indicate to allocate an IRQ
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
	return 0;
}


static int __init init_touchscreen(void)
{
	ts = kzalloc(sizeof(struct silvermoon_ts), GFP_KERNEL);
	if (ts == NULL)
	{
		printk(KERN_ERR "%s(): no memory!\n", __FUNCTION__);
		return -ENOMEM;
	}
	ts->dev = input_allocate_device();
	if (ts->dev == NULL)
	{
		printk(KERN_ERR "%s(): no memory!\n", __FUNCTION__);
		return -ENOMEM;
	}

	// Tell the linux event interface we support absolute X/Y, and key events (down / up)
	ts->dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);  // hail mary, i don't know what this does
	ts->dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	if(scaled_touchscreen) {
		input_set_abs_params(ts->dev, ABS_X, 0, SCREEN_W, 0, 0);
		input_set_abs_params(ts->dev, ABS_Y, 0, SCREEN_H, 0, 0);
	}
	else {
		input_set_abs_params(ts->dev, ABS_X, 0, 0x3FF, 0, 0);
		input_set_abs_params(ts->dev, ABS_Y, 0, 0x3FF, 0, 0);
	}
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
	if( input_register_device(ts->dev) )
	{
		printk("silvermoon-ts.c: failed to register input device!\n");
	}
	return 0;
}


static int __devinit silvermoon_ts_probe(struct platform_device *pdev)
{
	int rc;
	struct proc_dir_entry *pde;


	printk("init_spi\n" );
	rc = init_spi();
	if (rc)
	{
		silvermoon_ts_remove(pdev);
		return rc;
	}

	printk("init_touchscreen\n" );
	rc = init_touchscreen();
	if (rc)
	{
		silvermoon_ts_remove(pdev);
		return rc;
	}

	printk("init_gpio\n" );
	rc = init_gpios();
	if( rc )
	{
		printk( "init_gpio failed" );
		del_timer(&touch_timer);
		del_timer(&touch_timer_bkp);
		return rc;
	}

	//printk("init dbug\n" );
#ifdef GRH_DBUG_PRINTK
	dbug_proc_init();
#endif

	ssp_enable(&sspdev);

	printk("Setting some CP touchscreen controller parameters\n");

	ssp_write_word(&sspdev, WRITE_TS_REG(SPI_REGADR_COOLDOWN_DELAY, param_cooldown_delay));
	udelay(1000);
	ssp_write_word(&sspdev, WRITE_TS_REG(SPI_REGADR_PENUP_THRESH_HI, (param_penup_thresh >> 8)));
	udelay(1000);
	ssp_write_word(&sspdev, WRITE_TS_REG(SPI_REGADR_PENUP_THRESH_LO, (param_penup_thresh & 0xFF)));
	udelay(1000);
	ssp_write_word(&sspdev, WRITE_TS_REG(SPI_REGADR_SETTLE_DELAY, param_settle_delay/100));  //controller will scale it back (x100)
	udelay(1000);
	ssp_write_word(&sspdev, WRITE_TS_REG(SPI_REGADR_NUMSAMPS, param_numsamps));
	udelay(1000);


	proc_mkdir("driver/touchscreen", 0);
	pde = create_proc_read_entry("driver/touchscreen/sensitivity", 0, 0,
								sensitivity_read, NULL);
	pde->write_proc = sensitivity_write;


	printk(KERN_INFO "%s loaded successfully.\n", silvermoon_ts_name);

	return 0;
}


static struct platform_driver silvermoon_ts_driver = {
	.probe          = silvermoon_ts_probe,
	.remove         = silvermoon_ts_remove,
	.suspend        = NULL,
	.resume         = NULL,
	.driver     = {
	//        .owner  = THIS_MODULE,
	//        .name   = "silvermoon-ts", // platform_driver_register() keys off name!
	.name   = "silvermoon-ts", // platform_driver_register() keys off name!
	},
};


static char banner[] __initdata =
		KERN_INFO "Silvermoon Touchscreen driver v" TS_DRIVER_VER ", (c) 2008-2010 Chumby Industries\n";


static int __init silvermoon_ts_init(void)
{
	printk(banner);
	return platform_driver_register(&silvermoon_ts_driver);
}


static void __exit silvermoon_ts_exit(void)
{
	platform_driver_unregister(&silvermoon_ts_driver);
}


module_init(silvermoon_ts_init);
module_exit(silvermoon_ts_exit);
module_param(scaled_touchscreen, bool, 0644);
MODULE_PARM_DESC(scaled_touchscreen, "true if the touchscreen should report pre-scaled values");
module_param(debug_enabled, bool, 0644);
MODULE_PARM_DESC(debug_enabled, "true if the touchscreen should print touch screen coords to the console");

MODULE_AUTHOR("Chumby Industries -- Greg Hutchins (ghutchins@gmail.com)");
MODULE_DESCRIPTION("Silvermoon Touchscreen Driver");
MODULE_LICENSE("GPL");
