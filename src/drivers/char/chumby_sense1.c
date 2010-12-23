/*
	chumby_sense1.c
	bunnie -- June 2006 -- 1.4 -- linux 2.4.20
	bunnie -- March 2007 -- 2.0 -- port to Ironforge linux 2.6.16
        bunnie -- December 2007 -- 2.1 -- port to v3.8 and inclusion of watchdog timer feature, reset reason reporting
	henry@chumby.com - May 2009 - more changes for silvermoon hardware on linux 2.6.28

	This file is part of the chumby sensor suite driver in the linux kernel.
	Copyright (c) Chumby Industries, 2007-9

	The sensor suite driver is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License as published
	by the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	The sensor suite driver is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with the Chumby; if not, write to the Free Software
	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#define SENSE1_VERSION "0.12-$Rev: 7829 $-Silvermoon"

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/seq_file.h>
#include <linux/cdev.h>

#include <asm/io.h>
#include <asm/system.h>		/* cli(), *_flags */
#include <asm/uaccess.h>	/* copy_*_user */

#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/poll.h>
#include <linux/sysctl.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/rtc.h>

//#include <asm/hardware.h>

//#include <asm/arch/ttc-regs.h>
//#include <asm/arch/pxa-regs.h>
//#include <asm/arch/timex.h>

#include <linux/timer.h>

#include "chumby_sense1.h"

// Get gpio API
//#include <asm-arm/arch-pxa/hardware.h>
//#include <asm-arm/arch-pxa/pxa-regs.h>
//#include <asm-arm/arch-pxa/gpio.h>
#include <linux/gpio.h>

#undef CHUMBY_DIRECT_PORT_DIDDLE

// hgroover - copied from 2.6.24.7 mach-ttc
#if !defined(IO_ADDRESS)

//#include <asm-arm/arch-pxa/platform_pxa168.h>

//#ifdef CONFIG_MMU
///* macro to get at IO space when running virtually */
//#define IO_ADDRESS(x) ((x) | VIRT_IO_BASE)
//#else
#define IO_ADDRESS(x) (x)
//#endif

#endif

/*
 * basic parameters
 */

int sense1_major = 0;		// dynamic allocation
int sense1_minor = 0;
int sense1_nr_devs = 1;

module_param(sense1_major, int, S_IRUGO);
module_param(sense1_minor, int, S_IRUGO);
module_param(sense1_nr_devs, int, S_IRUGO);

MODULE_AUTHOR("bunnie@chumby.com");
MODULE_LICENSE("GPL");

#define PHYS_APB1CLOCK_BASE		0xD4015000
//#define PHYS_APB2CLOCK_BASE		0xD4015800
#define MY_APB1CLOCK_BASE	APB12CLOCK_BASE

#define PHYS_GPIO_BASE		0xD4019000
#define MY_GPIO_BASE		GPIO_BASE
#define MY_GPIO_BANK0_BASE	(MY_GPIO_BASE+0x000)
#define MY_GPIO_BANK1_BASE	(MY_GPIO_BASE+0x004)
#define MY_GPIO_BANK2_BASE	(MY_GPIO_BASE+0x008)
#define MY_GPIO_BANK3_BASE	(MY_GPIO_BASE+0x00C)

#define MY_GPIO_89_MASK  (1 << (89 - 64))
#define MY_GPIO_89_BASE  MY_GPIO_BANK2_BASE
#define MY_GPIO_84_MASK	 (1 << (84 - 64))
#define MY_GPIO_84_BASE	 MY_GPIO_BANK2_BASE
#define MY_GPIO_GCDR     0x60
#define MY_GPIO_GPLR     0x00

#define MY_GPIO_PLR		0x00
#define	MY_GPIO_PDR		0x0c
#define	MY_GPIO_PSR		0x18
#define	MY_GPIO_PCR		0x24
#define MY_GPIO_SDR		0x58
#define	MY_GPIO_CDR		0x60

#define PHYS_MFPR_BASE	0xD401E000
#define MY_MFPR_BASE	MFPR_BASE
#define MY_MFP84_OFF	0x150
#define MY_MFP89_OFF	0x164
#define MY_MFP98_OFF	0x188
#define MY_MFP99_OFF	0x18C

#if defined(CHUMBY_DIRECT_PORT_DIDDLE)
// Physical base address for GPIO registers
static void __iomem *GPIO_BASE;

struct my_gpio {
  void __iomem  *mmio_base;
  unsigned long phys_base;
//  int irq;
  unsigned long size;
} my_gpio;

// Physical base address for APB1CLOCK and APB2CLOCK registers
static void __iomem *APB12CLOCK_BASE;

// Physical base address for MFPR registers
static void __iomem *MFPR_BASE;

struct my_mfp {
  void __iomem  *mmio_base;
  unsigned long phys_base;
  unsigned long size;
} my_mfp;
#endif

// static data
static int gDone = 0;
static unsigned long sense1_status = 0;	/* bitmapped status byte.       */

static g_gpio_button_request_success = 0;
#define BUTTON_GPIO 89


/*
 * Sense1 sensor data logs, tracked by tasks that are scheduled by the
 * task scheduler
 */

#define FIXEDPOINT_NORM 1000

struct sense1data {
	unsigned char bent;
	unsigned char isDebounce;
	int lastTransitionState;
	unsigned long lastTransitionTime;
	unsigned long currentTransitionTime;
	struct timer_list timer;
	struct cdev *sense1_cdev;
} sense1task_data;

#define USE_SYSCTL  0

#define SENSE1_BLOCKING 0	// turn off blocking read on sense1 sensor

#if 1
#   if USE_SYSCTL
#       define CHUMSENSE1_DEBUG( flag, fmt, args... ) do { if ( gDebug ## flag ) printk( "%s: " fmt, __FUNCTION__ , ## args ); } while (0)
#   else
#       define CHUMSENSE1_DEBUG( flag, fmt, args... ) printk( "%s: " fmt, __FUNCTION__ , ## args )
#   endif
#else
#   define CHUMSENSE1_DEBUG( flag, fmt, args... )
#endif

// function protos
static int chumby_sense1_open(struct inode *inode, struct file *file);
static int chumby_sense1_release(struct inode *inode, struct file *file);
static int sense1_read_proc(char *page, char **start, off_t off,
			    int count, int *eof, void *data);
//static int resetReason_proc(char *page, char **start, off_t off,
//			    int count, int *eof, void *data);
static ssize_t chumby_sense1_read(struct file *file, char *buf,
				  size_t count, loff_t * ppos);
//static ssize_t chumby_sense1_write(struct file *file, const unsigned char *buf,
//                               size_t count, loff_t *ppos );
//static unsigned int chumby_sense1_poll(struct file * filp, struct poll_table_struct * wait);

static unsigned int gDebounce = 10;

#if USE_SYSCTL

// proc debug structure
static int gDebugTrace = 0;
static int gDebugIoctl = 0;
static int gDebugError = 1;
static unsigned int gSpkMute = 0;
static unsigned int gDimLevel = 0;
static unsigned int gHPin = 0;
static unsigned int gResetSerial = 0;

// create /proc/sys/debug-trace, etc...which can be written to set behavior of globals
// in this driver on the fly
static struct ctl_table_header *gSysCtlHeader;
static struct ctl_table gSysCtlChumsense1[] = {
	{CTL_CHUMSENSE1_DEBUG_TRACE, "dbg-trace", &gDebugTrace, sizeof(int), 0644, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_DEBUG_IOCTL, "dbg-ioctl", &gDebugIoctl, sizeof(int), 0644, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_DEBUG_ERROR, "dbg-error", &gDebugError, sizeof(int), 0644, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_SPKMUTE, "spkmute", &gSpkMute, sizeof(unsigned int), 0644, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_DIMLEVEL, "dimlevel", &gDimLevel, sizeof(unsigned int), 0644, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_HPIN, "hpin", &gHPin, sizeof(unsigned int), 0644, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_DEBOUNCE, "debounce", &gDebounce, sizeof(unsigned int), 0644, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_RESETSERIAL, "resetSerial", &gResetSerial, sizeof(unsigned int), 0644, NULL, &proc_dointvec},
	{0}
};

static struct ctl_table gSysCtl[] = {
	{CTL_CHUMSENSE1, "sense1", NULL, 0, 0555, gSysCtlChumsense1},
	{0}
};
#endif				// USE_SYSCTL

// map into the generic driver infrastructure
static struct file_operations sense1_fops = {
	.owner =	THIS_MODULE,
	.read =		chumby_sense1_read,
	.open =		chumby_sense1_open,
	.release =	chumby_sense1_release,
};

///////////// code /////////////

static int chumby_sense1_release(struct inode *inode, struct file *file)
{
	CHUMSENSE1_DEBUG(Trace, "Top of release.\n");
	sense1_status &= ~SENSE1_IS_OPEN;
	return 0;
}

static int sense1_proc_output(char *buf)
{
	int printlen = 0;
	int i;

	// insert proc debugging output here
	printlen =
	    sprintf(buf,
		    "Chumby sense1 driver version %s (bunnie@chumby.com)\n",
		    SENSE1_VERSION);

#if defined(CHUMBY_DIRECT_PORT_DIDDLE)
	printk( "%08x\n", readl(IO_ADDRESS(MY_GPIO_89_BASE + MY_GPIO_GPLR)) );
	for( i = 0; i < 20; i++ ) {
	  if( (i % 4) == 0 )
	    printk( "\n" );
	  printk( "%08x ", readl(IO_ADDRESS(MY_GPIO_89_BASE + i * 4)) );
	}
#endif
	buf += printlen;

	return printlen;
}

static int sense1_read_proc(char *page, char **start, off_t off,
			    int count, int *eof, void *data)
{
	int len = sense1_proc_output(page);
	if (len <= off + count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;

	return len;
}


static int chumby_sense1_open(struct inode *inode, struct file *file)
{
	// make sure we're not opened twice
	if (sense1_status & SENSE1_IS_OPEN)
		return -EBUSY;

	sense1_status |= SENSE1_IS_OPEN;
	return 0;
}

void sense1_task_record(unsigned long ptr)
{
	struct sense1data *data = (struct sense1data *)ptr;
	int state = 1;

	if (data->currentTransitionTime == 0xFFFFFFFF) {
		// handle roll-over case
		data->currentTransitionTime = 0;
		// this is cheesy but in practice this should work
		data->lastTransitionTime = 0;
	} else {
		data->currentTransitionTime++;
	}

	if (data->isDebounce) {
		// debouncing
		if (data->currentTransitionTime >
		    data->lastTransitionTime + gDebounce) {
#if defined(CHUMBY_DIRECT_PORT_DIDDLE)
			state = readl(IO_ADDRESS(MY_GPIO_89_BASE + MY_GPIO_GPLR)) & MY_GPIO_89_MASK ? 1 : 0;
#else

			if (g_gpio_button_request_success)
			{
                state = gpio_get_value_cansleep( BUTTON_GPIO );
				//printk( "[s=%x]", state );
			}

#endif
			if (state == data->lastTransitionState) {
				if (state == 0)	// 0 if switch is depressed
					data->bent = 1;
				else
					data->bent = 0;
			} else {
				// ignore, it was spurious
			}

			// reset our state machine
			data->lastTransitionTime = data->currentTransitionTime;
			data->lastTransitionState = state;
			data->isDebounce = 0;
		} else {
			// just wait until our turn comes
		}
	} else {
		// not a debounce period
#if defined(CHUMBY_DIRECT_PORT_DIDDLE)
		state = readl(IO_ADDRESS(MY_GPIO_89_BASE + MY_GPIO_GPLR)) & MY_GPIO_89_MASK ? 1 : 0;
#else
		if (g_gpio_button_request_success)
		{
            state = gpio_get_value_cansleep( BUTTON_GPIO );
			//printk( "[s=%x]", state );
		}

#endif
		if (state != data->lastTransitionState) {
			// something changed, enter the debounce state
			data->isDebounce = 1;
			data->lastTransitionState = state;
			data->lastTransitionTime = data->currentTransitionTime;
		} else {
			// else do nothing
		}
	}
#if 0
	// Handle LCD brightness control
	switch (gDimLevel) {
	case 0:		// on and bright
		imx_gpio_write(GPIO_PORTC | 28, 1);
		imx_gpio_write(GPIO_PORTKP | (2 + 8), 1);
		break;
	case 1:		// on and dim
		imx_gpio_write(GPIO_PORTC | 28, 0);
		imx_gpio_write(GPIO_PORTKP | (2 + 8), 1);
		break;
	case 2:		// off and (don't care, set to dim)
		imx_gpio_write(GPIO_PORTC | 28, 0);
		imx_gpio_write(GPIO_PORTKP | (2 + 8), 0);
		break;
	default:
		break;
	}

	// handle speaker muting
	if (gSpkMute == 1) {
		imx_gpio_write(GPIO_PORTKP | (4 + 8), 1);	// speakers off
	} else {
		imx_gpio_write(GPIO_PORTKP | (4 + 8), 0);	// speakers on
	}

	// handle headphone plug-in status reporting
	if (imx_gpio_read(GPIO_PORTKP | 3 | GPIO_IN))
		gHPin = 0;
	else
		gHPin = 1;
	// handle watchdog timer update, very important!
	WSR = 0x5555;
	WSR = 0xAAAA; // that should do it...
#endif

	// now handle retasking
	if (!gDone) {		// requires the done un-set to wait a few hundred ms to flush out the last timer added
		data->timer.expires += 1;	// next jiffie interval!
		add_timer(&data->timer);
	}
}

/*
  this driver handles the following sensors:

  * dimming level
  * speaker muting
  * DC voltage reading
  * battery voltage reading
  * headphone insertion status
  * bend sensor ('switch')

*/
static int __init chumby_sense1_init(void)
{
	dev_t dev = 0;
	int result, err;
	unsigned int last_val, last_val2, last_val3;
	int attempt;
	struct resource *res;
	int gpio84;
	int ret = 0;

	sense1task_data.sense1_cdev = cdev_alloc();

	// insert all device specific hardware initializations here
	printk("Chumby sensor suite 1 driver version %s initializing (bunnie@chumby.com)...\n", SENSE1_VERSION);

	/*
	 * Get a range of minor numbers to work with, asking for a dynamic
	 * major unless directed otherwise at load time.
	 */
	if (sense1_major) {
		dev = MKDEV(sense1_major, sense1_minor);
		result = register_chrdev_region(dev, sense1_nr_devs, "switch");
	} else {
		result = alloc_chrdev_region(&dev, sense1_minor, sense1_nr_devs, "switch");
		sense1_major = MAJOR(dev);
	}
	if (result < 0) {
		printk(KERN_WARNING "switch: can't get major %d\n",
		       sense1_major);
		return result;
	}

	printk( "Creating proc entries\n" );
	create_proc_read_entry("sense1", 0, 0, sense1_read_proc, NULL);

	cdev_init(sense1task_data.sense1_cdev, &sense1_fops);
	sense1task_data.sense1_cdev->owner = THIS_MODULE;
	sense1task_data.sense1_cdev->ops = &sense1_fops;
	err = cdev_add(sense1task_data.sense1_cdev, dev, 1);
	/* Fail gracefully if need be */
	if (err)
		printk(KERN_NOTICE "Error %d adding switch device\n", err);

	// put hardware initializations here

#if defined(CHUMBY_DIRECT_PORT_DIDDLE)
	printk( "initializing clocks\n" );
	APB12CLOCK_BASE = ioremap( PHYS_APB1CLOCK_BASE, 0x1000 );

	if (!APB12CLOCK_BASE)
	{
		printk( "ioremap(0x%08lx) failed\n", (unsigned long)PHYS_APB1CLOCK_BASE );
		return -ENODEV;
	}

	printk( "clock vm base address (APB1) = 0x%08lx from phys 0x%08lx\n", (unsigned long)APB12CLOCK_BASE, (unsigned long)PHYS_APB1CLOCK_BASE );

	printk( "turn on GPIO clocks\n" );

	// henry@chumby.com - much of this is copied from bunnie's code in
	// drivers/input/touchscreen/silvermoon-ts.c in gpios_init()

	writel(0x03,IO_ADDRESS(MY_APB1CLOCK_BASE + 0x808)); // APB2CLOCK
	writel(0x07,IO_ADDRESS(MY_APB1CLOCK_BASE + 0x808)); // APB2CLOCK
	writel(0x03,IO_ADDRESS(MY_APB1CLOCK_BASE + 0x808)); // APB2CLOCK

	writel(0x03,IO_ADDRESS(MY_APB1CLOCK_BASE + 0x008)); // APB1CLOCK
	writel(0x07,IO_ADDRESS(MY_APB1CLOCK_BASE + 0x008)); // APB1CLOCK
	writel(0x03,IO_ADDRESS(MY_APB1CLOCK_BASE + 0x008)); // APB1CLOCK
#endif

	// henry@chumby.com - we need to first request mem regions using the ??? API,
	// then we can request a physical to VM mapping

	// ------------ MFPR ---------------
#if defined(CHUMBY_DIRECT_PORT_DIDDLE)
	my_mfp.phys_base = PHYS_MFPR_BASE;
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

	MFPR_BASE = my_mfp.mmio_base;
#else
	// Handled in arch/arm/mach-pxa/aspenite.c
#endif

	// ------------- GPIO ---------------
#if defined(CHUMBY_DIRECT_PORT_DIDDLE)
	my_gpio.phys_base = PHYS_GPIO_BASE;
	my_gpio.size = 0x1A8;
	res = request_mem_region(my_gpio.phys_base, my_gpio.size, "gpio" );
	if( res == NULL ) {
		printk( "failed to get memory region for GPIO.\n" );
		goto err2;
	}

	my_gpio.phys_base = res->start;

	my_gpio.mmio_base = ioremap(res->start, res->end - res->start + 1);
	if( my_gpio.mmio_base == NULL ) {
		printk( "failed to ioremap GPIO area\n" );
		goto err5;
	}

	GPIO_BASE = my_gpio.mmio_base;

	writel(0xC0C0, IO_ADDRESS( MY_MFPR_BASE + MY_MFP89_OFF)); // setup the pin as an input
	writel(MY_GPIO_89_MASK, IO_ADDRESS(MY_GPIO_89_BASE + MY_GPIO_GCDR));  // make it an input

	// henry@chumby.com - get current direction for GPIO_84 and set to output
	last_val = readl( IO_ADDRESS( MY_GPIO_84_BASE + MY_GPIO_PLR ) ) & MY_GPIO_84_MASK;
	last_val2 = readl( IO_ADDRESS( MY_GPIO_84_BASE + MY_GPIO_PDR ) ) & MY_GPIO_84_MASK;
	last_val3 = readl( IO_ADDRESS( MY_MFPR_BASE + MY_MFP84_OFF ) );


#define	MFPR_PULL_SEL	(0x01<<15)
#define MFPR_PULLUP_EN	(0x01<<14)
#define MFPR_PULLDN_EN	(0x01<<13)
#define MFPR_AF_MASK	(0x07<<0)
	// Control pullup via PULLUP_EN and PULLDN_EN
	// Enable pullup
	// Disable pulldown
	writel( (last_val3 & ~(MFPR_PULL_SEL|MFPR_PULLUP_EN|MFPR_PULLDN_EN|MFPR_AF_MASK)) |
		MFPR_PULL_SEL |
		MFPR_PULLUP_EN |
		(0<<13) |
		(0<<0),
			IO_ADDRESS( MY_MFPR_BASE + MY_MFP84_OFF ) );
	printk( "Turned on bits %08lx in MFP84, was %08lx\n", MFPR_PULL_SEL|MFPR_PULLUP_EN, last_val3 );

	// Set MFP84 function to 0, enable pull-up resistor, disable pull-down resistor
	writel( MY_GPIO_84_MASK, IO_ADDRESS( MY_GPIO_84_BASE + MY_GPIO_SDR ) ); // Make it an output
	writel( MY_GPIO_84_MASK, IO_ADDRESS( MY_GPIO_84_BASE + MY_GPIO_PSR ) ); // Set it high
	printk( "set GPIO84 high, direction was %d and state was %d\n", last_val2, last_val );

	// Read back the values we just set
	last_val = readl( IO_ADDRESS( MY_GPIO_84_BASE + MY_GPIO_PLR ) ) & MY_GPIO_84_MASK;
	last_val2 = readl( IO_ADDRESS( MY_GPIO_84_BASE + MY_GPIO_PDR ) ) & MY_GPIO_84_MASK;
	last_val3 = readl( IO_ADDRESS( MY_MFPR_BASE + MY_MFP84_OFF ) );
	printk( "new GPIO84 pin level=%d, direction=%d, new MFPR84=%08lx\n", last_val, last_val2, last_val3 );

	// Retry 100 times
	for (attempt = 0; (last_val == 0 || last_val2 == 0) && attempt < 100; attempt++)
	{
		last_val = readl( IO_ADDRESS( MY_GPIO_84_BASE + MY_GPIO_PLR ) ) & MY_GPIO_84_MASK;
		last_val2 = readl( IO_ADDRESS( MY_GPIO_84_BASE + MY_GPIO_PDR ) ) & MY_GPIO_84_MASK;
		if (last_val && last_val2)
		{
			printk( "success after %d attempts\n", attempt );
			break;
		}
		if (attempt && (attempt % 10) == 0)
		{
			printk( "Attempt %d: still have plr=%d, pdr=%d\n", attempt, last_val, last_val2 );
		}

		if (attempt % 10 == 0)
		{
			// Try to set GPIO clocks
			writel(0x03,IO_ADDRESS(MY_APB1CLOCK_BASE + 0x808));
			writel(0x07,IO_ADDRESS(MY_APB1CLOCK_BASE + 0x808));
			writel(0x03,IO_ADDRESS(MY_APB1CLOCK_BASE + 0x808));

			//  printk( "attempt to turn on GPIO clocks, perspec addy\n" );
			writel(0x03,IO_ADDRESS(MY_APB1CLOCK_BASE + 0x008));
			writel(0x07,IO_ADDRESS(MY_APB1CLOCK_BASE + 0x008));
			writel(0x03,IO_ADDRESS(MY_APB1CLOCK_BASE + 0x008));
		}

		// Clear output first
		writel( MY_GPIO_84_MASK, IO_ADDRESS( MY_GPIO_84_BASE + MY_GPIO_CDR ) ); // Clear output flag (make it an input)
		writel( MY_GPIO_84_MASK, IO_ADDRESS( MY_GPIO_84_BASE + MY_GPIO_SDR ) ); // Make it an output
		writel( MY_GPIO_84_MASK, IO_ADDRESS( MY_GPIO_84_BASE + MY_GPIO_PCR ) ); // Clear it
		writel( MY_GPIO_84_MASK, IO_ADDRESS( MY_GPIO_84_BASE + MY_GPIO_PSR ) ); // Set it high
	}
	// Set direction on UART3
	last_val = readl( IO_ADDRESS( MY_MFPR_BASE + MY_MFP98_OFF ) );
	last_val2 = readl( IO_ADDRESS( MY_MFPR_BASE + MY_MFP99_OFF ) );

	// Alt fn 2 (UART3_RXD)
	writel( (last_val & ~(MFPR_AF_MASK)) |
		(2<<0),
			IO_ADDRESS( MY_MFPR_BASE + MY_MFP98_OFF ) );
	// Alt fn 2 (UART3_TXD)
	writel( (last_val2 & ~(MFPR_AF_MASK)) |
		(2<<0),
			IO_ADDRESS( MY_MFPR_BASE + MY_MFP99_OFF ) );
	printk( "Changed UART3 direction for 98/99, was %08lx / %08lx\n", last_val, last_val2 );
	last_val = readl( IO_ADDRESS( MY_MFPR_BASE + MY_MFP98_OFF ) );
	last_val2 = readl( IO_ADDRESS( MY_MFPR_BASE + MY_MFP99_OFF ) );
	printk( "New values: %08lx / %08lx\n", last_val, last_val2 );
#else
// No need to do this here - it is already being done in arch/arm/mach-pxa/aspenite.c
//#define LCD_PULLUP_INIT
#if defined(LCD_PULLUP_INIT)
	// Initialize using gpio
	printk( "%s() in %s:%d - Turning on LCD using gpio84\n", __FUNCTION__, __FILE__, __LINE__ );
	// Use API to set direction etc.
	gpio84 = 84;
	if (gpio_request( gpio84, "LCD_up" ))
	{
		printk( "Failed gpio request handle %d\n", gpio84 );
	}
	else
	{
		gpio_direction_output(gpio84, 0);
		mdelay(1000);
		gpio_direction_output(gpio84, 1);
		printk( "Changed direction for gpio84 (%d)\n", gpio84 );
		if (1)
		{
			printk( "Skipping set value\n" );
		}
		else
		{
			gpio_set_value( gpio84, 1 );
			printk( "Set value for gpio84\n" );
		}
	}
	gpio_free(gpio84);
#endif

	// Initialize sensor input

	if (gpio_request( BUTTON_GPIO, "BUTTON_GPIO" ))
	{
		printk( "failed gpio request %d\n", BUTTON_GPIO );
		goto err_button_gpio_request_failed;
	}
	else
	{
		g_gpio_button_request_success = 1;
		//export it only so we don't get a complaint when we free it later...
		gpio_export(BUTTON_GPIO, 0);

		ret = gpio_direction_input( BUTTON_GPIO );
		printk( "Changed gpio%d to input. ret = %d\n", BUTTON_GPIO, ret );

		ret = gpio_cansleep( BUTTON_GPIO );
		printk( "gpio_cansleep() returned: %d\n", ret );

	}

#endif

	/// end inits

#if USE_SYSCTL
	gSysCtlHeader = register_sysctl_table(gSysCtl, 0);
	if (gSysCtlHeader != NULL)
		gSysCtlHeader->ctl_table->child->de->owner = THIS_MODULE;
#endif

	///// initialize periodic task queue
	sense1task_data.bent = 0;
	sense1task_data.lastTransitionTime = jiffies;
	sense1task_data.isDebounce = 0;
	sense1task_data.currentTransitionTime = sense1task_data.lastTransitionTime;
	sense1task_data.lastTransitionState = 1;	// default state of the sensor

	init_timer(&sense1task_data.timer);
	sense1task_data.timer.data = (unsigned long)&sense1task_data;
	sense1task_data.timer.function = sense1_task_record;
	sense1task_data.timer.expires = jiffies + 1;
	add_timer(&sense1task_data.timer);

#if 0
	// initialize the watchdog timer module
	PCCR1 |= PCCR1_WDT_EN;
	WCR = 0x0430;  // set watchdog to 0x04 cycles (2 seconds) before trigger, plus force a system reset
	WCR = 0x0434;  // enable the watchdog timer! better have the update process running.

	resetReason = WRSR;
#endif

	return 0;

#if defined(CHUMBY_DIRECT_PORT_DIDDLE)
 err6:
  iounmap(my_gpio.mmio_base);
 err5:
  release_mem_region(my_gpio.phys_base, my_gpio.size);
// err4:
//  iounmap(my_gpioedge.mmio_base);
// err3:
//  release_mem_region(my_gpioedge.phys_base, my_gpioedge.size);
 err2:
  iounmap(my_mfp.mmio_base);
 err1:
  release_mem_region(my_mfp.phys_base, my_mfp.size);
#endif

 err_button_gpio_request_failed:

  return -ENODEV;
}

static ssize_t chumby_sense1_read(struct file *file, char *buf,
				  size_t count, loff_t * ppos)
{
	char retval = 0;
	size_t retlen = 1;

	retval = sense1task_data.bent;

	//	CHUMSENSE1_DEBUG(Trace, "sense1 read exit with: %d\n", retval);
	// printk("sense1 read exit with: %d\n", retval);
	copy_to_user(buf, &retval, retlen);

	return retlen;
}

static void __exit chumby_sense1_exit(void)
{
	dev_t devno = MKDEV(sense1_major, sense1_minor);

	CHUMSENSE1_DEBUG(Trace, "Top of exit.\n");
	// set global flag that we're out of here and force a call to remove ourselves
	gDone = 1;

	// delete our kernel task
	del_timer(&(sense1task_data.timer));

	// wait to dequeue self; if kernel panics on rmmod try adding 100 to this value
	mdelay(200);

	// insert all cleanup stuff here
	cdev_del(sense1task_data.sense1_cdev);
	remove_proc_entry("sense1", NULL);
	unregister_chrdev_region(devno, sense1_nr_devs);

#if USE_SYSCTL
	if (gSysCtlHeader != NULL)
		unregister_sysctl_table(gSysCtlHeader);
#endif

#if defined(CHUMBY_DIRECT_PORT_DIDDLE)
	// Free physical memory mapping
	iounmap(my_gpio.mmio_base);
	release_mem_region(my_gpio.phys_base, my_gpio.size);
	iounmap(my_mfp.mmio_base);
	release_mem_region(my_mfp.phys_base, my_mfp.size);
//	iounmap( MFPR_BASE );
//	iounmap( APB12CLOCK_BASE );
//	iounmap( GPIO_BASE );
#endif

	if (g_gpio_button_request_success)
	{
        gpio_free(BUTTON_GPIO);
	}

}

// entry and exit mappings
module_init(chumby_sense1_init);
module_exit(chumby_sense1_exit);
