/*
	$Id: s3c_regs.c 5097 2008-05-15 20:23:52Z henry $
	chumby_regs.c
	henry -- May 2008 -- cloned from chumby_sense1
	Mar 2009 - modified to expose Aspen registers

	Copyright (c) Chumby Industries, 2008-9

	The Chumby register interface driver is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License as published
	by the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	The chumby register interface driver is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with the Chumby; if not, write to the Free Software
	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#define CHUMBYREG_VERSION "2.3-Chumbyframe $Rev: 5097 $"

//#include <linux/config.h>
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
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <asm/arch/regs-timer.h>

#include "chumby_regs.h"

// following determines how frequently the s3c_regs_task_record() task will
// be called (Stormwind is configured at 200Hz, 5ms per jiffie)
#define TASK_JIFFY_DELTA    2

// following defines are used with PWM0 to control the brightness
#define PWM0_TCNT     66666 /*1000*/     // the base frequency
#define PWM0_TCMP    ((PWM0_TCNT*3)/4)  // maximum duty cycle


/*
 * basic parameters
 */

int s3cregs_major = 0;		// dynamic allocation
int s3cregs_minor = 0;
int s3cregs_nr_devs = 1;

module_param(s3cregs_major, int, S_IRUGO);
module_param(s3cregs_minor, int, S_IRUGO);
module_param(s3cregs_nr_devs, int, S_IRUGO);

MODULE_AUTHOR("henry@chumby.com");
MODULE_LICENSE("GPL");

// static data
static int gDone = 0;
//static unsigned long sense1_status = 0;	/* bitmapped status byte.       */

// Parent dir entry under /proc/driver (chumbyregs)
static struct proc_dir_entry *proc_s3regs = NULL;

// Dir entry under proc_s3regs for clock regs
static struct proc_dir_entry *proc_s3regs_clock = NULL;

// Dir entry under proc_s3regs for gpio and IIC regs
static struct proc_dir_entry *proc_s3regs_gpio = NULL;

// Physical base address for LCD I/O registers
static void __iomem *LCD_BASE;

// Physical base address for clock registers
static void __iomem *CLOCK_BASE;

// Physical base address for GPIO and IIC registers
static void __iomem *GPIO_BASE;

// Structure for defining registers
typedef struct _s3c_register_t
{
	const char *proc_entry_name; // lcd_window0_pos etc
	unsigned int reg_base; // Register base
} s3c_register_t;

// Array of registers. Must match following enum
static s3c_register_t s3cr[] = {
	{ "lcd_wincon0", S3C_WINCON0 }, // 0x0...
	{ "lcd_wincon1", S3C_WINCON1 }, // 0x0...
	{ "lcd_wincon2", S3C_WINCON2 }, // 0x0...
	//{ "lcd_winpos0", S3C_VIDOSD0A }, // ltx,lty,rbx,rby,a0r,a0g,a0b,a1r,a1g,a1b
	{ "lcd_winsize0", S3C_VIDOSD0C }, // This only has A, B, and C, no D
	{ "lcd_winpos1", S3C_VIDOSD1A },
		{ "lcd_vidosd1a", S3C_VIDOSD1A },
		{ "lcd_vidosd1b", S3C_VIDOSD1B },
		{ "lcd_vidosd1c", S3C_VIDOSD1C },
	{ "lcd_winsize1", S3C_VIDOSD1D },
	{ "lcd_winpos2", S3C_VIDOSD2A },
	{ "lcd_winsize2", S3C_VIDOSD2D },
	//{ "lcd_win0alpha0", S3C_VIDOSD0? }, // alpha doesn't exist for window 0
	{ "lcd_win1alpha0", S3C_VIDOSD1C }, // 0xrgb
	{ "lcd_win2alpha0", S3C_VIDOSD2C }, // 0xrgb
	//{ "lcd_win0alpha1", S3C_VIDOSD0? }, // alpha doesn't exist for window 0
	{ "lcd_win1alpha1", S3C_VIDOSD1C }, // 0xrgb
	{ "lcd_win2alpha1", S3C_VIDOSD2C }, // 0xrgb
	{ "lcdro_vidcon0", S3C_VIDCON0 },
	{ "lcdro_vidcon1", S3C_VIDCON1 },
	{ "lcdro_vidcon2", S3C_VIDCON2 },
	{ "lcd_vidtcon0", S3C_VIDTCON0 },
	{ "lcd_vidtcon1", S3C_VIDTCON1 },
	{ "lcd_vidtcon2", S3C_VIDTCON2 },
	{ "lcdro_vidw00add0b0", S3C_VIDW00ADD0B0 },
	{ "lcdro_vidw00add0b1", S3C_VIDW00ADD0B1 },
	{ "lcdro_vidw01add0b0", S3C_VIDW01ADD0B0 },
	{ "lcdro_vidw01add0b1", S3C_VIDW01ADD0B1 },
	{ "lcdro_vidw00add1b0", S3C_VIDW00ADD1B0 },
	{ "lcdro_vidw00add1b1", S3C_VIDW00ADD1B1 },
	{ "lcdro_vidw01add1b0", S3C_VIDW01ADD1B0 },
	{ "lcdro_vidw01add1b1", S3C_VIDW01ADD1B1 },
	{ "lcdro_vidw00add2", S3C_VIDW00ADD2 },
	{ "lcdro_vidw01add2", S3C_VIDW01ADD2 },
	{ "lcdro_vidw02add2", S3C_VIDW02ADD2 },
	{ "lcdro_vidw03add2", S3C_VIDW03ADD2 },
	{ "lcdro_vidw04add2", S3C_VIDW04ADD2 },
	{ "lcd_w1keycon0", S3C_W1KEYCON0 },
	{ "lcd_w1keycon1", S3C_W1KEYCON1 },
	{ "lcd_w2keycon0", S3C_W2KEYCON0 },
	{ "lcd_w2keycon1", S3C_W2KEYCON1 },
	{ "lcd_w3keycon0", S3C_W3KEYCON0 },
	{ "lcd_w3keycon1", S3C_W3KEYCON1 },
	{ "lcd_w4keycon0", S3C_W4KEYCON0 },
	{ "lcd_w4keycon1", S3C_W4KEYCON1 },
	{ "lcd_dithmode", S3C_DITHMODE },
	{ "lcd_win0map", S3C_WIN0MAP },
	{ "lcd_win1map", S3C_WIN1MAP },
	{ "lcd_win2map", S3C_WIN2MAP },
	{ "lcd_win3map", S3C_WIN3MAP },
	{ "lcd_win4map", S3C_WIN4MAP },
	{ "lcd_wpalcon", S3C_WPALCON },
	{ "lcd_trigcon", S3C_TRIGCON },
	{ "headphone_jack", 1 /* Bit in GPN */ },
	{ "infrared_data", 7 /* Bit in GPN */ },
};
#define S3CR_COUNT	(sizeof(s3cr)/sizeof(s3cr[0]))

// Must match s3cr[] definitions
enum _s3cr_idx {
	S3CRIDX_WINCON0 = 0,
	S3CRIDX_WINCON1,
	S3CRIDX_WINCON2,
	//S3CRIDX_WINPOS0,
	S3CRIDX_WINSIZE0,
	S3CRIDX_WINPOS1,
		S3CRIDX_VIDOSD1A,
		S3CRIDX_VIDOSD1B,
		S3CRIDX_VIDOSD1C,
	S3CRIDX_WINSIZE1,
	S3CRIDX_WINPOS2,
	S3CRIDX_WINSIZE2,
	//S3CRIDX_WIN0ALPHA0,
	S3CRIDX_WIN1ALPHA0,
	S3CRIDX_WIN2ALPHA0,
	//S3CRIDX_WIN0ALPHA1,
	S3CRIDX_WIN1ALPHA1,
	S3CRIDX_WIN2ALPHA1,
	S3CRIDXR_VIDCON0,
	S3CRIDXR_VIDCON1,
	S3CRIDXR_VIDCON2,
	S3CRIDX_VIDTCON0,
	S3CRIDX_VIDTCON1,
	S3CRIDX_VIDTCON2,
	S3CRIDXR_VIDW00ADD0B0,
	S3CRIDXR_VIDW00ADD0B1,
	S3CRIDXR_VIDW01ADD0B0,
	S3CRIDXR_VIDW01ADD0B1,
	S3CRIDXR_VIDW00ADD1B0,
	S3CRIDXR_VIDW00ADD1B1,
	S3CRIDXR_VIDW01ADD1B0,
	S3CRIDXR_VIDW01ADD1B1,
	S3CRIDXR_VIDW00ADD2,
	S3CRIDXR_VIDW01ADD2,
	S3CRIDXR_VIDW02ADD2,
	S3CRIDXR_VIDW03ADD2,
	S3CRIDXR_VIDW04ADD2,
	S3CRIDX_W1KEYCON0,
	S3CRIDX_W1KEYCON1,
	S3CRIDX_W2KEYCON0,
	S3CRIDX_W2KEYCON1,
	S3CRIDX_W3KEYCON0,
	S3CRIDX_W3KEYCON1,
	S3CRIDX_W4KEYCON0,
	S3CRIDX_W4KEYCON1,
	S3CRIDX_DITHMODE,
	S3CRIDX_WIN0MAP,
	S3CRIDX_WIN1MAP,
	S3CRIDX_WIN2MAP,
	S3CRIDX_WIN3MAP,
	S3CRIDX_WIN4MAP,
	S3CRIDX_WPALCON,
	S3CRIDX_TRIGCON,
	S3CRIDX_HEADPHONE_JACK,
	S3CRIDX_INFRARED_DATA,
};

// Structure for defining debug registers
typedef struct _s3c_debug_register_t
{
	struct proc_dir_entry **proc_dir;
	const char *proc_entry_name;
	void __iomem **base_addr;
	unsigned int reg_base;
} s3c_debug_register_t;

#define CLOCK_ENTRY(name,off)	{ &proc_s3regs_clock, name, &CLOCK_BASE, off }
#define GPIO_ENTRY(name,off)	{ &proc_s3regs_gpio, name, &GPIO_BASE, off }

// Debug registers need only be added here
static s3c_debug_register_t s3dbgr[] = {
	/**** From chapter 3 of s3c6400x user manual (page 104) ****/
	CLOCK_ENTRY( "APLL_LOCK", 		0xF000), // R/W Control PLL locking period for APLL 0x0000_FFFF
	CLOCK_ENTRY( "MPLL_LOCK", 		0xF004), // R/W Control PLL locking period for MPLL 0x0000_FFFF
	CLOCK_ENTRY( "EPLL_LOCK", 		0xF008), // R/W Control PLL locking period for EPLL 0x0000_FFFF
	CLOCK_ENTRY( "APLL_CON", 		0xF00C), // R/W Control PLL output frequency for APLL 0x0190_0302
	CLOCK_ENTRY( "MPLL_CON", 		0xF010), // R/W Control PLL output frequency for MPLL 0x0214_0603
	CLOCK_ENTRY( "EPLL_CON0", 		0xF014), // R/W Control PLL output frequency for EPLL 0x0020_0102
	CLOCK_ENTRY( "EPLL_CON1", 		0xF018), // R/W Control PLL output frequency for EPLL 0x0000_9111
	CLOCK_ENTRY( "CLK_SRC", 		0xF01C), // R/W Select clock source 0x0000_0000
	CLOCK_ENTRY( "CLK_DIV0", 		0xF020), // R/W Set clock divider ratio 0x0105_1000
	CLOCK_ENTRY( "CLK_DIV1", 		0xF024), // R/W Set clock divider ratio 0x0000_0000
	CLOCK_ENTRY( "CLK_DIV2", 		0xF028), // R/W Set clock divider ratio 0x0000_0000
	CLOCK_ENTRY( "CLK_OUT", 		0xF02C), // R/W Select clock output 0x0000_0000
	CLOCK_ENTRY( "HCLK_GATE", 		0xF030), // R/W Control HCLK clock gating 0xFFFF_FFFF
	CLOCK_ENTRY( "PCLK_GATE", 		0xF034), // R/W Control PCLK clock gating 0xFFFF_FFFF
	CLOCK_ENTRY( "SCLK_GATE", 		0xF038), // R/W Control SCLK clock gating 0xFFFF_FFFF
	CLOCK_ENTRY( "AHB_CON0", 		0xF100), // R/W Configure AHB I/P/X/F bus 0x0400_0000
	CLOCK_ENTRY( "AHB_CON1", 		0xF104), // R/W Configure AHB M1/M0/T1/T0 bus 0x0000_0000
	CLOCK_ENTRY( "AHB_CON2", 		0xF108), // R/W Configure AHB R/S1/S0 bus 0x0000_0000
	CLOCK_ENTRY( "SDMA_SEL", 		0xF110), // R/W Select secure DMA input 0x0000_0000
	CLOCK_ENTRY( "SW_RST", 			0xF114), // R/W Generate software reset 0x0000_0000
	CLOCK_ENTRY( "SYS_ID", 			0xF118), // R System ID for revision and pass 0x0000_0000
	CLOCK_ENTRY( "PRODUCT_ID",		0xFA1C), // R Product ID 0x3640_0202
	CLOCK_ENTRY( "MEM_SYS_CFG",		0xF120), // R/W Configure memory subsystem 0x0000_0080
	CLOCK_ENTRY( "QOS_OVERRIDE0",	0xF124), // R/W Override DMC0 QOS 0x0000_0000
	CLOCK_ENTRY( "QOS_OVERRIDE1",	0xF128), // R/W Override DMC1 QOS 0x0000_0000
	CLOCK_ENTRY( "MEM_CFG_STAT",	0xF12C), // R Memory subsystem setup status 0x0000_0000
	/**** From chapter 10 of s3c6400x user manual (page 271) ****/
	GPIO_ENTRY( "GPACON", 		0x8000 ),
	GPIO_ENTRY( "GPADAT", 		0x8004 ),
	GPIO_ENTRY( "GPAPUD", 		0x8008 ),
	GPIO_ENTRY( "GPACONSLP", 	0x800C ),
	GPIO_ENTRY( "GPAPUDSLP", 	0x8010 ),
	GPIO_ENTRY( "GPBCON", 		0x8020 ),
	GPIO_ENTRY( "GPBDAT", 		0x8024 ),
	GPIO_ENTRY( "GPBPUD", 		0x8028 ),
	GPIO_ENTRY( "GPBCONSLP", 	0x802C ),
	GPIO_ENTRY( "GPBPUDSLP", 	0x8030 ),
	GPIO_ENTRY( "GPCCON", 		0x8040 ),
	GPIO_ENTRY( "GPCDAT", 		0x8044 ),
	GPIO_ENTRY( "GPCPUD", 		0x8048 ),
	GPIO_ENTRY( "GPCCONSLP", 	0x804C ),
	GPIO_ENTRY( "GPCPUDSLP", 	0x8050 ),
	GPIO_ENTRY( "GPDCON", 		0x8060 ),
	GPIO_ENTRY( "GPDDAT", 		0x8064 ),
	GPIO_ENTRY( "GPDPUD", 		0x8068 ),
	GPIO_ENTRY( "GPDCONSLP", 	0x806C ),
	GPIO_ENTRY( "GPDPUDSLP", 	0x8070 ),
	GPIO_ENTRY( "GPECON", 		0x8080 ),
	GPIO_ENTRY( "GPEDAT", 		0x8084 ),
	GPIO_ENTRY( "GPEPUD", 		0x8088 ),
	GPIO_ENTRY( "GPECONSLP", 	0x808C ),
	GPIO_ENTRY( "GPEPUDSLP", 	0x8090 ),
	GPIO_ENTRY( "GPFCON", 		0x80A0 ),
	GPIO_ENTRY( "GPFDAT", 		0x80A4 ),
	GPIO_ENTRY( "GPFPUD", 		0x80A8 ),
	GPIO_ENTRY( "GPFCONSLP", 	0x80AC ),
	GPIO_ENTRY( "GPFPUDSLP", 	0x80B0 ),
	GPIO_ENTRY( "GPGCON",		0x80C0 ),
	GPIO_ENTRY( "GPGDAT",		0x80C4 ),
	GPIO_ENTRY( "GPGPUD",		0x80C8 ),
	GPIO_ENTRY( "GPGECONSLP",	0x80CC ),
	GPIO_ENTRY( "GPGPUDSLP",	0x80D0 ),
	GPIO_ENTRY( "GPHCON0",		0x80E0 ),
	GPIO_ENTRY( "GPHCON1",		0x80E4 ),
	GPIO_ENTRY( "GPHDAT",		0x80E8 ),
	GPIO_ENTRY( "GPHPUD",		0x80EC ),
	GPIO_ENTRY( "GPHCONSLP",	0x80F0 ),
	GPIO_ENTRY( "GPHPUDSLP",	0x80F4 ),
	GPIO_ENTRY( "GPICON",		0x8100 ),
	GPIO_ENTRY( "GPIDAT",		0x8104 ),
	GPIO_ENTRY( "GPIPUD",		0x8108 ),
	GPIO_ENTRY( "GPICONSLP",	0x810C ),
	GPIO_ENTRY( "GPIPUDSLP",	0x8110 ),
	GPIO_ENTRY( "GPJCON",		0x8120 ),
	GPIO_ENTRY( "GPJDAT",		0x8124 ),
	GPIO_ENTRY( "GPJPUD",		0x8128 ),
	GPIO_ENTRY( "GPJCONSLP",	0x812C ),
	GPIO_ENTRY( "GPJPUDSLP",	0x8130 ),
	GPIO_ENTRY( "GPKCON0",		0x8800 ),
	GPIO_ENTRY( "GPKCON1",		0x8804 ),
	GPIO_ENTRY( "GPKDAT",		0x8808 ),
	GPIO_ENTRY( "GPKPUD",		0x880C ),
	GPIO_ENTRY( "GPLCON0",		0x8810 ),
	GPIO_ENTRY( "GPLCON1",		0x8814 ),
	GPIO_ENTRY( "GPLDAT",		0x8818 ),
	GPIO_ENTRY( "GPLPUD",		0x881C ),
	GPIO_ENTRY( "GPMCON",		0x8820 ),
	GPIO_ENTRY( "GPMDAT",		0x8824 ),
	GPIO_ENTRY( "GPMPUD",		0x8828 ),
	GPIO_ENTRY( "GPNCON",		0x8830 ),
	GPIO_ENTRY( "GPNDAT",		0x8834 ),
	GPIO_ENTRY( "GPNPUD",		0x8838 ),
	GPIO_ENTRY( "GPOCON",		0x8140 ),
	GPIO_ENTRY( "GPODAT",		0x8144 ),
	GPIO_ENTRY( "GPOPUD",		0x8148 ),
	GPIO_ENTRY( "GPOCONSLP",	0x814C ),
	GPIO_ENTRY( "GPOPUDSLP",	0x8150 ),
	GPIO_ENTRY( "GPPCON",		0x8160 ),
	GPIO_ENTRY( "GPPDAT",		0x8164 ),
	GPIO_ENTRY( "GPPPUD",		0x8168 ),
	GPIO_ENTRY( "GPPCONSLP",	0x816C ),
	GPIO_ENTRY( "GPPPUDSLP",	0x8170 ),
	/**** From chapter 36 of s3c6400x user manual (page 1088) ****/
	GPIO_ENTRY( "IIS0CON", 		0x2000 ),
	GPIO_ENTRY( "IIS1CON", 		0x3000 ),
	GPIO_ENTRY( "IIS0MOD", 		0x2004 ),
	GPIO_ENTRY( "IIS1MOD", 		0x3004 ),
	GPIO_ENTRY( "IIS0FIC", 		0x2008 ),
	GPIO_ENTRY( "IIS1FIC", 		0x3008 ),
	GPIO_ENTRY( "IIS0PSR", 		0x200c ),
	GPIO_ENTRY( "IIS1PSR", 		0x300c ),
};
#define S3C_DEBUG_REGISTER_COUNT (sizeof(s3dbgr)/sizeof(s3dbgr[0]))

/*
 * Sense1 sensor data logs, tracked by tasks that are scheduled by the
 * task scheduler
 */

#define FIXEDPOINT_NORM 1000

#define USE_SYSCTL  1

#define SENSE1_BLOCKING 0	// turn off blocking read on sense1 sensor

#if 1
#   ifdef USE_SYSCTL
#       define CHUMSENSE1_DEBUG( flag, fmt, args... ) do { if ( gDebug ## flag ) printk( "%s: " fmt, __FUNCTION__ , ## args ); } while (0)
#   else
#       define CHUMSENSE1_DEBUG( flag, fmt, args... ) printk( "%s: " fmt, __FUNCTION__ , ## args )
#   endif
#else
#   define CHUMSENSE1_DEBUG( flag, fmt, args... )
#endif

// function protos
static int s3c_regs_open(struct inode *inode, struct file *file);
static int s3c_regs_release(struct inode *inode, struct file *file);
static ssize_t s3c_regs_read(struct file *file, char *buf,
				  size_t count, loff_t * ppos);
static ssize_t s3c_regs_write(struct file *file, const unsigned char *buf,
                               size_t count, loff_t *ppos );
//static unsigned int s3c_regs_poll(struct file * filp, struct poll_table_struct * wait);

#if USE_SYSCTL

// proc debug structure
static int gDebugTrace = 0;
static int gDebugIoctl = 0;
static int gDebugError = 1;
static unsigned int gSpkMute = 0;
static int gDCmillivolts = -1;
static int gBTmillivolts = -1;
static unsigned int gDimLevel = 0;
static unsigned int gBrightness = PWM0_TCMP;
static unsigned int gHPin = 0;
static unsigned int gDebounce = 10;
static unsigned int gResetSerial = 0;

static unsigned short resetReason = 0;

static struct s3c_data_struct {
    struct timer_list timer;
} s3c_data;


// create /proc/sys/debug-trace, etc...which can be written to set behavior of globals
// in this driver on the fly
static struct ctl_table_header *gSysCtlHeader;
static struct ctl_table gSysCtlChumsense1[] = {
	{CTL_CHUMSENSE1_DEBUG_TRACE, "dbg-trace", &gDebugTrace, sizeof(int), 0644, NULL, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_DEBUG_IOCTL, "dbg-ioctl", &gDebugIoctl, sizeof(int), 0644, NULL, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_DEBUG_ERROR, "dbg-error", &gDebugError, sizeof(int), 0644, NULL, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_SPKMUTE, "spkmute", &gSpkMute, sizeof(unsigned int), 0644, NULL, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_DIMLEVEL, "dimlevel", &gDimLevel, sizeof(unsigned int), 0644, NULL, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_DCPOWER, "DCvolts", &gDCmillivolts, sizeof(unsigned int), 0644, NULL, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_BTPOWER, "BTvolts", &gBTmillivolts, sizeof(unsigned int), 0644, NULL, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_HPIN, "hpin", &gHPin, sizeof(unsigned int), 0644, NULL, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_DEBOUNCE, "debounce", &gDebounce, sizeof(unsigned int), 0644, NULL, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_RESETSERIAL, "resetSerial", &gResetSerial, sizeof(unsigned int), 0644, NULL, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_BRIGHTNESS, "brightness", &gBrightness, sizeof(unsigned int), 0644, NULL, NULL, &proc_dointvec},
	{0}
};

static struct ctl_table gSysCtl[] = {
	{CTL_CHUMSENSE1, "s3c_regs", NULL, 0, 0755, gSysCtlChumsense1},
	{0}
};
#endif				// USE_SYSCTL

// map into the generic driver infrastructure
static struct file_operations sense1_fops = {
	.owner =	THIS_MODULE,
//	.llseek =	s3c_regs_llseek,
	.read =		s3c_regs_read,
//	.poll =		s3c_regs_poll,
//	.ioctl =	s3c_regs_ioctl,
	.open =		s3c_regs_open,
	.release =	s3c_regs_release,
	.write =	s3c_regs_write,
//	.fasync =	chubmy_sense1_fasync,
};

///////////// code /////////////

static unsigned long xtol( char **s )
{
	static char hexDigits[] = "00112233445566778899aAbBcCdDeEfF";
	unsigned long u = 0;
	while (**s)
	{
		char *f = strchr( hexDigits, **s );
		if (!f) break;
		u = (u << 4) | ((f-hexDigits) >> 1);
		(*s)++;
	}
	return u;
}

static int s3c_regs_release(struct inode *inode, struct file *file)
{
	CHUMSENSE1_DEBUG(Trace, "Top of release.\n");
//	sense1_status &= ~SENSE1_IS_OPEN;
	return 0;
}

static int s3c_regs_proc_output(char *buf)
{
	int printlen = 0;

	// insert proc debugging output here
	printlen =
	    sprintf(buf,
		    "s3c register driver version %s (henry@chumby.com)\n",
		    S3CREG_VERSION);

	return printlen;
}

// Read register as an int value, return bytes written
static int s3c_regs_read_register( int reg_idx, char *buf )
{
	int printlen;
	if (reg_idx < 0 || reg_idx >= S3CR_COUNT)
	{
		return sprintf( buf, "s3c_regs_read_register[%d] invalid index\n", reg_idx );
	}
	printlen = 0;
	//printlen += sprintf( buf, "register[%d] %s (%x) = %u\n",
	//	reg_idx, s3cr[reg_idx].proc_entry_name, s3cr[reg_idx].reg_base, 0 );
	//buf += printlen;
	// Add anything else we want and update printlen
	switch (reg_idx)
	{
		// If we did handle WINPOS0, we'd have to deal with a different set of offsets...
		//case S3CRIDX_WINPOS0:
		//	break;
		case S3CRIDX_WINPOS1:
		case S3CRIDX_WINPOS2:
			{
				unsigned int u_topleft = readl( S3C_ADDR( s3cr[reg_idx].reg_base ) );
				unsigned int u_bottomright = readl( S3C_ADDR( s3cr[reg_idx].reg_base + 4 ) );
				unsigned int u_alpha = readl( S3C_ADDR( s3cr[reg_idx].reg_base + 8 ) );
				printlen += sprintf( buf,
					"%u,%u,%u,%u"
						",%u,%u,%u"
							",%u,%u,%u\n",
					(u_topleft & S3C_VIDOSD_LeftTopX_F) >> 11,
					u_topleft & S3C_VIDOSD_LeftTopY_F,
					(u_bottomright & S3C_VIDOSD_RightBotX_F) >> 11,
					u_bottomright & S3C_VIDOSD_RightBotY_F,
						(u_alpha & S3C_VIDOSD_ALPHA0_R) >> (12+8),
						(u_alpha & S3C_VIDOSD_ALPHA0_G) >> (12+4),
						(u_alpha & S3C_VIDOSD_ALPHA0_B) >> (12+0),
							(u_alpha & S3C_VIDOSD_ALPHA1_R) >> 8,
							(u_alpha & S3C_VIDOSD_ALPHA1_G) >> 4,
							(u_alpha & S3C_VIDOSD_ALPHA1_B) >> 0
							);
			}
			break;
		case S3CRIDX_WINSIZE0:
		case S3CRIDX_WINSIZE1:
		case S3CRIDX_WINSIZE2:
			printlen += sprintf( buf, "0x%x\n", S3C_VIDOSD_SIZE & readl( S3C_ADDR( s3cr[reg_idx].reg_base ) ) );
			break;
		case S3CRIDX_WIN1ALPHA0:
		case S3CRIDX_WIN2ALPHA0:
			printlen += sprintf( buf, "0x%x\n", 0xfff & (readl( S3C_ADDR( s3cr[reg_idx].reg_base ) ) >> 12) );
			break;
		case S3CRIDX_WIN1ALPHA1:
		case S3CRIDX_WIN2ALPHA1:
			printlen += sprintf( buf, "0x%x\n", 0xfff & (readl( S3C_ADDR( s3cr[reg_idx].reg_base ) ) >> 0) );
			break;
		case S3CRIDX_HEADPHONE_JACK:
		case S3CRIDX_INFRARED_DATA:
			{
				unsigned int gpndat;
				gpndat = readl( GPIO_BASE + 0x8834 );
				gpndat &= (0x01 << s3cr[reg_idx].reg_base);
				printlen += sprintf( buf, "%u", (gpndat >> s3cr[reg_idx].reg_base) );
			}
			break;
		// case S3CRIDX_WINCON0: etc.
		default:
			printlen += sprintf( buf, "0x%x\n", readl( S3C_ADDR( s3cr[reg_idx].reg_base ) ) );
			break;
	}
	return printlen;
}

static int s3c_regs_read_proc(char *page, char **start, off_t off,
			    int count, int *eof, void *data)
{
	int len = s3c_regs_read_register( (int)data, page );
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

static int s3c_debugregs_read_proc( char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	int len, idx;
	idx = (int)data;
	if (idx < 0 || idx >= S3C_DEBUG_REGISTER_COUNT)
	{
		len = sprintf( page, "%s() - invalid index %d\n", __FUNCTION__, idx );
	}
	else
	{
		unsigned int u;
		// PRODUCT_ID requires a write before it can be read
		if (s3dbgr[idx].reg_base == 0xFA1C)
		{
			static int hasBeenWritten = 0;
			if (!hasBeenWritten)
			{
				writel( 0, s3dbgr[idx].reg_base + *s3dbgr[idx].base_addr );
				hasBeenWritten = 1;
			}
		}
		u = readl( s3dbgr[idx].reg_base + *s3dbgr[idx].base_addr );
		len = sprintf( page, "0x%x (%u)\n", u, u );
	}
	if (len <= off + count)
	{
		*eof = 1;
	}
	*start = page + off;
	len -= off;
	if (len > count)
	{
		len = count;
	}
	if (len < 0)
	{
		len = 0;
	}
	return len;
}

static int s3c_regs_write_register( int reg_idx, char *buf, int len )
{
	int shiftVal;
	if (reg_idx < 0 || reg_idx >= S3CR_COUNT)
	{
		printk( "s3c_regs_write_register[%d] invalid index\n", reg_idx );
		return -EFAULT;
	}
	shiftVal = 0;
	switch (reg_idx)
	{
		// If we did handle WINPOS0, we'd have to deal with a different set of offsets...
		//case S3CRIDX_WINPOS0:
		//	break;
		case S3CRIDX_WINPOS1:
		case S3CRIDX_WINPOS2:
			{
				unsigned int u_topleft = readl( S3C_ADDR( s3cr[reg_idx].reg_base ) );
				unsigned int u_bottomright = readl( S3C_ADDR( s3cr[reg_idx].reg_base + 4 ) );
				unsigned int u_alpha = readl( S3C_ADDR( s3cr[reg_idx].reg_base + 8 ) );
				unsigned int u_new_topleft_x, u_new_topleft_y, u_new_bottomright_x, u_new_bottomright_y,
					u_new_alpha0_r, u_new_alpha0_g, u_new_alpha0_b,
					u_new_alpha1_r, u_new_alpha1_g, u_new_alpha1_b;
				unsigned int u_new_topleft, u_new_bottomright, u_new_alpha;
				int num_scanned = sscanf( buf,
					"%u,%u,%u,%u"
						",%u,%u,%u"
							",%u,%u,%u\n",
					&u_new_topleft_x, &u_new_topleft_y, &u_new_bottomright_x, &u_new_bottomright_y,
						&u_new_alpha0_r, &u_new_alpha0_g, &u_new_alpha0_b,
							&u_new_alpha1_r, &u_new_alpha1_g, &u_new_alpha1_b
							);
				if (num_scanned != 4 + 3 + 3)
				{
					printk( "s3c_regs_write_register[%d] insufficient args (%d/10)\n", reg_idx, num_scanned );
					return -EFAULT;
				}
				// Reassemble
				u_new_topleft = (u_new_topleft_x & S3C_VIDOSD_XYMask) << 11;
				u_new_topleft |= (u_new_topleft_y & S3C_VIDOSD_XYMask);
				u_new_bottomright = (u_new_bottomright_x & S3C_VIDOSD_XYMask) << 11;
				u_new_bottomright |= (u_new_bottomright_y & S3C_VIDOSD_XYMask);
				u_new_alpha = 0
					| ((u_new_alpha0_r & 0x0f) << (12+8))
					| ((u_new_alpha0_g & 0x0f) << (12+4))
					| ((u_new_alpha0_b & 0x0f) << (12+0))
					| ((u_new_alpha1_r & 0x0f) << 8)
					| ((u_new_alpha1_g & 0x0f) << 4)
					| ((u_new_alpha1_b & 0x0f) << 0)
					;
				if (u_new_topleft != u_topleft || u_new_bottomright != u_bottomright || u_new_alpha != u_alpha)
				{
					printk( "s3c_regs_write_register[%d] old %x %x %x -> new %x %x %x\n",
						reg_idx,
						u_topleft, u_bottomright, u_alpha,
						u_new_topleft, u_new_bottomright, u_new_alpha );
					writel( u_new_topleft, S3C_ADDR( s3cr[reg_idx].reg_base ) );
					writel( u_new_bottomright, S3C_ADDR( s3cr[reg_idx].reg_base + 4 ) );
					writel( u_new_alpha, S3C_ADDR( s3cr[reg_idx].reg_base + 8 ) );
				}
				else
				{
					printk( "s3c_regs_write_register[%d] unchanged %x %x %x\n",
						reg_idx,
						u_topleft, u_bottomright, u_alpha );
				}
			}
			break;
		case S3CRIDX_HEADPHONE_JACK:
		case S3CRIDX_INFRARED_DATA:
			{
				unsigned int enable;
				unsigned int gpncon;
				unsigned int gpnpud;
				if (sscanf( buf, "%u\n", &enable ) != 1)
				{
					printk( "%s() [%s:%d] failed to get enable/disable\n", __FUNCTION__, __FILE__, __LINE__ );
					return -EFAULT;
				}
				gpncon = readl( GPIO_BASE + 0x8830 );
				gpnpud = readl( GPIO_BASE + 0x8838 );
				if (enable)
				{
					// Configure as input
					gpncon &= (1 << s3cr[reg_idx].reg_base);
					gpnpud &= ~(3 << (2*s3cr[reg_idx].reg_base) );
					gpnpud |= (((enable>>4) & 0x03) << (2*s3cr[reg_idx].reg_base));
					writel( gpncon, GPIO_BASE + 0x8830 );
					writel( gpnpud, GPIO_BASE + 0x8838 );
					printk( "%s() [%s:%d] GPN%d enabled with pullup %d\n", __FUNCTION__, __FILE__, __LINE__, s3cr[reg_idx].reg_base, enable>>4 );
				}
				else
				{
					// This just resets to input
					gpncon &= ~(1 << s3cr[reg_idx].reg_base);
					writel( gpncon, GPIO_BASE + 0x8830 );
					// Turn off pull up / pull down
					gpnpud &= ~(0x03 << (2*s3cr[reg_idx].reg_base));
					writel( gpncon, GPIO_BASE + 0x8838 );
					printk( "%s() [%s:%d] GPN%d disabled\n", __FUNCTION__, __FILE__, __LINE__, s3cr[reg_idx].reg_base );
				}
			}
			break;
		// Read-only registers
		case S3CRIDXR_VIDCON0:
		case S3CRIDXR_VIDCON1:
		case S3CRIDXR_VIDCON2:
		case S3CRIDXR_VIDW00ADD0B0:
		case S3CRIDXR_VIDW00ADD0B1:
		case S3CRIDXR_VIDW01ADD0B0:
		case S3CRIDXR_VIDW01ADD0B1:
		case S3CRIDXR_VIDW00ADD1B0:
		case S3CRIDXR_VIDW00ADD1B1:
		case S3CRIDXR_VIDW01ADD1B0:
		case S3CRIDXR_VIDW01ADD1B1:
		case S3CRIDXR_VIDW00ADD2:
		case S3CRIDXR_VIDW01ADD2:
		case S3CRIDXR_VIDW02ADD2:
		case S3CRIDXR_VIDW03ADD2:
		case S3CRIDXR_VIDW04ADD2:
			printk( "s3c_regs_write_register[%d] - read-only register\n", reg_idx );
			break;
		case S3CRIDX_WIN1ALPHA0:
		case S3CRIDX_WIN2ALPHA0:
			shiftVal = 12;
			// and fall through...
		case S3CRIDX_WIN1ALPHA1:
		case S3CRIDX_WIN2ALPHA1:
			{
				unsigned int u, olducomp;
				unsigned int oldu = readl( S3C_ADDR( s3cr[reg_idx].reg_base ) );
				int num_scanned;
				if (buf[0] != '0' || (buf[1] != 'x' && buf[1] != 'X'))
				{
					num_scanned = sscanf( &buf[0], "%u\n", &u );
				}
				else
				{
					num_scanned = sscanf( &buf[2], "%x\n", &u );
				}
				if (num_scanned < 1)
				{
					printk( "s3c_regs_write_register[%d] insufficient args (%d/1)\n", reg_idx, num_scanned );
					return -EFAULT;
				}
				u &= 0xfff;
				u <<= shiftVal;
				olducomp = (oldu & ~(0xfff << shiftVal));
				if ((u | olducomp) != oldu)
				{

					printk( "s3c_regs_write_register[%d] changed 0x%x -> 0x%x\n", reg_idx, oldu, u | olducomp );
					writel( u | olducomp, S3C_ADDR( s3cr[reg_idx].reg_base ) );
				}
				else
				{
					printk( "s3c_regs_write_register[%d] no change from 0x%x\n", reg_idx, oldu );
				}
			}
			break;
		// case S3CRIDX_WINCON0: etc.
		default:
			{
				unsigned int u, oldu, olducomp;
				int num_scanned;
				if (buf[0] != '0' || (buf[1] != 'x' && buf[1] != 'X'))
				{
					num_scanned = sscanf( &buf[0], "%u\n", &u );
				}
				else
				{
					num_scanned = sscanf( &buf[2], "%x\n", &u );
				}
				if (num_scanned < 1)
				{
					printk( "s3c_regs_write_register[%d] insufficient args (%d/1)\n", reg_idx, num_scanned );
					return -EFAULT;
				}
				olducomp = oldu = readl( S3C_ADDR( s3cr[reg_idx].reg_base ) );
				if (reg_idx == S3CRIDX_WINSIZE0
					|| reg_idx == S3CRIDX_WINSIZE1
					|| reg_idx == S3CRIDX_WINSIZE2)
				{
					u &= S3C_VIDOSD_SIZE;
					olducomp &= S3C_VIDOSD_SIZE;
					// Preserve reserved bits
					oldu &= ~S3C_VIDOSD_SIZE;
				}
				else
				{
					oldu = 0;
				}
				if (u != olducomp)
				{
					printk( "s3c_regs_write_register[%d] changed 0x%x -> 0x%x\n", reg_idx, olducomp, u );
					writel( u | oldu, S3C_ADDR( s3cr[reg_idx].reg_base ) );
				}
				else
				{
					printk( "s3c_regs_write_register[%d] no change from 0x%x\n", reg_idx, olducomp );
				}
			}
			break;
	}
	return 0;
}

static int s3c_regs_write_proc(struct file *file,
			     const char *buffer,
			     unsigned long count,
			     void *data)
{
	int len = count;
	char buff[256];
	if(count > sizeof(buff)-1)
		len = sizeof(buff)-1;

	if(copy_from_user(buff, buffer, len))
		return -EFAULT;

	buff[len] = '\0';

	s3c_regs_write_register( (int)data, buff, len );

	return len;
}

static int s3c_debugregs_write_proc( struct file *file,
				const char *buffer, unsigned long count, void *data)
{
	printk( "%s() - writing to debug registers not supported\n", __FUNCTION__ );
	return -EFAULT;
}

static int s3c_regs_open(struct inode *inode, struct file *file)
{
	// make sure we're not opened twice
	/*****
	if (sense1_status & SENSE1_IS_OPEN)
		return -EBUSY;

	sense1_status |= SENSE1_IS_OPEN;
	*****/
	return 0;
}

static void adjust_brightness(unsigned long tcmp)
{
    if (tcmp >= PWM0_TCNT)
        tcmp = PWM0_TCNT-1;
    __raw_writel(tcmp, S3C_TCMPB(/*chan*/0));
    gBrightness = tcmp;
}

static void initial_brightness(void)
{
	s3c6400_timer_setup(/*chan*/0, /*unused*/0, PWM0_TCNT, gBrightness); 
}

void s3c_regs_task_record(unsigned long ptr)
{
	#if 1
    struct s3c_data_struct *data = (struct s3c_data_struct *)ptr;
	unsigned long brightness = -1;
	if (brightness != gBrightness) {
		brightness = gBrightness;
		adjust_brightness(brightness);
	}
	// now handle retasking
	if (!gDone) {
		data->timer.expires += TASK_JIFFY_DELTA;
		add_timer(&data->timer);
	}
	#else
	// OLD chumby-imx stuff...
	struct sense1data *data = (struct sense1data *)ptr;
	int state = 0;

	if (gResetSerial) {
		gResetSerial = 0;
		UCR2(IMX_UART3_BASE) &= 0xFFFFFFFE;
	}

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
			state = imx_gpio_read(GPIO_PORTKP | 0);

			if (state == data->lastTransitionState) {
				if ((state & 1) == 0)	// 0 if switch is depressed
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
		state = imx_gpio_read(GPIO_PORTKP | 0);	// grab the current state
		if (state != data->lastTransitionState) {
			// something changed, enter the debounce state
			data->isDebounce = 1;
			data->lastTransitionState = state;
			data->lastTransitionTime = data->currentTransitionTime;
		} else {
			// else do nothing
		}
	}

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

	// now handle retasking
	if (!gDone) {		// requires the done un-set to wait a few hundred ms to flush out the last timer added
		data->timer.expires += 1;	// next jiffie interval!
		add_timer(&data->timer);
	}
	#endif
}

/*
  this driver handles the following registers:

  * frame buffer overlay alpha

*/
static int __init s3c_regs_init(void)
{
	dev_t dev = 0;
	int result, err;
	struct proc_dir_entry *pde;
	int n;
	int entriesAdded = 0;

//	sense1task_data.sense1_cdev = cdev_alloc();

	// insert all device specific hardware initializations here
	printk("chumby register driver version %s initializing (henry@chumby.com)...\n", S3CREG_VERSION);

	// Create proc dir s3cregs
//	create_proc_read_entry("s3cregs", 0, 0, s3c_regs_read_proc, NULL);
//	create_proc_read_entry("resetReason", 0, 0, resetReason_proc, NULL );
	if (proc_s3regs == NULL)
	{
		proc_s3regs = proc_mkdir("chumbyregs", proc_root_driver);
		if (proc_s3regs == NULL)
		{
			return -EBUSY;
		}
	}

	// Create other dirs
	if (proc_s3regs_clock == NULL)
	{
		if ((proc_s3regs_clock = proc_mkdir( "clock", proc_s3regs )) == NULL)
		{
			return -EBUSY;
		}
	}
	if (proc_s3regs_gpio == NULL)
	{
		if ((proc_s3regs_gpio = proc_mkdir( "gpio", proc_s3regs )) == NULL)
		{
			return -EBUSY;
		}
	}

	// Get base addresses
	LCD_BASE = ioremap( S3C_LCD_PHYS_BASE, S3C_LCD_PHYS_RANGE );
	CLOCK_BASE = ioremap( S3C_CLOCK_PHYS_BASE, S3C_CLOCK_PHYS_RANGE );
	GPIO_BASE = ioremap( S3C_GPIO_PHYS_BASE, S3C_GPIO_PHYS_RANGE );

	for (n = 0; n < S3CR_COUNT; n++)
	{
		pde = create_proc_entry(
			s3cr[n].proc_entry_name,
			S_IFREG | S_IWUSR | S_IRUSR,
			proc_s3regs
			);
		if (!pde)
		{
			printk( "s3regs_init - failed to create %s in /proc/driver/chumbyregs\n", s3cr[n].proc_entry_name );
			continue;
		}
		entriesAdded++;
		pde->owner = THIS_MODULE;
		pde->read_proc = s3c_regs_read_proc;
		pde->write_proc = s3c_regs_write_proc;
		pde->data = (void*)n;
	}
	printk( "s3regs_init - %d/%d entries added\n", entriesAdded, S3CR_COUNT );

	entriesAdded = 0;
	// Debug registers
	for (n = 0; n < S3C_DEBUG_REGISTER_COUNT; n++)
	{
		pde = create_proc_entry(
			s3dbgr[n].proc_entry_name,
			S_IFREG | S_IWUSR | S_IRUSR,
			*s3dbgr[n].proc_dir
			);
		if (!pde)
		{
			printk( "%s() - failed to create %s for debug\n", __FUNCTION__, s3dbgr[n].proc_entry_name );
			continue;
		}
		entriesAdded++;
		pde->owner = THIS_MODULE;
		pde->read_proc = s3c_debugregs_read_proc;
		pde->write_proc = s3c_debugregs_write_proc;
		pde->data = (void*)n;
	}
	printk( "%s() - %d/%d debug entries added\n", __FUNCTION__, entriesAdded, S3C_DEBUG_REGISTER_COUNT );

#if USE_SYSCTL
	gSysCtlHeader = register_sysctl_table(gSysCtl);
//	if (gSysCtlHeader != NULL)
//		gSysCtlHeader->ctl_table->child->owner = THIS_MODULE;
#endif

	initial_brightness();

    init_timer(&s3c_data.timer);
    s3c_data.timer.data = (unsigned long) &s3c_data;
    s3c_data.timer.function = s3c_regs_task_record;
    s3c_data.timer.expires = jiffies + TASK_JIFFY_DELTA;
    add_timer(&s3c_data.timer);

	return 0;
}

static ssize_t s3c_regs_read(struct file *file, char *buf,
				  size_t count, loff_t * ppos)
{
	char retval = 0;
	size_t retlen = 1;

//	retval = sense1task_data.bent;

	CHUMSENSE1_DEBUG(Trace, "s3cregs read exit with: %d\n", retval);
	copy_to_user(buf, &retval, retlen);

	return retlen;
}

static ssize_t s3c_regs_write(struct file *file, const unsigned char *buf,
                               size_t count, loff_t *ppos )
{
	char retval = 0;
	size_t retlen = 1;
	CHUMSENSE1_DEBUG(Trace, "s3regs write exit with: %d\n", retval);
	copy_to_user( buf, &retval, retlen );

	return retlen;
}

static void __exit s3c_regs_exit(void)
{
	int n;
	//dev_t devno = MKDEV(s3cregs_major, s3cregs_minor);

	CHUMSENSE1_DEBUG(Trace, "Top of exit.\n");
	// set global flag that we're out of here and force a call to remove ourselves
	gDone = 1;

#if 0
	// delete our kernel task
	del_timer(&(sense1task_data.timer));
#endif

	// wait to dequeue self; if kernel panics on rmmod try adding 100 to this value
	mdelay(200);

	// insert all cleanup stuff here
	for (n = 0; n < S3CR_COUNT; n++)
	{
		remove_proc_entry( s3cr[n].proc_entry_name, proc_s3regs );
	}

	// Debug registers
	for (n = 0; n < S3C_DEBUG_REGISTER_COUNT; n++)
	{
		remove_proc_entry( s3dbgr[n].proc_entry_name, *s3dbgr[n].proc_dir );
	}

	// Unmap physical address space
	iounmap( LCD_BASE );
	iounmap( CLOCK_BASE );
	iounmap( GPIO_BASE );

	if (proc_s3regs != NULL)
	{
		if (proc_s3regs_clock != NULL)
		{
			remove_proc_entry( "clock", proc_s3regs );
			proc_s3regs_clock = NULL;
		}
		if (proc_s3regs_gpio != NULL)
		{
			remove_proc_entry( "gpio", proc_s3regs );
			proc_s3regs_gpio = NULL;
		}

		remove_proc_entry( "s3regs", proc_root_driver );
		proc_s3regs = NULL;
	}

#ifdef USE_SYSCTL
	if (gSysCtlHeader != NULL)
		unregister_sysctl_table(gSysCtlHeader);
#endif

}

// entry and exit mappings
module_init(s3c_regs_init);
module_exit(s3c_regs_exit);
