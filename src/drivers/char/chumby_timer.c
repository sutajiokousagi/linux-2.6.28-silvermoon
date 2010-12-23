/*
       chumby_timerx.c
       bunnie    -- August 2006 -- 1.0 -- linux 2.4.20
       bunnie    -- April 2007  -- 2.0 -- port to Ironforge linux 2.6.16
       ghutchins -- Nov. 2007   -- 2.1 -- added minor 1, returns msecs
       hgroover  -- Apr 2010 -- 2.15 -- simplified timerm case for subms jiffies

       This file is part of the chumby sensor suite driver in the linux kernel.
       Copyright (c) Chumby Industries, 2007-2010

       The sensor suite driver is free software; you can redistribute it and/or modify
       it under the terms of the GNU General Public License as published by
       the Free Software Foundation; either version 2 of the License, or
       (at your option) any later version.

       The sensor suite driver is distributed in the hope that it will be useful,
       but WITHOUT ANY WARRANTY; without even the implied warranty of
       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
       GNU General Public License for more details.

       You should have received a copy of the GNU General Public License
       along with the Chumby; if not, write to the Free Software
       Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#define TIMERX_VERSION "2.15-Silvermoon"

//#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>       /* printk() */
#include <linux/stddef.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/slab.h>         /* kmalloc() */
#include <linux/fs.h>           /* everything... */
#include <linux/errno.h>        /* error codes */
#include <linux/types.h>        /* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>        /* O_ACCMODE */
#include <linux/seq_file.h>
#include <linux/cdev.h>

#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/poll.h>
#include <linux/sysctl.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/rtc.h>

#include <linux/root_dev.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/smp.h>

#include <asm/cpu.h>
#include <asm/elf.h>
#include <asm/io.h>
#include <asm/procinfo.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/system.h>         /* cli(), *_flags */
#include <asm/uaccess.h>        /* copy_*_user */

#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>

#include <linux/timer.h>

//#include <asm/arch/imx-regs.h>

#include "chumby_timer.h"

//#include "compat.h"

// Uncomment to support /dev/timerm
#define TIMERM_SUPPORT

/*
 * basic parameters
 */

int timerx_major = 0; // dynamic allocation
int timerx_minor = 0;
int timerx_nr_devs = 2;

module_param(timerx_major, int, S_IRUGO);
module_param(timerx_minor, int, S_IRUGO);
module_param(timerx_nr_devs, int, S_IRUGO);

MODULE_AUTHOR("bunnie@chumby.com");
MODULE_LICENSE("GPL");

// static data
static unsigned long timerx_active = 0;	/* bitmapped status byte.	*/
static unsigned long previous_timestamp = 0;    // minor==1 only!
static unsigned long previous_jiffie_msecs = 0;
static unsigned long previous_timer_offset = 0;

/*
 * Timerx sensor data logs, tracked by tasks that are scheduled by the
 * task scheduler
 */

#define FIXEDPOINT_NORM 1000

struct timerxdata {
  struct cdev *timerx_cdev;
} timerxtask_data;

#define USE_SYSCTL  0

#define TIMERX_BLOCKING 0  // turn off blocking read on timerx sensor
#define JIFFIES_ARE_SUBMILLISECOND 0
#define JIFFIE_READ_LOCKING	1

// Spew the first n timer offsets if nonzero - not supported on current Marvell BSP
#define TIMER_OFFSET_DUMP_SPEW	0

#if 1
#   if USE_SYSCTL
#       define CHUMTIMERX_DEBUG( flag, fmt, args... ) do { if ( gDebug ## flag ) printk( "%s: " fmt, __FUNCTION__ , ## args ); } while (0)
#   else
#       define CHUMTIMERX_DEBUG( flag, fmt, args... ) printk( "%s: " fmt, __FUNCTION__ , ## args )
#   endif
#else
#   define CHUMTIMERX_DEBUG( flag, fmt, args... )
#endif

// function protos
static int chumby_timerx_open(struct inode *inode, struct file *file);
static int chumby_timerx_release(struct inode *inode, struct file *file);
static int timerx_read_proc(char *page, char **start, off_t off,
                         int count, int *eof, void *data);
static ssize_t chumby_timerx_read(struct file *file, char *buf,
				size_t count, loff_t *ppos);

#if USE_SYSCTL

// proc debug structure
static  int gDebugTrace = 0;
static  int gDebugIoctl = 0;
static  int gDebugError = 1;

// create /proc/sys/debug-trace, etc...which can be written to set behavior of globals
// in this driver on the fly
static  struct ctl_table_header    *gSysCtlHeader;
static struct ctl_table gSysCtlChumtimerx[] =
{
    { CTL_CHUMTIMERX_DEBUG_TRACE,     "dbg-trace",  &gDebugTrace,   sizeof( int ), 0644, NULL, &proc_dointvec },
    { CTL_CHUMTIMERX_DEBUG_IOCTL,     "dbg-ioctl",  &gDebugIoctl,   sizeof( int ), 0644, NULL, &proc_dointvec },
    { CTL_CHUMTIMERX_DEBUG_ERROR,     "dbg-error",  &gDebugError,   sizeof( int ), 0644, NULL, &proc_dointvec },
    { 0 }
};

static struct ctl_table gSysCtl[] =
{
    { CTL_CHUMTIMERX, "timerx", NULL, 0, 0555, gSysCtlChumtimerx },
    { 0 }
};
#endif  // USE_SYSCTL

// map into the generic driver infrastructure
static struct file_operations timerx_fops = {
	owner:		THIS_MODULE,
	//	llseek:		chumby_timerx_llseek,
	read:		chumby_timerx_read,
	// poll:		chumby_timerx_poll,
	// ioctl:		chumby_timerx_ioctl,
	open:		chumby_timerx_open,
	release:	chumby_timerx_release,
	// write:          chumby_timerx_write,
	// fasync:		chubmy_timerx_fasync,
};

///////////// code /////////////

static int chumby_timerx_release(struct inode *inode, struct file *file) {
  int minor = iminor(inode);
  CHUMTIMERX_DEBUG( Trace, "Top of release(minor=%d).\n", minor );
  timerx_active &= ~(1<<minor);
  return 0;
}


static int timerx_proc_output (char *buf) {
  int printlen = 0;

  // insert proc debugging output here
  printlen = sprintf(buf,
                     "Chumby timerx driver version %s (bunnie@chumby.com)\n"
                     "The current time is: %08lX\n"
		     "JIFFIES_ARE_SUBMILLISECOND=%d\n",
                     TIMERX_VERSION, jiffies, JIFFIES_ARE_SUBMILLISECOND );

  return(printlen);
}


static int timerx_read_proc(char *page, char **start, off_t off,
                         int count, int *eof, void *data)
{
        int len = timerx_proc_output (page);
        if (len <= off+count) *eof = 1;
        *start = page + off;
        len -= off;
        if (len>count) len = count;
        if (len<0) len = 0;
        return len;
}


static int chumby_timerx_open(struct inode *inode, struct file *file) {
    int minor = iminor(inode);

    // make sure we're not opened twice
    if (timerx_active & (1<<minor))
        return -EBUSY;

    timerx_active |= (1<<minor);
    file->private_data = (void*) minor;
    if (minor == 1)
        previous_timestamp = 0;
    return(0);
}


static int __init chumby_timerx_init(void) {
  dev_t dev = 0;
  int result, err;

  timerxtask_data.timerx_cdev = cdev_alloc();

  if (timerx_nr_devs > 32)
      timerx_nr_devs = 32;

  // insert all device specific hardware initializations here
  printk( "Chumby timerx[%d] driver version %s initializing (bunnie@chumby.com)...show me your jiffies!!! subms=%d\n",
          timerx_nr_devs, TIMERX_VERSION, JIFFIES_ARE_SUBMILLISECOND );

  /*
   * Get a range of minor numbers to work with, asking for a dynamic
   * major unless directed otherwise at load time.
   */
  if (timerx_major) {
    dev = MKDEV(timerx_major, timerx_minor);
    result = register_chrdev_region(dev, timerx_nr_devs, "timerx");
  } else {
    result = alloc_chrdev_region(&dev, timerx_minor, timerx_nr_devs, "timerx");
    timerx_major = MAJOR(dev);
  }
  if (result < 0) {
    printk(KERN_WARNING "timerx: can't get major %d\n", timerx_major);
    return result;
  }

  create_proc_read_entry ("timerx", 0, 0, timerx_read_proc, NULL);

  cdev_init(timerxtask_data.timerx_cdev, &timerx_fops);
  timerxtask_data.timerx_cdev->owner = THIS_MODULE;
  timerxtask_data.timerx_cdev->ops = &timerx_fops;
  err = cdev_add (timerxtask_data.timerx_cdev, dev, timerx_nr_devs);
  /* Fail gracefully if need be */
  if (err)
    printk(KERN_NOTICE "Error %d adding timerx device\n", err);

#if USE_SYSCTL
  gSysCtlHeader = register_sysctl_table( gSysCtl, 0 );
  if ( gSysCtlHeader != NULL ) {
    gSysCtlHeader->ctl_table->child->de->owner = THIS_MODULE;
  }
#endif

  return (0);
}

static ssize_t chumby_timerx_read(struct file   *file,
                                  char          *buf,
                                  size_t        count,
                                  loff_t        *ppos) {
    int minor = (int) file->private_data;
    unsigned long timestamp;

    //    CHUMTIMERX_DEBUG( Trace, "Top of read(minor=%d).\n", minor );

    if (minor != 1)
        timestamp = jiffies;
#ifdef TIMERM_SUPPORT
    else {
        // if minor == 1 we return milliseconds instead of jiffies, note this
        // code is *NOT* portable and assumes that system_timer is exported
        // and system_timer->offset() returns microseconds -- reading of
        // jiffies and usecs was borrowed from do_gettimeofday()
	// On Silvermoon the timer offset is irrelevant since jiffies are
	// already sufficiently granular for instantaneous conversion to
	// milliseconds.
        unsigned long flags;
        unsigned long seq;
        unsigned long jiffie_based_msecs;
        unsigned long timer_offset;
        int ms_added;
#if TIMER_OFFSET_DUMP_SPEW
	static int offset_dump_count = 0;
#endif
#define JIFFIE_DIVISOR 1000
#if JIFFIE_READ_LOCKING
        do {
            seq = read_seqbegin_irqsave(&xtime_lock, flags);
#endif
            jiffie_based_msecs = jiffies_to_msecs(jiffies);
#if JIFFIES_ARE_SUBMILLISECOND || defined(CONFIG_GENERIC_TIME)
	    timer_offset = 0;
#else
		// This will fail to compile if CONFIG_GENERIC_TIME is defined
            timer_offset = system_timer->offset();
#endif
#if JIFFIE_READ_LOCKING
        } while (read_seqretry_irqrestore(&xtime_lock, seq, flags));
#endif

        // timestamp *MUST* be monotonically increasing, unless 32-bit wrap
        // has occurred -- every 29 days timestamp will wrap
        // Offset is platform-dependent but should be microseconds since
        // last timer tick.
#if JIFFIES_ARE_SUBMILLISECOND
	timestamp = jiffie_based_msecs;
#else
		timestamp = jiffie_based_msecs +
					(timer_offset/JIFFIE_DIVISOR) % 10;
#if TIMER_OFFSET_DUMP_SPEW
	if (offset_dump_count < TIMER_OFFSET_DUMP_SPEW)
	{
		printk( "[%d] jiffie_ms=%lu offset=%lu ts=%lu\n",
			offset_dump_count, jiffie_based_msecs, timer_offset, timestamp );
	}
	offset_dump_count++;
#endif
		// s3c64xx fires the timer twice per jiffie, making the maximum-ever offset
		// 4999. If wrap has occurred, add 5ms
		if (jiffie_based_msecs == previous_jiffie_msecs &&
			timer_offset < previous_timer_offset &&
			timer_offset < 5000)
		{
			// Add 5ms
			timestamp += 5;
			ms_added = 5;
		}
		else
		{
			ms_added = 0;
		}
#endif

	// Check for time travel
        if (timestamp < previous_timestamp) {
            if (timestamp > 100 && previous_timestamp < 0xFFFFFF00) {
                printk( KERN_ERR "%s(): time travel, %lu -> %lu (0x%08lx) msecs = %lu/%lu off = %lu/%lu added %d!\n",
                        __FUNCTION__, previous_timestamp, timestamp, timestamp,
                        previous_jiffie_msecs, jiffie_based_msecs,
                        previous_timer_offset, timer_offset,
                        ms_added );
            }
        }
        previous_timestamp = timestamp;
        previous_jiffie_msecs = jiffie_based_msecs;
        previous_timer_offset = timer_offset;
    }
#else
	else
		timestamp = 0;
#endif

    copy_to_user(buf, &timestamp, sizeof(timestamp));
    return sizeof(timestamp);
}

static void __exit chumby_timerx_exit(void) {
  dev_t devno = MKDEV(timerx_major, timerx_minor);

  CHUMTIMERX_DEBUG( Trace, "Top of exit.\n" );

  mdelay(200);  // wait to dequeue self; if kernel panics on rmmod try adding 100 to this value

  // insert all cleanup stuff here
  cdev_del( timerxtask_data.timerx_cdev );
  remove_proc_entry( "timerx", NULL );
  unregister_chrdev_region(devno, timerx_nr_devs);

#if USE_SYSCTL
  if ( gSysCtlHeader != NULL ) {
    unregister_sysctl_table( gSysCtlHeader );
  }
#endif

}

// entry and exit mappings
module_init(chumby_timerx_init);
module_exit(chumby_timerx_exit);

