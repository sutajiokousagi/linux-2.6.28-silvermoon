/*
 *  Xilinx FPGA driver for Silvermoon NeTV
 *
 *  Copyright (c) 2011 bunnie
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
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
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <asm/system.h>		/* cli(), *_flags */
#include <asm/uaccess.h>	/* copy_*_user */
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/irqs.h>

#include <linux/platform_device.h>
#include <linux/mutex.h>
#include "chumby_xilinx.h"
#include <mach/mfp-pxa168.h>
#include <mach/regs-apbc.h>
#include <mach/gpio.h>
#include <plat/generic.h>
#include <mach/addr-map.h>
#include <asm/delay.h>

#define ARRAY_AND_SIZE(x)       (x), ARRAY_SIZE(x)
#define REG32(x)                (*(volatile unsigned long *)(x))

#define SSP2_SSCR0    (APB_VIRT_BASE + 0x1c000)
#define SSP2_SSCR1    (APB_VIRT_BASE + 0x1c004)
#define SSP2_SSSR     (APB_VIRT_BASE + 0x1c008)
#define SSP2_SSITR    (APB_VIRT_BASE + 0x1c00c)
#define SSP2_SSDR     (APB_VIRT_BASE + 0x1c010)
#define SSP2_SSTO     (APB_VIRT_BASE + 0x1c028)
#define SSP2_SSPSP    (APB_VIRT_BASE + 0x1c02c)
#define SSP2_SSTSA    (APB_VIRT_BASE + 0x1c030)
#define SSP2_SSTSS    (APB_VIRT_BASE + 0x1c038)

#define SSP3_SSCR0    (APB_VIRT_BASE + 0x1f000)
#define SSP3_SSCR1    (APB_VIRT_BASE + 0x1f004)
#define SSP3_SSPSP    (APB_VIRT_BASE + 0x1f02c)
#define SSP3_SSDR     (APB_VIRT_BASE + 0x1f010)

#define TIMEOUT 1000000

/*
 * Our parameters which can be set at load time.
 */

int fpga_major =   FPGA_MAJOR;
int fpga_minor =   0;
int fpga_nr_devs = FPGA_NR_DEVS;	/* number of bare fpga devices */

module_param(fpga_major, int, S_IRUGO);
module_param(fpga_minor, int, S_IRUGO);
module_param(fpga_nr_devs, int, S_IRUGO);

struct fpga_dev *fpga_devices;	/* allocated in fpga_init_module */

#ifdef FPGA_DEBUG /* use proc only if debugging */
/*
 * The proc filesystem: function to read and entry
 */

int fpga_read_procmem(char *buf, char **start, off_t offset,
                   int count, int *eof, void *data)
{
	int i, j, len = 0;
	int limit = count - 80; /* Don't print more than this */

	for (i = 0; i < fpga_nr_devs && len <= limit; i++) {
		struct fpga_dev *d = &fpga_devices[i];

		if (down_interruptible(&d->sem))
			return -ERESTARTSYS;
		len += sprintf(buf+len,"\nDevice %i\n", i, d->test);
		up(&fpga_devices[i].sem);
	}
	*eof = 1;
	return len;
}

/*
 * Now to implement the /proc file we need only make an open
 * method which sets up the sequence operators.
 */
static int fpga_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &fpga_seq_ops);
}

/*
 * Create a set of file operations for our proc file.
 */
static struct file_operations fpga_proc_ops = {
	.owner   = THIS_MODULE,
	.open    = fpga_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};
	

/*
 * Actually create (and remove) the /proc file(s).
 */

static void fpga_create_proc(void)
{
	struct proc_dir_entry *entry;
	create_proc_read_entry("fpgamem", 0 /* default mode */,
			NULL /* parent dir */, fpga_read_procmem,
			NULL /* client data */);
	entry = create_proc_entry("fpgaseq", 0, NULL);
	if (entry)
		entry->proc_fops = &fpga_proc_ops;
}

static void fpga_remove_proc(void)
{
	/* no problem if it was not registered */
	remove_proc_entry("fpgamem", NULL /* parent dir */);
	remove_proc_entry("fpgaseq", NULL);
}


#endif /* FPGA_DEBUG */


/*
 * Open and close
 */

int fpga_open(struct inode *inode, struct file *filp)
{
	struct fpga_dev *dev; /* device information */

	dev = container_of(inode->i_cdev, struct fpga_dev, cdev);
	filp->private_data = dev; /* for other methods */

	/* now trim to 0 the length of the device if open was write-only */
	if ( (filp->f_flags & O_ACCMODE) == O_WRONLY) {
		if (down_interruptible(&dev->sem))
			return -ERESTARTSYS;

		// put the operation here that needs to be atomic
		// (just as an example placeholder)

		up(&dev->sem);
	}
	return 0;          /* success */
}

int fpga_release(struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t fpga_write(struct file *filp, const char __user *buf, size_t count,
                loff_t *f_pos)
{
	struct fpga_dev *dev = filp->private_data;
	char *data;
	int i;
	unsigned long ssp_word;
	int timeout = TIMEOUT;
	int retval = -ENOMEM;

	if (down_interruptible(&dev->sem))
		return -ERESTARTSYS;
	data = kmalloc( count * sizeof(char *), GFP_KERNEL );
	if( data == NULL ) {
	  printk( "can't allocate %d bytes for fpga input\n", count );
	  goto out;
	}
	  
	PDEBUG( "Writing %d bytes to fpga...\n", count );
	if(copy_from_user(data, buf, count)) {
	  retval = -EFAULT;
	  goto out;
	}

	if( (count % 4) != 0 ) {
	  printk( "Incoming data is not word-length aligned\n" );
	  printk( "Driver does not handle this correctly, but blasting data anyways...\n" );
	}

	i = 0;
	while( i < count ) {
	  if( gpio_get_value(120) ? 0 : 1 ) {
	    printk( "CRC error reported on byte %d, aborting. Please reset and retry.\n", i );
	    retval = -EILSEQ;
	    goto out;
	  }

	  ssp_word = be32_to_cpu(*((unsigned long *)(data + i)));
	  //	  tmp = ssp_word << 16;
	  //	  tmp |= (ssp_word >> 16) & 0xffff;
	  //	  ssp_word = tmp;

	  i += 4;
	  
	  while( (__raw_readl(SSP2_SSSR) & 0x4) == 0 ) {
	    if (!--timeout) {
	      retval = -ETIMEDOUT;
	      goto out;
	    }
	    cpu_relax();
	  }
	  
	  __raw_writel(ssp_word, SSP2_SSDR);
	  
	}
	PDEBUG( "wrote %d bytes\n", count );
	
	if( count % 2 ) {
	  gpio_direction_output(45, 1);
	  gpio_set_value(45, 1);
	} else {
	  gpio_direction_output(45, 1);
	  gpio_set_value(45, 0);
	}

	retval = count; // this means we got it all

  out:
	if (data)
		kfree(data);
	up(&dev->sem);
	return retval;
}

/*
 * The ioctl() implementation
 */

int fpga_ioctl(struct inode *inode, struct file *filp,
                 unsigned int cmd, unsigned long arg)
{

	int err = 0, tmp;
	int retval = 0;
    
	/*
	 * extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != FPGA_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > FPGA_IOC_MAXNR) return -ENOTTY;

	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	switch(cmd) {

	  case FPGA_IOCWTEST:
	    printk( "this is a test of writing via ioctl: %08x\n", (unsigned int) arg );
		break;

	case FPGA_IOCRESET:
	  printk( "chumby_xilinx ioctl: Resetting FPGA\n" );
	  gpio_direction_output(119,1);
	  gpio_set_value(119,0); // strobe low
	  udelay(2); // give it 2 usecs to settle
	  gpio_set_value(119,1);
	  break;
	  
	case FPGA_IOCLED0:
	  PDEBUG( "chumby_xilinx ioctl: led0 %d\n", arg );
	  if(arg) {
	    gpio_set_value(45, 0);
	  } else {
	    gpio_set_value(45, 1);
	  }
	  break;
        
	case FPGA_IOCLED1:
	  PDEBUG( "chumby_xilinx ioctl: led1 %d\n", arg );
	  if(arg) {
	    gpio_set_value(46, 0);
	  } else {
	    gpio_set_value(46, 1);
	  }
	  break;

	case FPGA_IOCDONE:
	  tmp = __gpio_get_value(97) ? 1 : 0;
	  __put_user(tmp, (int __user *)arg);
	  break;

	case FPGA_IOCINIT:
	  tmp = __gpio_get_value(120) ? 1 : 0;
	  __put_user(tmp, (int __user *)arg);
	  break;
        
	  default:  /* redundant, as cmd was checked against MAXNR */
		return -ENOTTY;
	}
	return retval;

}




struct file_operations fpga_fops = {
	.owner =    THIS_MODULE,
	.write =    fpga_write,
	.ioctl =    fpga_ioctl,
	.open =     fpga_open,
	.release =  fpga_release,
};

/*
 * Finally, the module stuff
 */

/*
 * The cleanup function is used to handle initialization failures as well.
 * Thefore, it must be careful to work correctly even if some of the items
 * have not been initialized
 */
void fpga_cleanup_module(void)
{
	int i;
	dev_t devno = MKDEV(fpga_major, fpga_minor);


	// turn off the LEDs
  gpio_direction_output(45, 1);
  gpio_set_value(45, 1);
  gpio_free(45);

  gpio_direction_output(46, 1);
  gpio_set_value(46, 1);
  gpio_free(46);

  free_irq(IRQ_GPIO(49), NULL);
  gpio_direction_input(120);
  gpio_direction_input(97);
  gpio_direction_input(49);
  gpio_direction_input(91);
  gpio_direction_input(119);

	/* Get rid of our char dev entries */
	if (fpga_devices) {
		for (i = 0; i < fpga_nr_devs; i++) {
		  // put any other cleanup code here
			cdev_del(&fpga_devices[i].cdev);
		}
		kfree(fpga_devices);
	}

#ifdef FPGA_DEBUG /* use proc only if debugging */
	fpga_remove_proc();
#endif

	/* cleanup_module is never called if registering failed */
	unregister_chrdev_region(devno, fpga_nr_devs);

}

#define LCD_BLOCKSIZE 0x42
static irqreturn_t genlock_isr(int irqno, void *param) {
  unsigned int backup[LCD_BLOCKSIZE];
  unsigned int clocks, clocks_reset;
  unsigned int temp;
  int i;

  // make backup copy of registers
  for( i = 0; i < LCD_BLOCKSIZE; i++ ) {
    backup[i] = __raw_readl(APB_VIRT_BASE + 0x20b0c0 + i*4);
  }

  // reset the LCD block
  clocks = __raw_readl(APB_VIRT_BASE + 0x28284c);
  clocks_reset = clocks & ~0x7;
  __raw_writel( clocks_reset, APB_VIRT_BASE + 0x28284c );
  __raw_writel( clocks, APB_VIRT_BASE + 0x28284c );

  // restore registers
  for( i = 0; i < LCD_BLOCKSIZE; i++ ) {
    __raw_writel( backup[i], APB_VIRT_BASE + 0x20b0c0 + i*4 );
  }

  temp = __raw_readl(APB_VIRT_BASE + 0x20b194); // spu_dma_ctrl1
  temp &= ~0x80000000;
  __raw_writel(temp, APB_VIRT_BASE + 0x20b194);
  temp |= 0x80000000;
  __raw_writel(temp, APB_VIRT_BASE + 0x20b194);

  //  temp = __raw_readl(APB_VIRT_BASE + 0x20b194); // spu_dma_ctrl1
  //  printk( "%08x\n", temp );

  return IRQ_HANDLED;
}


/*
 * Set up the char_dev structure for this device.
 */
static void fpga_setup_cdev(struct fpga_dev *dev, int index)
{
	int err, devno = MKDEV(fpga_major, fpga_minor + index);
    
	cdev_init(&dev->cdev, &fpga_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &fpga_fops;
	err = cdev_add (&dev->cdev, devno, 1);
	/* Fail gracefully if need be */
	if (err)
		printk(KERN_NOTICE "Error %d adding fpga%d", err, index);
}

void fpga_init_hw(void) {
  int ret;
  unsigned long pin_config[] = {
    MFP_CFG(GPIO45, AF0),
    MFP_CFG(GPIO46, AF0),
    MFP_CFG(GPIO118, AF1), // ssp2_clk
    MFP_CFG(GPIO119, AF0), // gpio -- fpga reset
    MFP_CFG(GPIO120, AF0), // gpio -- fpga init
    MFP_CFG(GPIO121, AF1), // ssp2_txd
    MFP_CFG(GPIO97, AF0),  // gpio -- fpga_done
    
    MFP_CFG(GPIO90, AF3), // ssp2_clk
  };
  mfp_config(ARRAY_AND_SIZE(pin_config));
  
  gpio_request(45, "LED 0");
  gpio_direction_output(45, 1);
  gpio_set_value(45, 0);

  gpio_request(46, "LED 1");
  gpio_direction_output(46, 1);
  gpio_set_value(46, 0);

  gpio_request(120, "FPGA init");
  gpio_direction_input(120);

  gpio_request(97, "FPGA done");
  gpio_direction_input(97);

  gpio_request(49, "vsync genlock");
  gpio_direction_input(49);

  gpio_request(91, "HPD report");
  gpio_direction_input(91);

  printk("SSP2 clock enable (52 MHz)\n");
  __raw_writel(0x33,(APB_VIRT_BASE + 0x15820));
  __raw_writel(0x37,(APB_VIRT_BASE + 0x15820)); // reset the unit
  __raw_writel(0x33,(APB_VIRT_BASE + 0x15820));
  
  printk("SSP3 clock enable (26 MHz)\n" );
  __raw_writel(0x23,(APB_VIRT_BASE + 0x1584c));
  __raw_writel(0x27,(APB_VIRT_BASE + 0x1584c)); // reset the unit
  __raw_writel(0x23,(APB_VIRT_BASE + 0x1584c));

  // sspsp
  // 0 000 000 0 00 0 011111 0000000 00 000 0 0 00
  // 0x001f0000
  // bit 30:28 - edmystop 000
  // bit 27:26 - edmystrt 000
  // bit 25 fsrt - 0
  // bit 24:23 - dmystop 00
  // bit 21:16 - sfrmwdth 011111
  // bit 15:9 - sfrmdly 0000000
  // bit 8:7 dmystrt 00
  // bit 6:4 strtdly 000
  // bit 3 etds 0
  // bit 2 sfrmp 0
  // bit 1 scmode 00  data driven on falling, sampled on rising, idle state 0
  __raw_writel(0x001f0000, SSP2_SSPSP);

  // sspcr1
  // 0 0 0 1 00 0 0 0 0 0 0 0 0 0 0 0 0 0000 0000 0 0 0 0 0
  // 0x1000000
  // bit 31 ttelp - 0
  // bit 30 tte - 0
  // bit 29 ebcei - 0
  // bit 28 scfr - 1  clk only runs during data transfers
  // bit 25 sclkdir - 0  master mode, ssp port drives clk
  // bit 24 sfrmdir - 0 master mode
  // bit 23 rwot - 0
  // bit 22 trail - 0 
  // bit 21 tsre - 0 tx dma disabled
  // bit 20 rsre - 0 rx dma disabled
  // bit 19 tinte - 0 timeout disabled
  // bit 18 pinte - 0 trail int disabled
  // bit 16 ifs - 0  no inversion of frame
  // bit 15 strf - 0
  // bit 14 efwr - 0
  // bit 13:10 rft - 0
  // bit 9:6 tft - 0  txfifo trigger threshold
  // bit 5 mwds - 0
  // bit 4 sph - 0
  // bit 3 spo - 0
  // bit 2 lbm - 0
  // bit 1 tie - 0 tx fifo interrupt disabled
  // bit 0 rie - 0
  //  __raw_writel(0x10000000, SSP2_SSCR1);
  __raw_writel(0x100000c2, SSP2_SSCR1);

  // sspcr0
  // 0 0 0 00 000 0 0 0 1 0000 0000 0000 0 0 11 1111
  // 0000 0000 0001 0000 0000 0000 0011 1111  0x0010003f
  // bit 31 mod - 0 normal ssp mode
  // bit 29 fpcke - 0 packing mode disabled
  // bit 26:24 frdc - 0
  // bit 23 tim - 1
  // bit 22 rim - 1
  // bit 20 edss - 1  select the larger size
  // bit 7 sse - 0  -- disable port initially
  // bit 5:4 frf - 11 -- programmable serial protocol
  // bit 3:0 dss 1111  -- 32 bits data
  __raw_writel(0x00d0003f, SSP2_SSCR0);

  __raw_writel(0x00d000bf, SSP2_SSCR0);  // enable the port

  __raw_writel(0x001f0000, SSP3_SSPSP); // continuous clock on SSP3
  __raw_writel(0x01000000, SSP3_SSCR1); // this is actually a bizarre configuration
  __raw_writel(0x0010003f, SSP3_SSCR0); // but it works.
  __raw_writel(0x001000ff, SSP3_SSCR0); 
  __raw_writel(0xdeadbeef, SSP3_SSDR); // dummy write to trigger clock
  __raw_writel(0xdeadbeef, SSP3_SSDR); // dummy write to trigger clock

  
  gpio_request(119, "FPGA reset (active low)");
  gpio_direction_output(119,1);
  gpio_set_value(119,0); // strobe low
  udelay(2); // give it 2 usecs to settle
  gpio_set_value(119,1);

  // allocate the sync IRQ
  ret = request_irq(IRQ_GPIO(49), genlock_isr, IRQF_TRIGGER_RISING, "genlock trigger", NULL);
  if( ret ) {
    printk( "Can't allocate IRQ 49 for vsync genlock trigger\n" );
  }

}

int fpga_init_module(void)
{
	int result, i;
	dev_t dev = 0;

/*
 * Get a range of minor numbers to work with, asking for a dynamic
 * major unless directed otherwise at load time.
 */
	if (fpga_major) {
		dev = MKDEV(fpga_major, fpga_minor);
		result = register_chrdev_region(dev, fpga_nr_devs, "fpga");
	} else {
		result = alloc_chrdev_region(&dev, fpga_minor, fpga_nr_devs,
				"fpga");
		fpga_major = MAJOR(dev);
	}
	if (result < 0) {
		printk(KERN_WARNING "fpga: can't get major %d\n", fpga_major);
		return result;
	}

        /* 
	 * allocate the devices -- we can't have them static, as the number
	 * can be specified at load time
	 */
	fpga_devices = kmalloc(fpga_nr_devs * sizeof(struct fpga_dev), GFP_KERNEL);
	if (!fpga_devices) {
		result = -ENOMEM;
		goto fail;  /* Make this more graceful */
	}
	memset(fpga_devices, 0, fpga_nr_devs * sizeof(struct fpga_dev));

        /* Initialize each device. */
	for (i = 0; i < fpga_nr_devs; i++) {
	  fpga_devices[i].test = 0;  // just an example of how to use this field
		init_MUTEX(&fpga_devices[i].sem);
		fpga_setup_cdev(&fpga_devices[i], i);
	}

        /* At this point call the init function for any friend device */
	dev = MKDEV(fpga_major, fpga_minor + fpga_nr_devs);

#ifdef FPGA_DEBUG /* only when debugging */
	fpga_create_proc();
#endif
	fpga_init_hw();

	return 0; /* succeed */

  fail:
	fpga_cleanup_module();
	return result;
}

module_init(fpga_init_module);
module_exit(fpga_cleanup_module);


MODULE_AUTHOR("bunnie <bunnie@chumby.com>");
MODULE_DESCRIPTION("chumby Silvermoon NeTV FPGA driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:silvermoon-fpga");
