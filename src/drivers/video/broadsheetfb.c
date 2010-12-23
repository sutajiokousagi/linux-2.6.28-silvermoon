/*
 * broadsheetfb.c -- FB driver for E-Ink Broadsheet controller
 *
 * Copyright (C) 2008, Jaya Kumar
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 * Layout is based on skeletonfb.c by James Simmons and Geert Uytterhoeven.
 *
 * This driver is written to be used with the Broadsheet display controller.
 *
 * It is intended to be architecture independent. A board specific driver
 * must be used to perform all the physical IO interactions.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/uaccess.h>

#include <video/broadsheetfb.h>

/* Display specific information */
#define DPY_H 800
#define DPY_W 600

static int rotmode = 0;

static struct fb_fix_screeninfo broadsheetfb_fix __devinitdata = {
	.id =           "broadsheetfb",
	.type =         FB_TYPE_PACKED_PIXELS,
	.visual =       FB_VISUAL_STATIC_PSEUDOCOLOR,
	.xpanstep =     0,
	.ypanstep =     0,
	.ywrapstep =    0,
	.line_length =  DPY_W,
	.accel =        FB_ACCEL_NONE,
};

static struct fb_var_screeninfo broadsheetfb_var __devinitdata = {
	.xres           = DPY_W,
	.yres           = DPY_H,
	.xres_virtual   = DPY_W,
	.yres_virtual   = DPY_H,
	.bits_per_pixel = 8,
	.grayscale      = 1,
	.red =          { 0, 4, 0 },
	.green =        { 0, 4, 0 },
	.blue =         { 0, 4, 0 },
	.transp =       { 0, 0, 0 },
};

u16 curr_width;
u16 curr_heigth;
u16 curr_sdcfg;
u16 curr_gdrvcfg;
u16 curr_lutindexformat;
u16 curr_fsync_len;
u16 curr_fend_fbegin_len;
u16 curr_line_sync_len;
u16 curr_line_end_begin_len;
u16 curr_pixel_clock_cfg;
u32 curr_waveform;

/* main broadsheetfb functions */

static void broadsheet_send_command(struct broadsheetfb_par *par, u16 data)
{
	par->board->writecmd(par, data);
}

static void broadsheet_send_cmdargs(struct broadsheetfb_par *par, u16 cmd,
				    int argc, u16 *argv)
{
	int i;

	par->board->writecmd(par, cmd);

	for (i = 0; i < argc; i++)
		par->board->writedata(par, argv[i]);
}

static void broadsheet_burst_write(struct broadsheetfb_par *par, int size,
				   u16 *data)
{
	int i;
	//	printk(KERN_INFO "Broadsheet write burst %d pixels\n", size*2);

	for (i = 0; i < size; i++) {
		par->board->writedata(par, data[i]);
	}

}

static u16 broadsheet_get_data(struct broadsheetfb_par *par)
{
	u16 res;

	res = par->board->read(par);

	return res;
}

static void broadsheet_write_reg(struct broadsheetfb_par *par, u16 reg,
				 u16 data)
{
	/* cs lo, dc lo for cmd, we lo for each data, db as usual */

	par->board->writecmd(par, BS_CMD_WR_REG);

	par->board->writedata(par, reg);
	par->board->writedata(par, data);

}

static u16 broadsheet_read_reg(struct broadsheetfb_par *par, u16 reg)
{
	par->board->writecmd(par, BS_CMD_RD_REG);

	par->board->writedata(par, reg);

	return par->board->read(par);

}

static void __devinit broadsheet_init(struct broadsheetfb_par *par)
{
	u16 a;
	u16 args[5];

	broadsheet_write_reg(par, 6, 0);

	par->board->writecmd(par, BS_CMD_INIT_SYS_RUN);

	broadsheet_write_reg(par, 0x106, 0x203);

	/* the controller needs a second */
	msleep(500);
	broadsheet_write_reg(par, 0x304, 0x0123);
	broadsheet_write_reg(par, 0x30A, 0x4567);
	a = broadsheet_read_reg( par, 0x0304 );
	if ( a != 0x0123 ) {
		printk ("read 0x304 - got 0x%x, should be 0x0123\n", a);
	}

	args[0] = curr_heigth;
	args[1] = curr_width;
	args[2] = curr_sdcfg;
	args[3] = curr_gdrvcfg;
	args[4] = curr_lutindexformat;
	broadsheet_send_cmdargs(par, BS_CMD_INIT_DSPE_CFG, 5, args);

#if 0
	/* did the controller really set it? */
	broadsheet_send_cmdargs(par, BS_CMD_INIT_DSPE_CFG, 5, args);
#endif

	args[0] = curr_fsync_len;
	args[1] = curr_fend_fbegin_len;
	args[2] = curr_line_sync_len;
	args[3] = curr_line_end_begin_len;
	args[4] = curr_pixel_clock_cfg;
	broadsheet_send_cmdargs(par, BS_CMD_INIT_DSPE_TMG, 5, args);

	/* setup waveform */
	args[0] = curr_waveform & 0xffff;
	args[1] = curr_waveform >> 16;
	broadsheet_send_cmdargs(par, BS_CMD_RD_WFM_INFO, 2, args);

	msleep(500);

	broadsheet_send_command(par, BS_CMD_UPD_GDRV_CLR);

	broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);

	broadsheet_write_reg(par, 0x330, 0x84);

	broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);

	args[0] = rotmode;
	broadsheet_send_cmdargs(par, BS_CMD_INIT_ROTMODE, 1, args);
	//	printk("init - set rotmode 0x%x\n", args[0]);

}

static void broadsheetfb_dpy_update(struct broadsheetfb_par *par)
{
	u16 args[5];

	printk(KERN_INFO "Broadsheet dpy update\n");

	args[0] = 3 << 4;     /* 8 bit display mode */
	broadsheet_send_cmdargs(par, BS_CMD_LD_IMG, 1, args);

	args[0] = 0x154;
	broadsheet_send_cmdargs(par, BS_CMD_WR_REG, 1, args);
	broadsheet_burst_write(par, DPY_W*DPY_H/2,
			       (u16 *) par->info->screen_base);

	broadsheet_send_command(par, BS_CMD_LD_IMG_END);

	args[0] = 0x4300;
	broadsheet_send_cmdargs(par, BS_CMD_UPD_FULL, 1, args);
	printk(KERN_INFO "Broadsheet dpy update - upd_full\n");

	broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);

	broadsheet_send_command(par, BS_CMD_WAIT_DSPE_FREND);


}

static void broadsheetfb_dpy_update_pages(struct broadsheetfb_par *par,
					  u16 y1, u16 y2)
{
	u16 args[5];
	unsigned char *buf = (unsigned char *)par->info->screen_base;

	printk(KERN_INFO "Broadsheet dpy update pages\n");

	/* y1 must be a multiple of 4 so drop the lower bits */
	y1 &= 0xFFFC;
	/* y2 must be a multiple of 4 , but - 1 so up the lower bits */
	y2 |= 0x0003;

	args[0] = 3 << 4;     /* 8 bit display mode */
	args[1] = 0;
	args[2] = y1;
	args[3] = cpu_to_le16(par->info->var.xres);
	args[4] = y2;
	broadsheet_send_cmdargs(par, BS_CMD_LD_IMG_AREA, 5, args);

	args[0] = 0x154;
	broadsheet_send_cmdargs(par, BS_CMD_WR_REG, 1, args);

	buf += y1 * par->info->var.xres;
	broadsheet_burst_write(par, ((1 + y2 - y1) * par->info->var.xres)/2,
			       (u16 *) buf);

	broadsheet_send_command(par, BS_CMD_LD_IMG_END);

	args[0] = 0x4300;
	broadsheet_send_cmdargs(par, BS_CMD_UPD_FULL, 1, args);
	printk(KERN_INFO "Broadsheet dpy update_pages - upd_full\n");

	broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);

	broadsheet_send_command(par, BS_CMD_WAIT_DSPE_FREND);

}

static int broadsheetfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	u16 args[5];
	struct broadsheetfb_par *par = info->par;

	switch (cmd) {
	case FBIO_FILLRECT:
	{
		struct fb_fillrect fr;
		int temp, i;
		char * fbbuf = par->info->screen_base;
		int sw = (rotmode & (1 << 8)) ? DPY_W : DPY_H;
		int sh = (rotmode & (1 << 8)) ? DPY_H : DPY_W;

		temp = copy_from_user((char *)&fr, __user (char *)arg, sizeof(fr));
		if (temp != 0) {
			printk(KERN_INFO "fill rect *arg: 0x%x\n", temp);
			return -EFAULT;
		}

		/*
		 * sanity check arguments.  The rotmode could have change without
		 * the caller knowing
		 */
		if(fr.width > sw || fr.dx > sw || fr.dy > sh || fr.height > sh) {
			int tmp = sw;
			sw = sh;
			sh = tmp;
			/* still bad, then user error */
			if(fr.width > sw || fr.dx > sw || fr.dy > sh || fr.height > sh) {
				return -EINVAL;
			}
		}

		if(fr.width < 0 || fr.dx < 0 || fr.dy < 0 || fr.height < 0) {
			return -EINVAL;
		}

#if 0
		printk(KERN_INFO "fillrect color: 0x%x dx: 0x%x dy: 0x%x w: 0x%x h: 0x%x sw: 0x%x  sh: 0x%x\n",
			fr.color, fr.dx, fr.dy, fr.width, fr.height, sw, sh );
#endif

		fbbuf += (fr.dy * sw) + fr.dx;

		for (i=0; i<fr.height; i++) {
			int j;
			for (j=0; j<fr.width; j++) {
				*fbbuf++ = fr.color;
			}
			fbbuf += (sw - fr.width);
		}

		args[0] = (3 << 4);     /* 8 bit display mode */
		args[1] = fr.dx;
		args[2] = fr.dy;
		args[3] = fr.width;
		args[4] = fr.height;
		broadsheet_send_cmdargs(par, BS_CMD_LD_IMG_AREA, 5, args);

		args[0] = 0x154;
		broadsheet_send_cmdargs(par, BS_CMD_WR_REG, 1, args);

		fbbuf = par->info->screen_base + (fr.dy * sw) + fr.dx;

		for (i=0; i<fr.height; i++) {
			broadsheet_burst_write(par, fr.width/2, (u16 *) fbbuf);
			fbbuf += sw;
		}

		broadsheet_send_command(par, BS_CMD_LD_IMG_END);

		broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);
  
		broadsheet_send_command(par, BS_CMD_WAIT_DSPE_FREND);
		break;
	}
	case FBIO_XCMD:
	{
		struct fb_xcmd xcmd;
		int temp;

		temp = copy_from_user((char *)&xcmd, __user (char *)arg, sizeof(xcmd));
		if (temp != 0) {
			printk(KERN_INFO "xcmd *arg: 0x%x\n", temp);
			return -EFAULT;
		}

		if (xcmd.cmd == BS_CMD_UPD_PART) {
#if 0
			int i;
			int a = 0;
			char * fbbuf = par->info->screen_base;

			printk(KERN_INFO "xcmd - cmd: 0x%x count: %d arg: 0x%x\n",
			       xcmd.cmd, xcmd.count, xcmd.args[0]);
			for (i=0; i<(DPY_H * DPY_W); i++) {
				if(*fbbuf++ != 0xff) a++;
			}
			if (a != 0)
				printk( "... data error a=%d\n", a);
			else
				printk( " ... data ok a=%d\n", a);
#endif
			{
				int prev_index = -1;
				struct page *cur;
				struct fb_deferred_io *fbdefio = info->fbdefio;

				/* walk the written page list and swizzle the data */
				list_for_each_entry(cur, &fbdefio->pagelist, lru) {
					prev_index = cur->index;
				}
			}
		}

#if 0
		printk(KERN_INFO "xcmd - cmd: 0x%x count: %d args: 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		       xcmd.cmd, xcmd.count, xcmd.args[0], xcmd.args[1],
		       xcmd.args[2], xcmd.args[3], xcmd.args[4]);
#endif
		broadsheet_send_cmdargs(par, xcmd.cmd, xcmd.count, &xcmd.args[0]);

		if (xcmd.cmd == BS_CMD_INIT_DSPE_CFG) {
			curr_heigth = xcmd.args[0];
			curr_width = xcmd.args[1];
			curr_sdcfg = xcmd.args[2];
			curr_gdrvcfg = xcmd.args[3];
			curr_lutindexformat = xcmd.args[4];
		}

		if (xcmd.cmd == BS_CMD_INIT_DSPE_TMG) {
			curr_fsync_len = xcmd.args[0];
			curr_fend_fbegin_len = xcmd.args[1];
			curr_line_sync_len = xcmd.args[2];
			curr_line_end_begin_len = xcmd.args[3];
			curr_pixel_clock_cfg = xcmd.args[4];
		}

		if (xcmd.cmd == BS_CMD_RD_WFM_INFO) {
			curr_waveform  = xcmd.args[0] | (xcmd.args[1] << 16);
		}

		if (xcmd.cmd == BS_CMD_INIT_ROTMODE) {
			rotmode  = xcmd.args[0];
			printk("xcmd - set rotmode 0x%x\n", rotmode);
		}


		break;
	}
	case FBIO_UPDFULL:
	{
		u16 sh = (rotmode & (1 << 8)) ? DPY_H : DPY_W;
		u16 sw = (rotmode & (1 << 8)) ? DPY_W : DPY_H;

#if 0
		printk(KERN_INFO "updfull arg: 0x%x w: %d  h:%d\n", (u32)arg, sw, sh);
#endif
		{
			int prev_index = -1;
			struct page *cur;
			struct fb_deferred_io *fbdefio = info->fbdefio;

			/* walk the written page list and swizzle the data */
			list_for_each_entry(cur, &fbdefio->pagelist, lru) {
				prev_index = cur->index;
			}
		}

		args[0] = (3 << 4);     /* 8 bit display mode */
		broadsheet_send_cmdargs(par, BS_CMD_LD_IMG, 1, args);

		args[0] = 0x154;
		broadsheet_send_cmdargs(par, BS_CMD_WR_REG, 1, args);

		broadsheet_burst_write(par,
				       (sh * sw)/2,
				       (u16 *) par->info->screen_base);

		broadsheet_send_command(par, BS_CMD_LD_IMG_END);

		broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);
  
		broadsheet_send_command(par, BS_CMD_WAIT_DSPE_FREND);

		args[0] = arg;
		broadsheet_send_cmdargs(par, BS_CMD_UPD_FULL, 1, args);

	}
	break;
	case FBIO_LDAREA:
	{
		/* reuse the fillrect for dx, dy, width, and height */
		struct fb_fillrect fr;
		int temp, i;
		char * fbbuf = par->info->screen_base;
		int sh = (rotmode & (1 << 8)) ? DPY_H : DPY_W;
		int sw = (rotmode & (1 << 8)) ? DPY_W : DPY_H;

		temp = copy_from_user((char *)&fr, __user (char *)arg, sizeof(fr));
		if (temp != 0) {
			printk(KERN_INFO "update part *arg: 0x%x\n", temp);
			return -EFAULT;
		}

#if 0
		printk(KERN_INFO "LDAREA part dx: %d dy: %d w: %d h: %d sw: %d  sh: %d\n",
		       fr.dx, fr.dy, fr.width, fr.height, sw, sh );
#endif

		/*
		 * sanity check arguments.  The rotmode could have change without
		 * the caller knowing
		 */
		if(fr.width > sw || fr.dx > sw || fr.dy > sh) {
			int tmp = sw;
			sw = sh;
			sh = tmp;
			printk(KERN_INFO "LDAREA part changed sw/sh - sw: %d  sh: %d\n",
			       sw, sh );
			/* still bad, then user error */
			if(fr.width > sw || fr.dx > sw || fr.dy > sh) {
				return -EFAULT;
			}
		}

		args[0] = fr.color;
		args[1] = fr.dx;
		args[2] = fr.dy;
		args[3] = fr.width;
		args[4] = fr.height;
		broadsheet_send_cmdargs(par, BS_CMD_LD_IMG_AREA, 5, args);

		args[0] = 0x154;
		broadsheet_send_cmdargs(par, BS_CMD_WR_REG, 1, args);

		fbbuf += (fr.dy * sw) + fr.dx;

		for (i=0; i<fr.height; i++) {
			broadsheet_burst_write(par, fr.width/2, (u16 *) fbbuf);
			fbbuf += sw;
		}

		broadsheet_send_command(par, BS_CMD_LD_IMG_END);
	}
	break;
	case FBIO_UPDPART:
	{
		/* reuse the fillrect for dx, dy, width, and height */
		struct fb_fillrect fr;
		int temp;

		temp = copy_from_user((char *)&fr, __user (char *)arg, sizeof(fr));
		if (temp != 0) {
			printk(KERN_INFO "update part *arg: 0x%x\n", temp);
			return -EFAULT;
		}

#if 0
		printk(KERN_INFO "UPDPART part arg: 0x%x dx: %d dy: %d w: %d h: %d\n",
		       fr.color, fr.dx, fr.dy, fr.width, fr.height);
#endif

		args[0] = fr.color;
		args[1] = fr.dx;
		args[2] = fr.dy;
		args[3] = fr.width;
		args[4] = fr.height;
		broadsheet_send_cmdargs(par, BS_CMD_UPD_PART_AREA, 5, args);

		{
			int prev_index = -1;
			struct page *cur;
			struct fb_deferred_io *fbdefio = info->fbdefio;

			/* walk the written page list and swizzle the data */
			list_for_each_entry(cur, &fbdefio->pagelist, lru) {
				prev_index = cur->index;
			}
		}
	}
	break;
	case FBIO_WRITEREG:
		args[0] = arg;
		broadsheet_send_cmdargs(par, BS_CMD_WR_REG, 1, args);
		break;
	case FBIO_READREG:
	{
		int val;
		/* get user space pointer to input register and output result */
		if (get_user(val, (int *)arg))
			return -EFAULT;
		par->board->writecmd(par, BS_CMD_RD_REG);
		par->board->writedata(par, (u16)val);
		val = (int)par->board->read(par);
		put_user(val, (__u32 __user *) arg);
		break;
	}
	case FBIO_WRITEDATA:
	{
		int data;
		u32 datalen;
		u32 copied_data;
		u16 *local_data;
		int i;
		int retval = 0;

		/* get user space pointer to output addresses */
		if (get_user(datalen, (int *)arg))
			return -EFAULT;

		if (get_user(data, (int *)(arg + 4)))
			return -EFAULT;

		/* copy the user data to the kernel space */
		local_data = (u16 *)kmalloc((datalen+1)*sizeof(u8), GFP_KERNEL);
		if (local_data == NULL)
			return -EFAULT;
		local_data[datalen/2] = 0;
		copied_data = copy_from_user((u8 *)local_data, __user (u8 *)data, datalen);
		if(copied_data == 0) {
#if 0
			printk(KERN_INFO "writedata: 0x%x count: %d\n", data, datalen);
#endif
			for(i=0;i<(datalen+1)/2;i++) {
				par->board->writedata(par, (u16)local_data[i]);
			}
			retval = 0;
		} else
			retval = -EINVAL;
		kfree(local_data);
		return retval;
	}
	case FBIO_READDATA:
	{
		int data;
		u32 datalen;
		u32 copied_data;
		u16 *local_data;
		int i;
		int retval = 0;

		/* get user space pointer to output addresses */
		if (get_user(datalen, (int *)arg))
			return -EFAULT;

		if (get_user(data, (int *)(arg + 4)))
			return -EFAULT;

		/* copy the user data to the kernel space */
		local_data = (u16 *)kmalloc((datalen+1)*sizeof(u8), GFP_KERNEL);
		if (local_data == NULL)
			return -EFAULT;

#if 1
		printk(KERN_INFO "readdata: 0x%x count: %d\n", data, datalen);
#endif
		for(i=0;i<(datalen+1)/2;i++) {
			local_data[i] = par->board->read(par);
		}
		copied_data = copy_to_user(__user (u8 *)data, local_data, datalen);

		if(copied_data == 0) {
			retval = 0;
		} else
			retval = -EINVAL;
		kfree(local_data);
		return retval;
	}
	default:
		break;
	}
	return 0;
}

/* this is called back from the deferred io workqueue */
static void broadsheetfb_dpy_deferred_io(struct fb_info *info,
					 struct list_head *pagelist)
{
	u16 y1 = 0, h = 0;
	int prev_index = -1;
	struct page *cur;
	struct fb_deferred_io *fbdefio = info->fbdefio;
	int h_inc;
	u16 yres = info->var.yres;
	u16 xres = info->var.xres;
	int do_update = 0;

	printk(KERN_INFO "Broadsheet dpy deferred io\n");

	/* height increment is fixed per page */
	h_inc = DIV_ROUND_UP(PAGE_SIZE , xres);

	/* walk the written page list and swizzle the data */
	list_for_each_entry(cur, &fbdefio->pagelist, lru) {
		if (prev_index < 0) {
			/* just starting so assign first page */
			y1 = (cur->index << PAGE_SHIFT) / xres;
			h = h_inc;
		} else if ((prev_index + 1) == cur->index) {
			/* this page is consecutive so increase our height */
			h += h_inc;
		} else {
			/* page not consecutive, issue previous update first */
			printk(KERN_INFO "Broadsheet dpy deferred io 1\n");
			//			broadsheetfb_dpy_update_pages(info->par, y1, y1 + h);
 			do_update = 1;
			/* start over with our non consecutive page */
			y1 = (cur->index << PAGE_SHIFT) / xres;
			h = h_inc;
		}
		prev_index = cur->index;
	}

	/* if we still have any pages to update we do so now */
	//	if (h >= yres) {
#if 0
	if (h >= yres || do_update) {
		/* its a full screen update, just do it */
		broadsheetfb_dpy_update(info->par);
	} else {
		printk(KERN_INFO "Broadsheet dpy deferred io 2\n");
		broadsheetfb_dpy_update_pages(info->par, y1,
					      min((u16) (y1 + h), yres));
	}
#endif
}

static void broadsheetfb_fillrect(struct fb_info *info,
				  const struct fb_fillrect *rect)
{
	struct broadsheetfb_par *par = info->par;

	printk(KERN_INFO "Broadsheet fillrect\n");

	sys_fillrect(info, rect);

	broadsheetfb_dpy_update(par);
}

static void broadsheetfb_copyarea(struct fb_info *info,
				  const struct fb_copyarea *area)
{
	struct broadsheetfb_par *par = info->par;

	printk(KERN_INFO "Broadsheet copyarea\n");

	sys_copyarea(info, area);

	broadsheetfb_dpy_update(par);
}

static void broadsheetfb_imageblit(struct fb_info *info,
				   const struct fb_image *image)
{
	struct broadsheetfb_par *par = info->par;

	printk(KERN_INFO "Broadsheet imageblit\n");

	sys_imageblit(info, image);

	broadsheetfb_dpy_update(par);
}

/*
 * this is the slow path from userspace. they can seek and write to
 * the fb. it's inefficient to do anything less than a full screen draw
 */
static ssize_t broadsheetfb_write(struct fb_info *info, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct broadsheetfb_par *par = info->par;
	unsigned long p = *ppos;
	void *dst;
	int err = 0;
	unsigned long total_size;

	printk(KERN_INFO "Broadsheet write\n");

	if (info->state != FBINFO_STATE_RUNNING)
		return -EPERM;

	total_size = info->fix.smem_len;

	if (p > total_size)
		return -EFBIG;

	if (count > total_size) {
		err = -EFBIG;
		count = total_size;
	}

	if (count + p > total_size) {
		if (!err)
			err = -ENOSPC;

		count = total_size - p;
	}

	dst = (void *)(info->screen_base + p);

	if (copy_from_user(dst, buf, count))
		err = -EFAULT;

	if  (!err)
		*ppos += count;

	broadsheetfb_dpy_update(par);

	return (err) ? err : count;
}

static struct fb_ops broadsheetfb_ops = {
	.owner	  	  = THIS_MODULE,
	.fb_read	  = fb_sys_read,
	.fb_write	  = broadsheetfb_write,
	.fb_fillrect	  = broadsheetfb_fillrect,
	.fb_copyarea	  = broadsheetfb_copyarea,
	.fb_imageblit	  = broadsheetfb_imageblit,
	.fb_ioctl	  = broadsheetfb_ioctl,
};

static struct fb_deferred_io broadsheetfb_defio = {
	.delay	  	  = HZ * 100000,
	.deferred_io	  = broadsheetfb_dpy_deferred_io,
};

static int __devinit broadsheetfb_probe(struct platform_device *dev)
{
	struct fb_info *info;
	struct broadsheet_board *board;
	int retval = -ENOMEM;
	int videomemorysize;
	unsigned char *videomemory;
	struct broadsheetfb_par *par;
	int i;

	/* pick up board specific routines */
	board = dev->dev.platform_data;
	if (!board)
		return -EINVAL;

	/* try to count device specific driver, if can't, platform recalls */
	if (!try_module_get(board->owner))
		return -ENODEV;

	info = framebuffer_alloc(sizeof(struct broadsheetfb_par), &dev->dev);
	if (!info)
		goto err;

	//	printk(KERN_INFO "Broadsheet probe vmalloc\n");

	videomemorysize = (DPY_W*DPY_H);
	videomemory = vmalloc(videomemorysize);
	if (!videomemory)
		goto err_fb_rel;

	memset(videomemory, 0xff, videomemorysize);

	info->screen_base = (char *)videomemory;
	info->fbops = &broadsheetfb_ops;

	info->var = broadsheetfb_var;
	info->fix = broadsheetfb_fix;
	info->fix.smem_len = videomemorysize;
	par = info->par;
	par->info = info;
	par->board = board;
	par->write_reg = broadsheet_write_reg;
	par->read_reg = broadsheet_read_reg;
	init_waitqueue_head(&par->waitq);

	info->flags = FBINFO_FLAG_DEFAULT;

	info->fbdefio = &broadsheetfb_defio;
	fb_deferred_io_init(info);

	retval = fb_alloc_cmap(&info->cmap, 16, 0);
	if (retval < 0) {
		dev_err(&dev->dev, "Failed to allocate colormap\n");
		goto err_vfree;
	}

	/* set cmap */
	for (i = 0; i < 16; i++)
		info->cmap.red[i] = (((2*i)+1)*(0xFFFF))/32;
	memcpy(info->cmap.green, info->cmap.red, sizeof(u16)*16);
	memcpy(info->cmap.blue, info->cmap.red, sizeof(u16)*16);

	retval = par->board->setup_irq(info);
	if (retval < 0)
		goto err_cmap;

	/* this inits the dpy */
	retval = board->init(par);
	if (retval < 0)
		goto err_free_irq;

	/* set initial default configuration - may be overridden
	   by application software */
	curr_width = DPY_W;
	curr_heigth = DPY_H;
	curr_sdcfg = (100 | (1 << 8) | (1 << 9)); /* sdcfg */
	curr_gdrvcfg = 2; /* gdrv cfg */
	curr_lutindexformat = (4 | (1 << 7)); /* lut index format */
	curr_fsync_len = 4;
	curr_fend_fbegin_len = (10 << 8) | 4;
	curr_line_sync_len = 10;
	curr_line_end_begin_len = (100 << 8) | 4;
	curr_pixel_clock_cfg = 6;
	curr_waveform = 0x886;

	/* initialize to portrait mode */
	rotmode = 3 <<8;

	broadsheet_init(par);

	retval = register_framebuffer(info);
	if (retval < 0)
		goto err_free_irq;
	platform_set_drvdata(dev, info);

	printk(KERN_INFO
	       "fb%d: Broadsheet frame buffer, using %dK of video memory\n",
	       info->node, videomemorysize >> 10);

	/* display a bank page */
	broadsheetfb_dpy_update(par);

	return 0;

err_free_irq:
	board->cleanup(par);
err_cmap:
	fb_dealloc_cmap(&info->cmap);
err_vfree:
	vfree(videomemory);
err_fb_rel:
	framebuffer_release(info);
err:
	module_put(board->owner);
	return retval;

}

static int __devexit broadsheetfb_remove(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);

	if (info) {
		struct broadsheetfb_par *par = info->par;
		unregister_framebuffer(info);
		fb_deferred_io_cleanup(info);
		par->board->cleanup(par);
		fb_dealloc_cmap(&info->cmap);
		vfree((void *)info->screen_base);
		module_put(par->board->owner);
		framebuffer_release(info);
	}
	return 0;
}

#ifdef CONFIG_PM
/**
 *broadsheetfb_suspend - Optional but recommended function. Suspend the device.
 *@dev: platform device
 *@msg: the suspend event code.
 *
 *      See Documentation/power/devices.txt for more information
 */
static int broadsheetfb_suspend(struct platform_device *dev, pm_message_t msg)
{
#if 0
	struct fb_info *info = platform_get_drvdata(dev);
	struct broadsheetfb_par *par = info->par;
	/* turn off Epsom controller? */
	printk(KERN_INFO "Broadsheet frame buffer send SLP\n");
	msleep(1000);
	broadsheet_send_command(par, BS_CMD_STBY);
#endif
	printk(KERN_INFO "Broadsheet frame buffer suspend complete\n");

	return 0;
}

/**
 *broadsheetfb_resume - Resume the device.
 *@dev: platform device
 *
 *      Performs a board init to setup GPIO pins and static memory controller
 */
static int broadsheetfb_resume(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct broadsheet_board *board = dev->dev.platform_data;

	int retval = board->resume(info->par);

#if 0
	/* reset the eInk controller */
	broadsheet_send_command(info->par, BS_CMD_RUN_PLL);
#else
	broadsheet_init(info->par);
#endif
	printk(KERN_INFO "Broadsheet frame buffer resume complete\n");

	return retval;
}
#else
#define broadsheetfb_suspend NULL
#define broadsheetfb_resume NULL
#endif /* CONFIG_PM */

static struct platform_driver broadsheetfb_driver = {
	.probe  = broadsheetfb_probe,
	.remove = broadsheetfb_remove,
	.suspend = broadsheetfb_suspend,
	.resume = broadsheetfb_resume,
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "broadsheetfb",
	},
};

static int __init broadsheetfb_init(void)
{
	return platform_driver_register(&broadsheetfb_driver);
}

static void __exit broadsheetfb_exit(void)
{
	platform_driver_unregister(&broadsheetfb_driver);
}

module_init(broadsheetfb_init);
module_exit(broadsheetfb_exit);

MODULE_DESCRIPTION("fbdev driver for Broadsheet controller");
MODULE_AUTHOR("Jaya Kumar");
MODULE_LICENSE("GPL");
