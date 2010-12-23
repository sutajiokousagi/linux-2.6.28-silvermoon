/*
 * drivers/video/cbp_pxa168_fb.c
 * (was: linux/drivers/video/pxa910_fb.c)
 *
 * Copyright (C) 2008 Marvell International Ltd.
 *
 * Written by Richard Tang <richardt@marvell.com>
 * Modified by Stanley Cai <swcai@marvell.com>
 * Modified by Jason Chagas <jason.chagas@marvell.com> - Enabled proper
 *                         binding with video core driver - Apr. 11, 2008
 * Modified by Jason Chagas <jason.chagas@marvell.com> - Code
 *                         cleanup - Apr. 22, 2008
 *
 * Adapted from:  linux/drivers/video/skeletonfb.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>

#include <mach/io.h>
#include <mach/irqs.h>
#include <mach/pxa168fb.h>
#include <mach/hardware.h>

#include <mach/gpio.h>
//#include <asm/arch/pxa910.h>
#include <linux/kthread.h>
//#include <asm/delay.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

#include "pxa168fb.h"
#include "cbp_pxa168_fb.h"

#define PXA910_FBNAME "pxa168-fb"

static void do_vsync_tasklet(unsigned long);
DECLARE_TASKLET(vsync_tasklet, do_vsync_tasklet, (unsigned long)0);

/*
 * Debug functions
 */
#define PXA910_FB_DEBUG
//#undef PXA910_FB_DEBUG
#ifdef PXA910_FB_DEBUG
#define DPRINTK(a,b...) printk(KERN_DEBUG "fb: %s: " a, __FUNCTION__ , ## b)
#else
#define DPRINTK(a,b...)
#endif

#define lcd_write(reg, val)	(*((volatile unsigned int *)((reg) + lcd_regs_base)) = (val))
#define lcd_read(reg)		(*((volatile unsigned int *)((reg) + lcd_regs_base)))

unsigned char __iomem *lcd_regs_base;

static int pxa910_fb_validate_var(struct fb_var_screeninfo *var);
static int pxa910_fb_map_RGB_memory( struct fb_info *info);
static int pxa910_fb_map_YUV_memory( struct fb_info *info);

static inline u_int chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static void set_timing_registers(struct fb_var_screeninfo *var)
{
        unsigned int temp;

	DPRINTK("Entering...\n");
	DPRINTK("\tvar->xres:%d\n", var->xres);
	DPRINTK("\tvar->yres:%d\n", var->yres);
	DPRINTK("\tvar->xres_virtual:%d\n", var->xres_virtual);
	DPRINTK("\tvar->yres_virtual:%d\n", var->yres_virtual);
	DPRINTK("\tvar->height:%d\n", var->height);
	DPRINTK("\tvar->bits_per_pixel:%d\n", var->bits_per_pixel);
	DPRINTK("\tvar->left_margin:%d\n", var->left_margin);
	DPRINTK("\tvar->right_margin:%d\n", var->right_margin);
	DPRINTK("\tvar->upper_margin:%d\n", var->upper_margin);
	DPRINTK("\tvar->lower_margin:%d\n", var->lower_margin);
	DPRINTK("\tvar->vsync_len:%d\n", var->vsync_len);
	DPRINTK("\tvar->hsync_len:%d\n", var->hsync_len);
	DPRINTK("\tvar->sync:%d\n", var->sync);
	DPRINTK("\tvar->nonstd:%d\n", var->nonstd);


        temp = ((var->yres << 16) | var->xres);
        lcd_write(LCD_SPU_V_H_ACTIVE, temp);
        DPRINTK("LCD_SPU_V_H_ACTIVE = %x\n", temp);

        temp = ((var->left_margin << 16) | var->right_margin);
        lcd_write(LCD_SPU_H_PORCH, temp);
        DPRINTK("LCD_SPU_H_PORCH = %x\n", temp);

        temp = ((var->upper_margin << 16) | var->lower_margin);
        lcd_write(LCD_SPU_V_PORCH, temp);
        DPRINTK("LCD_SPU_V_PORCH = %x\n", temp);

        temp = (((var->yres + var->upper_margin + var->lower_margin + var->vsync_len) << 16) |
                (var->xres + var->left_margin + var->right_margin + var->hsync_len));
        lcd_write(LCD_SPUT_V_H_TOTAL, temp);
        DPRINTK("LCD_SPUT_V_H_TOTAL = %x\n", temp);

        if (var->pixclock == 0)
                var->pixclock = 39; /* default for 800x480 */

        temp = PIX_CLK_SOURCE/var->pixclock;
        temp |= 0x80000000;
        lcd_write(LCD_CFG_SCLK_DIV, temp);
        DPRINTK("LCD_CFG_SCLK_DIV = %x\n", temp);

        /* TODO: just hard-code this one for now. */
#if defined(CONFIG_MACH_TETON) || defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	lcd_write(LCD_SPU_DUMB_CTRL, 0x210ff10f);
#else
        lcd_write(LCD_SPU_DUMB_CTRL, 0x610ff10f);
#endif
        DPRINTK("LCD_SPU_DUMB_CTRL = %x\n", temp);

        /* setup the IOPAD_CONTROL */
        temp = 0x0;
#if defined(CONFIG_MACH_TETON) || defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	/* hard-code 18 bit data and AXI control*/
	temp |= 0x22;
#else
	/* hard-code 24 bit data and AXI control*/
	temp |= 0x20;
#endif
	lcd_write(LCD_SPU_IOPAD_CONTROL, temp);
}

static int pxa910_fb_setcolreg(unsigned regno, unsigned red, unsigned green,
			   unsigned blue, unsigned transp,
			   struct fb_info *pinfo)
{
	unsigned int val;
	int ret = 1;

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (pinfo->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green
					+ 7471 * blue) >> 16;

	switch (pinfo->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = pinfo->pseudo_palette;

			val  = chan_to_field(red, &pinfo->var.red);
			val |= chan_to_field(green, &pinfo->var.green);
			val |= chan_to_field(blue, &pinfo->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;
	default:
		printk("ERROR: pxa910_fb: Color setting not supported.\n");
		break;
	}

	return ret;
}

void pxa910_fb_lcd_enable(struct pxa910_fb_info *inf)
{
	struct fb_var_screeninfo *var = &inf->fb.var;
	unsigned int dat_tmp;
	DPRINTK("Entering...\n");

	/* enable LCD controller clock */
	clk_enable(inf->clk);

	/* Video frame initialization */
	lcd_write(LCD_SPU_DMA_START_ADDR_Y0, 0x0);
	lcd_write(LCD_SPU_DMA_START_ADDR_U0, 0x0);
	lcd_write(LCD_SPU_DMA_START_ADDR_V0, 0x0);
	lcd_write(LCD_CFG_DMA_START_ADDR_C0, 0x0);
	lcd_write(LCD_SPU_DMA_START_ADDR_Y1, 0x0);
	lcd_write(LCD_SPU_DMA_START_ADDR_U1, 0x0);
	lcd_write(LCD_SPU_DMA_START_ADDR_V1, 0x0);
	lcd_write(LCD_CFG_DMA_START_ADDR_C1, 0x0);
	lcd_write(LCD_SPU_DMA_PITCH_YC, 0xf0);
	lcd_write(LCD_SPU_DMA_PITCH_UV, 0x0);
	lcd_write(LCD_SPUT_DMA_OVSA_HPXL_VLN, 0x0);
	lcd_write(LCD_SPU_DMA_HPXL_VLN, 0x0);
	lcd_write(LCD_SPU_DZM_HPXL_VLN, 0x0);

	/* Graphic frame initialization */
	lcd_write(LCD_CFG_GRA_START_ADDR0,
		  (unsigned long)inf->map_dma);
	lcd_write(LCD_CFG_GRA_START_ADDR1, (unsigned long)inf->map_dma);

	switch (var->bits_per_pixel) {
	case 32:
		lcd_write(LCD_CFG_GRA_PITCH, var->xres << 2);
		break;
	case 24:
		lcd_write(LCD_CFG_GRA_PITCH, var->xres * 3);
		break;
	case 16:
		lcd_write(LCD_CFG_GRA_PITCH, var->xres << 1);
		break;
	case 8:
		lcd_write(LCD_CFG_GRA_PITCH, var->xres);
		break;
	default:
		printk(KERN_ERR "pxa910_fb: unsupported pixel formats\n");
		break;
	}

	lcd_write(LCD_SPU_GRA_OVSA_HPXL_VLN, 0);

	lcd_write(LCD_SPU_GRA_HPXL_VLN, (var->yres << 16) | var->xres);
	lcd_write(LCD_SPU_GZM_HPXL_VLN, (var->height<< 16) | var->width);

	/* hardware cursor initialization */
	// CHUMBY - placement SHOULDN'T matter since horizontal and vertical size are set to 0 here
#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	lcd_write(LCD_SPU_HWC_OVSA_HPXL_VLN, 0x00000000);
#else
	lcd_write(LCD_SPU_HWC_OVSA_HPXL_VLN, 0x006800A8);
#endif
	lcd_write(LCD_SPU_HWC_HPXL_VLN, 0x00000000);

	/* Screen setup */
	/* We need a way to describe the screen */
	/* v_total = v_active + v_porch_head + v_porth_tail  */
	/* h_total = h_active + h_porch_head + h_porth_tail  */
	/* v_active = yres */
	/* h_active = xres */

	lcd_write(LCD_SPU_BLANKCOLOR, 0x00ff0000);

	/* alphy & color key setup */
	lcd_write(LCD_SPU_ALPHA_COLOR1, 0x00ff0000);
	lcd_write(LCD_SPU_ALPHA_COLOR2, 0x00ffff00);
	lcd_write(LCD_SPU_COLORKEY_Y, 0x0);
	lcd_write(LCD_SPU_COLORKEY_U, 0x0);
	lcd_write(LCD_SPU_COLORKEY_V, 0x0);

	/* internal SRAM setup */
	lcd_write(LCD_SPU_SRAM_CTRL, 0x0);
	lcd_write(LCD_SPU_SRAM_WRDAT, 0x0);
	lcd_write(LCD_SPU_SRAM_PARA0, 0x0);
	lcd_write(LCD_SPU_SRAM_PARA1, 0x0);

	/* DMA setup */
	lcd_write(LCD_SPU_DMA_CTRL0, 0x00000100);
	lcd_write(LCD_SPU_DMA_CTRL1, 0x2001ff00);

	lcd_write(LCD_SPU_CONTRAST, 0x00004000);
	lcd_write(LCD_SPU_SATURATION, 0x20004000);
	lcd_write(LCD_SPU_CBSH_HUE, 0x00004000);

	/* General setup XXX Hard Coded XXX*/
#if defined(CONFIG_MACH_TETON) || defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	lcd_write(LCD_SPU_DMA_CTRL0, 0x03001111);
#else
	lcd_write(LCD_SPU_DMA_CTRL0, 0x03000101);	//FIXED: SWAPRB(bit4) and YUV2RGB(bit1) are disabled by default
#endif

#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	printk( "%s() at %s:%d wrote hardcoded 0x03001111\n", __FUNCTION__, __FILE__, __LINE__ );
#endif
	set_timing_registers(var);

	dat_tmp = lcd_read(LCD_SPU_DUMB_CTRL);
	dat_tmp |= 0x1;
#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	printk( "%s() LCD_SPU_DUMB_CTRL was %08x writing hardcoded value\n", __FUNCTION__, dat_tmp );
	lcd_write(LCD_SPU_DUMB_CTRL, 0x210ff10f);
#else
	lcd_write(LCD_SPU_DUMB_CTRL, dat_tmp);
#endif

	lcd_write(LCD_SPU_IRQ_ENA, 0x0);
	lcd_write(LCD_SPU_IRQ_ISR, 0x0);
}


void pxa910_fb_gra_enable(struct pxa910_fb_info *inf)
{
	struct fb_var_screeninfo *var = &inf->fb.var;
	unsigned int dat_tmp;
        unsigned int val, fmt;

	DPRINTK("Entering...\n");

	/* Graphic frame initialization */

	if (var->width == 0)
		var->width = var->xres;
	if (var->height == 0)
		var->height = var->yres;

	lcd_write(LCD_CFG_GRA_START_ADDR0,
		      (unsigned long)inf->map_dma);
	lcd_write(LCD_CFG_GRA_START_ADDR1, (unsigned long)inf->map_dma);

	switch (var->bits_per_pixel) {

	case 32:
		lcd_write(LCD_CFG_GRA_PITCH, var->xres << 2 );
		break;
	case 24:
		lcd_write(LCD_CFG_GRA_PITCH, var->xres * 3);
		break;
	case 16:
		lcd_write(LCD_CFG_GRA_PITCH, var->xres << 1);
		break;
	case 8:
		lcd_write(LCD_CFG_GRA_PITCH, var->xres);
		break;
	default:
		printk(KERN_ERR "pxa910_fb: unsupported pixel formats\n");
		break;
	}

	lcd_write(LCD_SPU_GRA_OVSA_HPXL_VLN, 0);

	lcd_write(LCD_SPU_GRA_HPXL_VLN, (var->yres << 16) | var->xres);
	lcd_write(LCD_SPU_GZM_HPXL_VLN, (var->height<< 16) | var->width);

	/* hardware cursor initialization */
#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	lcd_write(LCD_SPU_HWC_OVSA_HPXL_VLN, 0x00000000);
#else
	lcd_write(LCD_SPU_HWC_OVSA_HPXL_VLN, 0x006800A8);
#endif
	lcd_write(LCD_SPU_HWC_HPXL_VLN, 0x00000000);

	/* Screen setup */
	/* We need a way to describe the screen */
	/* v_total = v_active + v_porch_head + v_porth_tail  */
	/* h_total = h_active + h_porch_head + h_porth_tail  */
	/* v_active = yres */
	/* h_active = xres */

	val = lcd_read(LCD_SPU_DMA_CTRL0);
	fmt = inf->format == FORMAT_RGB565?0:
		  inf->format == FORMAT_RGB1555?1:
		  inf->format == FORMAT_RGB888_PACKED?2:
		  inf->format == FORMAT_RGBA888_UNPACKED?3:
		  inf->format == FORMAT_RGBA888?4:
		  inf->format == FORMAT_YUV422_PACKED?5:
		  inf->format == FORMAT_YUV422_PLANAR?6:
		  inf->format == FORMAT_YUV420_PLANAR?7:
			0;
	DPRINTK("- %s(%d):\n", __func__, __LINE__);
	DPRINTK("\tinf->format:%ld\n", inf->format);
	DPRINTK("\tinf->buffer_num:%d\n", inf->buffer_num);
	DPRINTK("\tfmt:%d\n", fmt);
	DPRINTK("\tvar->xres:%d\n", var->xres);
	DPRINTK("\tvar->yres:%d\n", var->yres);
	DPRINTK("\tvar->xres_virtual:%d\n", var->xres_virtual);
	DPRINTK("\tvar->yres_virtual:%d\n", var->yres_virtual);
	DPRINTK("\tvar->height:%d\n", var->height);
	DPRINTK("\tvar->width:%d\n", var->width);
	DPRINTK("\tinf->xpos:%ld\n", inf->xpos);
	DPRINTK("\tinf->yres:%ld\n", inf->ypos);
	DPRINTK("\tinf->ypitch:%d\n", inf->ypitch);
	DPRINTK("\tinf->uvpitch:%d\n", inf->uvpitch);
	DPRINTK("\tinf->map_cpu:0x%08x\n", (unsigned int)inf->map_cpu);
	DPRINTK("\tinf->map_dma:0x%08x\n", inf->map_dma);
	DPRINTK("\tinf->map_size:0x%08x\n", inf->map_size);
	DPRINTK("\tinf->yoff:0x%08x\n", inf->yoff);
	DPRINTK("\tinf->cboff:0x%08x\n", inf->cboff);
	DPRINTK("\tinf->croff:0x%08x\n", inf->croff);

	val &= ~(0xf << LCD_SPU_DMA_CTRL0_CFG_GRAFORMAT_SHIFT);
	val |= fmt << LCD_SPU_DMA_CTRL0_CFG_GRAFORMAT_SHIFT;	//FIXED: LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPRB is removed
	if (inf->format == FORMAT_YUV422_PACKED ||
		inf->format == FORMAT_YUV422_PLANAR ||
		inf->format == FORMAT_YUV420_PLANAR) {
		val |= LCD_SPU_DMA_CTRL0_CFG_YUV2RGB_GRA;
	}
#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	printk( "%s() at %s:%d wrote 0x%08lx -> LCD_SPU_DMA_CTRL0\n", __FUNCTION__, __FILE__, __LINE__, val );
#endif
	lcd_write(LCD_SPU_DMA_CTRL0, val);


	set_timing_registers(var);

	dat_tmp = lcd_read(LCD_SPU_DUMB_CTRL);
	dat_tmp |= 0x1;
	lcd_write(LCD_SPU_DUMB_CTRL, dat_tmp);
}

static void do_vsync_tasklet(unsigned long param)
{
	u32 reg;

	reg = lcd_read(LCD_SPU_IRQ_ENA) | IRQ_VSYNC_IRQ_ENA;
	lcd_write(LCD_SPU_IRQ_ISR, reg);
}

static irqreturn_t pxa910_fb_handle_irq(int irq, void *dev_id)
{
	struct pxa910_fb_controller *controller = dev_id;
	u32 reg;

	reg = lcd_read(LCD_SPU_IRQ_ISR);
	lcd_write(LCD_SPU_IRQ_ISR, reg & ~reg);

	if (reg & ISR_VSYNC_IRQ) {
		wake_up(&controller->ctrlr_wait);
	}

	return IRQ_HANDLED;
}

static int pxa910_fb_mmap(struct fb_info *info,
		      struct vm_area_struct *vma)
{
	struct pxa910_fb_info *fbi = (struct pxa910_fb_info *)info;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned int ret;
	DPRINTK("Entering...\n");

	if (off < info->fix.smem_len) {
		ret = dma_mmap_writecombine(NULL, vma, fbi->map_cpu,
					     fbi->map_dma,
				 	     fbi->map_size);

		DPRINTK("fbi->map_cpu = %p\n", (void *)fbi->map_cpu);
		DPRINTK("fbi->map_dma = %p\n", (void *)fbi->map_dma);
		DPRINTK("fbi->map_size = %p\n", (void *)fbi->map_size);

		return ret;
	}
	return -EINVAL;
}

static int __init pxa910_fb_map_video_memory(struct pxa910_fb_info *fbi)
{
	DPRINTK("Entering...\n");
	if (fbi->map_cpu)
		dma_free_writecombine(fbi->dev,  fbi->map_size, (void*)fbi->map_cpu, fbi->map_dma);
	fbi->map_size = PAGE_ALIGN(fbi->fb.fix.smem_len + 0x4000);
	fbi->map_cpu = dma_alloc_writecombine(fbi->dev, fbi->map_size,
						&fbi->map_dma,
						GFP_KERNEL);
	DPRINTK("fbi->map_cpu = %p\n", (void *)fbi->map_cpu);
	DPRINTK("fbi->map_dma = %p\n", (void *)fbi->map_dma);
	DPRINTK("fbi->map_size = %p\n", (void *)fbi->map_size);


	if (fbi->map_cpu == NULL) {
		printk(KERN_ERR "Unable to allocate fb memory.\n");
		return -ENOMEM;
	}

	fbi->fb.screen_base = fbi->map_cpu;
	fbi->fb.fix.smem_start = (u32)fbi->map_dma;

	fbi->map_cpu2 = fbi->map_cpu + fbi->fb.fix.smem_len/2;
	fbi->map_dma2 = fbi->map_dma + fbi->fb.fix.smem_len/2;

	DPRINTK("fbi->fb.screen_base = %p\n",  (void *)fbi->fb.screen_base);
	DPRINTK("fbi->fb.fix.smem_start = %p\n",  (void *)fbi->fb.fix.smem_start);
	DPRINTK("fbi->map_dma2  = %p\n",(void *)fbi->map_dma2 );
	DPRINTK("fbi->map_dma2 = %p\n", (void *)fbi->map_dma2);

	return 0;
}

static int pxa910_vid_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{

  int blendval;
  int val;
  unsigned char param;
  struct pxa910_fb_info *pinf = (struct pxa910_fb_info*)info;
  void __user *argp = (void __user *)arg;
  unsigned int tmp = 0;
  unsigned int tmp2 = 0;

  DPRINTK("Entering...\n");


  switch (cmd) {

  case FBIOGET_FB0_DMA_BLANK_COLOR:

          pinf->dma_bc.red = (lcd_read(LCD_SPU_BLANKCOLOR) & 0xff);
          pinf->dma_bc.green = ((lcd_read(LCD_SPU_BLANKCOLOR) >> 8) & 0xff);
          pinf->dma_bc.blue = ((lcd_read(LCD_SPU_BLANKCOLOR) >> 16) & 0xff);
          pinf->dma_bc.disabled = (lcd_read(LCD_SPU_DMA_CTRL0) & (1<<8)) ? 0 : 1;

          return copy_to_user(argp, &pinf->dma_bc, sizeof(struct pxa910_fb_dma_blankcolor)) ? -EFAULT : 0;

  case FBIOPUT_FB0_DMA_BLANK_COLOR:
          if (copy_from_user(&pinf->dma_bc, argp, sizeof(struct pxa910_fb_dma_blankcolor )))
                  return -EFAULT;

          lcd_write(LCD_SPU_BLANKCOLOR,
                    pinf->dma_bc.red |
                    (pinf->dma_bc.green << 8) |
                    (pinf->dma_bc.blue << 16));

          tmp = lcd_read(LCD_SPU_DMA_CTRL0);
          if(pinf->dma_bc.disabled) {
                  lcd_write(LCD_SPU_DMA_CTRL0, tmp & 0xfffffeff);
#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	printk( "%s() at %s:%d wrote 0x%08lx -> LCD_SPU_DMA_CTRL0\n", __FUNCTION__, __FILE__, __LINE__, tmp );
#endif
          }
          else {
                  lcd_write(LCD_SPU_DMA_CTRL0, tmp | (1 << 8));
#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	printk( "%s() at %s:%d wrote 0x%08lx -> LCD_SPU_DMA_CTRL0\n", __FUNCTION__, __FILE__, __LINE__, tmp );
#endif
          }

          return 0;

  case FBIOGET_CHROMAKEYS:
          return copy_to_user(argp, &pinf->chroma, sizeof(struct pxa910_fb_chroma)) ? -EFAULT : 0;

  case FBIOPUT_CHROMAKEYS:
          if (copy_from_user(&pinf->chroma, argp, sizeof(struct pxa910_fb_chroma)))
                  return -EFAULT;

          tmp = ((pinf->chroma.y_alpha) |
                 (pinf->chroma.y << 8) |
                 (pinf->chroma.y1 << 16) |
                 (pinf->chroma.y2 << 24));
          lcd_write(LCD_SPU_COLORKEY_Y, tmp);

          tmp = ((pinf->chroma.u_alpha) |
                 (pinf->chroma.u << 8) |
                 (pinf->chroma.u1 << 16) |
                 (pinf->chroma.u2 << 24));
          lcd_write(LCD_SPU_COLORKEY_U, tmp);

          tmp = ((pinf->chroma.v_alpha) |
                 (pinf->chroma.v << 8) |
                 (pinf->chroma.v1 << 16) |
                 (pinf->chroma.v2 << 24));
          lcd_write(LCD_SPU_COLORKEY_V, tmp);

          tmp = ((pinf->chroma.mode & 0x7) << 24);
          tmp2 = lcd_read(LCD_SPU_DMA_CTRL1); /* get current value */
          tmp2 = (tmp2 & ~(0x7 << 24));       /* clear the mode */
          tmp |= tmp2;
          lcd_write(LCD_SPU_DMA_CTRL1, tmp);

          if (copy_to_user(argp, &pinf->chroma, sizeof(struct pxa910_fb_chroma)))
                  return -EFAULT;
          return 0;

  case FBIOPUT_VIDEO_ALPHABLEND:
          /*
           *  This puts the blending control to the Video layer.
           *
           */

          val = lcd_read(LCD_SPU_DMA_CTRL1);
          val &= ~LCD_SPU_DMA_CTRL1_CFG_ALPHA_MODE_MSK;
          val &= ~LCD_SPU_DMA_CTRL1_CFG_ALPHA_MSK;
          val |= (0x0 << LCD_SPU_DMA_CTRL1_CFG_ALPHA_MODE_SHIFT);
          val |= (0xff << LCD_SPU_DMA_CTRL1_CFG_ALPHA_SHIFT);
          lcd_write( LCD_SPU_DMA_CTRL1, val);
          return 0;

          break;


  case FBIOPUT_GLOBAL_ALPHABLEND:

    /*
     *  The userspace application can specify a byte value for the amount of global blend
     *  between the video layer and the graphic layer.
     *
     *  The alpha blending is per the formula below:
     *  P = (V[P] * blendval/255) + (G[P] * (1 - blendval/255))
     *
     *     where: P = Pixel value, V = Video Layer, and G = Graphic Layer
     */

    blendval = (arg & 0xff);

    val = lcd_read(LCD_SPU_DMA_CTRL1);
    val &= ~LCD_SPU_DMA_CTRL1_CFG_ALPHA_MODE_MSK;
    val &= ~LCD_SPU_DMA_CTRL1_CFG_ALPHA_MSK;
    val |= (0x2 << LCD_SPU_DMA_CTRL1_CFG_ALPHA_MODE_SHIFT);
    val |= (blendval << LCD_SPU_DMA_CTRL1_CFG_ALPHA_SHIFT);
    lcd_write( LCD_SPU_DMA_CTRL1, val);

    return 0;

    break;

  case FBIOPUT_GRAPHIC_ALPHABLEND:

    /*
     *  This puts the blending back to the default mode of allowing the
     *  graphic layer to do pixel level blending.
     *
     */

    val = lcd_read(LCD_SPU_DMA_CTRL1);
    val &= ~LCD_SPU_DMA_CTRL1_CFG_ALPHA_MODE_MSK;
    val &= ~LCD_SPU_DMA_CTRL1_CFG_ALPHA_MSK;
    val |= (0x1 << LCD_SPU_DMA_CTRL1_CFG_ALPHA_MODE_SHIFT);
    val |= (0xff << LCD_SPU_DMA_CTRL1_CFG_ALPHA_SHIFT);
    lcd_write( LCD_SPU_DMA_CTRL1, val);

    return 0;

    break;

  case FBIOPUT_SWAP_GRAPHIC_RED_BLUE:

    /*
     *  This swaps the red and blue color data from the frame buffer
     *  used for the graphic plane.
     *  A value of 1 enables the swap, 0 disables.
     *
     */

    param = (arg & 0x1);

    val = lcd_read(LCD_SPU_DMA_CTRL0);
    val &= ~LCD_SPU_DMA_CTRL0_CFG_GRA_SWAPRB;
    val |= (param << LCD_SPU_DMA_CTRL0_CFG_GRA_SWAPRB_SHIFT);
#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	printk( "%s() at %s:%d wrote 0x%08lx -> LCD_SPU_DMA_CTRL0\n", __FUNCTION__, __FILE__, __LINE__, val );
#endif
    lcd_write( LCD_SPU_DMA_CTRL0, val);

    return 0;

    break;

  case FBIOPUT_SWAP_VIDEO_RED_BLUE:

    /*
     *  This swaps the red and blue color data from the frame buffer
     *  used for the video plane.
     *  A value of 1 enables the swap, 0 disables.
     *
     */

    param = (arg & 0x1);

    val = lcd_read(LCD_SPU_DMA_CTRL0);
    val &= ~LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPRB;
    val |= (param << LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPRB_SHIFT);
#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	printk( "%s() at %s:%d wrote 0x%08lx -> LCD_SPU_DMA_CTRL0\n", __FUNCTION__, __FILE__, __LINE__, val );
#endif
    lcd_write( LCD_SPU_DMA_CTRL0, val);

    return 0;

    break;

  case FBIOPUT_SWAP_VIDEO_U_V:

    /*
     *  This swaps the U and V color data from the frame buffer
     *  used for the video plane.
     *  A value of 1 enables the swap, 0 disables.
     *
     */

    param = (arg & 0x1);

    val = lcd_read(LCD_SPU_DMA_CTRL0);
    val &= ~LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPUV;
    val |= (param << LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPUV_SHIFT);
#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	printk( "%s() at %s:%d wrote 0x%08lx -> LCD_SPU_DMA_CTRL0\n", __FUNCTION__, __FILE__, __LINE__, val );
#endif
    lcd_write( LCD_SPU_DMA_CTRL0, val);

    return 0;

    break;

  case FBIOPUT_SWAP_VIDEO_Y_UV:

    /*
     *  This swaps the Y and UV color data from the frame buffer
     *  used for the video plane.
     *  A value of 1 enables the swap, 0 disables.
     *
     */

    param = (arg & 0x1);

    val = lcd_read(LCD_SPU_DMA_CTRL0);
    val &= ~LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPYU;
    val |= (param << LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPYU_SHIFT);
#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	printk( "%s() at %s:%d wrote 0x%08lx -> LCD_SPU_DMA_CTRL0\n", __FUNCTION__, __FILE__, __LINE__, val );
#endif
    lcd_write( LCD_SPU_DMA_CTRL0, val);

    return 0;

    break;

  case FBIOPUT_SWAP_GRAPHIC_U_V:

    /*
     *  This swaps the U and V color data from the frame buffer
     *  used for the graphic plane.
     *  A value of 1 enables the swap, 0 disables.
     *
     */

    param = (arg & 0x1);

    val = lcd_read(LCD_SPU_DMA_CTRL0);
    val &= ~LCD_SPU_DMA_CTRL0_CFG_GRA_SWAPUV;
    val |= (param << LCD_SPU_DMA_CTRL0_CFG_GRA_SWAPUV_SHIFT);
#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	printk( "%s() at %s:%d wrote 0x%08lx -> LCD_SPU_DMA_CTRL0\n", __FUNCTION__, __FILE__, __LINE__, val );
#endif
    lcd_write( LCD_SPU_DMA_CTRL0, val);

    return 0;

    break;

  case FBIOPUT_SWAP_GRAPHIC_Y_UV:

    /*
     *  This swaps the Y and UV color data from the frame buffer
     *  used for the graphic plane.
     *  A value of 1 enables the swap, 0 disables.
     *
     */

    param = (arg & 0x1);

    val = lcd_read(LCD_SPU_DMA_CTRL0);
    val &= ~LCD_SPU_DMA_CTRL0_CFG_GRA_SWAPYU;
    val |= (param << LCD_SPU_DMA_CTRL0_CFG_GRA_SWAPYU_SHIFT);
#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	printk( "%s() at %s:%d wrote 0x%08lx -> LCD_SPU_DMA_CTRL0\n", __FUNCTION__, __FILE__, __LINE__, val );
#endif
    lcd_write( LCD_SPU_DMA_CTRL0, val);

    return 0;
    break;

  default:
    return -EINVAL;
  }
}
/*
 * Select the smallest mode that allows the desired resolution to be
 * displayed. If desired parameters can be rounded up.
 */
static struct pxa910_fb_mode_info *pxa910_fb_getmode(struct pxa910_fb_mach_info *mach, struct fb_var_screeninfo *var, int userevent)
{
        struct pxa910_fb_mode_info *mode = NULL;
        struct pxa910_fb_mode_info *modelist = mach->modes;
        unsigned int best_x = 0xffffffff, best_y = 0xffffffff;
        unsigned int i;
	unsigned long xres, yres;

        DPRINTK("Entering...\n");

	if (userevent) {
		xres = var->xres;
		yres = var->yres;
	} else {
		xres = var->width;
		yres = var->height;
	}

        for (i = 0 ; i < mach->num_modes ; i++) {
                if (modelist[i].xres >= var->xres && modelist[i].yres >= var->yres &&
                                modelist[i].xres < best_x && modelist[i].yres < best_y) {
                        best_x = modelist[i].xres;
                        best_y = modelist[i].yres;
                        mode = &modelist[i];
                }
        }

        DPRINTK("Best mode selected is best_x = %d, best_y = %d\n", best_x, best_y);
        return mode;
}

static void pxa910_fb_setmode(struct fb_var_screeninfo *var,
			  struct pxa910_fb_mode_info *mode)
{
	DPRINTK("Entering %s... %dbpp %dX%d\n", __FUNCTION__, mode->bpp, mode->xres, mode->yres );

	var->xres               = mode->xres;
	var->yres               = mode->yres;
	var->bits_per_pixel     = mode->bpp;
	var->left_margin        = mode->left_porch;
	var->right_margin       = mode->right_porch;
	var->upper_margin       = mode->upper_porch;
	var->lower_margin       = mode->lower_porch;
	var->hsync_len          = mode->h_sync;
	var->vsync_len          = mode->v_sync;
	var->sync               = mode->sync;
	var->pixclock           = mode->pixel_clk;

        var->xres_virtual       = mode->xres;
        var->yres_virtual       = mode->yres;

}

static int pxa910_fb_set_par(struct fb_info *info)
{
	struct pxa910_fb_info *fbi = (struct pxa910_fb_info *)info;
	struct fb_var_screeninfo *var = &fbi->fb.var;
        int err = EINVAL;
        unsigned char *ofs = (unsigned char *)fbi->map_dma;

	DPRINTK("Entering...\n");

        fbi->format = (var->nonstd>>20) & 0xf;

	pxa910_fb_gra_enable(fbi);

	if ( fbi->format == FORMAT_RGB565 ||
			fbi->format == FORMAT_RGB1555 ||
			fbi->format == FORMAT_RGB888_PACKED ||
			fbi->format == FORMAT_RGBA888_UNPACKED ||
			fbi->format == FORMAT_RGBA888)
		err = pxa910_fb_map_RGB_memory(info);
	else
		err = pxa910_fb_map_YUV_memory(info);

        /* page flip */
        ofs += ((fbi->fb.var.yoffset / fbi->fb.var.yres) * (fbi->fb.var.yres)
               * (fbi->fb.var.xres) * (fbi->fb.var.bits_per_pixel / 8));

        lcd_write(LCD_CFG_GRA_START_ADDR0, (unsigned long) ofs);

	return err;
}

static int pxa910_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
        struct pxa910_fb_info *fbi = (struct pxa910_fb_info *)info;
	struct pxa910_fb_mach_info *inf = fbi->dev->platform_data;
	int format = -1;

	DPRINTK("Entering...\n");

        if (var->width == 0)
                var->width = var->xres;

        if (var->height == 0)
                var->height = var->yres;

        if (inf->fixed_modes) {
                struct pxa910_fb_mode_info *mode;
                struct pxa910_fb_mode_info mode2;

		if (info->flags & FBINFO_MISC_USEREVENT)
	                mode = pxa910_fb_getmode(inf, var, 1);
		else
			mode = pxa910_fb_getmode(inf, var, 0);
                memcpy(&mode2, mode, sizeof(struct pxa910_fb_mode_info));
                mode = &mode2;
                mode->bpp = var->bits_per_pixel;
                if (!mode)
                        return -EINVAL;
                pxa910_fb_setmode(var, mode);		//TODO

        } else {
		if (var->width > inf->modes->xres)
			return -EINVAL;
		if (var->height > inf->modes->yres)
			return -EINVAL;
		if ((var->bits_per_pixel != 1) &&
				(var->bits_per_pixel != 2) &&
				(var->bits_per_pixel != 4) &&
				(var->bits_per_pixel != 8) &&
				(var->bits_per_pixel != 16) &&
				(var->bits_per_pixel != 18) &&
				(var->bits_per_pixel != 19))
			return -EINVAL;
	}

	var->xres_virtual =
		max(var->xres_virtual, var->xres);
	var->yres_virtual =
		max(var->yres_virtual, var->yres);

	if ((var->yres_virtual/var->yres) > fbi->buffer_num)
	{
		DPRINTK("buffer number = %d and max buffer number = %d\n",
			var->yres_virtual/var->yres, fbi->buffer_num);
		return -EINVAL;
	}

	switch(var->bits_per_pixel) {
	case 16:
		var->red.offset   = 11; var->red.length   = 5;
		var->green.offset = 5;  var->green.length = 6;
		var->blue.offset  = 0;  var->blue.length  = 5;
		var->transp.offset = var->transp.length = 0;
		break;
	case 18:
	case 19:
		var->red.offset   = 12; var->red.length   = 6;
		var->green.offset = 6;  var->green.length = 6;
		var->blue.offset  = 0;  var->blue.length  = 6;
		var->transp.offset = var->transp.length = 0;
		break;
	default:
		var->red.offset = var->green.offset = var->blue.offset = var->transp.offset = 0;
		var->red.length   = 8;
		var->green.length = 8;
		var->blue.length  = 8;
		var->transp.length = 0;
	}

        format = (var->nonstd >> 20) & 0xf;


        /* Palnar YCbCr444, YCbCr422, YCbCr420 */
        switch (format) {
        case FORMAT_RGB565:
        case FORMAT_RGB1555:
        case FORMAT_RGB888_PACKED:
        case FORMAT_RGBA888_UNPACKED:
        case FORMAT_RGBA888:
        case FORMAT_YUV422_PACKED:
	break;

        default:
                printk(KERN_ERR "No valid color format chosen\n");
                return -EINVAL;
        }


	return 0;
}

int pxa910_fb_release(struct fb_info *info, int user)
{
	struct pxa910_fb_info *fbi = (struct pxa910_fb_info *)info;
	int val;

	/* restore single buffer mode so that
	 * the base plane will not be affected when the overlay
	 * buffers are switched
	 */
	lcd_write(LCD_CFG_GRA_START_ADDR1, (unsigned long)fbi->map_dma);

	/* disable Vsync interrtup */
	val = lcd_read(LCD_SPU_IRQ_ENA);
	val &= ~IRQ_VSYNC_IRQ_ENA;
	lcd_write(LCD_SPU_IRQ_ENA, val);

	return 0;
}

static struct fb_ops pxa910_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= pxa910_fb_setcolreg,
	.fb_release	= pxa910_fb_release,
	.fb_fillrect    = cfb_fillrect,
	.fb_copyarea    = cfb_copyarea,
	.fb_imageblit   = cfb_imageblit,
	.fb_mmap	= pxa910_fb_mmap,
	.fb_check_var	= pxa910_fb_check_var,
	.fb_set_par	= pxa910_fb_set_par,
	.fb_ioctl	= pxa910_vid_ioctl,
};

static struct pxa910_fb_info * __init pxa910_fb_init_fbinfo(struct device *dev)
{
        struct pxa910_fb_info *fbi;
	struct pxa910_fb_mach_info *inf = dev->platform_data;
	struct pxa910_fb_mode_info *mode = inf->modes;
	void *addr;
	struct fb_videomode fb_mode;
	struct fb_var_screeninfo var;
	int i, smemlen;
	DPRINTK("Entering...\n");

	fbi = kmalloc(sizeof(struct pxa910_fb_info) + sizeof(u32) * 16, GFP_KERNEL);
	if (!fbi)
		return NULL;

	memset(fbi, 0, sizeof(struct pxa910_fb_info) + sizeof(u32)*16);

        INIT_LIST_HEAD(&fbi->fb.modelist);
	for (i = 0; i < inf->num_modes; i++) {
		pxa910_fb_setmode(&var, &inf->modes[i]);
		fb_var_to_videomode(&fb_mode, &var);
		fb_add_videomode(&fb_mode, &fbi->fb.modelist);
	}

	fbi->dev = dev;
	fbi->fb.device = dev;

	strcpy(fbi->fb.fix.id, PXA910_FBNAME);

	pxa910_fb_setmode(&fbi->fb.var, mode);

	/*
	 * Setup the RGB parameters for this display.
	 *
	 * The pixel packing format is described on page 7-11 of the
	 * PXA2XX Developer's Manual.
	 */
	if (fbi->fb.var.bits_per_pixel == 16) {
		fbi->fb.var.red.offset   = 11;
		fbi->fb.var.red.length   = 5;
		fbi->fb.var.green.offset = 5;
		fbi->fb.var.green.length = 6;
		fbi->fb.var.blue.offset  = 0;
		fbi->fb.var.blue.length  = 5;
		fbi->fb.var.transp.offset = fbi->fb.var.transp.length = 0;
	} else {
		fbi->fb.var.red.offset = 0;
		fbi->fb.var.green.offset = 0;
		fbi->fb.var.blue.offset = 0;
		fbi->fb.var.transp.offset = 0;
		fbi->fb.var.red.length   = 8;
		fbi->fb.var.green.length = 8;
		fbi->fb.var.blue.length  = 8;
		fbi->fb.var.transp.length = 0;
	}

	if (fbi->fb.var.bits_per_pixel == 16)
		fbi->fb.fix.visual = FB_VISUAL_TRUECOLOR;
	else {
		/*
		 * Some people have weird ideas about wanting static
		 * pseudocolor maps.  I suspect their user space
		 * applications are broken.
		 */
		fbi->fb.fix.visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	}
	fbi->fb.fix.line_length = fbi->fb.var.xres_virtual *
				fbi->fb.var.bits_per_pixel / 8;

	fbi->fb.fix.type = FB_TYPE_PACKED_PIXELS;
	fbi->fb.fix.type_aux = 0;
	fbi->fb.fix.xpanstep = 1;
	fbi->fb.fix.ypanstep = 1;
	fbi->fb.fix.ywrapstep = 0;
	fbi->fb.fix.accel = FB_ACCEL_NONE;

	fbi->fb.var.nonstd = 0;
	fbi->fb.var.activate = FB_ACTIVATE_NOW;
	fbi->fb.var.height      = 0;
	fbi->fb.var.width       = 0;
	fbi->fb.var.accel_flags = 0;
	fbi->fb.var.vmode       = FB_VMODE_NONINTERLACED;

	fbi->fb.fbops           = &pxa910_fb_ops;
	fbi->fb.flags           = FBINFO_DEFAULT;
	fbi->fb.node            = -1;
	fbi->buffer_num		= (inf->num_buffers < 1)?1:inf->num_buffers;

	addr = fbi;
	addr = addr + sizeof(struct pxa910_fb_info);
	fbi->fb.pseudo_palette  = addr;

	for (i = 0; i < inf->num_modes; i++) {
		smemlen = (mode[i].xres * mode[i].yres * mode[i].bpp / 8);
		smemlen *= fbi->buffer_num;
		if (smemlen > fbi->fb.fix.smem_len)
			fbi->fb.fix.smem_len = smemlen;
	}

        fbi->clk = clk_get(dev, "LCDCLK");

	return fbi;
}

static int pxa910_fb_map_YUV_memory( struct fb_info *info)
{
        struct pxa910_fb_info *fbi = (struct pxa910_fb_info *)info;
	unsigned int aylen, acblen, acrlen;
	unsigned int xres,yres;
	unsigned int nbytes;
	DPRINTK("Entering...\n");

	fbi->ylen = fbi->cblen = fbi->crlen = 0;
	fbi->yoff = fbi->cboff = fbi->croff = 0;
	aylen = acblen = acrlen = 0;

	yres = fbi->fb.var.yres;

	switch(fbi->format) {
	case FORMAT_YUV420_PLANAR:
		pr_debug("420 planar\n");
		/* 16 pixels per line */
		xres = (fbi->fb.var.xres + 0xf) & (~0xf);
		xres = fbi->fb.var.xres;
		fbi->fb.fix.line_length = xres;

		nbytes = xres * yres;
		fbi->ylen = nbytes;
		fbi->cblen = fbi->crlen = (nbytes/4);
		fbi->ypitch = xres;
		fbi->uvpitch = xres >> 1;

		break;
	case FORMAT_YUV422_PLANAR:
		/* 8 pixles per line */
		pr_debug("422 planar\n");
		xres = (fbi->fb.var.xres + 0x7) & (~0x7);
		fbi->fb.fix.line_length = xres;

		nbytes = xres * yres;
		fbi->ylen  = nbytes;
		fbi->cblen = fbi->crlen = (nbytes/2);
		fbi->ypitch = xres;
		fbi->uvpitch = xres >> 1;
		break;
	case FORMAT_YUV422_PACKED:
		/* 8 pixles per line */
		pr_debug("422 packed\n");
		xres = (fbi->fb.var.xres + 0x7) & (~0x7);
		fbi->fb.fix.line_length = xres*2;

		nbytes = xres * yres * 2;
		fbi->ylen  = nbytes;
		fbi->cblen = fbi->crlen = 0;
		fbi->ypitch = xres * 2;
		fbi->uvpitch = xres * 2;
		break;

	default:
		return -EIO;
	}

	/* 16-bytes alignment for DMA */
	aylen  = (fbi->ylen + 0xf) & (~0xf);
	acblen = (fbi->cblen + 0xf) & (~0xf);
	acrlen = (fbi->crlen + 0xf) & (~0xf);

	/* offset */
	fbi->yoff = 0;
	fbi->cboff = aylen;
	fbi->croff = fbi->cboff + acblen;

	/* adjust for user */
	fbi->fb.var.red.length   = fbi->ylen;
	fbi->fb.var.red.offset   = fbi->yoff;
	fbi->fb.var.green.length = fbi->cblen;
	fbi->fb.var.green.offset = fbi->cboff;
	fbi->fb.var.blue.length  = fbi->crlen;
	fbi->fb.var.blue.offset  = fbi->croff;

	if(fbi->buffer_num == 1){
		fbi->fb.fix.smem_len = aylen + acblen + acrlen;
	}else{
		fbi->fb.fix.smem_len = PAGE_ALIGN(aylen + acblen + acrlen) * fbi->buffer_num;
	}

	/* alloc memory */
	fbi->map_size = PAGE_ALIGN(aylen + acblen + acrlen) * fbi->buffer_num;

        DPRINTK("Current map_size = %x\n", fbi->map_size);

	fbi->fb.fix.smem_start = fbi->map_dma;
	fbi->fb.fix.smem_len = fbi->map_size;

	return 0;
};

static int pxa910_fb_map_RGB_memory( struct fb_info *info)
{
        struct pxa910_fb_info *fbi = (struct pxa910_fb_info *)info;
	struct fb_var_screeninfo *var = &fbi->fb.var;
	int pixels_per_line=0 , nbytes=0;

	DPRINTK("Entering...\n");

	switch(fbi->format) {
	case FORMAT_RGB565:
		/* 2 pixels per line */
		pixels_per_line = (fbi->fb.var.xres + 0x1) & (~0x1);
		nbytes = 2;

		var->transp.offset = var->transp.length = 0;
        	var->red.offset   = 11; var->red.length   = 5;
	        var->green.offset = 5;  var->green.length = 6;
        	var->blue.offset  = 0;  var->blue.length  = 5;
		break;

	case FORMAT_RGB1555:
		/* 2 pixels per line */
		pixels_per_line = (fbi->fb.var.xres + 0x1) & (~0x1);
		nbytes = 2;

		var->transp.offset= 15; var->transp.length = 1;
        	var->red.offset   = 10; var->red.length   = 5;
	        var->green.offset = 5;  var->green.length = 5;
        	var->blue.offset  = 0;  var->blue.length  = 5;
		break;

	case FORMAT_RGB888_PACKED:
		pixels_per_line = fbi->fb.var.xres;
		nbytes = 3;

		var->transp.offset = 24; var->transp.length = 0;
        	var->red.offset   = 16; var->red.length   = 8;
	       	var->green.offset = 8;  var->green.length = 8;
        	var->blue.offset  = 0;  var->blue.length  = 8;
		break;

	case FORMAT_RGBA888_UNPACKED:
		pixels_per_line = fbi->fb.var.xres;
		nbytes = 4;

		var->transp.offset = 24; var->transp.length = 8;
        	var->red.offset   = 16; var->red.length   = 8;
	       	var->green.offset = 8;  var->green.length = 8;
        	var->blue.offset  = 0;  var->blue.length  = 8;
		break;

	case FORMAT_RGBA888:
		pixels_per_line = fbi->fb.var.xres;
		nbytes = 4;

		var->transp.offset = 24; var->transp.length = 8;
		var->red.offset   = 16;  var->red.length   = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset  = 0;   var->blue.length  = 8;
		break;

	default:
		return -EIO;
	}

	fbi->fb.fix.line_length = nbytes * pixels_per_line ;
	fbi->ypitch = fbi->fb.fix.line_length;
	fbi->uvpitch = 0;
	fbi->fb.fix.smem_len = PAGE_ALIGN(fbi->fb.fix.line_length * fbi->fb.var.yres) * fbi->buffer_num;

	fbi->map_size = fbi->fb.fix.smem_len;

        DPRINTK("Current map_size = %x\n", fbi->map_size);

	fbi->fb.fix.smem_start = fbi->map_dma;
	fbi->fb.fix.smem_len = fbi->map_size;

	return 0;
}

int pxa910_fb_vid_enable(struct fb_info *info)
{
        struct pxa910_fb_info *fbi = (struct pxa910_fb_info *)info;
	struct fb_var_screeninfo *var = &fbi->fb.var;
	unsigned int val, fmt;
	unsigned int dat_tmp;
	DPRINTK("Entering...\n");

	if (var->width == 0)
		var->width = var->xres;
	if (var->height == 0)
		var->height = var->yres;

	/* Video frame initialization */
	lcd_write(LCD_SPU_DMA_START_ADDR_Y0, fbi->map_dma + fbi->yoff);
	lcd_write(LCD_SPU_DMA_START_ADDR_U0, fbi->map_dma + fbi->cboff);
	lcd_write(LCD_SPU_DMA_START_ADDR_V0, fbi->map_dma + fbi->croff);
	lcd_write(LCD_CFG_DMA_START_ADDR_C0, 0x0);
	lcd_write(LCD_SPU_DMA_START_ADDR_Y1, fbi->map_dma2 + fbi->yoff);
	lcd_write(LCD_SPU_DMA_START_ADDR_U1, fbi->map_dma2 + fbi->cboff);
	lcd_write(LCD_SPU_DMA_START_ADDR_V1, fbi->map_dma2 + fbi->croff);
	lcd_write(LCD_CFG_DMA_START_ADDR_C1, 0x0);
	lcd_write(LCD_SPU_DMA_PITCH_YC, fbi->ypitch);
	lcd_write(LCD_SPU_DMA_PITCH_UV, (fbi->uvpitch << 16) | fbi->uvpitch);
	lcd_write(LCD_SPUT_DMA_OVSA_HPXL_VLN, (fbi->ypos << 16) | fbi->xpos);
	lcd_write(LCD_SPU_DMA_HPXL_VLN, (var->yres << 16) | var->xres);
	lcd_write(LCD_SPU_DZM_HPXL_VLN, (var->height<< 16) | var->width);


	val = lcd_read(LCD_SPU_DMA_CTRL0);
	fmt = fbi->format == FORMAT_RGB565?0:
		  fbi->format == FORMAT_RGB1555?1:
		  fbi->format == FORMAT_RGB888_PACKED?2:
		  fbi->format == FORMAT_RGBA888_UNPACKED?3:
		  fbi->format == FORMAT_RGBA888?4:
		  fbi->format == FORMAT_YUV422_PACKED?5:
		  fbi->format == FORMAT_YUV422_PLANAR?6:
		  fbi->format == FORMAT_YUV420_PLANAR?7:
			0;

	DPRINTK("- %s(%d):\n", __func__, __LINE__);
	DPRINTK("\tfbi->format:%ld\n", fbi->format);
	DPRINTK("\tfbi->buffer_num:%d\n", fbi->buffer_num);
	DPRINTK("\tfmt:%d\n", fmt);
	DPRINTK("\tvar->xres:%d\n", var->xres);
	DPRINTK("\tvar->yres:%d\n", var->yres);
	DPRINTK("\tvar->xres_virtual:%d\n", var->xres_virtual);
	DPRINTK("\tvar->yres_virtual:%d\n", var->yres_virtual);
	DPRINTK("\tvar->height:%d\n", var->height);
	DPRINTK("\tvar->width:%d\n", var->width);
	DPRINTK("\tfbi->xpos:%ld\n", fbi->xpos);
	DPRINTK("\tfbi->yres:%ld\n", fbi->ypos);
	DPRINTK("\tfbi->ypitch:%d\n", fbi->ypitch);
	DPRINTK("\tfbi->uvpitch:%d\n", fbi->uvpitch);
	DPRINTK("\tfbi->map_cpu:0x%08x\n", (unsigned int)fbi->map_cpu);
	DPRINTK("\tfbi->map_dma:0x%08x\n", fbi->map_dma);
	DPRINTK("\tfbi->map_size:0x%08x\n", fbi->map_size);
	DPRINTK("\tfbi->yoff:0x%08x\n", fbi->yoff);
	DPRINTK("\tfbi->cboff:0x%08x\n", fbi->cboff);
	DPRINTK("\tfbi->croff:0x%08x\n", fbi->croff);

	val &= ~(0xf << LCD_SPU_DMA_CTRL0_CFG_DMAFORMAT_SHIFT);
	val |= fmt << LCD_SPU_DMA_CTRL0_CFG_DMAFORMAT_SHIFT	//FIXED: LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPRB is removed
			| LCD_SPU_DMA_CTRL0_CFG_DMA_ENA;
	if (fbi->format == FORMAT_YUV422_PACKED ||
		fbi->format == FORMAT_YUV422_PLANAR ||
		fbi->format == FORMAT_YUV420_PLANAR) {
		val |= LCD_SPU_DMA_CTRL0_CFG_YUV2RGB_DMA
			| LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPUV;	//U and V is swapped by default. reason is not clear.
	}
#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	printk( "%s() at %s:%d wrote 0x%08lx -> LCD_SPU_DMA_CTRL0\n", __FUNCTION__, __FILE__, __LINE__, val );
#endif
	lcd_write(LCD_SPU_DMA_CTRL0, val);

        dat_tmp = lcd_read(LCD_SPU_DUMB_CTRL);
        dat_tmp |= 0x1;
        lcd_write(LCD_SPU_DUMB_CTRL, dat_tmp);

	return 0;
}

static int pxa910_fb_validate_var(struct fb_var_screeninfo *var)
{
	DPRINTK("Entering...\n");
        var->xres_virtual =
	    var->xres_virtual < var->xres ? var->xres : (var->xres_virtual
        /var->xres)*var->xres;
	var->yres_virtual =
	    var->yres_virtual < var->yres ? var->yres : (var->yres_virtual
        /var->yres)*var->yres;

	var->xoffset = 0;
	var->yoffset = (var->yoffset/var->yres)*var->yres;
	if (var->yoffset >= var->yres_virtual) {
		var->yoffset = var->yres_virtual - var->yres;
	}
	return 0;
}

int pxa910_fb_vid_open(struct fb_info *info, int user)
{
  DPRINTK("Entering...\n");
  if (pxa910_fb_vid_enable(info))
    return 1;
  else
    return 0;
}

int pxa910_fb_vid_release(struct fb_info *info, int user)
{
	int val;
        DPRINTK("Entering...\n");
	val = lcd_read(LCD_SPU_DMA_CTRL0);
        val &= ~(LCD_SPU_DMA_CTRL0_CFG_DMA_ENA | LCD_SPU_DMA_CTRL0_CFG_YUV2RGB_DMA);

#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	printk( "%s() at %s:%d wrote 0x%08lx -> LCD_SPU_DMA_CTRL0\n", __FUNCTION__, __FILE__, __LINE__, val );
#endif
	lcd_write(LCD_SPU_DMA_CTRL0, val);

	/* disable Vsync interrtup */
	val = lcd_read(LCD_SPU_IRQ_ENA);
	val &= ~IRQ_VSYNC_IRQ_ENA;
	lcd_write(LCD_SPU_IRQ_ENA, val);

	return 0;
}

int pxa910_fb_vid_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	int xpos, ypos, xres, yres;
	int format;
        DPRINTK("Entering...\n");
	xres = yres = 0;

	xpos = (var->nonstd & 0x3ff);
	ypos = (var->nonstd >> 10) & 0x3ff;
	format = (var->nonstd >> 20) & 0xf;


	/* Palnar YCbCr444, YCbCr422, YCbCr420 */
	switch (format) {
	case FORMAT_RGB565:
	case FORMAT_RGB1555:
	case FORMAT_RGB888_PACKED:
	case FORMAT_RGBA888_UNPACKED:
	case FORMAT_RGBA888:
		xres = var->xres;
		break;
	case FORMAT_YUV444_PACKED:
		break;
	case FORMAT_YUV444_PLANAR:
		xres = (var->xres + 0x3) & ~(0x3);
		break;
	case FORMAT_YUV422_PLANAR:
		xres = (var->xres + 0x7) & ~(0x7);
		break;
	case FORMAT_YUV420_PLANAR:
		xres = (var->xres + 0xf) & ~(0xf);
		break;
	case FORMAT_YUV422_PACKED:
		xres = var->xres;
		break;
	default:
		printk(KERN_ERR "No Valid color format chosen\n");
                return -EINVAL;
	}

	yres = var->yres;
	var->activate = FB_ACTIVATE_NOW;

	if (var->width == 0)
		var->width = var->xres;

	if (var->height == 0)
		var->height = var->yres;

	return 0;
}

int pxa910_fb_vid_set_par(struct fb_info *info)
{
	unsigned int xpos, ypos;
	int format, err;

	struct pxa910_fb_info *fbi = (struct pxa910_fb_info *)info;
	struct fb_var_screeninfo *var = &fbi->fb.var;

	pxa910_fb_validate_var(var);

	info->flags &= ~FBINFO_MISC_USEREVENT;

	xpos = var->nonstd & 0x3ff;
	ypos = (var->nonstd>>10) & 0x3ff;
	format = (var->nonstd>>20) & 0xf;

	fbi->format = format;

	if ( fbi->format == FORMAT_RGB565 ||
			fbi->format == FORMAT_RGB1555 ||
			fbi->format == FORMAT_RGB888_PACKED ||
			fbi->format == FORMAT_RGBA888_UNPACKED ||
			fbi->format == FORMAT_RGBA888)
		err = pxa910_fb_map_RGB_memory(info);
	else
		err = pxa910_fb_map_YUV_memory(info);

	if (err) return err;

	/* position */
	fbi->xpos = var->nonstd & 0x3ff;
	fbi->ypos = (var->nonstd>>10) & 0x3ff;

	pxa910_fb_vid_enable(info);

	return 0;
}

static struct fb_ops pxa910_fb_vid_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= pxa910_fb_vid_open,
	.fb_release	= pxa910_fb_vid_release,
	.fb_fillrect    = cfb_fillrect,
	.fb_copyarea    = cfb_copyarea,
	.fb_imageblit   = cfb_imageblit,
	.fb_mmap	= pxa910_fb_mmap,
	.fb_check_var	= pxa910_fb_vid_check_var,
	.fb_set_par	= pxa910_fb_vid_set_par,
	.fb_ioctl	= pxa910_vid_ioctl,
};

static struct pxa910_fb_info * __init pxa910_fb_vid_init_fbinfo(struct device *dev)
{
        struct pxa910_fb_info *fbi;
	struct pxa910_fb_mach_info *inf = dev->platform_data;
	struct pxa910_fb_mode_info *mode = inf->modes;
	void *addr;
        struct fb_videomode fb_mode;
        struct fb_var_screeninfo var;
	int i, smemlen;

	DPRINTK("Entering...\n");

	fbi = kmalloc(sizeof(struct pxa910_fb_info) + sizeof(u32) * 16, GFP_KERNEL);
	if (!fbi){
		printk(KERN_ERR "Unable to allocate an fb_info structure\n");
                return NULL;
        }

	memset(fbi, 0, sizeof(struct pxa910_fb_info) + sizeof(u32)*16);
        INIT_LIST_HEAD(&fbi->fb.modelist);
        for (i = 0; i < inf->num_modes; i++) {
                pxa910_fb_setmode(&var, &inf->modes[i]);
                fb_var_to_videomode(&fb_mode, &var);
                fb_add_videomode(&fb_mode, &fbi->fb.modelist);
        }

	fbi->dev = dev;
        fbi->fb.device = dev;

	strcpy(fbi->fb.fix.id, "pxa910-fb-vid");
        pxa910_fb_setmode(&fbi->fb.var, mode);

	if (fbi->fb.var.bits_per_pixel == 16)
		fbi->fb.fix.visual = FB_VISUAL_TRUECOLOR;
	else {
		/*
		 * Some people have weird ideas about wanting static
		 * pseudocolor maps.  I suspect their user space
		 * applications are broken.
		 */
		fbi->fb.fix.visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	}
	fbi->fb.fix.type = FB_TYPE_PACKED_PIXELS;
	fbi->fb.fix.type_aux = 0;
	fbi->fb.fix.xpanstep = 1;
	fbi->fb.fix.ypanstep = 1;
	fbi->fb.fix.ywrapstep = 0;
	fbi->fb.fix.accel = FB_ACCEL_NONE;

	fbi->fb.var.nonstd = 0;
	fbi->fb.var.activate = FB_ACTIVATE_NOW;
	fbi->fb.var.height      = 0;
	fbi->fb.var.width       = 0;
	fbi->fb.var.accel_flags = 0;
	fbi->fb.var.vmode       = FB_VMODE_NONINTERLACED;

	fbi->fb.fbops           = &pxa910_fb_vid_ops;
	fbi->fb.flags           = FBINFO_DEFAULT;
	fbi->fb.node            = -1;
	fbi->buffer_num		= (inf->num_buffers < 1)?1:inf->num_buffers;

        addr = fbi;
        addr = addr + sizeof(struct pxa910_fb_info);
        fbi->fb.pseudo_palette  = addr;

        for (i = 0; i < inf->num_modes; i++) {
          smemlen = mode[i].xres * mode[i].yres * mode[i].bpp / 8 ;
	  smemlen *= fbi->buffer_num;


          if (smemlen > fbi->fb.fix.smem_len)
            fbi->fb.fix.smem_len = smemlen;
        }

        fbi->clk = clk_get(dev, "LCDCLK");

	return fbi;
}


int __init pxa910_fb_probe(struct platform_device *dev)
{
        int ret = -ENOMEM;
	struct pxa910_fb_info *fbi = NULL, *vid_fbi = NULL;
	struct pxa910_fb_mach_info *inf = NULL;
	struct pxa910_fb_controller *controller;
	struct resource *res;
	int irq;
        struct pxa910_fb_mode_info *mode=NULL;
	static int passcount = 0;

	DPRINTK("Entering...\n");
	inf = dev->dev.platform_data;

	mode=inf->modes;
	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	lcd_regs_base = ioremap_nocache(res->start, res->end - res->start + 1);
	if (lcd_regs_base == NULL) {
		printk(KERN_WARNING "failed to map LCD registers\n");
		return -ENOMEM;
	}

	controller = kmalloc(sizeof(struct pxa910_fb_controller), GFP_KERNEL);
	if (!controller) {
		printk(KERN_ERR "unable to allocate the pxa910_fb_controller structure\n");
                ret = -ENOMEM;
		goto failed;
	}
	//TODO
	init_waitqueue_head(&controller->ctrlr_wait);
	INIT_WORK(&controller->task, NULL);
	init_MUTEX(&controller->ctrlr_sem);

	fbi = pxa910_fb_init_fbinfo(&dev->dev);
	if (!fbi) {
		ret = -ENOMEM;
		goto failed;
	}
	fbi->controller = controller;

	if (pxa910_fb_map_video_memory(fbi) < 0) {
		ret = -EINVAL;
		goto failed;
	}

	pxa910_fb_lcd_enable(fbi);

	platform_set_drvdata(dev, fbi);

	if (register_framebuffer(&fbi->fb) < 0) {
		ret = -EINVAL;
		goto failed;
	}
	printk(KERN_INFO "fb%d: %s frame buffer device\n",
			fbi->fb.node, fbi->fb.fix.id);

	vid_fbi = pxa910_fb_vid_init_fbinfo(&dev->dev);
	if (!vid_fbi) {
		printk(KERN_ERR "Failed to initialize framebuffer device\n");
		ret = -ENOMEM;
		goto failed;
	}
	vid_fbi->controller = controller;

	if (pxa910_fb_map_video_memory(vid_fbi) < 0) {
		ret = -EINVAL;
		goto failed;
	}
	if (register_framebuffer(&vid_fbi->fb) < 0) {
		printk(KERN_ERR "unable to register overlay frame buffer\n");
		ret = -EINVAL;
		goto failed;
	}
#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	if (passcount++ == 0)
	{
		int gpio84;
		printk( KERN_INFO "%s() fb%d: turning on LCD for Silvermoon\n", __FUNCTION__, vid_fbi->fb.node );
		// Initialize using gpio
		// Use API to set direction etc.
		gpio84 = 84;
		if (gpio_request( gpio84, "LCD_pullup" ))
		{
			printk( KERN_ERR "Failed gpio request handle %d\n", gpio84 );
		}
		else
		{
			gpio_direction_output(gpio84, 0);
			mdelay(500);
			gpio_direction_output(gpio84, 1);
			printk( KERN_INFO "Changed direction for gpio84 (%d)\n", gpio84 );
		}
		gpio_free(gpio84);
	} // fb0 init
	else
	{
		printk(KERN_INFO "%s() fb%d: skipping LCD pullup for non-zero fb index, passcount=%d\n", __FUNCTION__, vid_fbi->fb.node, passcount );
	}
#else
		printk(KERN_INFO "not silvermoon\n" );
#endif
        printk(KERN_INFO "fb%d: [%s] frame buffer device\n",
                        vid_fbi->fb.node, vid_fbi->fb.fix.id);
	fbi->next = vid_fbi;

	irq = platform_get_irq(dev, 0);
	if (request_irq(irq, pxa910_fb_handle_irq, IRQF_DISABLED,
			"pxa168-fb", controller)) {
			printk(KERN_ERR "LCD interrupt hookup failed.\n");
			ret = -ENODEV;
			goto failed;
		}

	return 0;

failed:
	DPRINTK("Entering failed routine in Probe\n");
        platform_set_drvdata(dev, NULL);
	kfree(vid_fbi);
	kfree(fbi); /* kfree(NULL) should also be safe */
	kfree(controller);
	iounmap(lcd_regs_base);

	return ret;
}

static int __devexit pxa910_fb_remove(struct platform_device *dev)
{
        struct platform_device *pdev = dev;
	struct pxa910_fb_info *info = dev_get_drvdata(&pdev->dev);
	int irq;
	DPRINTK("Entering...\n");

	if (info->map_cpu)
		dma_free_writecombine(info->dev,  info->map_size,
				(void*)info->map_cpu, info->map_dma);
	if (info->next->map_cpu)
		dma_free_writecombine(info->next->dev,  info->next->map_size,
				(void*)info->next->map_cpu, info->next->map_dma);
	unregister_framebuffer(&info->fb);
	unregister_framebuffer(&info->next->fb);

	irq = platform_get_irq(pdev, 0);
	disable_irq(irq);
	free_irq(irq, info->controller);
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(info->controller);
	kfree(info->next);
	kfree(info);
	iounmap(lcd_regs_base);

	return 0;
}

static struct platform_driver pxa910_fb_driver = {
	.probe          = pxa910_fb_probe,
	.remove		= pxa910_fb_remove,
	.driver		= {
		.name	= PXA910_FBNAME,
	},
};

int __devinit pxa910_fb_init(void)
{
  DPRINTK("Entering...\n");
  return platform_driver_register(&pxa910_fb_driver);
}

static void __exit pxa910_fb_cleanup(void)
{
  DPRINTK("Entering...\n");
  platform_driver_unregister(&pxa910_fb_driver);
}

module_init(pxa910_fb_init);
module_exit(pxa910_fb_cleanup);
MODULE_DESCRIPTION("Loadable framebuffer driver for Chumby Silvermoon LCD");
MODULE_LICENSE("GPL");

