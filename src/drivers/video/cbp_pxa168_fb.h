/*
 * drivers/video/cbp_pxa168_fb.h
 * (was: linux-2.6.25/src/include/asm-arm/arch-pxa/pxa910_fb.h)
 *
 *
 * Copyright (C) 2008 Marvell International Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */


#ifndef __PXA910_FB_H__
#define __PXA910_FB_H__

#include <linux/fb.h>

#define PIX_CLK_SOURCE 312    /* value in MHz */

#define PXA910_LCD_CMD_COMMAND_WRITE           (0x1 << 8)
#define PXA910_LCD_CMD_DATA_WRITE              (0x1 << 8)
#define PXA910_LCD_CMD_A0_COMMAND              (0x0 << 15)
#define PXA910_LCD_CMD_A0_DATA                 (0x1 << 15)

#define PXA910_LCD_CMD_COMMAND_MASK            (0xff << 8)
#define PXA910_LCD_CMD_DATA_MASK               (0xff << 0)
#define PXA910_LCD_CMD_WAIT                    (0x05<<8)

#define PXA910_MAKEUP_CMD(x)                   (PXA910_LCD_CMD_COMMAND_WRITE | PXA910_LCD_CMD_A0_COMMAND | (x))
#define PXA910_MAKEUP_DATA(x)                  (PXA910_LCD_CMD_DATA_WRITE | PXA910_LCD_CMD_A0_DATA | (x))


struct pxa910_fb_mode_info {
	u_int    flags;
	u_char   bpp;
	u_short  xres;
	u_short  yres;
	u_char   upper_porch;        /* used for vertical back */
	u_char   lower_porch;        /* used for vertical front */
	u_char   left_porch;         /* used for horizontal back */
	u_char   right_porch;        /* used for horizontal front */
	u_short  h_sync;
	u_short  v_sync;
	u_short  sync;               /* sets h an v sync polarity */
	u_int    pixel_clk;          /* value is desired pixel clk/10000  */
	int      (*update_framedata)(struct fb_info *fbi);	
};

struct pxa910_fb_mach_info {
	struct pxa910_fb_mode_info *modes;
	unsigned int		num_modes;
	u_int	fixed_modes:1,
		cmap_inverse:1,
		cmap_static:1,
		unused:29;
	void (*pxa910_fb_backlight_power)(int);
	void (*pxa910_fb_lcd_power)(int, struct fb_var_screeninfo *);
	unsigned int		num_buffers;
};

struct pxa910_fb_controller {
	volatile u_char		state;
	volatile u_char		task_state;
	struct semaphore	ctrlr_sem;
	wait_queue_head_t	ctrlr_wait;
	struct work_struct	task;
};

struct pxa910_fb_dma_blankcolor {
        u_char     disabled;
        u_char     red;
        u_char     green;
        u_char     blue;
};

struct pxa910_fb_chroma {
        u_char     mode;
        u_char     y_alpha;
        u_char     y;
        u_char     y1;
        u_char     y2;
        u_char     u_alpha;
        u_char     u;
        u_char     u1;
        u_char     u2;
        u_char     v_alpha;
        u_char     v;
        u_char     v1;
        u_char     v2;
};


struct pxa910_fb_info {
	struct fb_info		fb;
	struct device		*dev;
	struct clk              *clk;

	/* frame buffer */
	u_char 		        *map_cpu;
	dma_addr_t		map_dma;
	u_char 		        *map_cpu2;
	dma_addr_t		map_dma2;
	u_int			map_size;

	unsigned long	xpos;		/* screen position (x, y)*/
	unsigned long	ypos;		
	unsigned long 	format;
	unsigned int	buffer_num;
	unsigned int	buffer_index;
	unsigned int	ylen;
	unsigned int	cblen;
	unsigned int	crlen;
	unsigned int	yoff;
	unsigned int	cboff; 
	unsigned int	croff;
	unsigned int	ypitch;
	unsigned int	uvpitch;

	struct pxa910_fb_info *next;
	struct pxa910_fb_controller *controller;
        struct pxa910_fb_chroma chroma;
        struct pxa910_fb_dma_blankcolor dma_bc;
};

/* non-stardard format which can not be indicated by bits_per_pixel */
#define FORMAT_RGB565               (0)
#define FORMAT_YUV444_PACKED        (1)
#define FORMAT_YUV444_PLANAR        (2)
#define FORMAT_YUV422_PLANAR        (3)
#define FORMAT_YUV420_PLANAR        (4)
/* 5 formats above is compatible with pxa2xx overlay format */
#define FORMAT_RGB1555        		(5)
#define FORMAT_RGB888_PACKED       	(6)
#define FORMAT_RGBA888_UNPACKED		(7)
#define FORMAT_RGBA888			(8)
#define FORMAT_YUV422_PACKED		(9)


/* ==================LCD Control Register ========================= */

#define LCD_SPU_DMA_START_ADDR_Y0        (0x00C0)
#define LCD_SPU_DMA_START_ADDR_U0        (0x00C4)
#define LCD_SPU_DMA_START_ADDR_V0        (0x00C8)
#define LCD_CFG_DMA_START_ADDR_C0        (0x00CC)
#define LCD_SPU_DMA_START_ADDR_Y1        (0x00D0)
#define LCD_SPU_DMA_START_ADDR_U1        (0x00D4)
#define LCD_SPU_DMA_START_ADDR_V1        (0x00D8)
#define LCD_CFG_DMA_START_ADDR_C1        (0x00DC)
#define LCD_SPU_DMA_PITCH_YC	         (0x00E0)
#define LCD_SPU_DMA_PITCH_UV	         (0x00E4)
#define LCD_SPUT_DMA_OVSA_HPXL_VLN       (0x00E8)
#define LCD_SPU_DMA_HPXL_VLN             (0x00EC)
#define LCD_SPU_DZM_HPXL_VLN             (0x00F0)
#define LCD_CFG_GRA_START_ADDR0	         (0x00F4)
#define LCD_CFG_GRA_START_ADDR1	         (0x00F8)
#define LCD_CFG_GRA_PITCH                (0x00FC)
#define LCD_SPU_GRA_OVSA_HPXL_VLN        (0x0100)
#define LCD_SPU_GRA_HPXL_VLN             (0x0104)
#define LCD_SPU_GZM_HPXL_VLN	         (0x0108)
#define LCD_SPU_HWC_OVSA_HPXL_VLN        (0x010C)
#define LCD_SPU_HWC_HPXL_VLN             (0x0110)
#define LCD_SPUT_V_H_TOTAL               (0x0114)
#define LCD_SPU_V_H_ACTIVE               (0x0118)
#define LCD_SPU_H_PORCH                  (0x011C)
#define LCD_SPU_V_PORCH                  (0x0120)
#define LCD_SPU_BLANKCOLOR               (0x0124)
#define LCD_SPU_ALPHA_COLOR1             (0x0128)
#define LCD_SPU_ALPHA_COLOR2             (0x012C)
#define LCD_SPU_COLORKEY_Y               (0x0130)
#define LCD_SPU_COLORKEY_U               (0x0134)
#define LCD_SPU_COLORKEY_V               (0x0138)
#define LCD_CFG_RDREG4F                  (0x013C)
#define LCD_SPU_SPI_RXDATA               (0x0140)
#define LCD_SPU_ISA_RXDATA               (0x0144)
#define LCD_SPU_DBG_ISA                  (0x0148)
#define LCD_SPU_DMAVLD_YC                (0x014C)
#define LCD_SPU_DMAVLD_UV                (0x0150)
#define LCD_SPU_DMAVLD_UVSPU_GRAVLD      (0x0154)
#define LCD_SPU_HWC_RDDAT                (0x0158)
#define LCD_SPU_GAMMA_RDDAT              (0x015C)
#define LCD_SPU_PALETTE_RDDAT            (0x0160)
#define LCD_SPU_DBG_DMATOP               (0x0164)
#define LCD_SPU_DBG_GRATOP               (0x0168)
#define LCD_SPU_DBG_TXCTRL               (0x016C)
#define LCD_SPU_DBG_SLVTOP               (0x0170)
#define LCD_SPU_DBG_MUXTOP               (0x0174)
#define LCD_SPU_IOPAD_IN                 (0x0178)
#define LCD_CFG_RDREG5F                  (0x017C)
#define LCD_SPU_SPI_CTRL                 (0x0180)
#define LCD_SPU_SPI_TXDATA               (0x0184)
#define LCD_SPU_SMPN_CTRL                (0x0188)
#define LCD_SPU_SLV_PORT                 (0x018C)
#define LCD_SPU_DMA_CTRL0                (0x0190)
#define LCD_SPU_DMA_CTRL1                (0x0194)
#define LCD_SPU_SRAM_CTRL	         (0x0198)
#define LCD_SPU_SRAM_WRDAT	         (0x019C)
#define LCD_SPU_SRAM_PARA0               (0x01A0)
#define LCD_SPU_SRAM_PARA1	         (0x01A4)
#define LCD_CFG_SCLK_DIV	         (0x01A8)
#define LCD_SPU_CONTRAST                 (0x01AC)
#define LCD_SPU_SATURATION               (0x01b0)
#define LCD_SPU_CBSH_HUE                 (0x01B4)
#define LCD_SPU_DUMB_CTRL                (0x01B8)
#define LCD_SPU_IOPAD_CONTROL            (0x01BC)
#define LCD_SPU_IRQ_ENA                  (0x01C0)
#define LCD_SPU_IRQ_ISR                  (0x01C4)
#define LCD_MISC_CNTL                    (0x01C8)


/************************************************************
 *  LCD Interrupt Control register area.
 ************************************************************/
/*
 * defined for Configure Color Space Conversion
 * LCD LCD I/O Pads control register bit[9:8]
 */
#define IRQ_DMA_FRAME_IRQ0_ENA      0x80000000
#define IRQ_DMA_FRAME_IRQ1_ENA      0x40000000
#define IRQ_DMA_FF_UNDERFLOW_ENA    0x20000000
#define IRQ_GRA_FRAME_IRQ0_ENA      0x8000000
#define IRQ_GRA_FRAME_IRQ1_ENA      0x4000000
#define IRQ_GRA_FF_UNDERFLOW_ENA    0x2000000
#define IRQ_VSYNC_IRQ_ENA           0x800000
#define IRQ_DUMB_FRAMEDONE_ENA      0x400000
#define IRQ_TXC_FRAMEDONE_ENA       0x200000
#define IRQ_HWC_FRAMEDONE_ENA       0x100000
#define IRQ_SLV_IRQ_ENA             0x80000
#define IRQ_SPI_IRQ_ENA             0x40000
#define IRQ_PWRDN_IRQ_ENA           0x20000
#define IRQ_ERR_IRQ_ENA             0x10000

#define ISR_DMA_FRAME_IRQ0          0x80000000
#define ISR_DMA_FRAME_IRQ1          0x40000000
#define ISR_DMA_FRAME_CNT           0x30000000  /* 2 bits */
#define ISR_GRA_FRAME_IRQ0          0x08000000
#define ISR_GRA_FRAME_IRQ1          0x04000000
#define ISR_GRA_FRAME_CNT           0x03000000      /* 2 bits */
#define ISR_VSYNC_IRQ               0x00800000
#define ISR_DUMB_FRAMEDONE          0x00400000
#define ISR_TWC_FRAMEDONE           0x00200000
#define ISR_HWC_FRAMEDONE           0x00100000
#define ISR_SLV_FF_EMPTY            0x00080000
#define ISR_DMA_FF_ALLEMPTY         0x00040000
#define ISR_GRA_FF_ALLEMPTY         0x00020000
#define ISR_PWRDN_IRQ               0x00010000

/*
 * some defines for the LCD_SPU_DUMB_CTRL register
 */

#define LCD_SPU_DUMB_CTRL_DUMB_ENA          0x00000001
#define LCD_SPU_DUMB_CTRL_INV_PCLK          0x00000002
#define LCD_SPU_DUMB_CTRL_INV_HSYNC         0x00000004
#define LCD_SPU_DUMB_CTRL_INV_VSYNC         0x00000008
#define LCD_SPU_DUMB_CTRL_INV_HENA          0x00000010
#define LCD_SPU_DUMB_CTRL_INV_COMPSYNC      0x00000020
#define LCD_SPU_DUMB_CTRL_INV_COMPBLANK     0x00000040
#define LCD_SPU_DUMB_CTRL_REVERSE_RGB       0x00000080
#define LCD_SPU_DUMB_CTRL_DUMB_MODE_MASK    0xf0000000

/*
 * define for LCD Interrupt Control register
 */
#define ICR_DMA_FRAME_IRQ0_ENA      31
#define ICR_DMA_FRAME_IRQ1_ENA      30
#define ICR_DMA_FF_UNDERFLOW_ENA    29
#define ICR_GRA_FRAME_IRQ0_ENA      27
#define ICR_GRA_FRAME_IRQ1_ENA      26
#define ICR_GRA_FF_UNDERFLOW_ENA    25
#define ICR_VSYNC_IRQ_ENA           23
#define ICR_DUMB_FRAMEDONE_ENA      22
#define ICR_TXC_FRAMEDONE_ENA       21
#define ICR_HWC_FRAMEDONE_ENA       20
#define ICR_SLV_IRQ_ENA             19
#define ICR_SPI_IRQ_ENA             18
#define ICR_PWRDN_IRQ_ENA           17
#define ICR_ERR_IRQ_ENA             16
#define ICR_RSR                     0    /* Read spu_irq_ISR Clean ISR[31:16]
					  *  Enable
					  */
/*
 * define for LCD DMA control 0 register
 */
#define LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPRB_SHIFT          4
#define LCD_SPU_DMA_CTRL0_CFG_GRA_SWAPRB_SHIFT          12
#define LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPUV_SHIFT          3 
#define LCD_SPU_DMA_CTRL0_CFG_GRA_SWAPUV_SHIFT          11
#define LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPYU_SHIFT          2 
#define LCD_SPU_DMA_CTRL0_CFG_GRA_SWAPYU_SHIFT          10
#define LCD_SPU_DMA_CTRL0_CFG_DMAFORMAT_SHIFT	20
#define LCD_SPU_DMA_CTRL0_CFG_GRAFORMAT_SHIFT	16

#define LCD_SPU_DMA_CTRL0_CFG_DMA_ENA		(1 << 0 )
#define LCD_SPU_DMA_CTRL0_CFG_YUV2RGB_DMA	(1 << 1)
#define LCD_SPU_DMA_CTRL0_CFG_YUV2RGB_GRA	(1 << 9)


#define LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPRB  (1 << LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPRB_SHIFT)
#define LCD_SPU_DMA_CTRL0_CFG_GRA_SWAPRB  (1 << LCD_SPU_DMA_CTRL0_CFG_GRA_SWAPRB_SHIFT)
#define LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPUV  (1 << LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPUV_SHIFT)
#define LCD_SPU_DMA_CTRL0_CFG_GRA_SWAPUV  (1 << LCD_SPU_DMA_CTRL0_CFG_GRA_SWAPUV_SHIFT)
#define LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPYU  (1 << LCD_SPU_DMA_CTRL0_CFG_DMA_SWAPYU_SHIFT)
#define LCD_SPU_DMA_CTRL0_CFG_GRA_SWAPYU  (1 << LCD_SPU_DMA_CTRL0_CFG_GRA_SWAPYU_SHIFT)

/*
 * define for LCD DMA control 1 register
 */
//#define LCD_SPU_DMA_CTRL1_CFG_ALPHA_MODE_MSK	(0xff << 16)
#define LCD_SPU_DMA_CTRL1_CFG_ALPHA_MODE_MSK	(0x3 << 16)
#define LCD_SPU_DMA_CTRL1_CFG_ALPHA_MODE_SHIFT 	16
#define LCD_SPU_DMA_CTRL1_CFG_ALPHA_MSK		(0xff << 8)
#define LCD_SPU_DMA_CTRL1_CFG_ALPHA_SHIFT	8

/*
 * IOCTL(s) 
 */

#define FBIOPUT_GLOBAL_ALPHABLEND         0xE1    /* Set mode to global blend and get a blend value */
#define FBIOPUT_GRAPHIC_ALPHABLEND        0xE2    /* Set mode to default Graphic blend and assume pixel blend */
#define FBIOPUT_SWAP_GRAPHIC_RED_BLUE     0xE3    /* Swap the Red and Blue color code for RGB frame */
#define FBIOPUT_SWAP_GRAPHIC_U_V          0xE4    /* Swap the U and the V of a YUV frame */
#define FBIOPUT_SWAP_GRAPHIC_Y_UV         0xE5    /* Swap the Y with the UV */
#define FBIOPUT_SWAP_VIDEO_RED_BLUE       0xE6    /* Swap the Red and Blue color code for RGB frame */
#define FBIOPUT_SWAP_VIDEO_U_V            0xE7    /* Swap the U and the V of a YUV frame */
#define FBIOPUT_SWAP_VIDEO_Y_UV           0xE8    /* Swap the Y with the UV */
#define FBIOGET_CHROMAKEYS                0xE9    /* Return the Chroma key struct to the user */
#define FBIOPUT_CHROMAKEYS                0xEA    /* Fill in the Chroma registers */
#define FBIOPUT_VIDEO_ALPHABLEND          0xEB    /* Set mode to Video alpha blend */
#define FBIOGET_FB0_DMA_BLANK_COLOR       0xEC    /* Return the FB0 DMA Enable status and current blank color to user */
#define FBIOPUT_FB0_DMA_BLANK_COLOR       0xED    /* Set the FB0 DMA ENABLE and set the blank color */

extern void __init set_pxa910_fb_info(struct pxa910_fb_mach_info *info);
int pxa910_fb_send_cmd( unsigned short *cmd, unsigned int num);
#endif

