/*
 * broadsheetfb.h - definitions for the broadsheet framebuffer driver
 *
 * Copyright (C) 2008 by Jaya Kumar
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 */

#ifndef _LINUX_BROADSHEETFB_H_
#define _LINUX_BROADSHEETFB_H_

#define FB_IOC_MAGIC                        'm'
#define FBIO_FILLRECT                       _IO(FB_IOC_MAGIC, 0)
#define FBIO_LDAREA                         _IO(FB_IOC_MAGIC, 1)
#define FBIO_UPDFULL                        _IO(FB_IOC_MAGIC, 2)
#define FBIO_UPDPART                        _IO(FB_IOC_MAGIC, 3)
#define FBIO_CLEAR_FRAMEBUFFER              _IO(FB_IOC_MAGIC, 4)
#define FBIO_ROTMODE                        _IO(FB_IOC_MAGIC, 5)
#define FBIO_READREG                        _IO(FB_IOC_MAGIC, 6)
#define FBIO_WRITEREG                       _IO(FB_IOC_MAGIC, 7)
#define FBIO_XCMD                           _IO(FB_IOC_MAGIC, 8)
#define FBIO_WRITEDATA                      _IO(FB_IOC_MAGIC, 9)
#define FBIO_READDATA                       _IO(FB_IOC_MAGIC, 10)
#define FBIO_READIRQ                        _IO(FB_IOC_MAGIC, 11)

struct  fb_xcmd {
  u16 cmd;
  u16 count;
  u16 args[10];
};

/* Broadsheet command defines */
#define BS_CMD_RUN_PLL		0x02
#define BS_CMD_STBY		0x04
#define BS_CMD_SLP		0x05
#define BS_CMD_INIT_SYS_RUN	0x06
#define	BS_CMD_INIT_DSPE_CFG	0x09
#define	BS_CMD_INIT_DSPE_TMG	0x0A
#define	BS_CMD_INIT_ROTMODE	0x0B
#define	BS_CMD_RD_REG		0x10
#define	BS_CMD_WR_REG		0x11
#define	BS_CMD_LD_IMG		0x20
#define	BS_CMD_LD_IMG_AREA	0x22
#define	BS_CMD_LD_IMG_END	0x23
#define	BS_CMD_WAIT_DSPE_TRG	0x28
#define	BS_CMD_WAIT_DSPE_FREND	0x29
#define	BS_CMD_RD_WFM_INFO	0x30
#define	BS_CMD_UPD_INIT		0x32
#define	BS_CMD_UPD_FULL		0x33
#define	BS_CMD_UPD_FULL_AREA	0x34
#define	BS_CMD_UPD_PART		0x35
#define	BS_CMD_UPD_PART_AREA	0x36
#define	BS_CMD_UPD_GDRV_CLR	0x37

/* struct used by broadsheet. board specific stuff comes from *board */
struct broadsheetfb_par {
	struct fb_info *info;
	struct broadsheet_board *board;
	void (*write_reg)(struct broadsheetfb_par *, u16 reg, u16 val);
	u16 (*read_reg)(struct broadsheetfb_par *, u16 reg);
	wait_queue_head_t waitq;
	int SMC_config;
};

/* board specific routines */
struct broadsheet_board {
	struct module *owner;
	int (*init)(struct broadsheetfb_par *);
	void (*writecmd)(struct broadsheetfb_par *, u16);
	void (*writedata)(struct broadsheetfb_par *, u16);
	u16 (*read)(struct broadsheetfb_par *);
	void (*cleanup)(struct broadsheetfb_par *);
	int (*get_panel_type)(void);
	int (*setup_irq)(struct fb_info *);
	int (*waitready)(void);
	void (*reset)(void);
	int (*resume)(struct broadsheetfb_par *);
};

#endif
