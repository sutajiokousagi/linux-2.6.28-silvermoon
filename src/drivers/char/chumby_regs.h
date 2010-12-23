/*
	$Id: s3c_regs.h 5097 2008-05-15 20:23:52Z henry $
	chumby_regs.h
	henry -- May 2008 -- cloned from chumby_sense1

	Copyright (c) Chumby Industries, 2008-9

	The chumby register interface driver is free software; you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation; either version 2 of the
	License, or (at your option) any later version.

	The chumby register interface driver is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with the Chumby; if not, write to the Free Software Foundation, Inc.,
	51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <linux/ioctl.h>



/* device numbers are allocated dynamically */
/* see /proc/devices */

#define SENSE1_IOCTL_MAGIC    'C'

/*
 * The following are for entries in /proc/sys/sense1
 */
#define CTL_CHUMSENSE1	0x4348554F	/* 'CHUP' in hex form */

enum {
	CTL_CHUMSENSE1_DEBUG_TRACE	= 101,
	CTL_CHUMSENSE1_DEBUG_IOCTL	= 102,
	CTL_CHUMSENSE1_DEBUG_ERROR	= 103,
	CTL_CHUMSENSE1_DIMLEVEL		= 105,
	CTL_CHUMSENSE1_HPIN		= 106,
	CTL_CHUMSENSE1_DEBOUNCE		= 107,
	CTL_CHUMSENSE1_SPKMUTE		= 111,
	CTL_CHUMSENSE1_DCPOWER		= 113,
	CTL_CHUMSENSE1_BTPOWER		= 114,
	CTL_CHUMSENSE1_DEBUG_CLEAR	= 116,
	CTL_CHUMSENSE1_RESETSERIAL	= 117,
    CTL_CHUMSENSE1_BRIGHTNESS   = 118,
};

// Base address for LCD controller
#define S3C_LCD_BASE		0

// Physical address range to be mapped for LCD controller
#define S3C_LCD_PHYS_BASE	0x77100000
#define S3C_LCD_PHYS_RANGE	   0x10000

// Physical address range to be mapped for clock registers (pp. 3-20 through 3-21)
#define	S3C_CLOCK_PHYS_BASE		0x7e000000
#define S3C_CLOCK_PHYS_RANGE	   0x10000

// Physical address range for GPIO registers (pp. 10-3 through 10-6) and IIC registers (pp. 36-9 through 36-10)
#define S3C_GPIO_PHYS_BASE		0x7f000000
#define S3C_GPIO_PHYS_RANGE		   0x10000

#define S3C_VIDCON0		(S3C_LCD_BASE+0x00)
#define S3C_VIDCON1		(S3C_LCD_BASE+0x04)
#define S3C_VIDCON2		(S3C_LCD_BASE+0x08)

#define S3C_VIDTCON0	(S3C_LCD_BASE+0x10)
#define S3C_VIDTCON1	(S3C_LCD_BASE+0x14)
#define S3C_VIDTCON2	(S3C_LCD_BASE+0x18)

#define S3C_WINCON0		(S3C_LCD_BASE+0x20)
#define S3C_WINCON1		(S3C_LCD_BASE+0x24)
// Bit values
#define S3C_WINCON_BLD_PIX		(0x0001<<6)	// 0x40
#define S3C_WINCON_ALPHA_SEL	(0x0001<<1)	// 0x02
#define S3C_WINCON2		(S3C_LCD_BASE+0x28)

#define S3C_VIDOSD0A	(S3C_LCD_BASE+0x40)	// Position
#define S3C_VIDOSD0B	(S3C_LCD_BASE+0x44)	// Position
#define S3C_VIDOSD0C	(S3C_LCD_BASE+0x48)	// Size

#define S3C_VIDOSD1A	(S3C_LCD_BASE+0x50) // Position
 #define S3C_VIDOSD_XYMask		(0x07ff)
 #define S3C_VIDOSD_LeftTopX_F	(S3C_VIDOSD_XYMask<<11)
 #define S3C_VIDOSD_LeftTopY_F	S3C_VIDOSD_XYMask
#define S3C_VIDOSD1B	(S3C_LCD_BASE+0x54) // Position
 #define S3C_VIDOSD_RightBotX_F	(S3C_VIDOSD_XYMask<<11)
 #define S3C_VIDOSD_RightBotY_F	S3C_VIDOSD_XYMask
#define S3C_VIDOSD1C	(S3C_LCD_BASE+0x58)	// Alpha
 #define S3C_VIDOSD_ALPHA0_R	(0x0f<<(12+8))
 #define S3C_VIDOSD_ALPHA0_G	(0x0f<<(12+4))
 #define S3C_VIDOSD_ALPHA0_B	(0x0f<<(12+0))
 #define S3C_VIDOSD_ALPHA1_R	(0x0f<<8)
 #define S3C_VIDOSD_ALPHA1_G	(0x0f<<4)
 #define S3C_VIDOSD_ALPHA1_B	(0x0f)
#define S3C_VIDOSD1D	(S3C_LCD_BASE+0x5C)	// Size
 #define S3C_VIDOSD_SIZE		0x00ffffff

#define S3C_VIDOSD2A	(S3C_LCD_BASE+0x60)
#define S3C_VIDOSD2B	(S3C_LCD_BASE+0x64) // Position
#define S3C_VIDOSD2C	(S3C_LCD_BASE+0x68)	// Alpha
#define S3C_VIDOSD2D	(S3C_LCD_BASE+0x6C)	// Size

#define S3C_VIDW00ADD0B0	(S3C_LCD_BASE+0xa0)	// Window 0 address 0 buffer 0 (start)
#define S3C_VIDW00ADD0B1	(S3C_LCD_BASE+0xa4)	// Window 0 address 0 buffer 1 (start)
#define S3C_VIDW01ADD0B0	(S3C_LCD_BASE+0xa8)	// Window 1 address 0 buffer 0 (start)
#define S3C_VIDW01ADD0B1	(S3C_LCD_BASE+0xac)	// Window 1 address 0 buffer 1 (start)
#define S3C_VIDW02ADD0		(S3C_LCD_BASE+0xb0)	// Window 2 address 0 (start)
#define S3C_VIDW03ADD0		(S3C_LCD_BASE+0xb4)	// Window 3 address 0 (start)
#define S3C_VIDW04ADD0		(S3C_LCD_BASE+0xb8)	// Window 4 address 0 (start)

#define S3C_VIDW00ADD1B0	(S3C_LCD_BASE+0xd0)	// Window 0 address 1 buffer 0 (end)
#define S3C_VIDW00ADD1B1	(S3C_LCD_BASE+0xd4)	// Window 0 address 1 buffer 1 (end)
#define S3C_VIDW01ADD1B0	(S3C_LCD_BASE+0xd8)	// Window 1 address 1 buffer 0 (end)
#define S3C_VIDW01ADD1B1	(S3C_LCD_BASE+0xdc)	// Window 1 address 1 buffer 1 (end)
#define S3C_VIDW02ADD1		(S3C_LCD_BASE+0xe0)	// Window 2 address 1 (end)
#define S3C_VIDW03ADD1		(S3C_LCD_BASE+0xe4)	// Window 3 address 1 (end)
#define S3C_VIDW04ADD1		(S3C_LCD_BASE+0xe8)	// Window 4 address 1 (end)

#define S3C_VIDW00ADD2		(S3C_LCD_BASE+0x100)	// Window 0 address 2 (buffer size)
#define S3C_VIDW01ADD2		(S3C_LCD_BASE+0x104)	// Window 1 address 2 (buffer size)
#define S3C_VIDW02ADD2		(S3C_LCD_BASE+0x108)	// Window 2 address 2 (buffer size)
#define S3C_VIDW03ADD2		(S3C_LCD_BASE+0x10c)	// Window 3 address 2 (buffer size)
#define S3C_VIDW04ADD2		(S3C_LCD_BASE+0x110)	// Window 4 address 2 (buffer size)

#define S3C_W1KEYCON0	(S3C_LCD_BASE+0x140)	// Color key control
#define	S3C_W1KEYCON1	(S3C_LCD_BASE+0x144)	// Color key value

#define S3C_W2KEYCON0	(S3C_LCD_BASE+0x148)	// Color key control
#define	S3C_W2KEYCON1	(S3C_LCD_BASE+0x14C)	// Color key value

#define S3C_W3KEYCON0	(S3C_LCD_BASE+0x150)	// Color key control
#define	S3C_W3KEYCON1	(S3C_LCD_BASE+0x154)	// Color key value

#define S3C_W4KEYCON0	(S3C_LCD_BASE+0x158)	// Color key control
#define	S3C_W4KEYCON1	(S3C_LCD_BASE+0x15C)	// Color key value

#define S3C_DITHMODE		(S3C_LCD_BASE+0x170)	// Dither mode

#define S3C_WIN0MAP			(S3C_LCD_BASE+0x180)	// Window 0 palette mapping
#define S3C_WIN1MAP			(S3C_LCD_BASE+0x184)	// Window 1 palette mapping
#define S3C_WIN2MAP			(S3C_LCD_BASE+0x188)	// Window 2 palette mapping
#define S3C_WIN3MAP			(S3C_LCD_BASE+0x18C)	// Window 3 palette mapping
#define S3C_WIN4MAP			(S3C_LCD_BASE+0x190)	// Window 4 palette mapping

#define S3C_WPALCON			(S3C_LCD_BASE+0x1A0)	// Window palette control

#define S3C_TRIGCON			(S3C_LCD_BASE+0x1A4)	// I80/RGB trigger control

#define S3C_ADDR(off)	(LCD_BASE+off)
