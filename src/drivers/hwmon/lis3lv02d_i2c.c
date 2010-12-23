/*
 *  lis3lv02d.c - ST LIS3LV02DL accelerometer driver
 *
 *  Copyright (C) 2007-2008 Yan Burman
 *  Copyright (C) 2008 Eric Piel
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/dmi.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/freezer.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/i2c.h>
#include <asm/atomic.h>
#include "lis3lv02d.h"

static struct i2c_client *g_client;

static int lis3lv02d_read(u8 reg, u8 *pval)
{
	int ret;
	int status;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;
	ret = i2c_smbus_read_byte_data(g_client, reg);
	if (ret >= 0) {
		*pval = ret;
		status = 0;
	} else {
		status = -EIO;
	}

	return status;
}

static int lis3lv02d_write(u8 reg, u8 val)
{
	int ret;
	int status;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;
	ret = i2c_smbus_write_byte_data(g_client, reg, val);
	if (ret == 0)
		status = 0;
	else
		status = -EIO;

	return status;
}

static void lis3lv02d_raw_xyz(u8 *x, u8 *y, u8 *z)
{
	lis3lv02d_read(0x29, x);
	lis3lv02d_read(0x2b, y);
	lis3lv02d_read(0x2d, z);
}


/*
 * The sensor can also generate interrupts (DRDY) but it's pretty pointless
 * because their are generated even if the data do not change. So it's better
 * to keep the interrupt for the free-fall event. The values are updated at
 * 40Hz (at the lowest frequency), but as it can be pretty time consuming on
 * some low processor, we poll the sensor only at 20Hz... enough for the
 * joystick.
 */

/* Maximum value our axis may get for the input device (signed 12 bits) */
#define MDPS_MAX_VAL 2048

struct axis_conversion {
	s8	x;
	s8	y;
	s8	z;
};

struct i2c_lis3lv02d {
	struct device	*device;   /* The device */
	struct input_dev	*idev;     /* input device */
	struct task_struct	*kthread;  /* kthread for input */
	struct completion	thread_exit_complete;
	int 			thread_exit;
	struct mutex            lock;
	atomic_t		count;     /* interrupt count after last read */
	int			xcalib;    /* calibrated null value for x */
	int			ycalib;    /* calibrated null value for y */
	int			zcalib;    /* calibrated null value for z */
	unsigned char		is_on;     /* whether the device is on or off */
	unsigned char		usage;     /* usage counter */
	struct axis_conversion	ac;        /* hw -> logical axis */
	unsigned int			delay;
	wait_queue_head_t	wait;
};

static struct i2c_lis3lv02d adev;

static int lis3lv02d_remove_fs(void);
static int lis3lv02d_add_fs(struct device *device);

static s16 lis3lv02d_read_16(int reg)
{
	u8 lo, hi;

	lis3lv02d_read(reg, &lo);
	lis3lv02d_read(reg + 1, &hi);
	/* In "12 bit right justified" mode, bit 6, bit 7, bit 8 = bit 5 */
	return (s16)((hi << 8) | lo);
}

/**
 * lis3lv02d_get_axis - For the given axis, give the value converted
 * @axis:      1,2,3 - can also be negative
 * @hw_values: raw values returned by the hardware
 *
 * Returns the converted value.
 */
static inline int lis3lv02d_get_axis(s8 axis, int hw_values[3])
{
	if (axis > 0)
		return hw_values[axis - 1];
	else
		return -hw_values[-axis - 1];
}

/**
 * lis3lv02d_get_xyz - Get X, Y and Z axis values from the accelerometer
 * @handle: the handle to the device
 * @x:      where to store the X axis value
 * @y:      where to store the Y axis value
 * @z:      where to store the Z axis value
 *
 * Note that 40Hz input device can eat up about 10% CPU at 800MHZ
 */
static void lis3lv02d_get_xyz(int *x, int *y, int *z)
{
	int position[3];

	position[0] = lis3lv02d_read_16(OUTX_L);
	position[1] = lis3lv02d_read_16(OUTY_L);
	position[2] = lis3lv02d_read_16(OUTZ_L);

	*x = lis3lv02d_get_axis(adev.ac.x, position);
	*y = lis3lv02d_get_axis(adev.ac.y, position);
	*z = lis3lv02d_get_axis(adev.ac.z, position);
}

static inline void lis3lv02d_poweroff(void)
{
	adev.is_on = 0;
	/* disable X,Y,Z axis and power down */
	lis3lv02d_write(CTRL_REG1, 0x00);
}

static void lis3lv02d_poweron(void)
{
	u8 val;

	adev.is_on = 1;

	lis3lv02d_write(CTRL_REG1, 
		CTRL1_PD1|CTRL1_PD0|CTRL1_Xen|CTRL1_Yen|CTRL1_Zen);

	lis3lv02d_write(FF_WU_CFG, 0);
	/*
	 * BDU: LSB and MSB values are not updated until both have been read.
	 *      So the value read will always be correct.
	 * IEN: Interrupt for free-fall and DD, not for data-ready.
	 */
	lis3lv02d_read(CTRL_REG2, &val);
	val |= CTRL2_BDU | CTRL2_IEN;
	lis3lv02d_write(CTRL_REG2, val);
}

/*
 * To be called before starting to use the device. It makes sure that the
 * device will always be on until a call to lis3lv02d_decrease_use(). Not to be
 * used from interrupt context.
 */
static void lis3lv02d_increase_use(struct i2c_lis3lv02d *dev)
{
	mutex_lock(&dev->lock);
	dev->usage++;
	if (dev->usage == 1) {
		if (!dev->is_on)
			lis3lv02d_poweron();
	}
	mutex_unlock(&dev->lock);
}

/*
 * To be called whenever a usage of the device is stopped.
 * It will make sure to turn off the device when there is not usage.
 */
static void lis3lv02d_decrease_use(struct i2c_lis3lv02d *dev)
{
	mutex_lock(&dev->lock);
	dev->usage--;
	if (dev->usage == 0)
		lis3lv02d_poweroff();
	mutex_unlock(&dev->lock);
}

/**
 * lis3lv02d_joystick_kthread - Kthread polling function
 * @data: unused - here to conform to threadfn prototype
 */
static int lis3lv02d_joystick_kthread(void *data)
{
	int x, y, z;
	int status;
	unsigned int delay = adev.delay;

	daemonize("lis3lv02");
	while(!adev.thread_exit) {
		wait_event_timeout(adev.wait, delay!=adev.delay, msecs_to_jiffies(delay));		
		delay = adev.delay;
		lis3lv02d_read(STATUS_REG, &status);
		if(status & 0x7 == 0x7) {
#if 0
			printk("raw: %d, %d, %d\n", 
					lis3lv02d_read_16(OUTX_L), 
					lis3lv02d_read_16(OUTY_L), 
					lis3lv02d_read_16(OUTZ_L));
			lis3lv02d_get_xyz(&x, &y, &z);
			input_report_abs(adev.idev, ABS_X, x - adev.xcalib);
			input_report_abs(adev.idev, ABS_Y, y - adev.ycalib);
			input_report_abs(adev.idev, ABS_Z, z - adev.zcalib);
			input_sync(adev.idev);
#else
			x = lis3lv02d_read_16(OUTX_L)/8;
			y = lis3lv02d_read_16(OUTY_L)/8;
			z = lis3lv02d_read_16(OUTZ_L)/8;
			input_report_abs(adev.idev, ABS_X, y);
			input_report_abs(adev.idev, ABS_Y, 0 - x);
			input_report_abs(adev.idev, ABS_Z, 0 - z);
			input_sync(adev.idev);
#endif
		}
	}

	complete_and_exit(&adev.thread_exit_complete, 0);
	return 0;
}

static int lis3lv02d_joystick_open(struct input_dev *input)
{
	adev.delay = 0xffffffff;
	lis3lv02d_increase_use(&adev);
	adev.thread_exit = 0;
	kernel_thread(lis3lv02d_joystick_kthread,
			NULL, 0);
	return 0;
}

static void lis3lv02d_joystick_close(struct input_dev *input)
{
	init_completion(&adev.thread_exit_complete);
	adev.thread_exit = 1;
	wait_for_completion(&adev.thread_exit_complete);
	lis3lv02d_decrease_use(&adev);
}


static inline void lis3lv02d_calibrate_joystick(void)
{
	lis3lv02d_get_xyz(&adev.xcalib, &adev.ycalib, &adev.zcalib);
}

static int lis3lv02d_joystick_enable(void)
{
	int err;

	if (adev.idev)
		return -EINVAL;

	adev.idev = input_allocate_device();
	if (!adev.idev)
		return -ENOMEM;

	lis3lv02d_calibrate_joystick();

	adev.idev->name       = "ST LIS3LV02DL Accelerometer";
	adev.idev->phys       =  "lis3lv02d/input0";
	adev.idev->id.bustype = BUS_I2C;
	adev.idev->id.vendor  = 0;
	adev.idev->dev.parent = adev.device;
	adev.idev->open       = lis3lv02d_joystick_open;
	adev.idev->close      = lis3lv02d_joystick_close;

	__set_bit(EV_ABS, adev.idev->evbit);
	__set_bit(ABS_X, adev.idev->absbit);
	__set_bit(ABS_Y, adev.idev->absbit);
	__set_bit(ABS_Z, adev.idev->absbit);

	__set_bit(EV_SYN, adev.idev->evbit);
	__set_bit(EV_KEY, adev.idev->evbit);

	input_set_abs_params(adev.idev, ABS_X, 
			-MDPS_MAX_VAL, MDPS_MAX_VAL, 3, 3);
	input_set_abs_params(adev.idev, ABS_Y, 
			-MDPS_MAX_VAL, MDPS_MAX_VAL, 3, 3);
	input_set_abs_params(adev.idev, ABS_Z, 
			-MDPS_MAX_VAL, MDPS_MAX_VAL, 3, 3);

	err = input_register_device(adev.idev);
	if (err) {
		printk(KERN_ERR "regist input driver error\n");
		input_free_device(adev.idev);
		adev.idev = NULL;
	}

	return err;
}

static void lis3lv02d_joystick_disable(void)
{
	if (!adev.idev)
		return;

	input_unregister_device(adev.idev);
	adev.idev = NULL;
}


/*
 * Initialise the accelerometer and the various subsystems.
 * Should be rather independant of the bus system.
 */
static int lis3lv02d_init_device(struct i2c_lis3lv02d *dev)
{
	mutex_init(&dev->lock);
	init_waitqueue_head(&adev.wait);
	lis3lv02d_add_fs(dev->device);
	lis3lv02d_increase_use(dev);

	if (lis3lv02d_joystick_enable())
		printk(KERN_ERR "lis3lv02d: joystick initialization failed\n");

	lis3lv02d_decrease_use(dev);
	return 0;
}

static int lis3lv02d_dmi_matched(const struct dmi_system_id *dmi)
{
	adev.ac = *((struct axis_conversion *)dmi->driver_data);
	printk(KERN_INFO "lis3lv02d: hardware type %s found.\n", dmi->ident);

	return 1;
}

/* Represents, for each axis seen by userspace, the corresponding hw axis (+1).
 * If the value is negative, the opposite of the hw value is used. */
static struct axis_conversion lis3lv02d_axis_normal = {1, 2, 3};
static struct axis_conversion lis3lv02d_axis_y_inverted = {1, -2, 3};
static struct axis_conversion lis3lv02d_axis_x_inverted = {-1, 2, 3};
static struct axis_conversion lis3lv02d_axis_z_inverted = {1, 2, -3};
static struct axis_conversion lis3lv02d_axis_xy_rotated_left = {-2, 1, 3};
static struct axis_conversion lis3lv02d_axis_xy_swap_inverted = {-2, -1, 3};

#define AXIS_DMI_MATCH(_ident, _name, _axis) {		\
	.ident = _ident,				\
	.callback = lis3lv02d_dmi_matched,		\
	.matches = {					\
		DMI_MATCH(DMI_PRODUCT_NAME, _name)	\
	},						\
	.driver_data = &lis3lv02d_axis_##_axis		\
}
static struct dmi_system_id lis3lv02d_dmi_ids[] = {
	/* product names are truncated to match all kinds of a same model */
	AXIS_DMI_MATCH("NC64x0", "HP Compaq nc64", x_inverted),
	AXIS_DMI_MATCH("NC84x0", "HP Compaq nc84", z_inverted),
	AXIS_DMI_MATCH("NX9420", "HP Compaq nx9420", x_inverted),
	AXIS_DMI_MATCH("NW9440", "HP Compaq nw9440", x_inverted),
	AXIS_DMI_MATCH("NC2510", "HP Compaq 2510", y_inverted),
	AXIS_DMI_MATCH("NC8510", "HP Compaq 8510", xy_swap_inverted),
	AXIS_DMI_MATCH("HP2133", "HP 2133", xy_rotated_left),
	{ NULL, }
/* Laptop models without axis info (yet):
 * "NC651xx" "HP Compaq 651"
 * "NC671xx" "HP Compaq 671"
 * "NC6910" "HP Compaq 6910"
 * HP Compaq 8710x Notebook PC / Mobile Workstation
 * "NC2400" "HP Compaq nc2400"
 * "NX74x0" "HP Compaq nx74"
 * "NX6325" "HP Compaq nx6325"
 * "NC4400" "HP Compaq nc4400"
 */
};

static int lis3lv02d_add(struct device *device)
{
	u8 val;

	if (!device)
		return -EINVAL;

	adev.device = device;

	lis3lv02d_read(WHO_AM_I, &val);
	if ((val != LIS3LV02DL_ID) && (val != LIS302DL_ID)) {
		printk(KERN_ERR	"lis3lv02d"
				": Accelerometer chip not LIS3LV02D{L,Q}\n");
	}


	/* If possible use a "standard" axes order */
	if (dmi_check_system(lis3lv02d_dmi_ids) == 0) {
		printk(KERN_INFO "lis3lv02d" ": laptop model unknown, "
				 "using default axes configuration\n");
		adev.ac = lis3lv02d_axis_normal;
	}

	return lis3lv02d_init_device(&adev);
}


/* Sysfs stuff */
static ssize_t lis3lv02d_position_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int x, y, z;

	lis3lv02d_increase_use(&adev);
	lis3lv02d_get_xyz(&x, &y, &z);
	lis3lv02d_decrease_use(&adev);
	return sprintf(buf, "(%d,%d,%d)\n", x, y, z);
}

static ssize_t lis3lv02d_calibrate_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "(%d,%d,%d)\n", adev.xcalib, adev.ycalib, adev.zcalib);
}

static ssize_t lis3lv02d_calibrate_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	lis3lv02d_increase_use(&adev);
	lis3lv02d_calibrate_joystick();
	lis3lv02d_decrease_use(&adev);
	return count;
}

/* conversion btw sampling rate and the register values */
static int lis3lv02dl_df_val[4] = {40, 160, 640, 2560};
static ssize_t lis3lv02d_rate_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 ctrl;
	int val;

	lis3lv02d_increase_use(&adev);
	lis3lv02d_read(CTRL_REG1, &ctrl);
	lis3lv02d_decrease_use(&adev);
	val = (ctrl & (CTRL1_DF0 | CTRL1_DF1)) >> 4;
	return sprintf(buf, "%d\n", lis3lv02dl_df_val[val]);
}

static DEVICE_ATTR(position, S_IRUGO, lis3lv02d_position_show, NULL);
static DEVICE_ATTR(calibrate, S_IRUGO|S_IWUSR, lis3lv02d_calibrate_show,
	lis3lv02d_calibrate_store);
static DEVICE_ATTR(rate, S_IRUGO, lis3lv02d_rate_show, NULL);

static struct attribute *lis3lv02d_attributes[] = {
	&dev_attr_position.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_rate.attr,
	NULL
};

static struct attribute_group lis3lv02d_attribute_group = {
	.attrs = lis3lv02d_attributes
};

static int lis3lv02d_add_fs(struct device *device)
{
	return sysfs_create_group(&adev.device->kobj, &lis3lv02d_attribute_group);
}

static int lis3lv02d_remove_fs(void)
{
	sysfs_remove_group(&adev.device->kobj, &lis3lv02d_attribute_group);
	return 0;
}


#ifdef	CONFIG_PM
static int lis3lv02d_suspend(struct i2c_client *client, pm_message_t state)
{
	lis3lv02d_poweroff();
	return 0;
}

static int lis3lv02d_resume(struct i2c_client *client)
{
	lis3lv02d_poweron();
	return 0;
}
#else
#define	lis3lv02d_suspend		NULL
#define	lis3lv02d_resume		NULL
#endif

#ifdef	CONFIG_PROC_FS
#define	LIS331DL_PROC_FILE	"driver/lis3lv02d"
static struct proc_dir_entry *lis3lv02d_proc_file;
static int index;

static ssize_t lis3lv02d_proc_read(struct file *filp,
		char *buffer, size_t length, loff_t *offset)
{
	u8 reg_val;

	if ((index < 0) || (index > 0x3f))
		return 0;

	lis3lv02d_read(index, &reg_val);
	printk(KERN_INFO "register 0x%x: 0x%x\n", index, reg_val);
	return 0;
}

static ssize_t lis3lv02d_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	u8 reg_val;
	char messages[256], vol[256];
	int i;
	u8 x, y, z;

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('-' == messages[0]) {
		/* set the register index */
		memcpy(vol, messages+1, len-1);
		index = (int) simple_strtoul(vol, NULL, 16);
	}else if ('g' == messages[0]){
		for (i=0; i<10; i++) {
			lis3lv02d_raw_xyz(&x, &y, &z);
			printk("G sensor: x=0x%x, y=0x%x, z=0x%x\n", x, y, z);
			msleep(999);
		}
	}else if ('d' == messages[0]){
		memcpy(vol, messages+1, len-1);
		adev.delay = (unsigned int) simple_strtoul(vol, NULL, 16);
		wake_up(&adev.wait);
		printk(KERN_INFO "set lis3lv02d delay to 0x%x\n", adev.delay);

	} else {
	    /* set the register value */
		reg_val = (int)simple_strtoul(messages, NULL, 16);
		lis3lv02d_write(index, reg_val & 0xFF);
	}

	return len;
}

static struct file_operations lis3lv02d_proc_ops = {
	.read = lis3lv02d_proc_read,
	.write = lis3lv02d_proc_write,
};

static void create_lis3lv02d_proc_file(void)
{
	lis3lv02d_proc_file = create_proc_entry(LIS331DL_PROC_FILE, 0644, NULL);
	if (lis3lv02d_proc_file) {
		lis3lv02d_proc_file->owner = THIS_MODULE;
		lis3lv02d_proc_file->proc_fops = &lis3lv02d_proc_ops;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_lis3lv02d_proc_file(void)
{
	extern struct proc_dir_entry proc_root;
	remove_proc_entry(LIS331DL_PROC_FILE, &proc_root);
}

#endif
static int __devinit lis3lv02d_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	int chip_id;

	chip_id = i2c_smbus_read_byte_data(client, 0x0F);
	if (chip_id < 0) {
		printk(KERN_WARNING "lis3lv02d unavailable!\n");
		g_client = NULL;
		return -ENXIO;
	} else {
		printk(KERN_INFO "lis3lv02d(chip id:0x%02x) detected.\n", chip_id);
	}

	g_client = client;

	lis3lv02d_add(&client->dev);


#ifdef	CONFIG_PROC_FS
	create_lis3lv02d_proc_file();
#endif
	return 0;
}

static int lis3lv02d_remove(struct i2c_client *client)
{
	lis3lv02d_joystick_disable();
	lis3lv02d_poweroff();
	lis3lv02d_remove_fs();

#ifdef	CONFIG_PROC_FS
	remove_lis3lv02d_proc_file();
#endif
	return 0;
}

static const struct i2c_device_id lis3lv02d_id[] = {
	{ "lis3lv02d", 0 },
	{ }
};

static struct i2c_driver lis3lv02d_driver = {
	.driver = {
		.name	= "lis3lv02d",
	},
	.id_table 	= lis3lv02d_id,
	.probe		= lis3lv02d_probe,
	.remove		= lis3lv02d_remove,
	.suspend	= lis3lv02d_suspend,
	.resume		= lis3lv02d_resume,
};

static int __init lis3lv02d_i2c_init(void)
{
	return i2c_add_driver(&lis3lv02d_driver);
}

static void __exit lis3lv02d_i2c_exit(void)
{
	i2c_del_driver(&lis3lv02d_driver);
}

/*====================================================================*/




MODULE_DESCRIPTION("ST LIS3LV02Dx three-axis digital accelerometer (I2C) driver");
MODULE_AUTHOR("Bin Yang <bin.yang@marvell.com>");
MODULE_LICENSE("GPL");

module_init(lis3lv02d_i2c_init);
module_exit(lis3lv02d_i2c_exit);
