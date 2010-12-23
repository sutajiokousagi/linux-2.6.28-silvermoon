/*
 *  linux/arch/arm/mach-mmp/clock.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include "clock.h"

static void apbc_clk_enable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst = APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(clk->fnclksel);
	__raw_writel(clk_rst, clk->clk_rst);
}

static void apbc_clk_disable(struct clk *clk)
{
	__raw_writel(0, clk->clk_rst);
}

struct clkops apbc_clk_ops = {
	.enable		= apbc_clk_enable,
	.disable	= apbc_clk_disable,
};

static int apbc_uart_clk_setrate(struct clk *clk, unsigned long rate)
{
	if ((clk->fnclksel && (rate <= clk->rate)) &&
	   (!clk->fnclksel && (rate > clk->rate)))
		return 0;

	if (clk->enabled)
		clk->ops->disable(clk);

	if (rate > clk->rate)
		clk->fnclksel = 0;
	else
		clk->fnclksel = 1;

	if (clk->enabled)
		clk->ops->enable(clk);

	return 0;
}

struct clkops apbc_uart_clk_ops = {
	.enable		= apbc_clk_enable,
	.disable	= apbc_clk_disable,
	.setrate	= apbc_uart_clk_setrate,
};

static void apmu_clk_enable(struct clk *clk)
{
	__raw_writel(clk->enable_val, clk->clk_rst);
}

static void apmu_clk_disable(struct clk *clk)
{
	__raw_writel(0, clk->clk_rst);
}

static int apmu_clk_setrate(struct clk *clk, unsigned long rate)
{
	__raw_writel(rate, clk->clk_rst);
	return 0;
}

struct clkops apmu_clk_ops = {
	.enable		= apmu_clk_enable,
	.disable	= apmu_clk_disable,
	.setrate	= apmu_clk_setrate,
};


static void pseudo_clk_enable(struct clk *clk)
{
}

static void pseudo_clk_disable(struct clk *clk)
{
}

struct clkops pseudo_clk_ops = {
        .enable         = pseudo_clk_enable,
        .disable        = pseudo_clk_disable,
};

static void u2o_clk_enable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);

	tmp |= 0x9;
	__raw_writel(tmp, clk->clk_rst);
}

static void u2o_clk_disable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);

	tmp &= ~0x9;
	__raw_writel(tmp, clk->clk_rst);
}

struct clkops u2o_clk_ops = {
	.enable		= u2o_clk_enable,
	.disable	= u2o_clk_disable,
};

static void u2h_clk_enable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);

	tmp |= 0x12;
	__raw_writel(tmp, clk->clk_rst);
}

static void u2h_clk_disable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);

	tmp &= ~0x12;
	__raw_writel(tmp, clk->clk_rst);
}

struct clkops u2h_clk_ops = {
	.enable		= u2h_clk_enable,
	.disable	= u2h_clk_disable,
};

static void cf_clk_enable(struct clk *clk)
{
	__raw_writel(0xD8, clk->clk_rst);
	udelay(10);
	__raw_writel(0xDB, clk->clk_rst);
}

static void cf_clk_disable(struct clk *clk)
{
	__raw_writel(0, clk->clk_rst);
}

struct clkops cf_clk_ops = {
	.enable		= cf_clk_enable,
	.disable	= cf_clk_disable,
};

static DEFINE_SPINLOCK(clocks_lock);

int clk_enable(struct clk *clk)
{
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);
	if (clk->enabled++ == 0)
		clk->ops->enable(clk);
	spin_unlock_irqrestore(&clocks_lock, flags);
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;

	WARN_ON(clk->enabled == 0);

	spin_lock_irqsave(&clocks_lock, flags);
	if (--clk->enabled == 0)
		clk->ops->disable(clk);
	spin_unlock_irqrestore(&clocks_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	unsigned long rate;

	if (clk->ops->getrate)
		rate = clk->ops->getrate(clk);
	else
		rate = clk->rate;

	return rate;
}
EXPORT_SYMBOL(clk_get_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long flags;
	int ret = -EINVAL;

	if (clk->ops->setrate) {
		spin_lock_irqsave(&clocks_lock, flags);
		ret = clk->ops->setrate(clk, rate);
		spin_unlock_irqrestore(&clocks_lock, flags);
	}

	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

void clks_register(struct clk_lookup *clks, size_t num)
{
	int i;

	for (i = 0; i < num; i++)
		clkdev_add(&clks[i]);
}
