/*
 * TimeSync API driver.
 *
 * Copyright 2015 Google Inc.
 * Copyright 2015 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 *
 * This code reads directly from an ARMv7 memory-mapped timer that lives in
 * MMIO space. Since this counter lives inside of MMIO space its shared between
 * cores and that means we don't have to worry about issues like TSC on x86 where
 * each TSC is local to a particular core..
 *
 * Register-level access code is based on
 * drivers/clocksource/arm_arch_timer.c
 */
#include <linux/cpufreq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <asm/arch_timer.h>

#include "greybus.h"

#define CTRL_ENABLE	(1 << 0)
#define CNTPCT_LO       0x00
#define CNTPCT_HI       0x04
#define CNTVCT_LO       0x08
#define CNTVCT_HI       0x0c
#define CNTFRQ          0x10
#define CNTP_TVAL       0x28
#define CNTP_CTL        0x2c
#define CNTV_TVAL       0x38
#define CNTV_CTL        0x3c

#ifndef readl_relaxed_no_log
#define readl_relaxed_no_log readl_relaxed
#endif
#ifndef writel_relaxed_no_log
#define writel_relaxed_no_log writel_relaxed
#endif

static DEFINE_MUTEX(gb_timesync_clock_mutex);
static void __iomem *gb_timesync_clock_regs;
static u32 gb_timesync_clock_frequency;
static u32 gb_timesync_clock_mult;

u64 gb_timesync_clock_get_counter(void)
{
	 u32 lo, hi, tmp;

	/* get_cycles() for ARM systems without greybus-armv7-timer-mem */
	if (unlikely(!gb_timesync_clock_regs))
		return (u64)get_cycles();

	do {
		hi = readl_relaxed_no_log(gb_timesync_clock_regs + CNTPCT_HI);
		lo = readl_relaxed_no_log(gb_timesync_clock_regs + CNTPCT_LO);
		tmp = readl_relaxed_no_log(gb_timesync_clock_regs + CNTPCT_HI);
	} while (hi != tmp);

	return ((u64) hi << 32) | lo;
}

u64 gb_timesync_clock2ns(u64 clocks)
{
	return clocks * gb_timesync_clock_mult;
}

u32 gb_timesync_clock_get_rate(void)
{
	if (unlikely(!gb_timesync_clock_regs))
		return cpufreq_get(0);

	return gb_timesync_clock_frequency;
}

void gb_timesync_clock_enable(void)
{
	u32 val;

	mutex_lock(&gb_timesync_clock_mutex);

	if (unlikely(!gb_timesync_clock_regs))
		goto done;
	val = readl_relaxed_no_log(gb_timesync_clock_regs + CNTP_CTL);
	val |= ARCH_TIMER_CTRL_ENABLE;
	writel_relaxed_no_log(val, gb_timesync_clock_regs + CNTP_CTL);
done:
	mutex_unlock(&gb_timesync_clock_mutex);
}

void gb_timesync_clock_disable(void)
{
	u32 val;

	mutex_lock(&gb_timesync_clock_mutex);

	if (unlikely(!gb_timesync_clock_regs))
		goto done;
	val = readl_relaxed_no_log(gb_timesync_clock_regs + CNTP_CTL);
	val &= ~ARCH_TIMER_CTRL_ENABLE;
	writel_relaxed_no_log(val, gb_timesync_clock_regs + CNTP_CTL);
done:
	mutex_unlock(&gb_timesync_clock_mutex);
}

static const struct of_device_id arch_timer_of_match[] = {
	{ .compatible   = "google,greybus-frame-time-counter", },
	{},
};

int __init gb_timesync_clock_init(void)
{
	struct device_node *np;
	u32 reg_frequency;

	np = of_find_matching_node(NULL, arch_timer_of_match);
	if (!np) {
		/* Tolerate not finding to allow BBB etc to continue */
		pr_warn("Unable to find a compatible ARMv7 timer\n");
		return 0;
	}

	if (of_property_read_u32(np, "clock-frequency",
				 &gb_timesync_clock_frequency)) {
		pr_err("Unable to find timer clock-frequency\n");
		return -ENODEV;
	}

	gb_timesync_clock_regs = of_iomap(np, 0);
	if (!gb_timesync_clock_regs) {
		pr_err("Cannot map armv7-timer registers\n");
		return -ENODEV;
	}

	reg_frequency = readl_relaxed_no_log(gb_timesync_clock_regs + CNTFRQ);
	if (reg_frequency != gb_timesync_clock_frequency) {
		pr_warn("dts clock frequency %d hardware reports %d\n",
			gb_timesync_clock_frequency,
			reg_frequency);
		gb_timesync_clock_frequency = reg_frequency;
	}
	gb_timesync_clock_mult = NSEC_PER_SEC / gb_timesync_clock_frequency;
	return 0;
}

void gb_timesync_clock_exit(void)
{
	gb_timesync_clock_disable();
	if (gb_timesync_clock_regs)
		iounmap(gb_timesync_clock_regs);
}
