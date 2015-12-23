/*
 * TimeSync API driver.
 *
 * Copyright 2016 Google Inc.
 * Copyright 2016 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 *
 * Note on x86 get_cycles() just reads from the local TSC which is a per core
 * counter and therefore will vary on multi-core x86 systems. This is not a
 * problem for us here since the objective is to enable x86 Greybus users to
 * compile and run Greybus - not to participate fully in the TimeSync protocol.
 * Greybus Timesynce requires the AP, SVC and AP/GPBridges to all reside on the
 * same PCB and to share common signalling lines.
 */

#include <asm/tsc.h>
#include <asm/timer.h>
#include "greybus.h"

u64 gb_timesync_platform_get_counter(void)
{
	return (u64)get_cycles();
}

u32 gb_timesync_platform_get_clock_rate(void)
{
	return (u32)tsc_khz * 1000;
}

void gb_timesync_platform_clock_enable(void){}

void gb_timesync_platform_clock_disable(void){}

int gb_timesync_platform_lock_bus(void)
{
	return 0;
}

void gb_timesync_platform_unlock_bus(void) {}

int __init gb_timesync_platform_init(void)
{
	return 0;
}

void gb_timesync_platform_exit(void){}
