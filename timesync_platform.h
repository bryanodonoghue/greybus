/*
 * TimeSync API driver.
 *
 * Copyright 2016 Google Inc.
 * Copyright 2016 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 */

#ifndef __TIMESYNC_PLATFORM_H
#define __TIMESYNC_PLATFORM_H

u64 gb_timesync_platform_get_counter(void);
u32 gb_timesync_platform_get_clock_rate(void);
int gb_timesync_platform_lock_bus(void);
void gb_timesync_platform_unlock_bus(void);
void gb_timesync_platform_clock_enable(void);
void gb_timesync_platform_clock_disable(void);

int gb_timesync_platform_init(void);
void gb_timesync_platform_exit(void);

#endif /* __TIMESYNC_PLATFORM_H */

