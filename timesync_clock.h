/*
 * TimeSync API driver.
 *
 * Copyright 2015 Google Inc.
 * Copyright 2015 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 */

#ifndef __TIMESYNC_CLOCK_H
#define __TIMESYNC_CLOCK_H

u64 gb_timesync_clock_get_counter(void);
u32 gb_timesync_clock_get_rate(void);
u64 gb_timesync_clock2ns(u64 clocks);
void gb_timesync_clock_enable(void);
void gb_timesync_clock_disable(void);

int gb_timesync_clock_init(void);
void gb_timesync_clock_exit(void);

#endif /* __TIMESYNC_CLOCK_H */

