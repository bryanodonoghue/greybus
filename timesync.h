/*
 * TimeSync API driver.
 *
 * Copyright 2016 Google Inc.
 * Copyright 2016 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 */

#ifndef __TIMESYNC_H
#define __TIMESYNC_H

#include <linux/interrupt.h>
#include "timesync_platform.h"

int gb_timesync_interface_add(struct gb_interface *interface);
void gb_timesync_interface_remove(struct gb_interface *interface);
int gb_timesync_hd_add(struct gb_host_device *hd);
void gb_timesync_hd_remove(struct gb_host_device *hd);
void gb_timesync_svc_add(struct gb_svc *svc);
void gb_timesync_svc_remove(struct gb_svc *svc);

int gb_timesync_init(void);
void gb_timesync_exit(void);
void gb_timesync_irq(void);
u64 gb_timesync_get_frame_time(void);
int gb_timesync_frame_time_to_timespec(u64 frame_time, struct timespec *ts);
int gb_timesync_schedule_asynchronous(void);
int gb_timesync_schedule_synchronous(void);

#endif /* __TIMESYNC_H */
