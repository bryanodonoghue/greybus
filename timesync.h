/*
 * TimeSync API driver.
 *
 * Copyright 2015 Google Inc.
 * Copyright 2015 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 */

#ifndef __TIMESYNC_H
#define __TIMESYNC_H

#include <linux/interrupt.h>

void gb_timesync_irq(void);
u64 gb_timesync_get_frame_time(void);

int gb_timesync_interface_add(struct gb_interface *interface);
void gb_timesync_interface_remove(struct gb_interface *interface);
void gb_timesync_svc_add(struct gb_svc *svc);
void gb_timesync_svc_remove(struct gb_svc *svc);

int gb_timesync_init(void);
void gb_timesync_exit(void);

#endif /* __TIMESYNC_H */
