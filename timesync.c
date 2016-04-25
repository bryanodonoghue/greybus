/*
 * TimeSync API driver.
 *
 * Copyright 2016 Google Inc.
 * Copyright 2016 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 */
#include <linux/debugfs.h>
#include <linux/hrtimer.h>
#include "greybus.h"
#include "greybus_trace.h"
#include "timesync_platform.h"

/*
 * Minimum inter-strobe value of one millisecond is chosen because it
 * just-about fits the common definition of a jiffy.
 *
 * Maximum value OTOH is constrained by the number of bits the SVC can fit
 * into a 16 bit up-counter. The SVC configures the timer in microseconds
 * so the maximum allowable value is 65535 microseconds. We clip that value
 * to 10000 microseconds for the sake of using nice round base 10 numbers
 * and since right-now there's no imaginable use-case requiring anything
 * other than a one millisecond inter-strobe time, let alone something
 * higher than ten milliseconds.
 */
#define GB_TIMESYNC_MIN_STROBE_DELAY_US		1000
#define GB_TIMESYNC_MAX_STROBE_DELAY_US		10000
#define GB_TIMESYNC_DEFAULT_OFFSET_US		1000

/* Work queue timers long, short and SVC strobe timeout */
#define GB_TIMESYNC_DELAYED_WORK_LONG		msecs_to_jiffies(1000)
#define GB_TIMESYNC_DELAYED_WORK_SHORT		msecs_to_jiffies(1)
#define GB_TIMESYNC_MAX_WAIT_SVC		msecs_to_jiffies(5000)
#define GB_TIMESYNC_KTIME_UPDATE		msecs_to_jiffies(1000)

/* Workqueue */
static void gb_timesync_worker(struct work_struct *work);
static struct workqueue_struct *gb_timesync_wq;
static struct delayed_work gb_timesync_delayed_work;

/* Spinlock for synchronizing external API, worker and ISR */
static DEFINE_SPINLOCK(gb_timesync_spinlock);
static DEFINE_MUTEX(gb_timesync_mutex);
static LIST_HEAD(gb_timesync_interface_list);
static LIST_HEAD(gb_timesync_host_device_list);

/* Waitqueue for time-sync completions */
static DECLARE_WAIT_QUEUE_HEAD(gb_timesync_wait_queue);

/* Debugfs */
static struct dentry *gb_timesync_dentry;

/* The current local frame-time */
static u64 __read_mostly gb_timesync_frame_time_offset;
struct gb_timesync_frame_time_data {
	u64 frame_time;
	struct timespec ts;
};

static struct gb_timesync_frame_time_data gb_timesync_strobe_data[GB_TIMESYNC_MAX_STROBES];
static struct gb_timesync_frame_time_data gb_timesync_ktime_data;

struct gb_timesync_interface {
	struct list_head list;
	struct gb_interface *interface;
	u64 ping_frame_time;
};

struct gb_timesync_host_device {
	struct list_head list;
	struct gb_host_device *hd;
	u64 ping_frame_time;
};

enum gb_timesync_state {
	GB_TIMESYNC_STATE_INVALID		= 0,
	GB_TIMESYNC_STATE_INACTIVE		= 1,
	GB_TIMESYNC_STATE_INIT			= 2,
	GB_TIMESYNC_STATE_WAIT_SVC		= 3,
	GB_TIMESYNC_STATE_AUTHORITATIVE		= 4,
	GB_TIMESYNC_STATE_PING			= 5,
	GB_TIMESYNC_STATE_ACTIVE		= 6,
};

static struct gb_svc *gb_timesync_svc;
static u64 gb_timesync_ap_ping_frame_time;
static u64 gb_timesync_svc_ping_frame_time;
static bool gb_timesync_offset_down;
static bool gb_timesync_print_ping;
static bool gb_timesync_capture_ping;
static int gb_timesync_state;
static int gb_timesync_strobe;
static u32 gb_timesync_strobe_delay;
static u32 gb_timesync_strobe_mask;
static int gb_timesync_strobe_count;
static u64 gb_timesync_ns_per_clock;
static u64 gb_timesync_fs_per_clock;
static u32 gb_timesync_max_ktime_diff;
static unsigned long gb_timesync_clock_rate;
static struct timer_list gb_timesync_ktime_timer;

static u64 gb_timesync_adjust_count(u64 counts)
{
	if (gb_timesync_offset_down)
		return counts - gb_timesync_frame_time_offset;
	else
		return counts + gb_timesync_frame_time_offset;
}

/*
 * This function provides the authoritative frame-time to a calling function. It
 * is designed to be lockless and should remain that way the caller is assumed
 * to be state-aware.
 */
static u64 gb_timesync_get_frame_time_raw(void)
{
	u64 clocks = gb_timesync_platform_get_counter();

	return gb_timesync_adjust_count(clocks);
}

static void gb_timesync_schedule_svc_timeout(void)
{
	queue_delayed_work(gb_timesync_wq, &gb_timesync_delayed_work,
			   GB_TIMESYNC_MAX_WAIT_SVC);
}

static void gb_timesync_set_state(int state)
{
	switch (state) {
	case GB_TIMESYNC_STATE_INVALID:
		gb_timesync_state = state;
		wake_up(&gb_timesync_wait_queue);
		break;
	case GB_TIMESYNC_STATE_INACTIVE:
		if (gb_timesync_state != GB_TIMESYNC_STATE_INIT) {
			gb_timesync_state = state;
			wake_up(&gb_timesync_wait_queue);
		}
		break;
	case GB_TIMESYNC_STATE_INIT:
		if (gb_timesync_state != GB_TIMESYNC_STATE_INVALID) {
			gb_timesync_strobe = 0;
			gb_timesync_frame_time_offset = 0;
			gb_timesync_state = state;
			cancel_delayed_work(&gb_timesync_delayed_work);
			queue_delayed_work(gb_timesync_wq,
					   &gb_timesync_delayed_work,
					   GB_TIMESYNC_DELAYED_WORK_LONG);
		}
		break;
	case GB_TIMESYNC_STATE_WAIT_SVC:
		if (gb_timesync_state == GB_TIMESYNC_STATE_INIT)
			gb_timesync_state = state;
		break;
	case GB_TIMESYNC_STATE_AUTHORITATIVE:
		if (gb_timesync_state == GB_TIMESYNC_STATE_WAIT_SVC) {
			gb_timesync_state = state;
			cancel_delayed_work(&gb_timesync_delayed_work);
			queue_delayed_work(gb_timesync_wq,
					   &gb_timesync_delayed_work, 0);
		}
		break;
	case GB_TIMESYNC_STATE_PING:
		if (gb_timesync_state == GB_TIMESYNC_STATE_ACTIVE) {
			gb_timesync_state = state;
			queue_delayed_work(gb_timesync_wq,
					   &gb_timesync_delayed_work,
					   GB_TIMESYNC_DELAYED_WORK_SHORT);
		}
		break;
	case GB_TIMESYNC_STATE_ACTIVE:
		if (gb_timesync_state == GB_TIMESYNC_STATE_AUTHORITATIVE ||
		    gb_timesync_state == GB_TIMESYNC_STATE_PING) {
			gb_timesync_state = state;
			wake_up(&gb_timesync_wait_queue);
		}
		break;
	}

	if (gb_timesync_state != state) {
		pr_err("Invalid state transition %d=>%d\n",
		       gb_timesync_state, state);
	}
}

static void gb_timesync_set_state_atomic(int state)
{
	unsigned long flags;

	spin_lock_irqsave(&gb_timesync_spinlock, flags);
	gb_timesync_set_state(state);
	spin_unlock_irqrestore(&gb_timesync_spinlock, flags);
}

static u64 gb_timesync_diff(u64 x, u64 y)
{
	if (x > y)
		return x - y;
	else
		return y - x;
}

static void gb_timesync_adjust_to_svc(u64 svc_frame_time, u64 ap_frame_time)
{
	if (svc_frame_time > ap_frame_time) {
		gb_timesync_frame_time_offset = svc_frame_time - ap_frame_time;
		gb_timesync_offset_down = false;
	} else {
		gb_timesync_frame_time_offset = ap_frame_time - svc_frame_time;
		gb_timesync_offset_down = true;
	}
}

/*
 * Associate a frame-time with a ktime timestamp represented as struct timespec
 * Requires the calling context to hold gb_timesync_mutex
 */
static void gb_timesync_store_ktime_data(struct timespec ts, u64 frame_time)
{
	gb_timesync_ktime_data.ts = ts;
	gb_timesync_ktime_data.frame_time = frame_time;
}

/*
 * Find the two pulses that best-match our expected inter-strobe gap and
 * then calculate the difference between the SVC time at the second pulse
 * to the local time at the second pulse.
 */
static void gb_timesync_collate_frame_time(u64 *frame_time)
{
	int i = 0;
	u64 delta, best_match, ap_frame_time;
	u64 strobe_delay_ns = gb_timesync_strobe_delay * NSEC_PER_USEC;
	u64 least = 0;

	for (i = 1; i < gb_timesync_strobe_count; i++) {
		delta = gb_timesync_strobe_data[i].frame_time -
			gb_timesync_strobe_data[i - 1].frame_time;
		delta *= gb_timesync_ns_per_clock;
		best_match = delta;
		delta = gb_timesync_diff(delta, strobe_delay_ns);

		if (!least || delta < least) {
			least = delta;
			gb_timesync_adjust_to_svc(frame_time[i],
						  gb_timesync_strobe_data[i].frame_time);

			ap_frame_time = gb_timesync_strobe_data[i].frame_time;
			ap_frame_time = gb_timesync_adjust_count(ap_frame_time);
			gb_timesync_store_ktime_data(gb_timesync_strobe_data[i].ts,
						     ap_frame_time);

			pr_debug("adjust %s local %llu svc %llu delta %llu\n",
				 gb_timesync_offset_down ? "down" : "up",
				 gb_timesync_strobe_data[i].frame_time,
				 frame_time[i], delta);
			pr_debug("strobe %d => %d diff %llu target %llu\n",
				 i - 1, i, best_match, strobe_delay_ns);
		}
	}
}

static void gb_timesync_teardown(void)
{
	struct gb_timesync_interface *timesync_interface;
	struct gb_timesync_host_device *timesync_host_device;
	struct gb_svc *svc = gb_timesync_svc;
	struct gb_interface *interface;
	struct gb_host_device *hd;
	int ret;

	list_for_each_entry(timesync_interface,
			    &gb_timesync_interface_list, list) {
		interface = timesync_interface->interface;
		ret = gb_control_timesync_disable(interface->control);
		if (ret) {
			dev_err(&interface->dev,
				"gb_control_timesync_disable %d\n", ret);
		}
	}

	list_for_each_entry(timesync_host_device,
			    &gb_timesync_host_device_list, list) {
		hd = timesync_host_device->hd;
		ret = hd->driver->timesync_disable(hd);
		if (ret < 0) {
			dev_err(&hd->dev, "host timesync_disable %d\n",
				ret);
		}
	}

	gb_svc_timesync_wake_pins_release(svc);
	gb_svc_timesync_disable(svc);
	gb_timesync_platform_unlock_bus();

	gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_INACTIVE);
}

static void gb_timesync_platform_lock_bus_fail(int ret)
{
	if (ret == -EAGAIN) {
		pr_info("bus busy timesync rescheduled\n");
		gb_timesync_set_state(gb_timesync_state);
	} else {
		pr_err("Failed to lock timesync bus %d\n", ret);
		gb_timesync_set_state(GB_TIMESYNC_STATE_INACTIVE);
	}
}

static void gb_timesync_enable(void)
{
	struct gb_svc *svc = gb_timesync_svc;
	struct gb_host_device *hd;
	struct gb_timesync_interface *timesync_interface;
	struct gb_timesync_host_device *timesync_host_device;
	struct gb_interface *interface;
	int strobe_count = gb_timesync_strobe_count;
	u64 init_frame_time;
	u32 strobe_delay = gb_timesync_strobe_delay;
	unsigned long clock_rate = gb_timesync_clock_rate;
	int ret;

	/*
	 * Get access to the wake pins in the AP and SVC
	 * Release these pins either in gb_timesync_teardown() or in
	 * gb_timesync_authoritative()
	 */
	ret = gb_timesync_platform_lock_bus();
	if (ret < 0) {
		gb_timesync_platform_lock_bus_fail(ret);
		return;
	}
	ret = gb_svc_timesync_wake_pins_acquire(svc,  gb_timesync_strobe_mask);
	if (ret) {
		dev_err(&svc->dev,
			"gb_svc_timesync_wake_pins_acquire %d\n", ret);
		gb_timesync_teardown();
		return;
	}

	/* Choose an initial time in the future */
	init_frame_time = gb_timesync_get_frame_time_raw() + 100000UL;

	/* Send enable command to all relevant participants */
	list_for_each_entry(timesync_interface, &gb_timesync_interface_list,
			    list) {
		interface = timesync_interface->interface;
		ret = gb_control_timesync_enable(interface->control,
						 strobe_count,
						 init_frame_time,
						 strobe_delay,
						 clock_rate);
		if (ret) {
			dev_err(&interface->dev,
				"gb_control_timesync_enable %d\n", ret);
		}
	}
	list_for_each_entry(timesync_host_device,
			    &gb_timesync_host_device_list, list) {
		hd = timesync_host_device->hd;
		ret = hd->driver->timesync_enable(hd, strobe_count,
						  init_frame_time,
						  strobe_delay,
						  clock_rate);
		if (ret < 0) {
			dev_err(&hd->dev, "host timesync_enable %d\n",
				ret);
		}
	}
	gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_WAIT_SVC);
	ret = gb_svc_timesync_enable(svc, strobe_count, init_frame_time,
				     strobe_delay, clock_rate);
	if (ret) {
		dev_err(&svc->dev,
			"gb_svc_timesync_enable %d\n", ret);
		gb_timesync_teardown();
		return;
	}

	/* Schedule a timeout waiting for SVC to complete strobing */
	gb_timesync_schedule_svc_timeout();
}

static void gb_timesync_authoritative(void)
{
	struct gb_svc *svc = gb_timesync_svc;
	struct gb_host_device *hd;
	struct gb_timesync_interface *timesync_interface;
	struct gb_timesync_host_device *timesync_host_device;
	struct gb_interface *interface;
	u64 svc_frame_time[GB_TIMESYNC_MAX_STROBES];
	int ret;

	/* Get authoritative time from SVC and adjust local clock */
	ret = gb_svc_timesync_authoritative(svc, svc_frame_time);
	if (ret) {
		dev_err(&svc->dev,
			"gb_svc_timesync_authoritative %d\n", ret);
		gb_timesync_teardown();
		return;
	}
	gb_timesync_collate_frame_time(svc_frame_time);

	/* Transmit authoritative time to downstream slaves */
	list_for_each_entry(timesync_host_device,
			    &gb_timesync_host_device_list, list) {
		hd = timesync_host_device->hd;
		ret = hd->driver->timesync_authoritative(hd,
							 svc_frame_time);
		if (ret < 0) {
			dev_err(&hd->dev,
				"host timesync_authoritative %d\n",
				ret);
		}
	}
	list_for_each_entry(timesync_interface,
			    &gb_timesync_interface_list, list) {
		interface = timesync_interface->interface;
		ret = gb_control_timesync_authoritative(
						interface->control,
						svc_frame_time);
		if (ret) {
			dev_err(&interface->dev,
				"gb_control_timesync_enable %d\n", ret);
		}
	}

	/* Release wake pins */
	gb_svc_timesync_wake_pins_release(svc);
	gb_timesync_platform_unlock_bus();

	/* Transition to state ACTIVE */
	gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_ACTIVE);

	/* Schedule a ping to verify the synchronized system time */
	gb_timesync_print_ping = true;
	gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_PING);
}

static size_t gb_timesync_log_frame_time(char *buf, size_t buflen)
{
	struct gb_svc *svc = gb_timesync_svc;
	struct gb_host_device *hd;
	struct gb_timesync_interface *timesync_interface;
	struct gb_timesync_host_device *timesync_host_device;
	struct gb_interface *interface;
	unsigned int len;
	size_t off;

	/* AP/SVC */
	memset(buf, 0x00, buflen);
	off = snprintf(buf, buflen, "timesync: ping-time ap=%llu %s=%llu ",
		       gb_timesync_ap_ping_frame_time, dev_name(&svc->dev),
		       gb_timesync_svc_ping_frame_time);
	len = buflen - off;

	/* APB/GPB */
	list_for_each_entry(timesync_host_device,
			    &gb_timesync_host_device_list, list) {
		if (len < buflen) {
			hd = timesync_host_device->hd;
			off += snprintf(&buf[off], len, "%s=%llu ",
					dev_name(&hd->dev),
					timesync_host_device->ping_frame_time);
			len = buflen - off;
		}
	}
	list_for_each_entry(timesync_interface,
			    &gb_timesync_interface_list, list) {
		if (len < buflen) {
			interface = timesync_interface->interface;
			off += snprintf(&buf[off], len, "%s=%llu ",
					dev_name(&interface->dev),
					timesync_interface->ping_frame_time);
			len = buflen - off;
		}
	}
	if (len < buflen)
		off += snprintf(&buf[off], len, "\n");
	return off;
}

/*
 * Send an SVC initiated wake 'ping' to each TimeSync participant.
 * Get the frame-time from each participant associated with the wake
 * ping.
 */
static void gb_timesync_ping(void)
{
	struct gb_svc *svc = gb_timesync_svc;
	struct gb_host_device *hd;
	struct gb_timesync_interface *timesync_interface;
	struct gb_timesync_host_device *timesync_host_device;
	struct gb_control *control;
	u64 *ping_frame_time;
	int ret;

	/* Get access to the wake pins in the AP and SVC */
	ret = gb_timesync_platform_lock_bus();
	if (ret < 0) {
		gb_timesync_platform_lock_bus_fail(ret);
		return;
	}
	ret = gb_svc_timesync_wake_pins_acquire(svc, gb_timesync_strobe_mask);
	if (ret) {
		dev_err(&svc->dev,
			"gb_svc_timesync_wake_pins_acquire %d\n", ret);
		gb_timesync_teardown();
		return;
	}

	/* Have SVC generate a timesync ping */
	gb_timesync_capture_ping = true;
	ret = gb_svc_timesync_ping(svc, &gb_timesync_svc_ping_frame_time);
	gb_timesync_capture_ping = false;
	if (ret) {
		dev_err(&svc->dev,
			"gb_svc_timesync_ping %d\n", ret);
		gb_timesync_teardown();
		return;
	}

	/* Get the ping frame-time from each APB/GPB */
	list_for_each_entry(timesync_host_device,
			    &gb_timesync_host_device_list, list) {
		hd = timesync_host_device->hd;
		ret = hd->driver->timesync_get_last_event(hd,
			&timesync_host_device->ping_frame_time);
		if (ret) {
			dev_err(&hd->dev,
				"host timesync_get_last_event %d\n",
				ret);
		}
	}
	list_for_each_entry(timesync_interface,
			    &gb_timesync_interface_list, list) {
		control = timesync_interface->interface->control;
		ping_frame_time = &timesync_interface->ping_frame_time;
		ret = gb_control_timesync_get_last_event(control,
							 ping_frame_time);
		if (ret) {
			dev_err(&timesync_interface->interface->dev,
				"gb_control_timesync_get_last_event %d\n", ret);
		}
	}

	/* Ping success - move to timesync active */
	gb_svc_timesync_wake_pins_release(svc);
	gb_timesync_platform_unlock_bus();
	gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_ACTIVE);
}

static void gb_timesync_log_ping_time(void)
{
	char *buf;

	if (!gb_timesync_print_ping)
		return;

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (buf) {
		gb_timesync_log_frame_time(buf, PAGE_SIZE);
		pr_info("%s", buf);
		kfree(buf);
	}
}

/*
 * Perform the actual work of scheduled TimeSync logic.
 */
static void gb_timesync_worker(struct work_struct *work)
{
	mutex_lock(&gb_timesync_mutex);

	switch (gb_timesync_state) {
	case GB_TIMESYNC_STATE_INIT:
		gb_timesync_enable();
		break;

	case GB_TIMESYNC_STATE_WAIT_SVC:
		dev_err(&gb_timesync_svc->dev,
			"timeout SVC strobe completion\n");
		gb_timesync_teardown();
		break;

	case GB_TIMESYNC_STATE_AUTHORITATIVE:
		gb_timesync_authoritative();
		break;

	case GB_TIMESYNC_STATE_PING:
		gb_timesync_ping();
		gb_timesync_log_ping_time();
		break;

	default:
		pr_err("Invalid state %d for delayed work\n",
		       gb_timesync_state);
		break;
	}

	mutex_unlock(&gb_timesync_mutex);
}

/*
 * Schedule a new TimeSync INIT or PING operation serialized w/r to
 * gb_timesync_worker().
 */
static int gb_timesync_schedule(int state)
{
	int ret = 0;

	if (state != GB_TIMESYNC_STATE_INIT && state != GB_TIMESYNC_STATE_PING)
		return -EINVAL;

	mutex_lock(&gb_timesync_mutex);
	if (gb_timesync_state ==  GB_TIMESYNC_STATE_INACTIVE ||
	    gb_timesync_state == GB_TIMESYNC_STATE_ACTIVE) {
		gb_timesync_set_state_atomic(state);
	} else {
		ret = -ENODEV;
	}
	mutex_unlock(&gb_timesync_mutex);
	return ret;
}

int gb_timesync_schedule_asynchronous(void)
{
	return gb_timesync_schedule(GB_TIMESYNC_STATE_INIT);
}
EXPORT_SYMBOL_GPL(gb_timesync_schedule_asynchronous);

static int gb_timesync_get_status(void)
{
	int ret;

	switch (gb_timesync_state) {
	case GB_TIMESYNC_STATE_INVALID:
	case GB_TIMESYNC_STATE_INACTIVE:
		ret = -ENODEV;
		break;
	case GB_TIMESYNC_STATE_INIT:
	case GB_TIMESYNC_STATE_WAIT_SVC:
	case GB_TIMESYNC_STATE_AUTHORITATIVE:
	case GB_TIMESYNC_STATE_PING:
		ret = -EAGAIN;
		break;
	case GB_TIMESYNC_STATE_ACTIVE:
		ret = 0;
		break;
	}
	return ret;
}

static int __gb_timesync_schedule_synchronous(int state)
{
	unsigned long flags;
	int ret;

	ret = gb_timesync_schedule(state);
	if (ret)
		return ret;

	wait_event_interruptible(
		gb_timesync_wait_queue,
		(gb_timesync_state == GB_TIMESYNC_STATE_ACTIVE ||
		 gb_timesync_state == GB_TIMESYNC_STATE_INACTIVE ||
		 gb_timesync_state == GB_TIMESYNC_STATE_INVALID));

	mutex_lock(&gb_timesync_mutex);
	spin_lock_irqsave(&gb_timesync_spinlock, flags);

	ret = gb_timesync_get_status();

	spin_unlock_irqrestore(&gb_timesync_spinlock, flags);
	mutex_unlock(&gb_timesync_mutex);

	return ret;
}

/*
 * Schedule system timesync waiting for completion.
 */
int gb_timesync_schedule_synchronous(void)
{
	return __gb_timesync_schedule_synchronous(GB_TIMESYNC_STATE_INIT);
}
EXPORT_SYMBOL_GPL(gb_timesync_schedule_synchronous);

/*
 * Add a Greybus Interface to the set of TimeSync Interfaces.
 */
int gb_timesync_interface_add(struct gb_interface *interface)
{
	struct gb_timesync_interface *timesync_interface;

	if (!(interface->features & GREYBUS_INTERFACE_FEATURE_TIMESYNC))
		return 0;

	timesync_interface = kzalloc(sizeof(*timesync_interface), GFP_KERNEL);
	if (!timesync_interface)
		return -ENOMEM;

	mutex_lock(&gb_timesync_mutex);
	timesync_interface->interface = interface;
	list_add(&timesync_interface->list, &gb_timesync_interface_list);
	gb_timesync_strobe_mask |= 1 << interface->interface_id;
	gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_INIT);
	mutex_unlock(&gb_timesync_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(gb_timesync_interface_add);

/*
 * Remove a Greybus Interface from the set of TimeSync Interfaces.
 */
void gb_timesync_interface_remove(struct gb_interface *interface)
{
	struct gb_timesync_interface *timesync_interface;

	if (!(interface->features & GREYBUS_INTERFACE_FEATURE_TIMESYNC))
		return;

	mutex_lock(&gb_timesync_mutex);
	list_for_each_entry(timesync_interface,
			    &gb_timesync_interface_list, list) {
		if (timesync_interface->interface == interface) {
			gb_timesync_strobe_mask &=
				~(1 << interface->interface_id);
			list_del(&timesync_interface->list);
			kfree(timesync_interface);
			break;
		}
	}
	mutex_unlock(&gb_timesync_mutex);
}
EXPORT_SYMBOL_GPL(gb_timesync_interface_remove);

int gb_timesync_hd_add(struct gb_host_device *hd)
{
	struct gb_timesync_host_device *timesync_hd;

	timesync_hd = kzalloc(sizeof(*timesync_hd), GFP_KERNEL);
	if (!timesync_hd)
		return -ENOMEM;

	mutex_lock(&gb_timesync_mutex);
	timesync_hd->hd = hd;
	list_add(&timesync_hd->list, &gb_timesync_host_device_list);
	mutex_unlock(&gb_timesync_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(gb_timesync_hd_add);

void gb_timesync_hd_remove(struct gb_host_device *hd)
{
	struct gb_timesync_host_device *timesync_hd;

	mutex_lock(&gb_timesync_mutex);
	list_for_each_entry(timesync_hd,
			    &gb_timesync_host_device_list, list) {
		if (timesync_hd->hd == hd) {
			list_del(&timesync_hd->list);
			kfree(timesync_hd);
			break;
		}
	}
	mutex_unlock(&gb_timesync_mutex);
}
EXPORT_SYMBOL_GPL(gb_timesync_hd_remove);

void gb_timesync_svc_add(struct gb_svc *svc)
{
	mutex_lock(&gb_timesync_mutex);
	gb_timesync_svc = svc;
	gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_INACTIVE);
	mutex_unlock(&gb_timesync_mutex);
}
EXPORT_SYMBOL_GPL(gb_timesync_svc_add);

void gb_timesync_svc_remove(struct gb_svc *svc)
{
	mutex_lock(&gb_timesync_mutex);
	gb_timesync_svc = NULL;
	gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_INVALID);
	mutex_unlock(&gb_timesync_mutex);
}
EXPORT_SYMBOL_GPL(gb_timesync_svc_remove);

/*
 * Give the authoritative frame-time to the calling function. Returns zero if we
 * are not in GB_TIMESYNC_STATE_ACTIVE.
 */
u64 gb_timesync_get_frame_time(void)
{
	unsigned long flags;
	u64 ret;

	spin_lock_irqsave(&gb_timesync_spinlock, flags);
	if (gb_timesync_state == GB_TIMESYNC_STATE_ACTIVE)
		ret = gb_timesync_get_frame_time_raw();
	else
		ret = 0;
	spin_unlock_irqrestore(&gb_timesync_spinlock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(gb_timesync_get_frame_time);

/* Incrementally updates the conversion base from frame-time to ktime */
static void gb_timesync_ktime_timer_fn(unsigned long data)
{
	unsigned long flags;
	u64 frame_time;
	struct timespec ts;

	spin_lock_irqsave(&gb_timesync_spinlock, flags);

	if (gb_timesync_state != GB_TIMESYNC_STATE_ACTIVE)
		goto done;

	ktime_get_ts(&ts);
	frame_time = gb_timesync_get_frame_time_raw();
	gb_timesync_store_ktime_data(ts, frame_time);

done:
	spin_unlock_irqrestore(&gb_timesync_spinlock, flags);
	mod_timer(&gb_timesync_ktime_timer, jiffies + GB_TIMESYNC_KTIME_UPDATE);
}

/*
 * This routine takes a frame-time and derives the difference with-respect
 * to a reference frame-time/ktime pair. It then returns the calculated
 * ktime based on the difference between the supplied frame-time and
 * the reference frame-time.
 *
 * The time difference is calculated to six decimal places. Taking 19.2MHz
 * as an example this means we have 52.083333~ nanoseconds per clock or
 * 52083333~ femtoseconds per clock.
 *
 * Naively taking the count difference and converting to
 * seconds/nanoseconds would quickly see the 0.0833 component produce
 * noticeable errors. For example a time difference of one second would
 * loose 19200000 * 0.08333x nanoseconds or 1.59 seconds.
 *
 * In contrast calculating in femtoseconds the same example of 19200000 *
 * 0.000000083333x nanoseconds per count of error is just 1.59 nanoseconds!
 *
 * Continuing the example of 19.2 MHz we cap the maximum error difference
 * at a worst-case 0.3 microseconds over a potential calculation window of
 * abount 16 seconds, meaning you can convert a frame-time that is <= 16
 * seconds older/younger than the reference time with an error of 0.3
 * useconds. Note 19.2MHz is an example frequency not a requirement.
 */
int gb_timesync_frame_time_to_timespec(u64 frame_time, struct timespec *ts)
{
	unsigned long flags;
	u64 delta_fs, counts;
	u32 sec, nsec;
	bool add;
	int ret;

	mutex_lock(&gb_timesync_mutex);
	spin_lock_irqsave(&gb_timesync_spinlock, flags);

	ret = gb_timesync_get_status();
	if (ret)
		goto done;

	/* Support calculating ktime upwards or downwards from the reference */
	if (frame_time < gb_timesync_ktime_data.frame_time) {
		add = false;
		counts = gb_timesync_ktime_data.frame_time - frame_time;
	} else {
		add = true;
		counts = frame_time - gb_timesync_ktime_data.frame_time;
	}

	/* Enforce the .3 of a usecond boundary @ 19.2MHz */
	if (counts > gb_timesync_max_ktime_diff) {
		ret = -EINVAL;
		goto done;
	}

	/* Determine the time difference in femtoseconds */
	delta_fs = counts * gb_timesync_fs_per_clock;
	sec = delta_fs / FSEC_PER_SEC;
	nsec = (delta_fs % FSEC_PER_SEC) / 1000000UL;

	if (add) {
		/* Add the calculated offset - overflow nanoseconds upwards */
		ts->tv_sec = gb_timesync_ktime_data.ts.tv_sec + sec;
		ts->tv_nsec = gb_timesync_ktime_data.ts.tv_nsec + nsec;
		if (ts->tv_nsec >= NSEC_PER_SEC) {
			ts->tv_sec++;
			ts->tv_nsec -= NSEC_PER_SEC;
		}
	} else {
		/* Subtract the difference over/underflow as necessary */
		if (nsec > gb_timesync_ktime_data.ts.tv_nsec) {
			sec++;
			nsec = nsec + gb_timesync_ktime_data.ts.tv_nsec;
			nsec %= NSEC_PER_SEC;
		} else {
			nsec = gb_timesync_ktime_data.ts.tv_nsec - nsec;
		}
		/* Cannot return a negative second value */
		if (sec > gb_timesync_ktime_data.ts.tv_sec) {
			ret = -EINVAL;
			goto done;
		}
		ts->tv_sec = gb_timesync_ktime_data.ts.tv_sec - sec;
		ts->tv_nsec = nsec;
	}
done:
	spin_unlock_irqrestore(&gb_timesync_spinlock, flags);
	mutex_unlock(&gb_timesync_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(gb_timesync_frame_time_to_timespec);

void gb_timesync_irq(void)
{
	unsigned long flags;
	u64 strobe_time;
	bool strobe_is_ping = true;
	struct timespec ts;

	ktime_get_ts(&ts);
	strobe_time = gb_timesync_get_frame_time_raw();

	spin_lock_irqsave(&gb_timesync_spinlock, flags);

	if (gb_timesync_state == GB_TIMESYNC_STATE_PING) {
		if (!gb_timesync_capture_ping)
			goto done_nolog;
		gb_timesync_ap_ping_frame_time = strobe_time;
		goto done_log;
	} else if (gb_timesync_state != GB_TIMESYNC_STATE_WAIT_SVC) {
		goto done_nolog;
	}

	gb_timesync_strobe_data[gb_timesync_strobe].frame_time = strobe_time;
	gb_timesync_strobe_data[gb_timesync_strobe].ts = ts;

	if (++gb_timesync_strobe == gb_timesync_strobe_count)
		gb_timesync_set_state(GB_TIMESYNC_STATE_AUTHORITATIVE);
	strobe_is_ping = false;
done_log:
	trace_gb_timesync_irq(strobe_is_ping, gb_timesync_strobe,
			      gb_timesync_strobe_count, strobe_time);
done_nolog:
	spin_unlock_irqrestore(&gb_timesync_spinlock, flags);
}
EXPORT_SYMBOL(gb_timesync_irq);

static int gb_timesync_ping_read(struct seq_file *seq, void *data)
{
	char *pbuf;
	ssize_t ret;

	gb_timesync_print_ping = false;
	ret = __gb_timesync_schedule_synchronous(GB_TIMESYNC_STATE_PING);
	if (ret)
		return ret;

	pbuf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!pbuf)
		return -ENOMEM;

	ret = gb_timesync_log_frame_time(pbuf, PAGE_SIZE);
	seq_printf(seq, "%s", pbuf);
	kfree(pbuf);

	return 0;
}

static int gb_timesync_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, gb_timesync_ping_read, NULL);
}

static const struct file_operations gb_timesync_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= gb_timesync_seq_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int __init gb_timesync_init(void)
{
	u64 tmp;
	int ret = 0;

	ret = gb_timesync_platform_init();
	if (ret) {
		pr_err("timesync platform init fail!\n");
		return ret;
	}

	gb_timesync_wq = create_singlethread_workqueue("gb-timesync-wq");
	if (!gb_timesync_wq)
		return -ENOMEM;
	INIT_DELAYED_WORK(&gb_timesync_delayed_work, gb_timesync_worker);

	gb_timesync_clock_rate = gb_timesync_platform_get_clock_rate();
	tmp = (10 * FSEC_PER_SEC) / gb_timesync_clock_rate;
	gb_timesync_frame_time_offset = 0;
	gb_timesync_capture_ping = false;
	gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_INACTIVE);
	gb_timesync_dentry = debugfs_create_file("frame-time", S_IRUGO,
						 gb_debugfs_get(), NULL,
						 &gb_timesync_debugfs_ops);

	gb_timesync_strobe_delay = GB_TIMESYNC_MIN_STROBE_DELAY_US;
	gb_timesync_strobe_count = GB_TIMESYNC_MAX_STROBES;
	gb_timesync_ns_per_clock = NSEC_PER_SEC / gb_timesync_clock_rate;
	gb_timesync_fs_per_clock = tmp / 10;
	tmp %= 10;
	gb_timesync_max_ktime_diff = (FSEC_PER_SEC / USEC_PER_SEC) / tmp;

	init_timer(&gb_timesync_ktime_timer);
	gb_timesync_ktime_timer.function = gb_timesync_ktime_timer_fn;
	gb_timesync_ktime_timer.expires = jiffies + GB_TIMESYNC_KTIME_UPDATE;
	add_timer(&gb_timesync_ktime_timer);

	tmp = gb_timesync_max_ktime_diff / gb_timesync_clock_rate;
	tmp -= GB_TIMESYNC_KTIME_UPDATE / MSEC_PER_SEC;

	pr_info("Time-Sync @ %lu Hz max ktime conversion +/- %llu seconds\n",
		gb_timesync_clock_rate, tmp);
	return 0;
}

void gb_timesync_exit(void)
{
	struct gb_timesync_interface *timesync_interface;

	del_timer(&gb_timesync_ktime_timer);
	debugfs_remove(gb_timesync_dentry);

	cancel_delayed_work(&gb_timesync_delayed_work);
	destroy_workqueue(gb_timesync_wq);

	mutex_lock(&gb_timesync_mutex);
	gb_timesync_teardown();
	list_for_each_entry(timesync_interface,
			    &gb_timesync_interface_list, list) {
		list_del(&timesync_interface->list);
		kfree(timesync_interface);
	}
	gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_INVALID);
	mutex_unlock(&gb_timesync_mutex);

	gb_timesync_platform_exit();
}
