/*
 * TimeSync API driver.
 *
 * Copyright 2015 Google Inc.
 * Copyright 2015 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 */
#include "greybus.h"
#include "timesync_clock.h"

#define GB_TIMESYNC_DEBUG
#define GB_TIMESYNC_DELAYED_WORK_JIFFIES	100
#define GB_TIMESYNC_DEFAULT_OFFSET_US		1000

/*
 * Minimum value of 1 milliseconds is set for Linux assuming a scheduler tick of
 * 1 millisecond sending in singals at a frequency < 1 millisecond is a bit
 * anti-social. This floor value is complete arbitrary.
 *
 * Maximum value is constrained by the number of bits the SVC can fit into a 16
 * bit up-counter, also to stop funny clock divisions on the SVC side we will
 * support 1 millisecond or 10 milliseconds since we need to account for SVC
 * side pulse timers based on 48MHz/84MHz/168MHz or 21Mhz/52MHz/104MHz and
 * potentially different values in future SVC versions. For that reason even
 * clock divisions to calcuate the necessary clocks for SVC timers are best.
 */
#define GB_TIMESYNC_MIN_STROBE_DELAY_US		1000
#define GB_TIMESYNC_MAX_STROBE_DELAY_US		10000

/* Workqueue */
static void gb_timesync_worker(struct work_struct *work);
static DECLARE_DELAYED_WORK(gb_timesync_work, gb_timesync_worker);

/* Spinlock for synchronizing external API, worker and ISR */
static DEFINE_SPINLOCK(gb_timesync_spinlock);
static DEFINE_MUTEX(gb_timesync_mutex);
static LIST_HEAD(gb_timesync_interface_list);

/* Represents the frequency returned by get_cycles() in Hz */
static unsigned long __read_mostly gb_timesync_clock_rate;

/* The current local frame-time */
static u64 __read_mostly gb_timesync_frame_time_offset;
static u64 __read_mostly gb_timesync_strobe_time[GB_TIMESYNC_MAX_STROBES];
static u64 gb_timesync_init_frame_time;
static bool gb_timesync_offset_down;

struct gb_timesync_interface {
	struct list_head list;
	struct gb_interface *interface;
	__u32 prop_offset;
};

/* TimeSync finite state machine */
enum gb_timesync_state {
	GB_TIMESYNC_STATE_INVALID		= 0,
	GB_TIMESYNC_STATE_INACTIVE		= 1,
	GB_TIMESYNC_STATE_INIT			= 2,
	GB_TIMESYNC_STATE_WAIT_SVC		= 3,
	GB_TIMESYNC_STATE_SYNC_AUTHORITATIVE	= 4,
	GB_TIMESYNC_STATE_ACTIVE		= 5,
};

static int gb_timesync_state;
static struct gb_svc *gb_timesync_svc;
static int gb_timesync_strobe;
static u32 gb_timesync_strobe_delay;
static int gb_timesync_strobe_count;

static void gb_timesync_set_state(int state)
{
	gb_timesync_state = state;
	gb_timesync_strobe = 0;
}

static void gb_timesync_set_state_atomic(int state)
{
	unsigned long flags;

	spin_lock_irqsave(&gb_timesync_spinlock, flags);
	gb_timesync_set_state(state);
	spin_unlock_irqrestore(&gb_timesync_spinlock, flags);
}

static void gb_timesync_set_authoritative_request(
	struct gb_svc_timesync_authoritative_response *response,
	struct gb_control_timesync_authoritative_request *request)
{
	int i;

	for (i = 0; i < GB_TIMESYNC_MAX_STROBES; i++)
		request->frame_time[i] = response->frame_time[i];
}

static void gb_timesync_collate_frame_time(
	struct gb_svc_timesync_authoritative_response *response)
{
	int i = 0;
	u64 delta, least = 0;

	for (i = 1; i < GB_TIMESYNC_MAX_STROBES; i++) {

		delta = gb_timesync_strobe_time[i] - gb_timesync_strobe_time[i-1];
		if (!least || delta < least) {
			least = delta;
			if (response->frame_time[i] > gb_timesync_strobe_time[i]) {
				dev_warn(&gb_timesync_svc->dev, "Revising frame offset upwards!\n");
				delta = response->frame_time[i] - gb_timesync_strobe_time[i];
				pr_info("%s adjusting frame time upwards. my frame %llu svc %llu delta %llu\n",
					__func__, gb_timesync_strobe_time[i], response->frame_time[i], delta);

				gb_timesync_frame_time_offset = delta;
				gb_timesync_offset_down = true;
			} else {
				delta = gb_timesync_strobe_time[i] - response->frame_time[i];
				pr_info("%s adjusting frame time downwards. my frame %llu svc %llu delta %llu\n",
					__func__, gb_timesync_strobe_time[i], response->frame_time[i], delta);
				gb_timesync_frame_time_offset = delta;
				gb_timesync_offset_down = true;
			}
		}
	}
}

static void gb_timesync_worker(struct work_struct *work)
{
	struct gb_timesync_interface *timesync_interface;
	struct gb_control_timesync_authoritative_request control_auth_request;
	struct gb_control_timesync_authoritative_response control_auth_response;
	struct gb_svc_timesync_authoritative_response svc_auth_response;
	struct gb_interface *interface;
	u32 strobe_mask;
	int ret;

	mutex_lock(&gb_timesync_mutex);

	switch (gb_timesync_state) {
	case GB_TIMESYNC_STATE_INIT:
		strobe_mask = 0;
		gb_timesync_frame_time_offset = 0;
		gb_timesync_init_frame_time = 0;

		list_for_each_entry(timesync_interface,
				    &gb_timesync_interface_list, list) {
			interface = timesync_interface->interface;
			ret = gb_control_timesync_enable(interface->control,
							 gb_timesync_strobe_count,
							 gb_timesync_init_frame_time,
							 gb_timesync_strobe_delay,
							 gb_timesync_clock_rate);
			/* An interface may have been unplugged - keep going */
			if (ret) {
				dev_err(&interface->dev,
					"gb_control_timesync_enable %d\n", ret);
			} else {
				strobe_mask |= 1 << interface->interface_id;
			}
		}
		gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_WAIT_SVC);

		/* Initiate SVC driven TIME_SYNC strobe set */
		ret = gb_svc_timesync_enable(gb_timesync_svc,
					     gb_timesync_strobe_count,
					     gb_timesync_init_frame_time,
					     gb_timesync_strobe_delay,
					     strobe_mask,
					     gb_timesync_clock_rate);
		if (ret) {
			dev_err(&gb_timesync_svc->dev,
				"gb_svc_timesync_enable %d\n", ret);
			gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_INACTIVE);
			break;
		}

		break;
	case GB_TIMESYNC_STATE_SYNC_AUTHORITATIVE:
		ret = gb_svc_timesync_authoritative(gb_timesync_svc,
						    &svc_auth_response);
		if (ret) {
			dev_err(&gb_timesync_svc->dev,
				"gb_svc_timesync_authoritative %d\n", ret);
			gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_INACTIVE);
			break;
		}
		gb_timesync_collate_frame_time(&svc_auth_response);
		gb_timesync_set_authoritative_request(&svc_auth_response,
						      &control_auth_request);
		list_for_each_entry(timesync_interface,
				    &gb_timesync_interface_list, list) {
			interface = timesync_interface->interface;
			ret = gb_control_timesync_authoritative(
							interface->control,
							&control_auth_request,
							&control_auth_response);
			/* An interface may have been unplugged - keep going */
			if (ret) {
				dev_err(&interface->dev,
					"gb_control_timesync_enable %d\n", ret);
			} else {
				timesync_interface->prop_offset =
					control_auth_response.prop_offset;
				pr_info("%s propogation offset for interface %d is %u micorseconds\n",
					__func__, interface->interface_id, timesync_interface->prop_offset);
			}
		}
		gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_ACTIVE);
		break;
	default:
		pr_err("Invalid state %d for delayed work\n",
		       gb_timesync_state);
		break;
	}

	mutex_unlock(&gb_timesync_mutex);
}

static void gb_timesync_schedule_work(void)
{
	schedule_delayed_work(&gb_timesync_work,
			      GB_TIMESYNC_DELAYED_WORK_JIFFIES);
}

/*
 * Add a Greybus Interface to the set of TimeSync Interfaces. Only Greybus
 * Interfaces with an associated TimeSync bit in a Manifest should be added to
 * the TimeSync set in this way.
 */
int gb_timesync_interface_add(struct gb_interface *interface)
{
	struct gb_timesync_interface *timesync_interface;

	timesync_interface = kzalloc(sizeof(*timesync_interface), GFP_KERNEL);
	if (!timesync_interface)
		return -ENOMEM;

	cancel_delayed_work_sync(&gb_timesync_work);

	mutex_lock(&gb_timesync_mutex);
	gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_INIT);
	timesync_interface->interface = interface;
	list_add(&timesync_interface->list, &gb_timesync_interface_list);
	mutex_unlock(&gb_timesync_mutex);

	gb_timesync_schedule_work();

	return 0;
}
EXPORT_SYMBOL_GPL(gb_timesync_interface_add);

/*
 * Remove a Greybus Interface from the set of TimeSync Interfaces.
 */
void gb_timesync_interface_remove(struct gb_interface *interface)
{
	struct gb_timesync_interface *timesync_interface;

	mutex_lock(&gb_timesync_mutex);
	list_for_each_entry(timesync_interface,
			    &gb_timesync_interface_list, list) {
		if (timesync_interface->interface == interface) {
			list_del(&timesync_interface->list);
			kfree(timesync_interface);
			break;
		}
	}
	mutex_unlock(&gb_timesync_mutex);
}
EXPORT_SYMBOL_GPL(gb_timesync_interface_remove);

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
 * This function provides the authoritative frame-time to a calling function. It
 * is designed to be lockless and should remain that way.
 */
u64 gb_timesync_get_frame_time(void)
{
	u64 clocks = gb_timesync_clock_get_counter();

	clocks = gb_timesync_clock2ns(clocks);

	if (likely(gb_timesync_offset_down))
		return clocks - gb_timesync_frame_time_offset;
	else
		return clocks + gb_timesync_frame_time_offset;
}
EXPORT_SYMBOL_GPL(gb_timesync_get_frame_time);

#ifdef GB_TIMESYNC_DEBUG
static void gb_timesync_strobe_debug(void)
{
	if (gb_timesync_state == GB_TIMESYNC_STATE_ACTIVE)
		pr_info("%s frame-time %llu\n", __func__, gb_timesync_get_frame_time());
}
#else
static void gb_timesync_strobe_debug(void);
#endif

void gb_timesync_irq(void)
{
	unsigned long flags;
	u64 strobe_time;

	gb_timesync_strobe_debug();

	if (gb_timesync_state != GB_TIMESYNC_STATE_WAIT_SVC)
		return;

	spin_lock_irqsave(&gb_timesync_spinlock, flags);

	if (!gb_timesync_strobe)
		strobe_time = gb_timesync_init_frame_time;
	else
		strobe_time = gb_timesync_get_frame_time();
	gb_timesync_strobe_time[gb_timesync_strobe] = strobe_time;
	gb_timesync_strobe++;
	if (gb_timesync_strobe == gb_timesync_strobe_count) {
		gb_timesync_set_state(
			GB_TIMESYNC_STATE_SYNC_AUTHORITATIVE);
		gb_timesync_schedule_work();
	}

	spin_unlock_irqrestore(&gb_timesync_spinlock, flags);
}
EXPORT_SYMBOL(gb_timesync_irq);

int __init gb_timesync_init(void)
{
	int ret = 0;

	ret = gb_timesync_clock_init();
	if (ret) {
		pr_err("timesync platform init fail!\n");
		return ret;
	}

	gb_timesync_clock_rate = gb_timesync_clock_get_rate();
	gb_timesync_frame_time_offset = 0;
	gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_INVALID);

	/* TODO: get these two values out of sysfs data-points */
	gb_timesync_strobe_delay = GB_TIMESYNC_MIN_STROBE_DELAY_US;
	gb_timesync_strobe_count = GB_TIMESYNC_MAX_STROBES;

	pr_info("Time-Sync timer frequency %lu Hz\n", gb_timesync_clock_rate);
	return 0;
}

void gb_timesync_exit(void)
{
	struct gb_timesync_interface *timesync_interface;

	mutex_lock(&gb_timesync_mutex);
	list_for_each_entry(timesync_interface,
			    &gb_timesync_interface_list, list) {
		list_del(&timesync_interface->list);
		kfree(timesync_interface);
	}
	gb_timesync_set_state_atomic(GB_TIMESYNC_STATE_INVALID);
	mutex_unlock(&gb_timesync_mutex);

	cancel_delayed_work_sync(&gb_timesync_work);
	gb_timesync_clock_exit();
}
