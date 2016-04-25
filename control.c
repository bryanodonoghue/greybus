/*
 * Greybus CPort control protocol.
 *
 * Copyright 2015 Google Inc.
 * Copyright 2015 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include "greybus.h"

/* Highest control-protocol version supported */
#define GB_CONTROL_VERSION_MAJOR	0
#define GB_CONTROL_VERSION_MINOR	1


static int gb_control_get_version(struct gb_control *control)
{
	struct gb_interface *intf = control->connection->intf;
	struct gb_control_version_request request;
	struct gb_control_version_response response;
	int ret;

	request.major = GB_CONTROL_VERSION_MAJOR;
	request.minor = GB_CONTROL_VERSION_MINOR;

	ret = gb_operation_sync(control->connection,
				GB_CONTROL_TYPE_VERSION,
				&request, sizeof(request), &response,
				sizeof(response));
	if (ret) {
		dev_err(&intf->dev,
				"failed to get control-protocol version: %d\n",
				ret);
		return ret;
	}

	if (response.major > request.major) {
		dev_err(&intf->dev,
				"unsupported major control-protocol version (%u > %u)\n",
				response.major, request.major);
		return -ENOTSUPP;
	}

	control->protocol_major = response.major;
	control->protocol_minor = response.minor;

	dev_dbg(&intf->dev, "%s - %u.%u\n", __func__, response.major,
			response.minor);

	return 0;
}

static int gb_control_get_bundle_version(struct gb_control *control,
						struct gb_bundle *bundle)
{
	struct gb_interface *intf = control->connection->intf;
	struct gb_control_bundle_version_request request;
	struct gb_control_bundle_version_response response;
	int ret;

	request.bundle_id = bundle->id;

	ret = gb_operation_sync(control->connection,
				GB_CONTROL_TYPE_BUNDLE_VERSION,
				&request, sizeof(request),
				&response, sizeof(response));
	if (ret) {
		dev_err(&intf->dev,
				"failed to get bundle %u class version: %d\n",
				bundle->id, ret);
		return ret;
	}

	bundle->class_major = response.major;
	bundle->class_minor = response.minor;

	dev_dbg(&intf->dev, "%s - %u: %u.%u\n", __func__, bundle->id,
			response.major, response.minor);

	return 0;
}

int gb_control_get_bundle_versions(struct gb_control *control)
{
	struct gb_interface *intf = control->connection->intf;
	struct gb_bundle *bundle;
	int ret;

	if (!control->has_bundle_version)
		return 0;

	list_for_each_entry(bundle, &intf->bundles, links) {
		ret = gb_control_get_bundle_version(control, bundle);
		if (ret)
			return ret;
	}

	return 0;
}

/* Get Manifest's size from the interface */
int gb_control_get_manifest_size_operation(struct gb_interface *intf)
{
	struct gb_control_get_manifest_size_response response;
	struct gb_connection *connection = intf->control->connection;
	int ret;

	ret = gb_operation_sync(connection, GB_CONTROL_TYPE_GET_MANIFEST_SIZE,
				NULL, 0, &response, sizeof(response));
	if (ret) {
		dev_err(&connection->intf->dev,
				"failed to get manifest size: %d\n", ret);
		return ret;
	}

	return le16_to_cpu(response.size);
}

/* Reads Manifest from the interface */
int gb_control_get_manifest_operation(struct gb_interface *intf, void *manifest,
				      size_t size)
{
	struct gb_connection *connection = intf->control->connection;

	return gb_operation_sync(connection, GB_CONTROL_TYPE_GET_MANIFEST,
				NULL, 0, manifest, size);
}

int gb_control_connected_operation(struct gb_control *control, u16 cport_id)
{
	struct gb_control_connected_request request;

	request.cport_id = cpu_to_le16(cport_id);
	return gb_operation_sync(control->connection, GB_CONTROL_TYPE_CONNECTED,
				 &request, sizeof(request), NULL, 0);
}

int gb_control_disconnected_operation(struct gb_control *control, u16 cport_id)
{
	struct gb_control_disconnected_request request;

	request.cport_id = cpu_to_le16(cport_id);
	return gb_operation_sync(control->connection,
				 GB_CONTROL_TYPE_DISCONNECTED, &request,
				 sizeof(request), NULL, 0);
}

int gb_control_get_interface_version_operation(struct gb_interface *intf)
{
	struct gb_control_interface_version_response response;
	struct gb_connection *connection = intf->control->connection;
	int ret;

	if (intf->quirks & GB_INTERFACE_QUIRK_NO_INTERFACE_VERSION)
		return 0;

	ret = gb_operation_sync(connection, GB_CONTROL_TYPE_INTERFACE_VERSION,
				NULL, 0, &response, sizeof(response));
	if (ret) {
		dev_err(&connection->intf->dev,
			"failed to get interface version: %d\n", ret);
		/*
		 * FIXME: Return success until the time we bump version of
		 * control protocol. The interface-version is already set to
		 * 0.0, so no need to update that.
		 */
		return 0;
	}

	intf->version_major = le16_to_cpu(response.major);
	intf->version_minor = le16_to_cpu(response.minor);

	return 0;
}

int gb_control_timesync_enable(struct gb_control *control, u8 count,
			       u64 frame_time, u32 strobe_delay, u32 refclk)
{
	struct gb_control_timesync_enable_request request;

	request.count = count;
	request.frame_time = cpu_to_le64(frame_time);
	request.strobe_delay = cpu_to_le32(strobe_delay);
	request.refclk = cpu_to_le32(refclk);
	return gb_operation_sync(control->connection,
				 GB_CONTROL_TYPE_TIMESYNC_ENABLE, &request,
				 sizeof(request), NULL, 0);
}

int gb_control_timesync_disable(struct gb_control *control)
{
	return gb_operation_sync(control->connection,
				 GB_CONTROL_TYPE_TIMESYNC_DISABLE, NULL, 0,
				 NULL, 0);
}

int gb_control_timesync_authoritative(struct gb_control *control,
				      u64 *frame_time, u8 count)
{
	struct gb_control_timesync_authoritative_request request;
	int i;

	for (i = 0; i < GB_TIMESYNC_MAX_STROBES; i++)
		request.frame_time[i] = cpu_to_le64(frame_time[i]);

	return gb_operation_sync(control->connection,
				 GB_CONTROL_TYPE_TIMESYNC_AUTHORITATIVE,
				 &request, sizeof(request),
				 NULL, 0);
}

static ssize_t vendor_string_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gb_control *control = to_gb_control(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", control->vendor_string);
}
static DEVICE_ATTR_RO(vendor_string);

static ssize_t product_string_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gb_control *control = to_gb_control(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", control->product_string);
}
static DEVICE_ATTR_RO(product_string);

static struct attribute *control_attrs[] = {
	&dev_attr_vendor_string.attr,
	&dev_attr_product_string.attr,
	NULL,
};
ATTRIBUTE_GROUPS(control);

static void gb_control_release(struct device *dev)
{
	struct gb_control *control = to_gb_control(dev);

	gb_connection_destroy(control->connection);

	kfree(control->vendor_string);
	kfree(control->product_string);

	kfree(control);
}

struct device_type greybus_control_type = {
	.name =		"greybus_control",
	.release =	gb_control_release,
};

struct gb_control *gb_control_create(struct gb_interface *intf)
{
	struct gb_connection *connection;
	struct gb_control *control;

	control = kzalloc(sizeof(*control), GFP_KERNEL);
	if (!control)
		return ERR_PTR(-ENOMEM);

	control->intf = intf;

	connection = gb_connection_create_control(intf);
	if (IS_ERR(connection)) {
		dev_err(&intf->dev,
				"failed to create control connection: %ld\n",
				PTR_ERR(connection));
		kfree(control);
		return ERR_CAST(connection);
	}

	control->connection = connection;

	control->dev.parent = &intf->dev;
	control->dev.bus = &greybus_bus_type;
	control->dev.type = &greybus_control_type;
	control->dev.groups = control_groups;
	control->dev.dma_mask = intf->dev.dma_mask;
	device_initialize(&control->dev);
	dev_set_name(&control->dev, "%s.ctrl", dev_name(&intf->dev));

	gb_connection_set_data(control->connection, control);

	return control;
}

int gb_control_enable(struct gb_control *control)
{
	int ret;

	dev_dbg(&control->connection->intf->dev, "%s\n", __func__);

	ret = gb_connection_enable_tx(control->connection);
	if (ret) {
		dev_err(&control->connection->intf->dev,
				"failed to enable control connection: %d\n",
				ret);
		return ret;
	}

	ret = gb_control_get_version(control);
	if (ret)
		goto err_disable_connection;

	if (control->protocol_major > 0 || control->protocol_minor > 1)
		control->has_bundle_version = true;

	return 0;

err_disable_connection:
	gb_connection_disable(control->connection);

	return ret;
}

void gb_control_disable(struct gb_control *control)
{
	dev_dbg(&control->connection->intf->dev, "%s\n", __func__);

	gb_connection_disable(control->connection);
}

int gb_control_add(struct gb_control *control)
{
	int ret;

	ret = device_add(&control->dev);
	if (ret) {
		dev_err(&control->dev,
				"failed to register control device: %d\n",
				ret);
		return ret;
	}

	return 0;
}

void gb_control_del(struct gb_control *control)
{
	if (device_is_registered(&control->dev))
		device_del(&control->dev);
}

void gb_control_put(struct gb_control *control)
{
	put_device(&control->dev);
}
