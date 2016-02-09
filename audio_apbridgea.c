/*
 * Greybus Audio Device Class Protocol helpers
 *
 * Copyright 2015-2016 Google Inc.
 *
 * Released under the GPLv2 only.
 */

#include "greybus.h"
#include "greybus_protocols.h"
#include "audio_apbridgea.h"
#include "audio_codec.h"

int gb_audio_apbridgea_set_config(struct gb_connection *connection,
				  __u16 i2s_port, __u32 format, __u32 rate,
				  __u32 mclk_freq)
{
	struct audio_apbridgea_set_config_request req;

	req.hdr.type = AUDIO_APBRIDGEA_TYPE_SET_CONFIG;
	req.hdr.i2s_port = cpu_to_le16(i2s_port);
	req.format = cpu_to_le32(format);
	req.rate = cpu_to_le32(rate);
	req.mclk_freq = cpu_to_le32(mclk_freq);

	return gb_hd_output(connection->hd, &req, sizeof(req),
			    GB_APB_REQUEST_AUDIO_CONTROL, true);
}
EXPORT_SYMBOL_GPL(gb_audio_apbridgea_set_config);

int gb_audio_apbridgea_register_cport(struct gb_connection *connection,
				      __u16 i2s_port, __u16 cportid)
{
	struct audio_apbridgea_register_cport_request req;

	req.hdr.type = AUDIO_APBRIDGEA_TYPE_REGISTER_CPORT;
	req.hdr.i2s_port = cpu_to_le16(i2s_port);
	req.cport = cpu_to_le16(cportid);

	return gb_hd_output(connection->hd, &req, sizeof(req),
			    GB_APB_REQUEST_AUDIO_CONTROL, true);
}
EXPORT_SYMBOL_GPL(gb_audio_apbridgea_register_cport);

int gb_audio_apbridgea_unregister_cport(struct gb_connection *connection,
					__u16 i2s_port, __u16 cportid)
{
	struct audio_apbridgea_unregister_cport_request req;

	req.hdr.type = AUDIO_APBRIDGEA_TYPE_UNREGISTER_CPORT;
	req.hdr.i2s_port = cpu_to_le16(i2s_port);
	req.cport = cpu_to_le16(cportid);

	return gb_hd_output(connection->hd, &req, sizeof(req),
			    GB_APB_REQUEST_AUDIO_CONTROL, true);
}
EXPORT_SYMBOL_GPL(gb_audio_apbridgea_unregister_cport);

int gb_audio_apbridgea_set_tx_data_size(struct gb_connection *connection,
					__u16 i2s_port, __u16 size)
{
	struct audio_apbridgea_set_tx_data_size_request req;

	req.hdr.type = AUDIO_APBRIDGEA_TYPE_SET_TX_DATA_SIZE;
	req.hdr.i2s_port = cpu_to_le16(i2s_port);
	req.size = cpu_to_le16(size);

	return gb_hd_output(connection->hd, &req, sizeof(req),
			    GB_APB_REQUEST_AUDIO_CONTROL, true);
}
EXPORT_SYMBOL_GPL(gb_audio_apbridgea_set_tx_data_size);

int gb_audio_apbridgea_get_tx_delay(struct gb_connection *connection,
				    __u16 i2s_port, __u32 *delay)
{
	/* TODO: implement */
	return -ENOSYS;
}
EXPORT_SYMBOL_GPL(gb_audio_apbridgea_get_tx_delay);

int gb_audio_apbridgea_start_tx(struct gb_connection *connection,
				__u16 i2s_port, __u64 timestamp)
{
	struct audio_apbridgea_start_tx_request req;

	req.hdr.type = AUDIO_APBRIDGEA_TYPE_START_TX;
	req.hdr.i2s_port = cpu_to_le16(i2s_port);
	req.timestamp = cpu_to_le64(timestamp);

	return gb_hd_output(connection->hd, &req, sizeof(req),
			    GB_APB_REQUEST_AUDIO_CONTROL, true);
}
EXPORT_SYMBOL_GPL(gb_audio_apbridgea_start_tx);

int gb_audio_apbridgea_stop_tx(struct gb_connection *connection, __u16 i2s_port)
{
	struct audio_apbridgea_stop_tx_request req;

	req.hdr.type = AUDIO_APBRIDGEA_TYPE_STOP_TX;
	req.hdr.i2s_port = cpu_to_le16(i2s_port);

	return gb_hd_output(connection->hd, &req, sizeof(req),
			    GB_APB_REQUEST_AUDIO_CONTROL, true);
}
EXPORT_SYMBOL_GPL(gb_audio_apbridgea_stop_tx);

int gb_audio_apbridgea_set_rx_data_size(struct gb_connection *connection,
					__u16 i2s_port, __u16 size)
{
	struct audio_apbridgea_set_rx_data_size_request req;

	req.hdr.type = AUDIO_APBRIDGEA_TYPE_SET_RX_DATA_SIZE;
	req.hdr.i2s_port = cpu_to_le16(i2s_port);
	req.size = cpu_to_le16(size);

	return gb_hd_output(connection->hd, &req, sizeof(req),
			    GB_APB_REQUEST_AUDIO_CONTROL, true);
}
EXPORT_SYMBOL_GPL(gb_audio_apbridgea_set_rx_data_size);

int gb_audio_apbridgea_get_rx_delay(struct gb_connection *connection,
				    __u16 i2s_port, __u32 *delay)
{
	/* TODO: implement */
	return -ENOSYS;
}
EXPORT_SYMBOL_GPL(gb_audio_apbridgea_get_rx_delay);

int gb_audio_apbridgea_start_rx(struct gb_connection *connection,
				__u16 i2s_port)
{
	struct audio_apbridgea_start_rx_request req;

	req.hdr.type = AUDIO_APBRIDGEA_TYPE_START_RX;
	req.hdr.i2s_port = cpu_to_le16(i2s_port);

	return gb_hd_output(connection->hd, &req, sizeof(req),
			    GB_APB_REQUEST_AUDIO_CONTROL, true);
}
EXPORT_SYMBOL_GPL(gb_audio_apbridgea_start_rx);

int gb_audio_apbridgea_stop_rx(struct gb_connection *connection, __u16 i2s_port)
{
	struct audio_apbridgea_stop_rx_request req;

	req.hdr.type = AUDIO_APBRIDGEA_TYPE_STOP_RX;
	req.hdr.i2s_port = cpu_to_le16(i2s_port);

	return gb_hd_output(connection->hd, &req, sizeof(req),
			    GB_APB_REQUEST_AUDIO_CONTROL, true);
}
EXPORT_SYMBOL_GPL(gb_audio_apbridgea_stop_rx);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("greybus:audio-apbridgea");
MODULE_DESCRIPTION("Greybus Special APBridgeA Audio Protocol library");
MODULE_AUTHOR("Mark Greer <mgreer@animalcreek.com>");
