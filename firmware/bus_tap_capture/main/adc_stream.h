#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lwip/sockets.h"

/* Resolve channels, set up calibration and start the capture task, which streams
 * sample packets on `sock` to the most recent client. */
void adc_stream_start(int sock);

/* Record the sender of a HELLO as the stream destination and refresh its lease. */
void adc_stream_touch_client(const struct sockaddr_in *addr);

/* Copy the current client into *out; false if none or its lease has expired. */
bool adc_stream_get_client(struct sockaddr_in *out);

/* Queue a new rate/channel mask; applied by the capture task between reads.
 * Returns false if the request is invalid. rate_hz 0 or mask 0 keep current values. */
bool adc_stream_request_config(uint32_t rate_hz, uint8_t mask);

/* Look up a channel name ("DC", "AC", "FC", case-insensitive); -1 if unknown. */
int adc_stream_channel_index(const char *name, size_t len);

/* Build a BT_TYPE_INFO datagram (header + JSON) into buf; returns its length. */
size_t adc_stream_build_info(uint8_t *buf, size_t len);
