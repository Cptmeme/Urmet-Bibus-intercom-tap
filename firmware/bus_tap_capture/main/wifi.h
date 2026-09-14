#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Connect as a Wi-Fi station using the Kconfig credentials; reconnects forever. */
void wifi_start(void);

/* Block until an IP address is assigned. */
void wifi_wait_connected(void);

/* RSSI of the current AP in dBm, or 0 when not associated. */
int8_t wifi_rssi(void);
