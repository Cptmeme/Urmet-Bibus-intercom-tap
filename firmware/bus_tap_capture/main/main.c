/*
 * bus_tap_capture: stream the bus_tap board's three ADC channels over Wi-Fi/UDP.
 *
 * SAFETY: bus_tap GND is Urmet bus L2. While J1 is connected to the intercom, power
 * this ESP32 from an isolated USB supply and never plug it into a computer.
 * Data leaves over Wi-Fi precisely so no wired connection is needed.
 */
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "adc_stream.h"
#include "protocol.h"
#include "wifi.h"

static const char *TAG = "bus_tap";

/* "UBTHELLO rate=60000 ch=DC,AC" -> rate 60000, mask 0b011. Absent options stay 0. */
static void parse_hello(const char *msg, uint32_t *rate, uint8_t *mask)
{
    *rate = 0;
    *mask = 0;
    const char *r = strstr(msg, "rate=");
    if (r) *rate = strtoul(r + 5, NULL, 10);

    const char *c = strstr(msg, "ch=");
    if (!c) return;
    for (c += 3; *c && *c != ' ';) {
        const char *end = c;
        while (*end && *end != ',' && *end != ' ') end++;
        int idx = adc_stream_channel_index(c, end - c);
        if (idx >= 0) *mask |= 1u << idx;
        c = (*end == ',') ? end + 1 : end;
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    wifi_start();
    wifi_wait_connected();

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_BUS_TAP_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (sock < 0 || bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "UDP socket/bind failed");
        abort();
    }
    struct timeval tv = {.tv_sec = 1};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    adc_stream_start(sock);
    ESP_LOGI(TAG, "waiting for %s on UDP %d", BT_HELLO, CONFIG_BUS_TAP_UDP_PORT);

    static uint8_t info[1400];
    char rx[128];
    int64_t last_info_us = 0;

    for (;;) {
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);
        int n = recvfrom(sock, rx, sizeof(rx) - 1, 0, (struct sockaddr *)&src, &src_len);
        bool reply = false;
        if (n > 0) {
            rx[n] = '\0';
            if (strncmp(rx, BT_HELLO, strlen(BT_HELLO)) == 0) {
                uint32_t rate;
                uint8_t mask;
                parse_hello(rx, &rate, &mask);
                if (!adc_stream_request_config(rate, mask)) ESP_LOGW(TAG, "ignoring invalid config in: %s", rx);
                adc_stream_touch_client(&src);
                reply = true; /* lets the host discover this ESP from a broadcast HELLO */
            }
        }

        struct sockaddr_in client;
        int64_t now = esp_timer_get_time();
        if (adc_stream_get_client(&client) && (reply || now - last_info_us > 1000000)) {
            size_t len = adc_stream_build_info(info, sizeof(info));
            if (len) sendto(sock, info, len, 0, (struct sockaddr *)&client, sizeof(client));
            last_info_us = now;
        }
    }
}
