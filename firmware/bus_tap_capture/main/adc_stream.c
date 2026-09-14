#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"

#include "adc_stream.h"
#include "protocol.h"
#include "wifi.h"

_Static_assert(sizeof(bt_hdr_t) == BT_HDR_SIZE, "bt_hdr_t layout drifted from protocol.h");

static const char *TAG = "adc";

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define ADC_FORMAT      ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define ADC_GET_CH(p)   ((p)->type1.channel)
#define ADC_GET_DATA(p) ((p)->type1.data)
#else
#define ADC_FORMAT      ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define ADC_GET_CH(p)   ((p)->type2.channel)
#define ADC_GET_DATA(p) ((p)->type2.data)
#endif

#define ADC_ATTEN       ADC_ATTEN_DB_12
#define READ_LEN        (256 * SOC_ADC_DIGI_RESULT_BYTES)
#define POOL_LEN        (8192 * SOC_ADC_DIGI_RESULT_BYTES)
#define CLIENT_LEASE_US (5 * 1000 * 1000)
#define CAL_POINTS      17

typedef struct {
    const char *name;
    int gpio;
    adc_channel_t channel;
    bool valid;
} channel_t;

static channel_t s_ch[BT_MAX_CHANNELS] = {
    {.name = "DC", .gpio = CONFIG_BUS_TAP_GPIO_DC},
    {.name = "AC", .gpio = CONFIG_BUS_TAP_GPIO_AC},
    {.name = "FC", .gpio = CONFIG_BUS_TAP_GPIO_FC},
};
static int8_t s_phys_to_logical[16];

static adc_continuous_handle_t s_adc;
static adc_cali_handle_t s_cali;
static const char *s_cali_scheme = "none";
static int s_sock = -1;

/* Everything below is shared between the capture and control tasks. */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static struct sockaddr_in s_client;
static int64_t s_client_seen_us;
static uint32_t s_rate_hz = CONFIG_BUS_TAP_SAMPLE_RATE;
static uint8_t s_mask;
static uint32_t s_epoch;
static uint32_t s_seq;
static bool s_pending;
static uint32_t s_pending_rate;
static uint8_t s_pending_mask;
static volatile uint32_t s_overflows;
static uint32_t s_tx_drops;

static bool IRAM_ATTR on_pool_ovf(adc_continuous_handle_t h, const adc_continuous_evt_data_t *e, void *u)
{
    s_overflows++;
    return false;
}

static int popcount8(uint8_t m)
{
    int n = 0;
    for (; m; m >>= 1) n += m & 1;
    return n;
}

static uint32_t next_seq(void)
{
    taskENTER_CRITICAL(&s_lock);
    uint32_t seq = s_seq++;
    taskEXIT_CRITICAL(&s_lock);
    return seq;
}

static void channels_init(void)
{
    memset(s_phys_to_logical, -1, sizeof(s_phys_to_logical));
    for (int i = 0; i < BT_MAX_CHANNELS; i++) {
        adc_unit_t unit;
        if (adc_continuous_io_to_channel(s_ch[i].gpio, &unit, &s_ch[i].channel) == ESP_OK && unit == ADC_UNIT_1) {
            s_ch[i].valid = true;
            s_mask |= 1u << i;
            s_phys_to_logical[s_ch[i].channel & 0xF] = i;
        } else {
            ESP_LOGE(TAG, "%s: GPIO%d is not an ADC1 input, channel disabled", s_ch[i].name, s_ch[i].gpio);
        }
    }
}

static void cali_init(void)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t curve = {
        .unit_id = ADC_UNIT_1,
        .chan = s_ch[0].channel,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&curve, &s_cali) == ESP_OK) {
        s_cali_scheme = "curve_fitting";
        return;
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t line = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
#if CONFIG_IDF_TARGET_ESP32
        .default_vref = 1100,
#endif
    };
    if (adc_cali_create_scheme_line_fitting(&line, &s_cali) == ESP_OK) {
        s_cali_scheme = "line_fitting";
        return;
    }
#endif
    ESP_LOGW(TAG, "no ADC calibration scheme; host will assume a nominal transfer curve");
}

static esp_err_t adc_apply(uint32_t rate_hz, uint8_t mask)
{
    adc_digi_pattern_config_t pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    int n = 0;
    for (int i = 0; i < BT_MAX_CHANNELS; i++) {
        if (!(mask & (1u << i)) || !s_ch[i].valid) continue;
        pattern[n].atten = ADC_ATTEN;
        pattern[n].channel = s_ch[i].channel & 0x7;
        pattern[n].unit = ADC_UNIT_1;
        pattern[n].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
        n++;
    }
    if (n == 0) return ESP_ERR_INVALID_ARG;
    adc_continuous_config_t cfg = {
        .pattern_num = n,
        .adc_pattern = pattern,
        .sample_freq_hz = rate_hz,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_FORMAT,
    };
    return adc_continuous_config(s_adc, &cfg);
}

void adc_stream_touch_client(const struct sockaddr_in *addr)
{
    taskENTER_CRITICAL(&s_lock);
    s_client = *addr;
    s_client_seen_us = esp_timer_get_time();
    taskEXIT_CRITICAL(&s_lock);
}

bool adc_stream_get_client(struct sockaddr_in *out)
{
    taskENTER_CRITICAL(&s_lock);
    int64_t seen = s_client_seen_us;
    *out = s_client;
    taskEXIT_CRITICAL(&s_lock);
    return seen != 0 && esp_timer_get_time() - seen < CLIENT_LEASE_US;
}

int adc_stream_channel_index(const char *name, size_t len)
{
    for (int i = 0; i < BT_MAX_CHANNELS; i++) {
        if (strlen(s_ch[i].name) == len && strncasecmp(s_ch[i].name, name, len) == 0) return i;
    }
    return -1;
}

bool adc_stream_request_config(uint32_t rate_hz, uint8_t mask)
{
    if (rate_hz && (rate_hz < SOC_ADC_SAMPLE_FREQ_THRES_LOW || rate_hz > SOC_ADC_SAMPLE_FREQ_THRES_HIGH)) return false;
    uint8_t valid = 0;
    for (int i = 0; i < BT_MAX_CHANNELS; i++) valid |= s_ch[i].valid ? (1u << i) : 0;
    if (mask & ~valid) return false;

    taskENTER_CRITICAL(&s_lock);
    uint32_t rate = rate_hz ? rate_hz : s_rate_hz;
    uint8_t m = mask ? mask : s_mask;
    if (rate != s_rate_hz || m != s_mask) {
        s_pending_rate = rate;
        s_pending_mask = m;
        s_pending = true;
    }
    taskEXIT_CRITICAL(&s_lock);
    return true;
}

typedef struct {
    char *p;
    size_t cap, n;
    bool ok;
} sbuf_t;

static void sb_printf(sbuf_t *b, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void sb_printf(sbuf_t *b, const char *fmt, ...)
{
    if (!b->ok) return;
    va_list ap;
    va_start(ap, fmt);
    int r = vsnprintf(b->p + b->n, b->cap - b->n, fmt, ap);
    va_end(ap);
    if (r < 0 || (size_t)r >= b->cap - b->n) {
        b->ok = false;
        return;
    }
    b->n += r;
}

size_t adc_stream_build_info(uint8_t *buf, size_t len)
{
    if (len <= BT_HDR_SIZE) return 0;
    taskENTER_CRITICAL(&s_lock);
    uint32_t rate = s_rate_hz, epoch = s_epoch;
    uint8_t mask = s_mask;
    taskEXIT_CRITICAL(&s_lock);

    sbuf_t b = {.p = (char *)buf + BT_HDR_SIZE, .cap = len - BT_HDR_SIZE, .ok = true};
    sb_printf(&b, "{\"fw\":\"bus_tap_capture/1\",\"target\":\"%s\",\"epoch\":%" PRIu32 ",\"rate_hz\":%" PRIu32
              ",\"atten_db\":12,\"bits\":12,\"channels\":[", CONFIG_IDF_TARGET, epoch, rate);
    for (int i = 0; i < BT_MAX_CHANNELS; i++) {
        sb_printf(&b, "%s{\"idx\":%d,\"name\":\"%s\",\"gpio\":%d,\"valid\":%s,\"enabled\":%s}", i ? "," : "", i,
                  s_ch[i].name, s_ch[i].gpio, s_ch[i].valid ? "true" : "false",
                  (mask & (1u << i)) ? "true" : "false");
    }
    sb_printf(&b, "],\"cal\":{\"scheme\":\"%s\",\"raw\":[", s_cali_scheme);
    for (int k = 0; k < CAL_POINTS; k++) sb_printf(&b, "%s%d", k ? "," : "", k == CAL_POINTS - 1 ? 4095 : k * 256);
    sb_printf(&b, "],\"mv\":[");
    for (int k = 0; k < CAL_POINTS; k++) {
        int mv = -1;
        if (s_cali) adc_cali_raw_to_voltage(s_cali, k == CAL_POINTS - 1 ? 4095 : k * 256, &mv);
        sb_printf(&b, "%s%d", k ? "," : "", mv);
    }
    sb_printf(&b, "]},\"overflows\":%" PRIu32 ",\"tx_drops\":%" PRIu32 ",\"uptime_s\":%" PRId64 ",\"rssi\":%d}",
              s_overflows, s_tx_drops, esp_timer_get_time() / 1000000, wifi_rssi());
    if (!b.ok) return 0;

    bt_hdr_t h = {
        .magic = BT_MAGIC,
        .type = BT_TYPE_INFO,
        .n_channels = popcount8(mask),
        .seq = next_seq(),
        .epoch = epoch,
        .esp_time_us = esp_timer_get_time(),
        .rate_hz = rate,
        .overflow_count = s_overflows,
    };
    memcpy(buf, &h, BT_HDR_SIZE);
    return BT_HDR_SIZE + b.n;
}

static void capture_task(void *arg)
{
    static uint8_t rd[READ_LEN];
    static uint8_t pkt[BT_HDR_SIZE + 2 * BT_MAX_SAMPLES_PER_PKT];

    taskENTER_CRITICAL(&s_lock);
    uint32_t rate = s_rate_hz, epoch = s_epoch;
    uint8_t mask = s_mask;
    taskEXIT_CRITICAL(&s_lock);

    uint64_t index = 0, pkt_first = 0;
    int64_t pkt_time = 0;
    uint16_t n = 0;

    for (;;) {
        taskENTER_CRITICAL(&s_lock);
        bool apply = s_pending;
        uint32_t new_rate = s_pending_rate;
        uint8_t new_mask = s_pending_mask;
        if (apply) {
            /* Commit the intent now so a HELLO arriving mid-reconfigure is not re-queued. */
            s_pending = false;
            s_rate_hz = new_rate;
            s_mask = new_mask;
        }
        taskEXIT_CRITICAL(&s_lock);

        if (apply) {
            adc_continuous_stop(s_adc);
            if (adc_apply(new_rate, new_mask) != ESP_OK) {
                ESP_LOGE(TAG, "config %" PRIu32 " Hz mask 0x%x rejected, keeping previous", new_rate, new_mask);
                adc_apply(rate, mask);
                taskENTER_CRITICAL(&s_lock);
                s_rate_hz = rate;
                s_mask = mask;
                taskEXIT_CRITICAL(&s_lock);
            } else {
                rate = new_rate;
                mask = new_mask;
            }
            adc_continuous_start(s_adc);
            taskENTER_CRITICAL(&s_lock);
            epoch = ++s_epoch;
            taskEXIT_CRITICAL(&s_lock);
            index = 0;
            n = 0;
            ESP_LOGI(TAG, "epoch %" PRIu32 ": %" PRIu32 " Hz total, mask 0x%x", epoch, rate, mask);
        }

        uint32_t got = 0;
        esp_err_t err = adc_continuous_read(s_adc, rd, READ_LEN, &got, 100);
        if (err == ESP_ERR_TIMEOUT) continue;
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "read: %s", esp_err_to_name(err));
            vTaskDelay(1);
            continue;
        }
        int64_t now = esp_timer_get_time();
        struct sockaddr_in client;
        bool streaming = adc_stream_get_client(&client);

        for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= got; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&rd[i];
            int8_t logical = s_phys_to_logical[ADC_GET_CH(p) & 0xF];
            if (logical < 0) continue;
            uint16_t w = (uint16_t)((logical << BT_CH_SHIFT) | (ADC_GET_DATA(p) & BT_DATA_MASK));
            if (n == 0) {
                pkt_first = index;
                pkt_time = now;
            }
            pkt[BT_HDR_SIZE + 2 * n] = w & 0xFF;
            pkt[BT_HDR_SIZE + 2 * n + 1] = w >> 8;
            n++;
            index++;
            if (n < BT_MAX_SAMPLES_PER_PKT) continue;

            if (streaming) {
                bt_hdr_t h = {
                    .magic = BT_MAGIC,
                    .type = BT_TYPE_SAMPLES,
                    .n_channels = popcount8(mask),
                    .n_samples = n,
                    .seq = next_seq(),
                    .epoch = epoch,
                    .first_index = pkt_first,
                    .esp_time_us = pkt_time,
                    .rate_hz = rate,
                    .overflow_count = s_overflows,
                };
                memcpy(pkt, &h, BT_HDR_SIZE);
                if (sendto(s_sock, pkt, BT_HDR_SIZE + 2u * n, 0, (struct sockaddr *)&client, sizeof(client)) < 0) {
                    s_tx_drops++; /* the seq gap tells the host */
                }
            }
            n = 0;
        }
    }
}

void adc_stream_start(int sock)
{
    s_sock = sock;
    channels_init();
    cali_init();
    if (s_mask == 0) {
        ESP_LOGE(TAG, "no usable ADC1 channels; check the GPIO settings in menuconfig");
        return;
    }

    adc_continuous_handle_cfg_t hcfg = {.max_store_buf_size = POOL_LEN, .conv_frame_size = READ_LEN};
    ESP_ERROR_CHECK(adc_continuous_new_handle(&hcfg, &s_adc));
    adc_continuous_evt_cbs_t cbs = {.on_pool_ovf = on_pool_ovf};
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(s_adc, &cbs, NULL));
    ESP_ERROR_CHECK(adc_apply(s_rate_hz, s_mask));
    ESP_ERROR_CHECK(adc_continuous_start(s_adc));
    s_epoch = 1;
    ESP_LOGI(TAG, "epoch 1: %" PRIu32 " Hz total, mask 0x%x, calibration %s", s_rate_hz, s_mask, s_cali_scheme);

    /* Wi-Fi runs on core 0 on dual-core chips; keep the capture loop off it. */
    xTaskCreatePinnedToCore(capture_task, "capture", 4096, NULL, 10, NULL,
                            portNUM_PROCESSORS > 1 ? 1 : tskNO_AFFINITY);
}
