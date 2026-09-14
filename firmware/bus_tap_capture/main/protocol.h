/*
 * Wire protocol between bus_tap_capture (ESP32) and the host tools (host/bustap).
 *
 * Keep this header free of ESP-IDF includes: the host test suite compiles it with
 * the system C compiler to check the layout against host/bustap/protocol.py.
 *
 * Transport: UDP, default port 7777.
 *
 *   host -> ESP   ASCII  "UBTHELLO [rate=<hz>] [ch=DC,AC,FC]"
 *                 Sent about once a second. The ESP streams to the source address of
 *                 the most recent HELLO and stops ~5 s after the last one. Options are
 *                 applied if they differ from the running configuration, which starts
 *                 a new epoch.
 *
 *   ESP -> host   bt_hdr_t followed by a payload:
 *                 BT_TYPE_SAMPLES  n_samples little-endian uint16 sample words
 *                 BT_TYPE_INFO     UTF-8 JSON (configuration, calibration, stats)
 *
 * Sample word: bits 15..12 logical channel index (0=DC, 1=AC, 2=FC),
 *              bits 11..0  raw 12-bit ADC code.
 *
 * first_index counts sample words (all channels) since the start of the epoch, so a
 * jump between consecutive packets means samples were lost in transit. Losses inside
 * the ESP's DMA pool are not visible that way; overflow_count increments instead.
 */
#pragma once

#include <stdint.h>

#define BT_MAGIC            0x31544255u /* "UBT1" little-endian */
#define BT_TYPE_SAMPLES     1
#define BT_TYPE_INFO        2
#define BT_HELLO            "UBTHELLO"

#define BT_MAX_CHANNELS     3
#define BT_CH_SHIFT         12
#define BT_DATA_MASK        0x0FFF

/* Keep datagrams under a 1500-byte MTU: 40 + 2*686 = 1412 bytes. */
#define BT_MAX_SAMPLES_PER_PKT 686

typedef struct __attribute__((packed)) {
    uint32_t magic;          /* BT_MAGIC */
    uint8_t  type;           /* BT_TYPE_* */
    uint8_t  n_channels;     /* channels enabled in the conversion pattern */
    uint16_t n_samples;      /* sample words following the header (0 for INFO) */
    uint32_t seq;            /* per-datagram sequence number, never reset */
    uint32_t epoch;          /* incremented on every ADC (re)configuration */
    uint64_t first_index;    /* sample-word index of the first word in this packet */
    uint64_t esp_time_us;    /* esp_timer time when the first word was read from DMA */
    uint32_t rate_hz;        /* configured total conversion rate, all channels */
    uint32_t overflow_count; /* cumulative DMA pool overflows (samples lost on-chip) */
} bt_hdr_t;

#define BT_HDR_SIZE 40
