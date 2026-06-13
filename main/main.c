/*
 * Node Sensor - ESP32-S3 + SHT30 + BMP180 + BH1750 -> LoRa SX1278 (Ra-02)
 *
 * Tinh nang:
 *   - SHT30 + BMP180 + BH1750, tu dong probe 2 bus I2C, IN dia chi I2C ra log.
 *   - Chu ky nhanh 3-4 giay (CONFIG_NODE_SEND_INTERVAL_S).
 *   - MA HOA goi tin (AES-128-GCM hoac ASCON-128, chon trong crypto_cfg.h).
 *   - Co che XAC NHAN (ACK) 2 chieu: gui -> doi ACK -> moi ket thuc chu ky.
 *   - Tu do: thoi gian xu ly (proc_us), thoi gian ma hoa (enc_us),
 *     round-trip (rtt_ms), uoc luong KHOANG CACH toi gateway tu RSSI.
 *     Cac so lieu nay duoc gui kem trong goi sau de gateway/tool theo doi.
 *   - HYBRID light-sleep (timer HOAC GPIO/DIO0 wake) + Watchdog.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "sdkconfig.h"

#include "board_pins.h"
#include "packet.h"
#include "crypto.h"
#include "crypto_cfg.h"
#include "sx127x.h"
#include "sht3x.h"
#include "bmp180.h"
#include "bh1750.h"

/* ---- Gia tri mac dinh phong khi chua chay menuconfig lai ---- */
#ifndef CONFIG_NODE_ID
#define CONFIG_NODE_ID                1
#endif
#ifndef CONFIG_NODE_SEND_INTERVAL_S
#define CONFIG_NODE_SEND_INTERVAL_S   4
#endif
#ifndef CONFIG_LORA_FREQ_HZ
#define CONFIG_LORA_FREQ_HZ           433000000
#endif
#ifndef CONFIG_LORA_SF
#define CONFIG_LORA_SF                9
#endif
#ifndef CONFIG_LORA_TX_POWER_DBM
#define CONFIG_LORA_TX_POWER_DBM      17
#endif
#ifndef CONFIG_NODE_ACK_TIMEOUT_MS
#define CONFIG_NODE_ACK_TIMEOUT_MS    800
#endif
#ifndef CONFIG_NODE_ACK_RETRIES
#define CONFIG_NODE_ACK_RETRIES       2
#endif
#ifndef CONFIG_NODE_WDT_TIMEOUT_S
#define CONFIG_NODE_WDT_TIMEOUT_S     10
#endif
#if defined(CONFIG_NODE_LIGHT_SLEEP)
#define NODE_LIGHT_SLEEP_EN           1
#else
#define NODE_LIGHT_SLEEP_EN           1
#endif

#define LORA_TX_TIMEOUT_MS            3000

/* Mo hinh suy hao duong truyen de uoc luong khoang cach tu RSSI:
 *   RSSI(d) = RSSI_REF_1M - 10*n*log10(d)
 *   => d = 10^((RSSI_REF_1M - RSSI)/(10*n))   (d tinh bang met)
 * RSSI_REF_1M: RSSI do duoc o 1m (nen HIEU CHUAN theo thuc te).
 * PATH_LOSS_N: he so suy hao moi truong (2.0 free space ... 3-4 trong nha). */
#define RSSI_REF_1M     (-43.0)
#define PATH_LOSS_N     (2.7)

static const char *TAG = "node";

static i2c_master_bus_handle_t s_bus[2];

static sht3x_t  s_sht;
static bmp180_t s_bmp;
static bh1750_t s_bh;
static bool s_has_sht, s_has_bmp, s_has_bh;
static int  s_sht_bus = -1, s_bmp_bus = -1, s_bh_bus = -1;
static uint8_t s_sht_addr, s_bmp_addr, s_bh_addr;

/* So lieu mang chu ky truoc (gui kem trong goi sau) */
static uint16_t s_last_rtt_ms = 0;
static uint16_t s_last_dist_dm = 0;
static int16_t  s_last_dl_rssi = 0;
static uint16_t s_last_enc_us = 0;

/* ---------------- LED ---------------- */

static void leds_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_LED_R) | (1ULL << PIN_LED_G) | (1ULL << PIN_LED_B),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(PIN_LED_R, !LED_ON_LEVEL);
    gpio_set_level(PIN_LED_G, !LED_ON_LEVEL);
    gpio_set_level(PIN_LED_B, !LED_ON_LEVEL);
}

static void led_blink(gpio_num_t pin, uint32_t ms)
{
    gpio_set_level(pin, LED_ON_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(ms));
    gpio_set_level(pin, !LED_ON_LEVEL);
}

/* ---------------- I2C + sensor ---------------- */

static void i2c_init(void)
{
    i2c_master_bus_config_t bus0 = {
        .i2c_port = 0,
        .sda_io_num = PIN_I2C0_SDA,
        .scl_io_num = PIN_I2C0_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus0, &s_bus[0]));

    i2c_master_bus_config_t bus1 = {
        .i2c_port = 1,
        .sda_io_num = PIN_I2C1_SDA,
        .scl_io_num = PIN_I2C1_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus1, &s_bus[1]));
}

static int find_on_buses(const uint8_t *addrs, int n_addr, uint8_t *out_addr)
{
    for (int bus = 0; bus < 2; bus++) {
        for (int i = 0; i < n_addr; i++) {
            if (i2c_master_probe(s_bus[bus], addrs[i], 50) == ESP_OK) {
                *out_addr = addrs[i];
                return bus;
            }
        }
    }
    return -1;
}

/* Quet va in toan bo dia chi I2C tim thay tren 2 bus (de tien check phan cung) */
static void i2c_scan_dump(void)
{
    for (int bus = 0; bus < 2; bus++) {
        char line[128];
        int n = snprintf(line, sizeof(line), "I2C scan bus%d:", bus);
        bool any = false;
        for (uint8_t a = 0x08; a < 0x78; a++) {
            if (i2c_master_probe(s_bus[bus], a, 20) == ESP_OK) {
                n += snprintf(line + n, sizeof(line) - n, " 0x%02X", a);
                any = true;
            }
        }
        if (!any) n += snprintf(line + n, sizeof(line) - n, " (trong)");
        ESP_LOGI(TAG, "%s", line);
    }
}

static void sensors_init(void)
{
    uint8_t addr = 0;
    int bus;

    i2c_scan_dump();

    bus = find_on_buses((const uint8_t[]){ 0x44, 0x45 }, 2, &addr);
    if (bus >= 0 && sht3x_init(s_bus[bus], addr, &s_sht) == ESP_OK) {
        s_has_sht = true; s_sht_bus = bus; s_sht_addr = addr;
        ESP_LOGI(TAG, "SHT30  : bus%d, dia chi 0x%02X", bus, addr);
    } else {
        ESP_LOGW(TAG, "SHT30  : KHONG tim thay (kiem tra J18/J19)");
    }

    bus = find_on_buses((const uint8_t[]){ 0x77 }, 1, &addr);
    if (bus >= 0 && bmp180_init(s_bus[bus], addr, &s_bmp) == ESP_OK) {
        s_has_bmp = true; s_bmp_bus = bus; s_bmp_addr = addr;
        ESP_LOGI(TAG, "BMP180 : bus%d, dia chi 0x%02X", bus, addr);
    } else {
        ESP_LOGW(TAG, "BMP180 : KHONG tim thay (kiem tra J15, dia chi 0x77)");
    }

    bus = find_on_buses((const uint8_t[]){ 0x23, 0x5C }, 2, &addr);
    if (bus >= 0 && bh1750_init(s_bus[bus], addr, &s_bh) == ESP_OK) {
        s_has_bh = true; s_bh_bus = bus; s_bh_addr = addr;
        ESP_LOGI(TAG, "BH1750 : bus%d, dia chi 0x%02X", bus, addr);
    } else {
        ESP_LOGW(TAG, "BH1750 : KHONG tim thay (kiem tra J16/J17)");
    }

    ESP_LOGI(TAG, "== I2C MAP ==  SHT30=%s(0x%02X)  BMP180=%s(0x%02X)  BH1750=%s(0x%02X)",
             s_has_sht ? "OK" : "--", s_sht_addr,
             s_has_bmp ? "OK" : "--", s_bmp_addr,
             s_has_bh  ? "OK" : "--", s_bh_addr);
}

/* ---------------- Uoc luong khoang cach ---------------- */

static uint16_t estimate_distance_dm(int rssi_dbm)
{
    double e = (RSSI_REF_1M - (double)rssi_dbm) / (10.0 * PATH_LOSS_N);
    double d_m = pow(10.0, e);
    if (d_m < 0) d_m = 0;
    if (d_m > 6000.0) d_m = 6000.0;
    return (uint16_t)(d_m * 10.0 + 0.5);   /* decimet */
}

/* ---------------- Gui kem ACK ---------------- */

/* Gui frame da seal, doi ACK. Tra ve true neu nhan ACK dung seq.
 * Xuat rtt_ms, dl_rssi (RSSI cua ACK), ul_rssi (RSSI gateway bao). */
static bool send_with_ack(const uint8_t *frame, size_t flen, uint16_t seq,
                          uint16_t *out_rtt_ms, int16_t *out_dl_rssi, int16_t *out_ul_rssi)
{
    uint8_t buf[SX127X_MAX_PAYLOAD];

    for (int attempt = 0; attempt <= CONFIG_NODE_ACK_RETRIES; attempt++) {
        esp_task_wdt_reset();

        int64_t t_tx = esp_timer_get_time();
        esp_err_t err = sx127x_send(frame, (uint8_t)flen, LORA_TX_TIMEOUT_MS);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "TX #%u that bai (%s), thu lai", (unsigned)seq, esp_err_to_name(err));
            continue;
        }

        if (sx127x_start_rx() != ESP_OK) {
            continue;
        }
        int64_t t0 = esp_timer_get_time();
        int64_t limit_us = (int64_t)CONFIG_NODE_ACK_TIMEOUT_MS * 1000;
        while ((esp_timer_get_time() - t0) < limit_us) {
            uint8_t len = 0;
            int16_t rssi = 0;
            float snr = 0;
            esp_err_t r = sx127x_receive(buf, sizeof(buf), &len, &rssi, &snr);
            if (r == ESP_OK && len == sizeof(ack_packet_t)) {
                ack_packet_t ack;
                memcpy(&ack, buf, sizeof(ack));
                if (ack_pkt_valid(&ack) && ack.node_id == CONFIG_NODE_ID && ack.seq == seq) {
                    uint32_t rtt = (uint32_t)((esp_timer_get_time() - t_tx) / 1000);
                    if (rtt > 65000) rtt = 65000;
                    if (out_rtt_ms)  *out_rtt_ms = (uint16_t)rtt;
                    if (out_dl_rssi) *out_dl_rssi = rssi;
                    if (out_ul_rssi) *out_ul_rssi = ack.ul_rssi;
                    ESP_LOGI(TAG, "ACK #%u OK (dl_rssi %d, ul_rssi %d, rtt %u ms)",
                             (unsigned)seq, (int)rssi, (int)ack.ul_rssi, (unsigned)rtt);
                    return true;
                }
            }
            vTaskDelay(1);
            esp_task_wdt_reset();
        }
        ESP_LOGW(TAG, "Khong co ACK cho #%u (lan %d/%d)",
                 (unsigned)seq, attempt + 1, CONFIG_NODE_ACK_RETRIES + 1);
    }
    return false;
}

/* ---------------- Hybrid light sleep ---------------- */

static void wake_pin_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_WAKE_EVENT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en   = (WAKE_EVENT_LEVEL == 0) ? GPIO_PULLUP_ENABLE   : GPIO_PULLUP_DISABLE,
        .pull_down_en = (WAKE_EVENT_LEVEL == 0) ? GPIO_PULLDOWN_DISABLE: GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
}

static void hybrid_light_sleep(uint32_t sleep_ms)
{
#if NODE_LIGHT_SLEEP_EN
    if (sleep_ms == 0) return;

    sx127x_start_rx();   /* arm radio de DIO0 keo len khi co goi den */

    esp_sleep_enable_timer_wakeup((uint64_t)sleep_ms * 1000ULL);
    gpio_wakeup_enable(PIN_WAKE_EVENT,
                       (WAKE_EVENT_LEVEL == 1) ? GPIO_INTR_HIGH_LEVEL : GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    esp_task_wdt_reset();
    esp_light_sleep_start();

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
        ESP_LOGI(TAG, "** Thuc day boi SU KIEN (GPIO/DIO0) **");
        led_blink(PIN_LED_B, 40);
    }
    esp_task_wdt_reset();
#else
    vTaskDelay(pdMS_TO_TICKS(sleep_ms));
#endif
}

/* ---------------- Task chinh ---------------- */

static void sensor_cycle_task(void *arg)
{
    (void)arg;
    esp_task_wdt_add(NULL);

    uint16_t seq = 0;
    while (1) {
        int64_t t_start = esp_timer_get_time();
        esp_task_wdt_reset();

        app_payload_t pl;
        memset(&pl, 0, sizeof(pl));
        pl.node_id = CONFIG_NODE_ID;
        pl.seq = seq;
        pl.algo = CRYPTO_ALGO;

        float v1, v2;
        if (s_has_sht && sht3x_read(&s_sht, &v1, &v2) == ESP_OK) {
            pl.sht_temp_c = v1; pl.sht_hum_pct = v2; pl.flags |= PKT_F_SHT30_OK;
            ESP_LOGI(TAG, "SHT30  : %.2f C, %.1f %%RH", (double)v1, (double)v2);
        }
        if (s_has_bmp && bmp180_read(&s_bmp, &v1, &v2) == ESP_OK) {
            pl.bmp_temp_c = v1; pl.bmp_press_hpa = v2; pl.flags |= PKT_F_BMP180_OK;
            ESP_LOGI(TAG, "BMP180 : %.2f C, %.2f hPa", (double)v1, (double)v2);
        }
        if (s_has_bh && bh1750_read(&s_bh, &v1) == ESP_OK) {
            pl.lux = v1; pl.flags |= PKT_F_BH1750_OK;
            ESP_LOGI(TAG, "BH1750 : %.1f lux", (double)v1);
        }

        /* So lieu mang chu ky truoc */
        pl.rtt_ms  = s_last_rtt_ms;
        pl.dist_dm = s_last_dist_dm;
        pl.dl_rssi = s_last_dl_rssi;
        pl.enc_us  = s_last_enc_us;

        /* proc_us = thoi gian doc sensor + dung goi (truoc khi ma hoa) */
        uint32_t proc_us = (uint32_t)(esp_timer_get_time() - t_start);
        if (proc_us > 65000) proc_us = 65000;
        pl.proc_us = (uint16_t)proc_us;

        /* Ma hoa (do thoi gian) */
        uint8_t frame[ENV_MAX_LEN];
        size_t flen = 0;
        int64_t e0 = esp_timer_get_time();
        int sr = packet_seal(&pl, CRYPTO_ALGO, CRYPTO_KEY, frame, &flen);
        uint32_t enc_us = (uint32_t)(esp_timer_get_time() - e0);
        if (enc_us > 65000) enc_us = 65000;
        s_last_enc_us = (uint16_t)enc_us;

        if (sr != 0) {
            ESP_LOGE(TAG, "Ma hoa loi (%d)", sr);
            led_blink(PIN_LED_R, 200);
            seq++;
            hybrid_light_sleep(1000);
            continue;
        }

        ESP_LOGI(TAG, "Goi #%u (%s, %u byte) proc %u us, enc %u us",
                 (unsigned)seq, crypto_algo_name(CRYPTO_ALGO), (unsigned)flen,
                 (unsigned)proc_us, (unsigned)enc_us);

        uint16_t rtt = 0; int16_t dl = 0, ul = 0;
        bool acked = send_with_ack(frame, flen, seq, &rtt, &dl, &ul);
        if (acked) {
            s_last_rtt_ms = rtt;
            s_last_dl_rssi = dl;
            s_last_dist_dm = estimate_distance_dm(dl);
            ESP_LOGI(TAG, "Chu ky #%u OK | rtt %u ms | ~%.1f m | da xac nhan",
                     (unsigned)seq, (unsigned)rtt, (double)s_last_dist_dm / 10.0);
            led_blink(PIN_LED_G, 80);
        } else {
            ESP_LOGE(TAG, "Chu ky #%u: KHONG nhan duoc ACK", (unsigned)seq);
            led_blink(PIN_LED_R, 200);
        }
        seq++;

        int64_t active_us = esp_timer_get_time() - t_start;
        uint32_t cycle_ms = (uint32_t)CONFIG_NODE_SEND_INTERVAL_S * 1000U;
        uint32_t active_ms = (uint32_t)(active_us / 1000);
        uint32_t sleep_ms = (active_ms < cycle_ms) ? (cycle_ms - active_ms) : 0;

        ESP_LOGI(TAG, "Active %u ms -> light-sleep %u ms (heap %u)",
                 (unsigned)active_ms, (unsigned)sleep_ms, (unsigned)esp_get_free_heap_size());
        hybrid_light_sleep(sleep_ms);
    }
}

/* ---------------- main ---------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "=== NODE SENSOR (id=%d) ===", CONFIG_NODE_ID);
    ESP_LOGI(TAG, "Payload %u byte, envelope toi da %u byte, ma hoa: %s",
             (unsigned)sizeof(app_payload_t), (unsigned)ENV_MAX_LEN,
             crypto_algo_name(CRYPTO_ALGO));
    ESP_LOGI(TAG, "Chu ky %d s, ACK timeout %d ms, retries %d",
             CONFIG_NODE_SEND_INTERVAL_S, CONFIG_NODE_ACK_TIMEOUT_MS, CONFIG_NODE_ACK_RETRIES);

    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms = CONFIG_NODE_WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    if (esp_task_wdt_init(&wdt_cfg) == ESP_ERR_INVALID_STATE) {
        esp_task_wdt_reconfigure(&wdt_cfg);
    }

    leds_init();
    wake_pin_init();
    i2c_init();
    sensors_init();

    sx127x_config_t lora = {
        .spi_host = SPI2_HOST,
        .pin_sck  = PIN_LORA_SCK,
        .pin_miso = PIN_LORA_MISO,
        .pin_mosi = PIN_LORA_MOSI,
        .pin_nss  = PIN_LORA_NSS,
        .pin_rst  = PIN_LORA_RST,
        .freq_hz  = CONFIG_LORA_FREQ_HZ,
        .sf       = CONFIG_LORA_SF,
        .bw_hz    = LORA_BW_HZ,
        .cr_denom = LORA_CR_DENOM,
        .tx_power_dbm = CONFIG_LORA_TX_POWER_DBM,
        .sync_word    = LORA_SYNC_WORD,
        .preamble_len = 8,
    };
    while (sx127x_init(&lora) != ESP_OK) {
        ESP_LOGE(TAG, "Khoi tao LoRa that bai, thu lai sau 5s...");
        led_blink(PIN_LED_R, 200);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    xTaskCreatePinnedToCore(sensor_cycle_task, "sensor_cycle", 8192, NULL, 5, NULL, 0);
}
