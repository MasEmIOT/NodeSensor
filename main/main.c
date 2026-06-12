/*
 * Node Sensor - ESP32-S3 + SHT30 + BMP280 + BH1750 -> LoRa SX1278 (Ra-02)
 *
 * - 2 bus I2C theo schematic, tu dong do tim sensor tren ca 2 bus
 *   (nen cam sensor vao J14..J19 bus nao cung chay).
 * - Doc sensor dinh ky, dong goi sensor_packet_t (28 byte) va gui qua LoRa.
 * - LED xanh la: gui thanh cong. LED do: loi.
 *
 * Cau hinh (node id, chu ky gui, tan so/SF LoRa): idf.py menuconfig -> "Node Sensor Configuration"
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "sdkconfig.h"

#include "board_pins.h"
#include "packet.h"
#include "sx127x.h"
#include "sht3x.h"
#include "bmp280.h"
#include "bh1750.h"

static const char *TAG = "node";

static i2c_master_bus_handle_t s_bus[2];

static sht3x_t  s_sht;
static bmp280_t s_bmp;
static bh1750_t s_bh;
static bool s_has_sht, s_has_bmp, s_has_bh;

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

/* Do tim 1 sensor tren ca 2 bus. Tra ve so bus (0/1) hoac -1 neu khong thay. */
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

static void sensors_init(void)
{
    uint8_t addr = 0;
    int bus;

    bus = find_on_buses((const uint8_t[]){ 0x44, 0x45 }, 2, &addr);
    if (bus >= 0 && sht3x_init(s_bus[bus], addr, &s_sht) == ESP_OK) {
        s_has_sht = true;
        ESP_LOGI(TAG, "SHT30  : bus%d, dia chi 0x%02X", bus, addr);
    } else {
        ESP_LOGW(TAG, "SHT30  : KHONG tim thay (kiem tra J18/J19)");
    }

    bus = find_on_buses((const uint8_t[]){ 0x76, 0x77 }, 2, &addr);
    if (bus >= 0 && bmp280_init(s_bus[bus], addr, &s_bmp) == ESP_OK) {
        s_has_bmp = true;
        ESP_LOGI(TAG, "BMP280 : bus%d, dia chi 0x%02X", bus, addr);
    } else {
        ESP_LOGW(TAG, "BMP280 : KHONG tim thay (kiem tra J14/J15)");
    }

    bus = find_on_buses((const uint8_t[]){ 0x23, 0x5C }, 2, &addr);
    if (bus >= 0 && bh1750_init(s_bus[bus], addr, &s_bh) == ESP_OK) {
        s_has_bh = true;
        ESP_LOGI(TAG, "BH1750 : bus%d, dia chi 0x%02X", bus, addr);
    } else {
        ESP_LOGW(TAG, "BH1750 : KHONG tim thay (kiem tra J16/J17)");
    }
}

/* ---------------- main ---------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "=== NODE SENSOR (id=%d) ===", CONFIG_NODE_ID);
    ESP_LOGI(TAG, "Goi tin %u byte, chu ky %d s", (unsigned)sizeof(sensor_packet_t),
             CONFIG_NODE_SEND_INTERVAL_S);

    leds_init();
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

    uint16_t seq = 0;
    while (1) {
        sensor_packet_t pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.node_id = CONFIG_NODE_ID;
        pkt.seq = seq++;

        float v1, v2;
        if (s_has_sht) {
            if (sht3x_read(&s_sht, &v1, &v2) == ESP_OK) {
                pkt.sht_temp_c = v1;
                pkt.sht_hum_pct = v2;
                pkt.flags |= PKT_F_SHT30_OK;
                ESP_LOGI(TAG, "SHT30  : %.2f C, %.1f %%RH", (double)v1, (double)v2);
            } else {
                ESP_LOGW(TAG, "SHT30  : loi doc");
            }
        }
        if (s_has_bmp) {
            if (bmp280_read(&s_bmp, &v1, &v2) == ESP_OK) {
                pkt.bmp_temp_c = v1;
                pkt.bmp_press_hpa = v2;
                pkt.flags |= PKT_F_BMP280_OK;
                ESP_LOGI(TAG, "BMP280 : %.2f C, %.2f hPa", (double)v1, (double)v2);
            } else {
                ESP_LOGW(TAG, "BMP280 : loi doc");
            }
        }
        if (s_has_bh) {
            if (bh1750_read(&s_bh, &v1) == ESP_OK) {
                pkt.lux = v1;
                pkt.flags |= PKT_F_BH1750_OK;
                ESP_LOGI(TAG, "BH1750 : %.1f lux", (double)v1);
            } else {
                ESP_LOGW(TAG, "BH1750 : loi doc");
            }
        }

        sensor_pkt_seal(&pkt);
        esp_err_t err = sx127x_send((const uint8_t *)&pkt, sizeof(pkt), 5000);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "LoRa TX #%u OK (flags=0x%02X)", (unsigned)pkt.seq, (unsigned)pkt.flags);
            led_blink(PIN_LED_G, 80);
        } else {
            ESP_LOGE(TAG, "LoRa TX #%u LOI: %s", (unsigned)pkt.seq, esp_err_to_name(err));
            led_blink(PIN_LED_R, 300);
        }

        vTaskDelay(pdMS_TO_TICKS((uint32_t)CONFIG_NODE_SEND_INTERVAL_S * 1000U));
    }
}
