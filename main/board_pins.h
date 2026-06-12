/*
 * board_pins.h - Pin map theo schematic "Node Sensor" Rev 1.0 (ESP32-S3-WROOM-1)
 *
 * I2C bus 0 : net SDA  = GPIO8,  net SCL  = GPIO14  -> J14, J17 (BH1750), J12
 * I2C bus 1 : net I2C_SDA1 = GPIO6, net I2C_SCL1 = GPIO7 -> J15 (BMP280), J16, J13 (RTC)
 * (J19/SHT30 nam tren 1 trong 2 bus - code tu dong probe ca 2 bus nen khong can quan tam)
 *
 * LoRa SX1278 (Ra-02): SCK=GPIO10, MISO=GPIO11, MOSI=GPIO12, NSS=GPIO13,
 *                      RST=GPIO9, DIO0=GPIO15 (khong dung, driver poll IRQ flags)
 *
 * LED: B=GPIO39, G=GPIO40, R=GPIO41
 */
#pragma once

#include "driver/gpio.h"

/* ---------- I2C ---------- */
#define PIN_I2C0_SDA        GPIO_NUM_8
#define PIN_I2C0_SCL        GPIO_NUM_14
#define PIN_I2C1_SDA        GPIO_NUM_6
#define PIN_I2C1_SCL        GPIO_NUM_7
#define I2C_SPEED_HZ        100000

/* ---------- LoRa SX1278 ---------- */
#define PIN_LORA_SCK        GPIO_NUM_10
#define PIN_LORA_MISO       GPIO_NUM_11
#define PIN_LORA_MOSI       GPIO_NUM_12
#define PIN_LORA_NSS        GPIO_NUM_13
#define PIN_LORA_RST        GPIO_NUM_9
#define PIN_LORA_DIO0       GPIO_NUM_15   /* khong dung trong code (polling) */

/* Sync word rieng cho mang nay - PHAI GIONG NHAU o Node va Gateway */
#define LORA_SYNC_WORD      0x12
#define LORA_BW_HZ          125000
#define LORA_CR_DENOM       5             /* coding rate 4/5 */

/* ---------- LED bao trang thai ---------- */
#define PIN_LED_B           GPIO_NUM_39
#define PIN_LED_G           GPIO_NUM_40
#define PIN_LED_R           GPIO_NUM_41
#define LED_ON_LEVEL        1             /* doi thanh 0 neu LED noi kieu active-low */
