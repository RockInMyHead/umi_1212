#pragma once
// Copy to board_config.h and enter your actual wiring. No board is assumed.
#define IMU_SDA_PIN -1
#define IMU_SCL_PIN -1
#define IMU_DRDY_PIN -1
#define CALIBRATION_BUTTON_PIN -1  // momentary switch to GND, INPUT_PULLUP
#define LORA_SCK_PIN -1
#define LORA_MISO_PIN -1
#define LORA_MOSI_PIN -1
#define LORA_NSS_PIN -1
#define LORA_RESET_PIN -1
#define LORA_DIO0_PIN -1
#define LORA_FREQUENCY_HZ 0L      // module/antenna/local-authorized frequency
#define VERIFIED_REST_PIN -1     // disabled by default; external truth, NOT IMU quiet
#define VERIFIED_REST_ACTIVE_LEVEL LOW
#define IMU_ADDRESS 0x68          // AD0 low; use 0x69 only when AD0 high
#define TELEMETRY_PERIOD_MS 10000UL // radio airtime/duty limits remain deployment-specific
#define TELEMETRY_SERIAL false

#if IMU_SDA_PIN < 0 || IMU_SCL_PIN < 0 || IMU_DRDY_PIN < 0 || CALIBRATION_BUTTON_PIN < 0
#error "Set actual MPU6050 and calibration button pins in board_config.h"
#endif
#if LORA_SCK_PIN < 0 || LORA_MISO_PIN < 0 || LORA_MOSI_PIN < 0 || LORA_NSS_PIN < 0 || LORA_RESET_PIN < 0 || LORA_DIO0_PIN < 0 || LORA_FREQUENCY_HZ <= 0
#error "Set actual SX1278 SPI, reset, DIO0 pins and frequency in board_config.h"
#endif
