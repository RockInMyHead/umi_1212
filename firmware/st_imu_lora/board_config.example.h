#pragma once
// Copy to board_config.h and enter your actual wiring. No board is assumed.
#define IMU_SDA_PIN -1
#define IMU_SCL_PIN -1
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
#define ACCEL_ADDRESS 0x18
#define GYRO_ADDRESS 0x68
#define MAG_ADDRESS 0x1c
#define BARO_ADDRESS 0x5c
#define TELEMETRY_PERIOD_MS 10000UL // radio airtime/duty limits remain deployment-specific
#define TELEMETRY_SERIAL false

#if IMU_SDA_PIN < 0 || IMU_SCL_PIN < 0 || CALIBRATION_BUTTON_PIN < 0
#error "Set actual ST IMU and calibration button pins in board_config.h"
#endif
#if LORA_SCK_PIN < 0 || LORA_MISO_PIN < 0 || LORA_MOSI_PIN < 0 || LORA_NSS_PIN < 0 || LORA_RESET_PIN < 0 || LORA_DIO0_PIN < 0 || LORA_FREQUENCY_HZ <= 0
#error "Set actual SX1278 SPI, reset, DIO0 pins and frequency in board_config.h"
#endif
