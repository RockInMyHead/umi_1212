#include "Wire.h"
#include "MPU6050.h"
#include <RadioLib.h>
#include "algorithm_core.h" // Includes Vector3D, Quaternion and tactical navigation engines
#include "calibration.h"

// UNIQUE CONFIGURATION PARAMETER FOR EACH TRANSMITTER NODE
const uint8_t DEVICE_ID = 1; // Change this ID (1 to 20) before flashing each specific board

MPU6050 mpu;

// RADIO PIN CONFIGURATION------------------------------------------
const int LORA_CS   = 10; // Chip Select (NSS) pin
const int LORA_DIO0 = 2;  // Hardware Interrupt pin (DIO0)
const int LORA_RST  = 9;  // Hardware Reset (NRST) pin

Module mod(LORA_CS, LORA_DIO0, LORA_RST);
SX1278 radio(&mod);

//------------------------------------------------------------------

// HARDWARE INTERACTIVE BUTTON CONFIGURATION-------------------------
const int CALIBRATE_BUTTON_PIN = 4; // Tactile button connected between this pin and GND
unsigned long last_debounce_time = 0;
const unsigned long DEBOUNCE_DELAY = 250; // Milliseconds to prevent button double-trigger
//------------------------------------------------------------------

// OPTIMIZED COMPACT TELEMETRY PACKET--------------------------------
typedef struct __attribute__((__packed__)) {
    uint8_t  device_id;     // Transmitter unique identifier (1 byte)
    uint32_t packet_id;     // Sequential tracking number to detect data loss (4 bytes)
    float    distance;      // Processed linear displacement magnitude (4 bytes)
    float    alpha;         // True movement angle relative to Global X (4 bytes)
    float    beta;          // True movement angle relative to Global Y (4 bytes)
    float    gamma;         // True movement angle relative to Global Z (4 bytes)
} CleanTelemetryPacket;

CleanTelemetryPacket tx_packet;
uint32_t global_packet_counter = 0; // Persistent sequence counter
unsigned long last_timestamp = 0;   // High-resolution delta-time tracker

// Persistent global state for current orientation tracking
Quaternion current_orientation = {1.0f, 0.0f, 0.0f, 0.0f};
//------------------------------------------------------------------

// ASYNCHRONOUS LORA STATE REGISTERS--------------------------------
volatile bool transmission_flag = false; // Triggered inside hardware ISR when TX finishes
bool lora_is_transmitting = false;       // Flags if the RF chip is currently occupied

// ISR (Interrupt Service Routine) - Invoked by hardware when transmission completes
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR void set_flag(void) {
#else
void set_flag(void) {
#endif
    transmission_flag = true;
}
//------------------------------------------------------------------

void setup() {
    Wire.begin();
    Wire.setClock(400000UL); // Overclock I2C to 400kHz for minimum sensor lag
    Serial.begin(115200);    // Fast serial monitoring

    // Initialize tactile button pin with internal pullup resistor
    pinMode(CALIBRATE_BUTTON_PIN, INPUT_PULLUP);

    // 1. Initialize MPU6050 Sensor
    Serial.println("Initializing MPU6050 Core...");
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("CRITICAL ERROR: MPU6050 Communication Failed!");
        while (1);
    }

    // Run initial calibration on startup to clear volatile register states
    calibrate_mpu_after_button();

    // 2. Initialize LoRa Radio Module via RadioLib
    Serial.println("Initializing LoRa Radio Module...");
    int state = radio.begin(434.0, 125.0, 7, 5);

    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("Radio Core Status: ONLINE (Asynchronous P2P Mode Enabled)");
    } else {
        Serial.print("CRITICAL ERROR: Radio Initialization Failed, Code: ");
        Serial.println(state);
        while (1);
    }

    radio.setOutputPower(20);
    radio.setPacketSentAction(set_flag);

    last_timestamp = micros();
}

void loop() {
    // 0. Check tactile button state for on-the-fly mid-operation calibration
    if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW) {
        unsigned long current_time = millis();
        if (current_time - last_debounce_time > DEBOUNCE_DELAY) {
            if (lora_is_transmitting) {
                // ИСПРАВЛЕНО: Синтаксис точки заменен на стрелку (->)
                radio.finishTransmit();
                lora_is_transmitting = false;
                transmission_flag = false;
            }

            calibrate_mpu_after_button();

            // Safe element-by-element resets to bypass array initialization constraint
            acc_global_old.x = 0.0f; acc_global_old.y = 0.0f; acc_global_old.z = 0.0f;
            vel_global.x     = 0.0f; vel_global.y     = 0.0f; vel_global.z     = 0.0f;

            current_orientation.w = 1.0f; current_orientation.x = 0.0f;
            current_orientation.y = 0.0f; current_orientation.z = 0.0f;

            last_timestamp = micros();
            last_debounce_time = millis();
        }
    }

    // 1. High-precision delta time calculation
    unsigned long current_timestamp = micros();
    float dt = (current_timestamp - last_timestamp) / 1000000.0f;
    last_timestamp = current_timestamp;

    if (dt <= 0.0f || dt > 0.1f) dt = 0.02f;

    // 2. Direct hardware memory read from sensor registers
    int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    mpu.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

    // 3. Scale raw register integers into physical metric units
    Vector3D acc_local_phys;
    acc_local_phys.x = (float)raw_ax / 16384.0f * 9.80665f;
    acc_local_phys.y = (float)raw_ay / 16384.0f * 9.80665f;
    acc_local_phys.z = (float)raw_az / 16384.0f * 9.80665f;

    // 4. Orientation Quaternion estimation via Tactical Data Filtration Engine
    float gx_rad = ((float)raw_gx / 131.0f) * 0.0174533f;
    float gy_rad = ((float)raw_gy / 131.0f) * 0.0174533f;
    float gz_rad = ((float)raw_gz / 131.0f) * 0.0174533f;

    Vector3D gyro_rad_vec = {gx_rad, gy_rad, gz_rad};
    execute_data_filtration(&current_orientation, acc_local_phys, gyro_rad_vec, dt);

    // 5. Compute Dead Reckoning locally on the node to offload the base receiver
    TacticalMovement navigation_result = execute_tactical_navigation(acc_local_phys, current_orientation, dt);

    // 6. Asynchronous Telemetry Radio Handler
    bool skip_immediate_transmit = false;

    if (transmission_flag) {
        transmission_flag = false;
        lora_is_transmitting = false;
        skip_immediate_transmit = true;

        int completion_state = radio.finishTransmit();
        if (completion_state == RADIOLIB_ERR_NONE) {
            Serial.print("TX SUCCESS | Node: "); Serial.print(tx_packet.device_id);
            Serial.print(" | Packet ID: "); Serial.print(tx_packet.packet_id);
            Serial.print(" | Distance: "); Serial.println(tx_packet.distance, 3);
        } else {
            Serial.print("TX FAILED! Error Code: "); Serial.println(completion_state);
        }
    }

    if (!lora_is_transmitting && !skip_immediate_transmit) {
        tx_packet.device_id = DEVICE_ID;
        tx_packet.packet_id = global_packet_counter++;
        tx_packet.distance  = navigation_result.distance;
        tx_packet.alpha     = navigation_result.alpha;
        tx_packet.beta      = navigation_result.beta;
        tx_packet.gamma     = navigation_result.gamma;
        int transmission_state = radio.startTransmit((uint8_t*)&tx_packet, sizeof(CleanTelemetryPacket));
        if (transmission_state == RADIOLIB_ERR_NONE) {
            lora_is_transmitting = true;
        } else {
            Serial.print("TX FAILED! Initiation error: "); Serial.println(transmission_state);
        }
    }
}
