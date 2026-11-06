// ESP32 integration example. Read docs/HARDWARE.md before commissioning.
// Host algorithm tests are not a hardware build or timing qualification.
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <RadioLib.h>
#include <new>
#include "board_config.h"
#include "calibration_config.h"
#include "navigation.h"
#include "acquisition.h"
#include "telemetry.h"

#if !defined(ESP32)
#error "This adapter targets Arduino-ESP32; port and validate for other MCUs"
#endif
SET_LOOP_TASK_STACK_SIZE(32 * 1024); // qualify actual stack high-water mark on target

namespace {
constexpr uint32_t sample_period_us=5000, calibration_samples=2000;
constexpr uint32_t tx_timeout_ms=2000;
constexpr double radians_per_degree=0.017453292519943295;
enum AdapterFault : uint32_t { BusFault=1u<<16, SampleLoss=1u<<17, ConfigurationFault=1u<<18, RadioFault=1u<<19 };
imu::Config config;
imu::Navigator navigator;
MPU6050 sensor(IMU_ADDRESS);
Module radio_module(LORA_NSS_PIN,LORA_DIO0_PIN,LORA_RESET_PIN,RADIOLIB_NC);
SX1278 radio(&radio_module);
portMUX_TYPE irq_mutex=portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t irq_sequence=0,irq_tick=0;
volatile bool radio_done=false;
uint32_t consumed_sequence=0,adapter_fault=0,packet_sequence=0,epoch=0;
uint32_t dropped_samples=0,sample_count=0,max_compute_us=0;
uint32_t last_telemetry_ms=0,tx_start_ms=0;
uint64_t last_sample_us=0,acquisition_started_us=0;
acquisition::Micros64 monotonic;
acquisition::Moments alignment_accel,alignment_gyro;
double last_temperature=25;
bool stopped=false,collecting=false,radio_available=false,transmitting=false;

void IRAM_ATTR dataReady() {
    portENTER_CRITICAL_ISR(&irq_mutex);
    irq_tick=micros(); ++irq_sequence;
    portEXIT_CRITICAL_ISR(&irq_mutex);
}
void IRAM_ATTR txDone() { radio_done=true; }
void snapshot(uint32_t& sequence,uint32_t& tick) {
    portENTER_CRITICAL(&irq_mutex); sequence=irq_sequence; tick=irq_tick; portEXIT_CRITICAL(&irq_mutex);
}
bool externallyVerifiedRest() {
#if VERIFIED_REST_PIN >= 0
    return digitalRead(VERIFIED_REST_PIN)==VERIFIED_REST_ACTIVE_LEVEL;
#else
    return false;
#endif
}
void fail(uint32_t reason) {
    adapter_fault|=reason; stopped=true; collecting=false;
}
bool registerEquals(uint8_t reg,uint8_t expected,uint8_t mask=0xff) {
    uint8_t value=0;
    return I2Cdev::readByte(IMU_ADDRESS,reg,&value,5)==1 && (value&mask)==expected;
}
bool configureSensor() {
    sensor.reset(); delay(100); sensor.initialize(); sensor.setSleepEnabled(false);
    sensor.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    sensor.setTempSensorEnabled(true);
    sensor.setDLPFMode(MPU6050_DLPF_BW_42); // accel 44 Hz, gyro 42 Hz
    sensor.setRate(4);                      // 1 kHz / (1+4) = 200 Hz
    sensor.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    sensor.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    sensor.setDMPEnabled(false); sensor.setFIFOEnabled(false);
    sensor.setIntEnabled(0); sensor.setInterruptMode(false); sensor.setInterruptDrive(false);
    sensor.setInterruptLatch(false); sensor.setInterruptLatchClear(false);
    // Verify transactions rather than trusting void setters.
    return sensor.testConnection()
        && registerEquals(MPU6050_RA_SMPLRT_DIV,4)
        && registerEquals(MPU6050_RA_CONFIG,MPU6050_DLPF_BW_42,7)
        && registerEquals(MPU6050_RA_ACCEL_CONFIG,2<<3,0x18)
        && registerEquals(MPU6050_RA_GYRO_CONFIG,1<<3,0x18)
        && registerEquals(MPU6050_RA_PWR_MGMT_1,1,0x7f)
        && registerEquals(MPU6050_RA_INT_PIN_CFG,0,0xf0);
}
void beginAlignment() {
    // The button is an explicit operator declaration of rest for the next 10 s.
    // Also resets the local origin. Epoch changes when alignment completes.
    if(adapter_fault&ConfigurationFault) return;
    alignment_accel={}; alignment_gyro={}; collecting=true; stopped=false;
    adapter_fault&=RadioFault; last_sample_us=0; acquisition_started_us=monotonic.update(micros());
    navigator.~Navigator(); new (&navigator) imu::Navigator(config); // no large stack temporary
    uint32_t tick; snapshot(consumed_sequence,tick);
}
void serviceButton() {
    static bool previous_raw=HIGH,stable=HIGH;
    static uint32_t changed_ms=0;
    const bool raw=digitalRead(CALIBRATION_BUTTON_PIN);
    const uint32_t now=millis();
    if(raw!=previous_raw) { previous_raw=raw; changed_ms=now; }
    if(raw!=stable && uint32_t(now-changed_ms)>=50) {
        stable=raw; if(stable==LOW) beginAlignment(); // edge only, never repeat while held
    }
}
int16_t signedWord(const uint8_t* p) {
    const uint16_t u=(uint16_t(p[0])<<8)|p[1];
    return u<32768?int16_t(u):int16_t(int32_t(u)-65536);
}
void align(const imu::Sample& sample) {
    const auto& c=config.calibration;
    const auto a=acquisition::corrected(sample.accel,sample.temperature_c,c.accel_matrix,c.accel_offset,c.accel_temp_slope,c.reference_temperature_c);
    const auto g=acquisition::corrected(sample.gyro,sample.temperature_c,c.gyro_matrix,c.gyro_offset,c.gyro_temp_slope,c.reference_temperature_c);
    if(std::abs(imu::norm(a)-config.gravity)>0.7 || imu::norm(g)>0.05) {
        alignment_accel={}; alignment_gyro={}; return;
    }
    alignment_accel.add(a); alignment_gyro.add(g);
    if(alignment_accel.n>=100 && (alignment_accel.rms()>0.12 || alignment_gyro.rms()>0.015)) {
        alignment_accel={}; alignment_gyro={}; return;
    }
    if(alignment_accel.n<calibration_samples) return;
    const imu::Quat q=imu::gravityAlignment(alignment_accel.mean);
    const imu::Vec3 earth_body=imu::rotate(imu::conjugate(q),config.earth_rate_nav);
    const imu::Vec3 gyro_bias=alignment_gyro.mean-earth_body;
    if(!navigator.initialize(sample.time_s,q,{},{},{},gyro_bias)) { fail(imu::NumericalFailure); return; }
    collecting=false; ++epoch;
}
void acquire() {
    uint32_t sequence,tick; snapshot(sequence,tick);
    const uint32_t count=sequence-consumed_sequence;
    if(count==0) {
        const uint64_t reference=last_sample_us?last_sample_us:acquisition_started_us;
        if(monotonic.update(micros())-reference>20000) fail(SampleLoss);
        return;
    }
    consumed_sequence=sequence;
    if(count!=1) { dropped_samples+=count-1; fail(SampleLoss); return; }
    const uint32_t now32=micros(); const uint64_t now64=monotonic.update(now32);
    const uint32_t age=now32-tick;
    // Avoid associating a new register set with an old event near the next edge.
    if(age>3000) { ++dropped_samples; fail(SampleLoss); return; }
    const uint64_t sample_us=now64-age;
    uint8_t bytes[14];
    const int8_t got=I2Cdev::readBytes(IMU_ADDRESS,MPU6050_RA_ACCEL_XOUT_H,14,bytes,5);
    uint32_t after,after_tick; snapshot(after,after_tick);
    if(got!=14) { fail(BusFault); return; }
    if(after!=sequence) { ++dropped_samples; consumed_sequence=after; fail(SampleLoss); return; }
    if(last_sample_us && (sample_us-last_sample_us<sample_period_us/2 || sample_us-last_sample_us>sample_period_us*3/2)) {
        ++dropped_samples; fail(SampleLoss); return;
    }
    last_sample_us=sample_us;
    int16_t raw[7]; for(int i=0;i<7;++i) raw[i]=signedWord(bytes+2*i);
    imu::Sample sample;
    sample.time_s=double(sample_us)*1e-6;
    sample.accel={raw[0]*9.80665/4096.0,raw[1]*9.80665/4096.0,raw[2]*9.80665/4096.0};
    sample.temperature_c=raw[3]/340.0+36.53; last_temperature=sample.temperature_c;
    sample.gyro={raw[4]*radians_per_degree/65.5,raw[5]*radians_per_degree/65.5,raw[6]*radians_per_degree/65.5};
    for(int i=0;i<7;++i) if(i!=3 && (raw[i]>=32700 || raw[i]<=-32700)) sample.saturated=true;
    ++sample_count;
    if(sample.saturated) { fail(imu::Saturation); return; }
    if(stopped) return;
    const uint32_t started=micros();
    if(collecting) align(sample);
    else if(navigator.state().initialized && !navigator.predict(sample,externallyVerifiedRest())) fail(navigator.state().fault);
    const uint32_t compute=micros()-started;
    if(compute>max_compute_us) max_compute_us=compute;
}
void serviceTelemetry() {
    const uint32_t now=millis();
    if(transmitting) {
        if(radio_done) {
            transmitting=false; radio_done=false;
            if(radio.finishTransmit()!=RADIOLIB_ERR_NONE) { radio_available=false; adapter_fault|=RadioFault; }
        }
        else if(uint32_t(now-tx_start_ms)>tx_timeout_ms) {
            radio.standby(); transmitting=false; radio_available=false; adapter_fault|=RadioFault;
        }
    }
    if(transmitting || uint32_t(now-last_telemetry_ms)<TELEMETRY_PERIOD_MS) return;
    uint32_t seq,tick; snapshot(seq,tick);
    if(!stopped && (seq!=consumed_sequence || uint32_t(micros()-tick)>2500)) return; // IMU first
    last_telemetry_ms=now;
    const auto& s=navigator.state();
    uint32_t status=(s.initialized?wire::Initialized:0)|(s.quiet?wire::Quiet:0)
        |(s.stationary?wire::VerifiedStationary:0)|(collecting?wire::Calibrating:0)
        |(stopped?wire::AcquisitionStopped:0)|(!radio_available?wire::RadioFailed:0)
        |(!s.initialized&&!collecting?wire::WaitingForCalibration:0)
        |(!measuredCalibrationProvided()?wire::Uncalibrated:0);
    uint8_t packet[wire::packet_size];
    wire::encode(packet,navigator,packet_sequence++,epoch,status,adapter_fault,last_temperature,dropped_samples,sample_count,max_compute_us);
    if(TELEMETRY_SERIAL && Serial.availableForWrite()>=int(sizeof packet)) Serial.write(packet,sizeof packet);
    if(radio_available) {
        radio_done=false;
        if(radio.startTransmit(packet,sizeof packet)==RADIOLIB_ERR_NONE) { transmitting=true; tx_start_ms=now; }
        else { adapter_fault|=RadioFault; radio_available=false; }
    }
}
} // namespace

void adapterSetup() {
    Serial.begin(115200);
    pinMode(CALIBRATION_BUTTON_PIN,INPUT_PULLUP);
#if VERIFIED_REST_PIN >= 0
    pinMode(VERIFIED_REST_PIN,VERIFIED_REST_ACTIVE_LEVEL==LOW?INPUT_PULLUP:INPUT_PULLDOWN);
#endif
    config.calibration=measuredCalibration();
    navigator.~Navigator(); new (&navigator) imu::Navigator(config);
    Wire.begin(IMU_SDA_PIN,IMU_SCL_PIN); Wire.setClock(400000); Wire.setTimeOut(5);
    I2Cdev::readTimeout=5;
    if(!configureSensor()) fail(ConfigurationFault);
    SPI.begin(LORA_SCK_PIN,LORA_MISO_PIN,LORA_MOSI_PIN,LORA_NSS_PIN);
    // Frequency MHz, bandwidth kHz, SF7, CR4/5, sync0x12, power10 dBm.
    radio_available=radio.begin(LORA_FREQUENCY_HZ*1e-6,125.0,7,5,0x12,10,8,0)==RADIOLIB_ERR_NONE;
    if(radio_available) {
        radio_available=radio.setCRC(true)==RADIOLIB_ERR_NONE;
        radio.setPacketSentAction(txDone);
        if(!radio_available) adapter_fault|=RadioFault;
    } else adapter_fault|=RadioFault;
    pinMode(IMU_DRDY_PIN,INPUT);
    acquisition_started_us=monotonic.update(micros());
    attachInterrupt(digitalPinToInterrupt(IMU_DRDY_PIN),dataReady,RISING);
    sensor.setIntDataReadyEnabled(true);
    if(!registerEquals(MPU6050_RA_INT_ENABLE,1)) fail(ConfigurationFault);
    // No navigation until operator presses calibration button at genuine rest.
}
void adapterLoop() {
    monotonic.update(micros());
    serviceButton(); acquire(); serviceTelemetry();
}
