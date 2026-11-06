// Actual Troyka IMU v2 sensor set; ESP32 controller is an explicit integration assumption.
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RadioLib.h>
#include <new>
#include "board_config.h"
#include "calibration_config.h"
#include "sensor_config.h"
#include "st_driver.h"
#include "acquisition.h"
#include "telemetry.h"
#if !defined(ESP32)
#error "Port the Wire/clock/stack/RadioLib adapter for the actual MCU before building"
#endif
SET_LOOP_TASK_STACK_SIZE(32*1024);

namespace {
acquisition::Micros64 clock64;
class WireBus : public stimu::Bus {
public:
    bool read(uint8_t a,uint8_t r,uint8_t* b,size_t n) override {
        Wire.beginTransmission(a); Wire.write(uint8_t(r|(n>1?0x80:0)));
        if(Wire.endTransmission(false)!=0) return false;
        if(Wire.requestFrom(a,uint8_t(n),true)!=n) return false;
        for(size_t i=0;i<n;++i) { if(!Wire.available()) return false; b[i]=uint8_t(Wire.read()); }
        return true;
    }
    bool write(uint8_t a,uint8_t r,uint8_t v) override {
        Wire.beginTransmission(a); Wire.write(r); Wire.write(v); return Wire.endTransmission(true)==0;
    }
    uint64_t nowUs() override { return clock64.update(micros()); }
} bus;
stimu::Driver sensors(bus,{ACCEL_ADDRESS,GYRO_ADDRESS,MAG_ADDRESS,BARO_ADDRESS});
imu::Config config;
imu::Navigator navigator;
Module radio_module(LORA_NSS_PIN,LORA_DIO0_PIN,LORA_RESET_PIN,RADIOLIB_NC);
SX1278 radio(&radio_module);
volatile bool radio_done=false;
bool radio_available=false,transmitting=false,stopped=false,collecting=false;
uint32_t adapter_fault=0,sequence=0,epoch=0,samples=0,dropped=0,max_compute=0;
uint32_t last_poll=0,last_telemetry=0,tx_start=0;
uint64_t acquisition_start=0,last_gyro=0,alignment_start=0,last_mag_used=0,last_baro_used=0;
acquisition::Moments acc_stats,gyro_stats,mag_stats;
double pressure_sum=0,reference_pressure=0,temperature=25;
uint32_t pressure_count=0;

void IRAM_ATTR sent() { radio_done=true; }
bool verifiedRest() {
#if VERIFIED_REST_PIN >= 0
    return digitalRead(VERIFIED_REST_PIN)==VERIFIED_REST_ACTIVE_LEVEL;
#else
    return false;
#endif
}
void stop(uint32_t fault) { adapter_fault|=fault; stopped=true; collecting=false; }
bool validMounting() {
    using namespace sensor_config;
    return mounting_verified&&stimu::properRotation(accel_to_body)&&stimu::properRotation(gyro_to_body)&&stimu::properRotation(mag_to_body);
}
imu::Vec3 correctedMag(imu::Vec3 raw) {
    const auto calibrated=stimu::matrixVector(magnetic_calibration::soft_iron,raw-magnetic_calibration::hard_iron_uT);
    return stimu::matrixVector(sensor_config::mag_to_body,calibrated);
}
void resetCollection() {
    acc_stats={}; gyro_stats={}; mag_stats={}; alignment_start=0; pressure_count=0; pressure_sum=0;
}
void beginAlignment() {
    if(adapter_fault&stimu::ConfigurationFault) return;
    if(!sensors.begin()) { stop(sensors.fault()); return; }
    resetCollection(); last_gyro=last_mag_used=last_baro_used=0; reference_pressure=0;
    adapter_fault&=stimu::RadioFault; stopped=false; collecting=true;
    navigator.~Navigator(); new (&navigator) imu::Navigator(config);
    acquisition_start=bus.nowUs();
}
void button() {
    static bool raw_old=HIGH,stable=HIGH; static uint32_t changed=0;
    const bool raw=digitalRead(CALIBRATION_BUTTON_PIN); const uint32_t t=millis();
    if(raw!=raw_old) { raw_old=raw; changed=t; }
    if(raw!=stable && uint32_t(t-changed)>=50) { stable=raw; if(stable==LOW) beginAlignment(); }
}
void collectAuxiliary() {
    const auto& d=sensors.data();
    if(d.mag.fresh&&sensor_config::magnetometer_enabled) mag_stats.add(correctedMag(d.mag.value));
    if(d.pressure_fresh) { pressure_sum+=d.pressure_hpa; ++pressure_count; }
}
void align(const imu::Sample& s) {
    const auto& c=config.calibration;
    const auto a=acquisition::corrected(s.accel,s.temperature_c,c.accel_matrix,c.accel_offset,c.accel_temp_slope,c.reference_temperature_c);
    const auto w=acquisition::corrected(s.gyro,s.temperature_c,c.gyro_matrix,c.gyro_offset,c.gyro_temp_slope,c.reference_temperature_c);
    // A button asserts rest; statistics reject common movement but cannot prove it.
    if(std::abs(imu::norm(a)-config.gravity)>.7 || imu::norm(w)>.5) { resetCollection(); return; }
    if(!alignment_start) alignment_start=uint64_t(s.time_s*1e6);
    acc_stats.add(a); gyro_stats.add(w);
    if(acc_stats.n>100 && (acc_stats.rms()>.15 || gyro_stats.rms()>.025)) { resetCollection(); return; }
    if(s.time_s*1e6-alignment_start<1e7 || acc_stats.n<900) return;
    if(sensor_config::barometer_enabled && pressure_count<100) return;
    imu::Quat q=imu::gravityAlignment(acc_stats.mean);
    if(sensor_config::magnetometer_enabled) {
        if(mag_stats.n<100) return;
        const auto m=imu::rotate(q,mag_stats.mean),r=config.magnetic_field_nav;
        if(std::hypot(m.x,m.y)<5 || std::hypot(r.x,r.y)<5 || std::abs(imu::norm(m)/imu::norm(r)-1)>.2 || mag_stats.rms()>2) {
            resetCollection(); return;
        }
        q=imu::multiply(imu::expRotation({0,0,std::atan2(r.y,r.x)-std::atan2(m.y,m.x)}),q);
    }
    const auto bg=gyro_stats.mean-imu::rotate(imu::conjugate(q),config.earth_rate_nav);
    if(!navigator.initialize(s.time_s,q,{},{},{},bg)) { stop(navigator.state().fault); return; }
    reference_pressure=pressure_count?pressure_sum/pressure_count:0;
    collecting=false; ++epoch;
    last_mag_used=sensors.data().mag.time_us; last_baro_used=sensors.data().pressure_time_us;
}
void updateAuxiliary(uint64_t state_time_us) {
    const auto& d=sensors.data(); const double t=navigator.state().time_s;
    // Explicit approximation: fuse latest fresh aid at the current state time,
    // with age-dependent variance. Production high dynamics needs delayed replay.
    if(sensor_config::magnetometer_enabled && d.mag.time_us>last_mag_used && d.mag.time_us<=state_time_us) {
        const double age=(state_time_us-d.mag.time_us+d.mag.uncertainty_us)*1e-6;
        if(age<.04) {
            const double latency=imu::norm(config.magnetic_field_nav)*sensor_config::magnetic_latency_rotation_bound_rad_s*age;
            const double variance=sensor_config::magnetic_sigma_uT*sensor_config::magnetic_sigma_uT+latency*latency;
            navigator.updateMagnetometer(correctedMag(d.mag.value),{variance,variance,variance},t);
        }
        last_mag_used=d.mag.time_us;
    }
    if(sensor_config::barometer_enabled&&reference_pressure>0&&d.pressure_time_us>last_baro_used&&d.pressure_time_us<=state_time_us) {
        const double age=(state_time_us-d.pressure_time_us+d.pressure_uncertainty_us)*1e-6;
        if(age<.08) {
            const double altitude=stimu::relativeAltitude(d.pressure_hpa,reference_pressure,sensor_config::mean_air_temperature_k,config.gravity);
            const double latency=sensor_config::barometric_vertical_speed_bound_m_s*age;
            navigator.updateBarometer(altitude,sensor_config::barometer_sigma_m*sensor_config::barometer_sigma_m+latency*latency,t);
        }
        last_baro_used=d.pressure_time_us;
    }
}
void acquire() {
    const uint32_t now=micros(); if(uint32_t(now-last_poll)<1000) return;
    last_poll=now;
    if(stopped) return;
    if(!sensors.poll()) { ++dropped; stop(sensors.fault()); return; }
    const auto& d=sensors.data(); const uint64_t current=bus.nowUs();
    if(current-(last_gyro?last_gyro:acquisition_start)>30000) { ++dropped; stop(stimu::SamplingFault); return; }
    if(d.pressure_time_us) temperature=d.baro_temperature_c;
    if(collecting) collectAuxiliary();
    if(!d.gyro.fresh) return;
    const uint64_t t=d.gyro.time_us;
    if(!d.accel.time_us) return;
    const int64_t skew=int64_t(t)-int64_t(d.accel.time_us);
    if(std::abs(skew)>15000 || d.gyro.uncertainty_us>4000 || d.accel.uncertainty_us>4000
        || (last_gyro&&(t<=last_gyro||t-last_gyro>30000))) {
        ++dropped; stop(stimu::SamplingFault); return;
    }
    last_gyro=t; ++samples;
    imu::Sample sample; sample.time_s=t*1e-6;
    sample.accel=stimu::matrixVector(sensor_config::accel_to_body,d.accel.value);
    sample.gyro=stimu::matrixVector(sensor_config::gyro_to_body,d.gyro.value);
    sample.temperature_c=temperature; // barometer-chip proxy; calibrate using THIS same proxy
    const uint32_t start=micros();
    if(collecting) align(sample);
    else if(navigator.state().initialized) {
        if(!navigator.predict(sample,verifiedRest())) stop(navigator.state().fault);
        else updateAuxiliary(t);
    }
    const uint32_t duration=micros()-start; if(duration>max_compute) max_compute=duration;
}
void telemetry() {
    const uint32_t now=millis();
    if(transmitting) {
        if(radio_done) {
            radio_done=false; transmitting=false;
            if(radio.finishTransmit()!=RADIOLIB_ERR_NONE) { radio_available=false; adapter_fault|=stimu::RadioFault; }
        } else if(uint32_t(now-tx_start)>2000) {
            radio.standby(); transmitting=false; radio_available=false; adapter_fault|=stimu::RadioFault;
        }
    }
    if(transmitting || uint32_t(now-last_telemetry)<TELEMETRY_PERIOD_MS) return;
    // Polling continues every loop; RF operations remain short but require profiling.
    last_telemetry=now; const auto& s=navigator.state();
    uint32_t status=(s.initialized?wire::Initialized:0)|(s.quiet?wire::Quiet:0)|(s.stationary?wire::VerifiedStationary:0)
        |(collecting?wire::Calibrating:0)|(stopped?wire::AcquisitionStopped:0)|(!radio_available?wire::RadioFailed:0)
        |(!s.initialized&&!collecting?wire::WaitingForCalibration:0)|(!measuredCalibrationProvided()?wire::Uncalibrated:0)
        |(1u<<9)|(sensor_config::magnetometer_enabled?1u<<10:0)|(sensor_config::barometer_enabled?1u<<11:0);
    const auto& d=sensors.data();
    uint8_t packet[148];
    wire::encode(packet,navigator,sequence++,epoch,status,adapter_fault,temperature,dropped,samples,max_compute);
    packet[2]=3; packet[3]=sizeof packet;
    // v3 keeps v2 bytes0..107; extension replaces v2 CRC. Four timestamp bounds.
    const uint64_t current=bus.nowUs();
    const auto age=[&](uint64_t stamp)->uint32_t { return stamp&&current>=stamp?uint32_t(std::fmin(double(current-stamp),double(UINT32_MAX))):UINT32_MAX; };
    wire::u32(packet+108,age(d.accel.time_us)); wire::u32(packet+112,age(d.gyro.time_us));
    wire::u32(packet+116,age(d.mag.time_us)); wire::u32(packet+120,age(d.pressure_time_us));
    wire::u32(packet+124,d.accel.uncertainty_us); wire::u32(packet+128,d.gyro.uncertainty_us);
    wire::f32(packet+132,d.pressure_hpa,status); wire::f32(packet+136,navigator.covariance(15,15),status);
    wire::u32(packet+140,d.auxiliary_overruns); wire::u32(packet+20,status); wire::u32(packet+144,wire::crc32(packet,144));
    if(TELEMETRY_SERIAL&&Serial.availableForWrite()>=int(sizeof packet)) Serial.write(packet,sizeof packet);
    if(radio_available) {
        radio_done=false;
        if(radio.startTransmit(packet,sizeof packet)==RADIOLIB_ERR_NONE) { transmitting=true; tx_start=now; }
        else { radio_available=false; adapter_fault|=stimu::RadioFault; }
    }
}
} // namespace

void adapterSetup() {
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxBufferSize(512); // a v3 frame exceeds the unbuffered UART FIFO
#endif
    Serial.begin(115200); pinMode(CALIBRATION_BUTTON_PIN,INPUT_PULLUP);
#if VERIFIED_REST_PIN >= 0
    pinMode(VERIFIED_REST_PIN,VERIFIED_REST_ACTIVE_LEVEL==LOW?INPUT_PULLUP:INPUT_PULLDOWN);
#endif
    config.calibration=measuredCalibration(); config.gyro_noise=.0006;
    config.gravity=sensor_config::local_gravity_m_s2;
    config.earth_rate_nav=sensor_config::earth_rate_nav_rad_s;
    config.magnetic_field_nav=sensor_config::magnetometer_enabled?sensor_config::magnetic_field_nav:imu::Vec3{};
    navigator.~Navigator(); new (&navigator) imu::Navigator(config);
    Wire.begin(IMU_SDA_PIN,IMU_SCL_PIN); Wire.setClock(400000); Wire.setTimeOut(5);
    SPI.begin(LORA_SCK_PIN,LORA_MISO_PIN,LORA_MOSI_PIN,LORA_NSS_PIN);
    radio_available=radio.begin(LORA_FREQUENCY_HZ*1e-6,125.,7,5,0x12,10,8,0)==RADIOLIB_ERR_NONE;
    if(radio_available) { radio_available=radio.setCRC(true)==RADIOLIB_ERR_NONE; radio.setPacketSentAction(sent); }
    if(!radio_available) adapter_fault|=stimu::RadioFault;
    if(!validMounting() || (sensor_config::magnetometer_enabled&&(!magnetic_calibration::provided||imu::norm(config.magnetic_field_nav)<10))
       ||(imu::norm(config.earth_rate_nav)>0&&!sensor_config::magnetometer_enabled)) stop(stimu::ConfigurationFault);
    if(!stopped&&!sensors.begin()) stop(sensors.fault());
    acquisition_start=bus.nowUs();
}
void adapterLoop() { bus.nowUs(); button(); acquire(); telemetry(); }
