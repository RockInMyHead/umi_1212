#pragma once
#include "navigation.h"
#include <cmath>
#include <cstddef>
#include <cstdint>

// Independently implemented from ST register maps; no third-party driver copied.
namespace stimu {
constexpr double standard_gravity=9.80665, degree_to_rad=0.017453292519943295;
enum Fault : uint32_t { BusFault=1u<<16, SamplingFault=1u<<17, ConfigurationFault=1u<<18,
                       RadioFault=1u<<19, Overrun=1u<<20, Clipping=1u<<21 };
struct Bus {
    virtual ~Bus()=default;
    virtual bool read(uint8_t address,uint8_t reg,uint8_t* bytes,size_t count)=0;
    virtual bool write(uint8_t address,uint8_t reg,uint8_t value)=0;
    virtual uint64_t nowUs()=0;
};
struct Addresses { uint8_t accel=0x18,gyro=0x68,mag=0x1c,baro=0x5c; };
struct VectorReading { imu::Vec3 value{}; uint64_t time_us=0; uint32_t uncertainty_us=0; bool fresh=false; };
struct Readings {
    VectorReading accel,gyro,mag;
    double pressure_hpa=0,baro_temperature_c=25;
    int8_t gyro_temperature_raw=0; // relative only; no invented absolute offset
    uint64_t pressure_time_us=0;
    uint32_t pressure_uncertainty_us=0,auxiliary_overruns=0;
    bool pressure_fresh=false;
};
inline int16_t signed16(const uint8_t* p) {
    const uint16_t v=uint16_t(p[0])|(uint16_t(p[1])<<8);
    return v<32768?int16_t(v):int16_t(int32_t(v)-65536);
}
inline double accel4g(int16_t packed) { return (int32_t(packed)/16)*0.002*standard_gravity; }
inline double gyro500(int16_t raw) { return raw*0.0175*degree_to_rad; }
inline double mag4gauss(int16_t raw) { return raw*(100.0/6842.0); }
inline double relativeAltitude(double p,double p0,double mean_air_temperature_k,double gravity) {
    return (p>0 && p0>0 && mean_air_temperature_k>0 && gravity>0)
        ?287.05*mean_air_temperature_k/gravity*std::log(p0/p):NAN;
}
inline imu::Vec3 matrixVector(const double m[3][3],imu::Vec3 v) {
    imu::Vec3 result; for(int i=0;i<3;++i) for(int j=0;j<3;++j) result[i]+=m[i][j]*v[j]; return result;
}
inline bool properRotation(const double m[3][3]) {
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) {
        double d=0; for(int k=0;k<3;++k) d+=m[i][k]*m[j][k];
        if(!std::isfinite(d) || std::abs(d-(i==j?1:0))>1e-6) return false;
    }
    const double d=m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
    return std::abs(d-1)<1e-6;
}
class Driver {
    Bus& bus_; Addresses a_; Readings data_{}; uint32_t fault_=0;
    uint64_t negative_[4]{};
    bool read(uint8_t a,uint8_t r,uint8_t* p,size_t n) {
        if(bus_.read(a,r,p,n)) return true; fault_|=BusFault; return false;
    }
    bool match(uint8_t a,uint8_t r,uint8_t expected) {
        uint8_t v=0; return read(a,r,&v,1) && v==expected;
    }
    bool set(uint8_t a,uint8_t r,uint8_t value) {
        if(!bus_.write(a,r,value)) { fault_|=BusFault; return false; }
        return match(a,r,value);
    }
    uint64_t estimateTime(int index,uint64_t ready_time,uint32_t& uncertainty) {
        const uint64_t n=negative_[index];
        const uint64_t interval=n && ready_time>=n?ready_time-n:10000;
        uncertainty=uint32_t(interval/2+1);
        return ready_time-interval/2;
    }
    bool vector(uint8_t address,int index,VectorReading& v,int type) {
        uint8_t status=0; const uint64_t before=bus_.nowUs();
        if(!read(address,0x27,&status,1)) return false;
        if(!(status&8)) { negative_[index]=before; return true; }
        uint8_t bytes[6]; if(!read(address,0x28,bytes,6)) return false;
        if(status&0x80) {
            if(type==0) fault_|=Overrun; else ++data_.auxiliary_overruns;
            return true; // discard uncertain old observation
        }
        const uint64_t observed=bus_.nowUs();
        for(int i=0;i<3;++i) {
            const int16_t raw=signed16(bytes+2*i);
            const int limit=type==0?31840:27200; // guard nominal ±4g / ±4gauss limits
            if(std::abs(int(raw))>=limit) { if(type==0) fault_|=Clipping; else ++data_.auxiliary_overruns; return true; }
            v.value[i]=type==0?accel4g(raw):mag4gauss(raw);
        }
        v.time_us=estimateTime(index,observed,v.uncertainty_us); v.fresh=true;
        return true;
    }
public:
    Driver(Bus& bus,Addresses addresses={}):bus_(bus),a_(addresses){}
    const Readings& data() const { return data_; }
    uint32_t fault() const { return fault_; }
    bool begin() {
        fault_=0; data_={}; for(auto& n:negative_) n=0;
        // Reject old-board L3G4200D/LPS331AP and all wrong addresses explicitly.
        bool ok=match(a_.accel,0x0f,0x32)&&match(a_.gyro,0x0f,0xd3)
            &&match(a_.mag,0x0f,0x3d)&&match(a_.baro,0x0f,0xbd);
        // LIS331DLH: normal 100 Hz, no high-pass (retain gravity), ±4g, BDU.
        ok=ok&&set(a_.accel,0x20,0x07)&&set(a_.accel,0x21,0x00)
            &&set(a_.accel,0x22,0x00)&&set(a_.accel,0x23,0x90)&&set(a_.accel,0x24,0x00);
        // I3G4250D: nominal 100 Hz /25 Hz bandwidth, ±500 deg/s.
        // Bit7 of CTRL_REG4 is RESERVED ZERO, not a BDU control.
        // Stream FIFO keeps each six-byte sample coherent without exposed DRDY.
        ok=ok&&set(a_.gyro,0x20,0x07)&&set(a_.gyro,0x21,0x00)&&set(a_.gyro,0x22,0x00)
            &&set(a_.gyro,0x23,0x10)&&set(a_.gyro,0x24,0x40)&&set(a_.gyro,0x2e,0x00)&&set(a_.gyro,0x2e,0x40);
        // LIS3MDL: 40 Hz, ultrahigh-performance XYZ, ±4 gauss, BDU.
        ok=ok&&set(a_.mag,0x22,0x03)&&set(a_.mag,0x20,0x78)&&set(a_.mag,0x21,0x00)
            &&set(a_.mag,0x23,0x0c)&&set(a_.mag,0x24,0x40)&&set(a_.mag,0x22,0x00);
        // LPS25HB: 25 Hz, BDU, pressure512/temp64 internal averages, FIFO off.
        ok=ok&&set(a_.baro,0x20,0x00)&&set(a_.baro,0x10,0x0f)&&set(a_.baro,0x21,0x00)
            &&set(a_.baro,0x22,0x00)&&set(a_.baro,0x23,0x00)&&set(a_.baro,0x20,0xc4);
        ok=ok&&set(a_.accel,0x20,0x2f)&&set(a_.gyro,0x20,0x1f);
        if(!ok) fault_|=ConfigurationFault;
        return ok;
    }
    bool poll() {
        data_.accel.fresh=data_.gyro.fresh=data_.mag.fresh=data_.pressure_fresh=false;
        if(fault_) return false;
        if(!vector(a_.accel,0,data_.accel,0)) return false;
        uint8_t fifo=0; const uint64_t before=bus_.nowUs();
        if(!read(a_.gyro,0x2f,&fifo,1)) return false;
        if(fifo&0x20) negative_[1]=before;
        else {
            if((fifo&0x40) || (fifo&0x1f)>1) { fault_|=Overrun; return false; }
            uint8_t bytes[6]; if(!read(a_.gyro,0x28,bytes,6)) return false;
            const uint64_t observed=bus_.nowUs();
            for(int i=0;i<3;++i) {
                const int16_t raw=signed16(bytes+2*i);
                // ±500 deg/s is reached before signed ADC rail: 500/0.0175=28571.
                if(std::abs(int(raw))>=28400) { fault_|=Clipping; return false; }
                data_.gyro.value[i]=gyro500(raw);
            }
            data_.gyro.time_us=estimateTime(1,observed,data_.gyro.uncertainty_us);
            data_.gyro.fresh=true;
            uint8_t t; if(!read(a_.gyro,0x26,&t,1)) return false;
            data_.gyro_temperature_raw=t<128?int8_t(t):int8_t(int(t)-256);
        }
        if(!vector(a_.mag,2,data_.mag,1)) return false;
        uint8_t status=0; const uint64_t pbefore=bus_.nowUs();
        if(!read(a_.baro,0x27,&status,1)) return false;
        if(!(status&2)) negative_[3]=pbefore;
        else {
            uint8_t bytes[5]; if(!read(a_.baro,0x28,bytes,5)) return false;
            if(status&0x20) ++data_.auxiliary_overruns;
            else {
                const uint32_t raw=uint32_t(bytes[0])|(uint32_t(bytes[1])<<8)|(uint32_t(bytes[2])<<16);
                data_.pressure_hpa=raw/4096.; data_.baro_temperature_c=42.5+signed16(bytes+3)/480.;
                if(data_.pressure_hpa<260 || data_.pressure_hpa>1260) { fault_|=Clipping; return false; }
                data_.pressure_time_us=estimateTime(3,bus_.nowUs(),data_.pressure_uncertainty_us);
                data_.pressure_fresh=true;
            }
        }
        return fault_==0;
    }
};
} // namespace stimu
