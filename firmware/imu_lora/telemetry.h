#pragma once
#include "navigation.h"
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace wire {
constexpr size_t packet_size=112;
enum Status : uint32_t {
    Initialized=1u, Quiet=2u, VerifiedStationary=4u, Calibrating=8u,
    AcquisitionStopped=16u, RadioFailed=32u, EncodingOverflow=64u,
    WaitingForCalibration=128u, Uncalibrated=256u
};
inline uint32_t crc32(const uint8_t* p,size_t n) {
    uint32_t c=0xffffffffu;
    for(size_t i=0;i<n;++i) { c^=p[i]; for(int j=0;j<8;++j) c=(c>>1)^(0xedb88320u&uint32_t(-int32_t(c&1u))); }
    return ~c;
}
inline void u32(uint8_t* p,uint32_t v) { for(int i=0;i<4;++i) p[i]=uint8_t(v>>(8*i)); }
inline void f32(uint8_t* p,double v,uint32_t& flags) {
    static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559,"IEEE754 float required");
    if(!std::isfinite(v) || std::abs(v)>FLT_MAX) flags|=EncodingOverflow;
    float f=std::isfinite(v)?float(std::fmax(-double(FLT_MAX),std::fmin(double(FLT_MAX),v))):std::numeric_limits<float>::quiet_NaN();
    uint32_t bits; std::memcpy(&bits,&f,4); u32(p,bits);
}
inline void encode(uint8_t* out,const imu::Navigator& nav,uint32_t sequence,uint32_t epoch,
                   uint32_t status,uint32_t acquisition_fault,double temperature,
                   uint32_t dropped,uint32_t sample_count,uint32_t max_compute_us) {
    const auto& s=nav.state();
    out[0]='I'; out[1]='N'; out[2]=2; out[3]=uint8_t(packet_size);
    u32(out+4,sequence); u32(out+8,epoch);
    const uint64_t t=s.time_s>=0 && std::isfinite(s.time_s)?uint64_t(s.time_s*1e6+0.5):0;
    u32(out+12,uint32_t(t)); u32(out+16,uint32_t(t>>32));
    u32(out+24,s.fault|acquisition_fault);
    for(int i=0;i<3;++i) {
        f32(out+28+4*i,s.position[i],status); f32(out+40+4*i,s.velocity[i],status);
        f32(out+52+4*i,nav.covariance(i,i),status); f32(out+64+4*i,nav.covariance(i+3,i+3),status);
    }
    f32(out+76,s.orientation.w,status); f32(out+80,s.orientation.x,status);
    f32(out+84,s.orientation.y,status); f32(out+88,s.orientation.z,status);
    f32(out+92,temperature,status); u32(out+96,dropped); u32(out+100,sample_count);
    u32(out+104,max_compute_us); u32(out+20,status); u32(out+108,crc32(out,108));
}
} // namespace wire
