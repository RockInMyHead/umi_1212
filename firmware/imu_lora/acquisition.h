#pragma once
#include "navigation.h"
#include <cmath>
#include <cstdint>

namespace acquisition {
// Call at least once per 2^32 microseconds; the normal loop does so continuously.
class Micros64 {
    uint32_t last_=0; uint64_t total_=0;
public:
    uint64_t update(uint32_t now) { total_+=uint32_t(now-last_); last_=now; return total_; }
};
struct Moments {
    uint32_t n=0; imu::Vec3 mean{},m2{};
    void add(imu::Vec3 v) {
        ++n; const imu::Vec3 d=v-mean; mean=mean+d/double(n);
        for(int i=0;i<3;++i) m2[i]+=d[i]*(v[i]-mean[i]);
    }
    double rms() const { return n>1?std::sqrt((m2.x+m2.y+m2.z)/(n-1)):0; }
};
inline imu::Vec3 corrected(imu::Vec3 raw,double temp,const double m[3][3],imu::Vec3 offset,imu::Vec3 slope,double ref) {
    const imu::Vec3 v=raw-offset-slope*(temp-ref); imu::Vec3 result;
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) result[i]+=m[i][j]*v[j];
    return result;
}
} // namespace acquisition
