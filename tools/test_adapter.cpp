// Portable adapter primitives only: does not simulate Arduino, I2C, IRQ or RF.
#include "acquisition.h"
#include "telemetry.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

int main(int argc,char** argv) {
    acquisition::Micros64 clock;
    assert(clock.update(0xfffffff0u)==0xfffffff0ULL);
    assert(clock.update(0x20u)==0x100000020ULL);
    assert(clock.update(0x1000u)==0x100001000ULL);
    assert(wire::crc32(reinterpret_cast<const uint8_t*>("123456789"),9)==0xcbf43926u);
    acquisition::Moments moments; moments.add({1,2,3}); moments.add({3,4,5});
    assert(std::abs(moments.mean.x-2)<1e-12 && std::abs(moments.rms()-std::sqrt(6))<1e-12);
    imu::Navigator nav; assert(nav.initialize(28800.,{},{1.25,-2.5,9.75}));
    uint8_t out[wire::packet_size];
    wire::encode(out,nav,77,4,wire::Initialized,0,26.5,3,5760000,700);
    assert(out[0]=='I' && out[1]=='N' && out[2]==2 && out[3]==112);
    uint32_t status=0; uint8_t overflow[4]; wire::f32(overflow,1e300,status);
    assert(status&wire::EncodingOverflow);
    if(argc==2) { FILE* f=std::fopen(argv[1],"wb"); assert(f); assert(std::fwrite(out,1,sizeof(out),f)==sizeof(out)); std::fclose(f); }
    std::puts("Adapter primitive checks passed (host only).");
}
