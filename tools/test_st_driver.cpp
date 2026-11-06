// Host register/conversion checks, not an electrical or hardware timing model.
#include "st_driver.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdio>

struct FakeBus : stimu::Bus {
    uint8_t regs[128][128]{}; uint64_t t=100000;
    FakeBus() { regs[0x18][15]=0x32; regs[0x68][15]=0xd3; regs[0x1c][15]=0x3d; regs[0x5c][15]=0xbd; regs[0x68][0x2f]=0x20; }
    bool read(uint8_t a,uint8_t r,uint8_t* out,size_t n) override { std::memcpy(out,&regs[a][r],n); return true; }
    bool write(uint8_t a,uint8_t r,uint8_t v) override { regs[a][r]=v; return true; }
    uint64_t nowUs() override { return t; }
    void word(int a,int r,int16_t v) { regs[a][r]=uint8_t(v); regs[a][r+1]=uint8_t(uint16_t(v)>>8); }
};
int main() {
    assert(std::abs(stimu::accel4g(8000)-9.80665)<1e-12);
    assert(std::abs(stimu::accel4g(-8000)+9.80665)<1e-12);
    assert(std::abs(stimu::gyro500(1000)-17.5*stimu::degree_to_rad)<1e-12);
    assert(std::abs(stimu::mag4gauss(6842)-100.)<1e-12);
    const double identity[3][3]={{1,0,0},{0,1,0},{0,0,1}},reflection[3][3]={{1,0,0},{0,1,0},{0,0,-1}};
    assert(stimu::properRotation(identity)&&!stimu::properRotation(reflection));
    assert(stimu::relativeAltitude(1000,1013.25,288.15,9.80665)>100);
    assert(stimu::relativeAltitude(1013.25,1013.25,288.15,9.80665)==0);
    FakeBus bus; stimu::Driver driver(bus); assert(driver.begin());
    assert(bus.regs[0x68][0x23]==0x10); // never set nonexistent gyro BDU bit
    assert(bus.regs[0x18][0x23]==0x90 && bus.regs[0x68][0x2e]==0x40);
    assert(driver.poll()); // establish last not-ready poll timestamps
    bus.t+=1000;
    bus.regs[0x18][0x27]=8; bus.word(0x18,0x2c,8000);
    bus.regs[0x68][0x2f]=1; bus.word(0x68,0x28,1000);
    bus.regs[0x1c][0x27]=8; bus.word(0x1c,0x28,6842);
    bus.regs[0x5c][0x27]=2; const uint32_t p=4150272; // 1013.25 hPa *4096
    for(int i=0;i<3;++i) bus.regs[0x5c][0x28+i]=uint8_t(p>>(8*i));
    assert(driver.poll()); const auto& d=driver.data();
    assert(d.gyro.fresh&&d.accel.fresh&&d.mag.fresh&&d.pressure_fresh);
    assert(d.gyro.time_us==100500 && d.gyro.uncertainty_us==501);
    assert(std::abs(d.pressure_hpa-1013.25)<1e-10);
    assert(std::abs(d.baro_temperature_c-42.5)<1e-10);
    bus.regs[0x68][0x2f]=2; assert(!driver.poll()); assert(driver.fault()&stimu::Overrun);
    FakeBus oldBoard; oldBoard.regs[0x68][15]=0xd0;
    stimu::Driver wrong(oldBoard); assert(!wrong.begin()); assert(wrong.fault()&stimu::ConfigurationFault);
    std::puts("ST register/conversion checks passed (host fake bus only).");
}
