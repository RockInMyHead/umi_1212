#include "navigation.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

// Analytic smooth moving/rotating oracle, independent of the mechanization.
namespace {
constexpr double gravity=9.80665,earth_component=7.292115e-5/1.4142135623730950488;
struct Truth { std::array<double,3> p,v,a,e,de; };
Truth truth(double t){return {{8*sin(.15*t),6*(cos(.15*t)-1),.6*sin(.3*t)},
    {1.2*cos(.15*t),-.9*sin(.15*t),.18*cos(.3*t)},
    {-.18*sin(.15*t),-.135*cos(.15*t),-.054*sin(.3*t)},
    {.2*sin(.071*t),.16*sin(.053*t),.6*sin(.037*t)},
    {.0142*cos(.071*t),.00848*cos(.053*t),.0222*cos(.037*t)}};}
imu::Vec3 vector(const std::array<double,3>& a){return {a[0],a[1],a[2]};}
std::array<double,9> matrix(const Truth& t){const double r=t.e[0],p=t.e[1],y=t.e[2],sr=sin(r),cr=cos(r),sp=sin(p),cp=cos(p),sy=sin(y),cy=cos(y);return {cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,-sp,cp*sr,cp*cr};}
imu::Vec3 transpose(const std::array<double,9>& a,imu::Vec3 v){return {a[0]*v.x+a[3]*v.y+a[6]*v.z,a[1]*v.x+a[4]*v.y+a[7]*v.z,a[2]*v.x+a[5]*v.y+a[8]*v.z};}
imu::Sample sample(double time,double dt){
    const double nodes[3]={-sqrt(3.0/5),0,sqrt(3.0/5)},weights[3]={5.0/18,4.0/9,5.0/18};imu::Sample s;s.time_s=time;
    for(int j=0;j<3;++j){
        const auto t=truth(time-dt/2+dt*nodes[j]/2);const auto R=matrix(t);const double r=t.e[0],p=t.e[1];
        const imu::Vec3 omega={t.de[0]-t.de[2]*sin(p),t.de[1]*cos(r)+t.de[2]*sin(r)*cos(p),-t.de[1]*sin(r)+t.de[2]*cos(r)*cos(p)};
        const imu::Vec3 coriolis={2*earth_component*(t.v[2]-t.v[1]),2*earth_component*t.v[0],-2*earth_component*t.v[0]};
        s.accel=s.accel+transpose(R,vector(t.a)+coriolis+imu::Vec3{0,0,gravity})*weights[j];
        s.gyro=s.gyro+(omega+transpose(R,{0,earth_component,earth_component}))*weights[j];
    }
    return s;
}
double run(double dt){
    imu::Config c;c.earth_rate_nav={0,earth_component,earth_component};imu::Navigator nav(c);
    const auto initial=truth(0);if(!nav.initialize(0,{},vector(initial.p),vector(initial.v)))return -1;
    const int count=static_cast<int>(240/dt+.5);for(int i=1;i<=count;++i)if(!nav.predict(sample(i*dt,dt)))return -1;
    return imu::norm(nav.state().position-vector(truth(count*dt).p));
}
}
int main(){
    const double coarse=run(.02),middle=run(.01),fine=run(.005);
    std::cout<<"Noiseless 240 s moving/rotating Earth-frame final errors (dt .02/.01/.005): "<<coarse<<", "<<middle<<", "<<fine<<" m\n";
    const bool ok=coarse>0&&coarse<.1&&middle>0&&middle<.35*coarse&&fine>0&&fine<.35*middle;
    if(!ok)std::cerr<<"FAIL: deterministic mechanization must converge at second order\n";
    return ok?0:1;
}
