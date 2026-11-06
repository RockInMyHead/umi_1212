#include "navigation.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {
int failures=0,checks=0;
void check(bool ok,const std::string& name) { ++checks; if(!ok) { ++failures; std::cerr<<"FAIL: "<<name<<'\n'; } }
void near(double a,double b,double tol,const std::string& name) { check(std::isfinite(a)&&std::abs(a-b)<=tol,name+" got="+std::to_string(a)+" expected="+std::to_string(b)); }
void nearVec(imu::Vec3 a,imu::Vec3 b,double tol,const std::string& name) { near(a.x,b.x,tol,name+" x");near(a.y,b.y,tol,name+" y");near(a.z,b.z,tol,name+" z"); }
imu::Quat euler(double r,double p,double y) { const double cr=cos(r/2),sr=sin(r/2),cp=cos(p/2),sp=sin(p/2),cy=cos(y/2),sy=sin(y/2);return {cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy}; }
bool psd(const imu::Navigator& n) {
    double L[imu::N][imu::N]{};
    for(int i=0;i<imu::N;++i) for(int j=0;j<=i;++j) {
        if(!std::isfinite(n.covariance(i,j))||std::abs(n.covariance(i,j)-n.covariance(j,i))>1e-9) return false;
        double v=n.covariance(i,j)+(i==j?1e-14:0);
        for(int k=0;k<j;++k) v-=L[i][k]*L[j][k];
        if(i==j) { if(v<=0) return false; L[i][j]=sqrt(v); } else L[i][j]=v/L[j][j];
    }
    return true;
}
imu::Sample rest(double t,imu::Quat q={}) { return {t,imu::rotate(imu::conjugate(q),{0,0,9.80665}),{},25,false}; }
void conventions() {
    const double pi=acos(-1.0);
    nearVec(imu::rotate(imu::expRotation({pi/2,0,0}),{0,1,0}),{0,0,1},1e-12,"Hamilton active x rotation");
    nearVec(imu::rotate(imu::expRotation({0,0,pi/2}),{1,0,0}),{0,1,0},1e-12,"Hamilton active yaw rotation");
    for(double roll:{-1.2,-0.3,0.6,1.3}) for(double pitch:{-0.8,0.0,0.7}) {
        const imu::Quat q=euler(roll,pitch,0);
        const imu::Vec3 f=imu::rotate(imu::conjugate(q),{0,0,9.80665});
        const imu::Quat a=imu::gravityAlignment(f);
        nearVec(imu::rotate(a,f),{0,0,9.80665},1e-10,"gravityAlignment maps measured gravity up");
    }
    const imu::Quat q=euler(.47,-.31,.8);
    imu::Navigator n;check(n.initialize(0,q),"initialize tilted");
    for(int i=1;i<=1000;++i) check(n.predict(rest(i*.01,q)),"tilted rest predict");
    nearVec(n.state().position,{},1e-9,"tilted rest position");
    nearVec(n.state().velocity,{},1e-10,"tilted rest velocity");
}
void propagation() {
    imu::Navigator n;check(n.initialize(0),"initialize small acceleration");
    for(int i=1;i<=2000;++i) check(n.predict({i*.01,{.01,-.02,9.80665},{},25,false}),"small acceleration predict");
    nearVec(n.state().position,{2,-4,0},1e-8,"acceleration below historical dead zone survives");
    nearVec(n.state().velocity,{.2,-.4,0},1e-9,"small acceleration velocity");
    imu::Navigator v;check(v.initialize(0,{}, {},{3,-2,.5}),"initialize moving");
    for(int i=1;i<=2000;++i) check(v.predict(rest(i*.01)),"constant velocity predict");
    nearVec(v.state().position,{60,-40,10},1e-8,"quiet constant velocity does not imply rest");
    check(!v.state().stationary,"no ZUPT without independent rest evidence");
    imu::Navigator spin;check(spin.initialize(0),"initialize spin");
    for(int i=1;i<=1000;++i) check(spin.predict({i*.01,{0,0,9.80665},{0,0,.3},25,false}),"spin predict");
    nearVec(imu::rotate(spin.state().orientation,{1,0,0}),{cos(3),sin(3),0},1e-10,"exact constant rate quaternion integration");
    nearVec(spin.state().position,{},1e-8,"yaw rotation creates no acceleration");
}
void failStop() {
    const double nan=std::numeric_limits<double>::quiet_NaN();
    for(int mode=0;mode<5;++mode) {
        imu::Navigator n;check(n.initialize(1),"initialize fault test");
        imu::Sample s=rest(1.01);
        if(mode==0)s.time_s=1;if(mode==1)s.time_s=1.2;if(mode==2)s.accel.x=nan;if(mode==3)s.saturated=true;if(mode==4)s.temperature_c=nan;
        check(!n.predict(s),"bad input rejected mode "+std::to_string(mode));
        check(n.state().fault!=0,"fault latched mode "+std::to_string(mode));
        nearVec(n.state().position,{},0,"invalid sample does not change position");
        check(!n.predict(rest(1.02)),"latched fault blocks further prediction");
    }
}
void aidingAndCovariance() {
    imu::Navigator n;check(n.initialize(0),"initialize aiding");
    check(!n.updatePosition({},{1,1,1},-.01),"delayed position rejected");
    check(!n.updateVelocity({},{1,1,1},.01),"future velocity rejected");
    check(!n.updatePosition({},{0,1,1},0),"zero measurement variance rejected");
    check(!n.updatePosition({1000,0,0},{.01,.01,.01},0),"position outlier gated");
    nearVec(n.state().position,{},0,"outlier leaves position unchanged");
    check(n.updatePosition({.02,-.01,.01},{.01,.01,.01},0),"small position innovation accepted");
    check(psd(n),"PSD after gated/accepted observations");
    for(int i=1;i<=3000;++i) {
        check(n.predict(rest(i*.01),true),"stationary aided predict");
        if(i%50==0) {
            check(n.updatePosition({},{.01,.01,.01},i*.01),"periodic position update");
            check(psd(n),"Joseph/reset covariance symmetric positive definite");
        }
    }
    check(n.state().stationary,"verified quiet rest activates stationary update");
    check(n.state().accepted_updates>20,"rest generates updates");
    // Independent first-order covariance oracle for dv/dtheta = -[g]x.
    imu::Config c;c.accel_noise=0;c.gyro_noise=0;c.accel_bias_walk=0;c.gyro_bias_walk=0;
    imu::Navigator d(c);check(d.initialize(0),"initialize covariance derivative");
    const double dt=1e-6;check(d.predict(rest(dt)),"infinitesimal covariance predict");
    near(d.covariance(3,7)/dt,c.gravity*c.tilt_initial_sigma*c.tilt_initial_sigma,1e-7,"dvx/dtheta_y covariance sign");
    near(d.covariance(4,6)/dt,-c.gravity*c.tilt_initial_sigma*c.tilt_initial_sigma,1e-7,"dvy/dtheta_x covariance sign");
    near(d.covariance(0,3)/dt,c.velocity_initial_sigma*c.velocity_initial_sigma,1e-7,"position velocity cross covariance");
}
}
int main() { conventions();propagation();failStop();aidingAndCovariance();std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0; }
