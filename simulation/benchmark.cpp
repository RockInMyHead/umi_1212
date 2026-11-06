// An independent analytic local-frame simulator. No filter functions generate truth.
#include "navigation.h"
#include "algorithm_core.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

namespace {
constexpr double pi=3.14159265358979323846,g=9.80665;
struct V { double x=0,y=0,z=0; double& operator[](int i){return i==0?x:i==1?y:z;}double operator[](int i)const{return i==0?x:i==1?y:z;} };
V operator+(V a,V b){return {a.x+b.x,a.y+b.y,a.z+b.z};}V operator-(V a,V b){return {a.x-b.x,a.y-b.y,a.z-b.z};}V operator*(V a,double b){return {a.x*b,a.y*b,a.z*b};}
double length(V a){return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);}double inner(V a,V b){return a.x*b.x+a.y*b.y+a.z*b.z;}
V vectorProduct(V a,V b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
// Surveyed local ENU at 45 degrees north, with effective local gravity.
const V earth={0,7.292115e-5/std::sqrt(2.0),7.292115e-5/std::sqrt(2.0)};
imu::Vec3 iv(V a){return {a.x,a.y,a.z};}V vv(imu::Vec3 a){return {a.x,a.y,a.z};}
struct Truth { V p,v,a,rpy,rpy_rate; };
struct Smooth {double s,d,dd;};
Smooth smooth(double t,double duration){if(t<=0)return {0,0,0};if(t>=duration)return {1,0,0};const double u=t/duration;return {u*u*u*(10+u*(-15+6*u)),30*u*u*(1-u)*(1-u)/duration,60*u*(1-u)*(1-2*u)/(duration*duration)};}
Truth truth(int scenario,double t){
    Truth r;
    if(scenario==0){r.rpy={.25,-.20,.4};return r;}
    if(scenario==1||scenario==6){
        const long cycle=static_cast<long>(std::floor(t/120));const double local=t-120*cycle;
        const Smooth h=smooth(local-20,80);const double sign=(cycle%2)?-1:1,offset=(cycle%2)?1:0;
        const V span={20,10,2},angle={.18,-.12,.6};
        r.p=span*(offset+sign*h.s);r.v=span*(sign*h.d);r.a=span*(sign*h.dd);
        r.rpy=angle*(offset+sign*h.s);r.rpy_rate=angle*(sign*h.d);return r;
    }
    const double w=.15;const Smooth h=smooth(t,30);
    V p={8*sin(w*t),6*(cos(w*t)-1),.6*sin(2*w*t)};
    V v={8*w*cos(w*t),-6*w*sin(w*t),1.2*w*cos(2*w*t)};
    V a={-8*w*w*sin(w*t),-6*w*w*cos(w*t),-2.4*w*w*sin(2*w*t)};
    const double cadence=2*pi*1.8;
    if(scenario==8){
        p=p+V{.025*sin(cadence*t),.015*(1-cos(cadence*t)),.035*sin(cadence*t)};
        v=v+V{.025*cadence*cos(cadence*t),.015*cadence*sin(cadence*t),.035*cadence*cos(cadence*t)};
        a=a+V{-.025*cadence*cadence*sin(cadence*t),.015*cadence*cadence*cos(cadence*t),-.035*cadence*cadence*sin(cadence*t)};
    }
    r.p=p*h.s;r.v=v*h.s+p*h.d;r.a=a*h.s+v*(2*h.d)+p*h.dd;
    V ang={.20*sin(.071*t),.16*sin(.053*t),.6*sin(.037*t)};
    V rate={.20*.071*cos(.071*t),.16*.053*cos(.053*t),.6*.037*cos(.037*t)};
    if(scenario==8){
        ang=ang+V{.10*sin(cadence*t),.08*(sin(cadence*t+.4)-sin(.4)),.06*(sin(cadence*t-.3)+sin(.3))};
        rate=rate+V{.10*cadence*cos(cadence*t),.08*cadence*cos(cadence*t+.4),.06*cadence*cos(cadence*t-.3)};
    }
    r.rpy=ang*h.s;r.rpy_rate=rate*h.s+ang*h.d;return r;
}
bool verifiedRest(int scenario,double t,double dt){if(scenario==0)return true;if(scenario!=1&&scenario!=6)return false;const double local=t-120*floor(t/120);return local>=dt&&((local<=20)||(local>=100+dt));}
std::array<double,9> matrix(V e){
    const double cr=cos(e.x),sr=sin(e.x),cp=cos(e.y),sp=sin(e.y),cy=cos(e.z),sy=sin(e.z);
    return {cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,-sp,cp*sr,cp*cr};
}
V transposeTimes(const std::array<double,9>& R,V v){return {R[0]*v.x+R[3]*v.y+R[6]*v.z,R[1]*v.x+R[4]*v.y+R[7]*v.z,R[2]*v.x+R[5]*v.y+R[8]*v.z};}
imu::Quat quaternion(V e){const double cr=cos(e.x/2),sr=sin(e.x/2),cp=cos(e.y/2),sp=sin(e.y/2),cy=cos(e.z/2),sy=sin(e.z/2);return {cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy};}
V angularVelocity(const Truth& t){return {t.rpy_rate.x-t.rpy_rate.z*sin(t.rpy.y),t.rpy_rate.y*cos(t.rpy.x)+t.rpy_rate.z*sin(t.rpy.x)*cos(t.rpy.y),-t.rpy_rate.y*sin(t.rpy.x)+t.rpy_rate.z*cos(t.rpy.x)*cos(t.rpy.y)};}
void validateTruth(){
    const double eps=1e-4;
    for(int scenario=0;scenario<9;++scenario)for(double t:{1.0,19.0,20.3,29.2,48.0,99.2,100.3,119.9,150.0}){
        const auto a=truth(scenario,t-eps),b=truth(scenario,t+eps),c=truth(scenario,t);
        if(length((b.p-a.p)*(1/(2*eps))-c.v)>2e-6||length((b.v-a.v)*(1/(2*eps))-c.a)>2e-6)throw std::runtime_error("Analytic position derivatives inconsistent");
        const auto A=matrix(a.rpy),B=matrix(b.rpy),R=matrix(c.rpy);const V w=angularVelocity(c);
        const double skew[3][3]={{0,-w.z,w.y},{w.z,0,-w.x},{-w.y,w.x,0}};
        for(int i=0;i<3;++i)for(int j=0;j<3;++j){double expected=0;for(int k=0;k<3;++k)expected+=R[3*i+k]*skew[k][j];if(std::abs((B[3*i+j]-A[3*i+j])/(2*eps)-expected)>5e-7)throw std::runtime_error("Analytic angular rate inconsistent");}
    }
}
void intervalSignals(int scenario,double time,double dt,V& accel,V& gyro){
    // Three-point Gaussian quadrature: interval-average body measurements.
    const double a=sqrt(3.0/5.0);const double nodes[3]={-a,0,a},weights[3]={5.0/18,4.0/9,5.0/18};
    accel={};gyro={};for(int j=0;j<3;++j){const Truth tr=truth(scenario,time-dt/2+dt*nodes[j]/2);const auto R=matrix(tr.rpy);accel=accel+transposeTimes(R,tr.a+V{0,0,g}+vectorProduct(earth,tr.v)*2)*weights[j];gyro=gyro+(angularVelocity(tr)+transposeTimes(R,earth))*weights[j];}
}
std::pair<double,double> attitudeErrors(imu::Quat q,V actual){
    // Compare gravity in body coordinates (tilt), and local horizontal heading.
    const V z={2*(q.x*q.z-q.w*q.y),2*(q.y*q.z+q.w*q.x),1-2*(q.x*q.x+q.y*q.y)};
    const auto R=matrix(actual);const V tz={R[6],R[7],R[8]};
    const double tilt=acos(std::clamp(inner(z,tz)/std::max(1e-30,length(z)*length(tz)),-1.0,1.0))*180/pi;
    const double yaw=atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    return {tilt,std::abs(std::remainder(yaw-actual.z,2*pi))*180/pi};
}
struct Metrics {
    double final_error=0,max_error=0,sum_sq_error=0,final_tilt=0,final_yaw=0,sum_sq_tilt=0,sum_sq_yaw=0,estimate_path=0,truth_path=0;
    double final_horizontal=0,final_vertical=0,sum_sq_horizontal=0,sum_sq_vertical=0;
    uint64_t count=0,covered=0,failures=0,mag_accepted=0,mag_rejected=0,baro_accepted=0,baro_rejected=0;
    void add(V p,imu::Quat q,const Truth& t,double path,double true_path,const imu::Navigator* n){
        final_error=length(p-t.p);max_error=std::max(max_error,final_error);sum_sq_error+=final_error*final_error;
        final_horizontal=std::hypot(p.x-t.p.x,p.y-t.p.y);final_vertical=std::abs(p.z-t.p.z);sum_sq_horizontal+=final_horizontal*final_horizontal;sum_sq_vertical+=final_vertical*final_vertical;
        const auto errors=attitudeErrors(q,t.rpy);final_tilt=errors.first;final_yaw=errors.second;sum_sq_tilt+=final_tilt*final_tilt;sum_sq_yaw+=final_yaw*final_yaw;estimate_path=path;truth_path=true_path;++count;
        if(n){const V e=p-t.p,sigma=vv(n->positionSigma());if(std::abs(e.x)<=3*sigma.x&&std::abs(e.y)<=3*sigma.y&&std::abs(e.z)<=3*sigma.z)++covered;}
        if(!std::isfinite(final_error)||!std::isfinite(path)||!std::isfinite(final_tilt))++failures;
    }
};
const char* names[]={"stationary_verified","stop_go_verified","continuous_imu_only","continuous_position_1hz","continuous_position_outages_outliers","continuous_10dof_only","stop_go_10dof_verified","continuous_10dof_position_1hz","walking_10dof_no_stance"};
bool finalCovarianceCholesky(const imu::Navigator& n){
    double L[imu::N][imu::N]{};
    for(int i=0;i<imu::N;++i)for(int j=0;j<=i;++j){
        const double scale=std::sqrt(n.covariance(i,i))*std::sqrt(n.covariance(j,j));
        if(!(scale>0)||!std::isfinite(scale))return false;
        double v=n.covariance(i,j)/scale;
        for(int k=0;k<j;++k)v-=L[i][k]*L[j][k];
        if(i==j){if(!(v>0)||!std::isfinite(v))return false;L[i][j]=sqrt(v);}else L[i][j]=v/L[j][j];
    }
    return true;
}
void row(std::ostream& o,const char* scenario,int seed,const char* estimator,double hours,double dt,const Metrics& m,const imu::Navigator* n,bool mag_systematics){
    o<<scenario<<','<<seed<<','<<estimator<<','<<hours<<','<<dt<<','<<m.count<<','<<m.final_error<<','<<m.max_error<<','<<sqrt(m.sum_sq_error/m.count)<<','<<m.final_tilt<<','<<sqrt(m.sum_sq_tilt/m.count)<<','<<m.final_yaw<<','<<sqrt(m.sum_sq_yaw/m.count)<<','<<m.truth_path<<','<<m.estimate_path<<','<<m.estimate_path-m.truth_path<<',';
    if(n)o<<double(m.covered)/m.count;else o<<"nan";
    o<<','<<m.failures<<','<<(n?n->state().fault:0)<<','<<(n?n->state().accepted_updates:0)<<','<<(n?n->state().rejected_updates:0)<<',';
    if(n)o<<finalCovarianceCholesky(*n);else o<<"nan";
    o<<','<<m.final_horizontal<<','<<m.final_vertical<<','<<sqrt(m.sum_sq_horizontal/m.count)<<','<<sqrt(m.sum_sq_vertical/m.count)<<','<<m.mag_accepted<<','<<m.mag_rejected<<','<<m.baro_accepted<<','<<m.baro_rejected<<",st10dof_calibrated_synthetic,"<<mag_systematics<<'\n';
}
void run(int scenario,int seed,double hours,double dt,std::ostream& summary,std::ostream& series,bool mag_systematics){
    // The seed and stream are the same for scenarios 2 and 3. Position noise has
    // a separate stream, so aiding cannot change any IMU realization.
    std::mt19937_64 rng(seed),aid_rng(seed+99991),mag_rng(seed+333331),baro_rng(seed+777773);
    std::normal_distribution<double> gaussian(0,1),aid_gaussian(0,1),mag_gaussian(0,1),baro_gaussian(0,1);
    imu::Config config;config.max_dt=std::max(.05,dt*1.01);config.earth_rate_nav=iv(earth);config.gyro_noise=.0006;
    if(scenario>=5)config.magnetic_field_nav={0,20,-40};
    imu::Navigator nav(config);
    const auto initial=truth(scenario,0);const auto q0=quaternion(initial.rpy);nav.initialize(0,q0,iv(initial.p),iv(initial.v));
    reset_navigation();reset_data_filtration();Quaternion baseline_q={float(q0.w),float(q0.x),float(q0.y),float(q0.z)};NavigationState baseline{};
    V ba={.006,-.004,.008},bg={.00003,-.00002,.00004};const V slope_a={.0008,-.0005,.0010},slope_g={.000003,-.000002,.000002};
    const double accel_density=.004,gyro_density=.0006,accel_walk=.00002,gyro_walk=.0000002;
    // ST10DOF: LIS331DLH 12-bit +/-4g; I3G4250D +/-500dps.
    const double aq=g*.002,gq=(pi/180)*.0175;Metrics current,old;double true_path=0;
    const uint64_t samples=static_cast<uint64_t>(std::llround(hours*3600/dt)),aid_every=static_cast<uint64_t>(std::llround(1/dt)),log_every=static_cast<uint64_t>(std::llround(60/dt));
    const uint64_t mag_every=static_cast<uint64_t>(std::llround(.1/dt)),baro_every=static_cast<uint64_t>(std::llround(.04/dt));
    for(uint64_t k=1;k<=samples;++k){
        const double time=k*dt,temperature=25+8*(1-exp(-time/1800))+2*sin(2*pi*time/7200);
        V accel,gyro;intervalSignals(scenario,time,dt,accel,gyro);
        for(int j=0;j<3;++j){ba[j]+=accel_walk*sqrt(dt)*gaussian(rng);bg[j]+=gyro_walk*sqrt(dt)*gaussian(rng);accel[j]=aq*std::round((accel[j]+ba[j]+slope_a[j]*(temperature-25)+accel_density/std::sqrt(dt)*gaussian(rng))/aq);gyro[j]=gq*std::round((gyro[j]+bg[j]+slope_g[j]*(temperature-25)+gyro_density/std::sqrt(dt)*gaussian(rng))/gq);}
        const bool ok=nav.predict({time,iv(accel),iv(gyro),temperature,false},verifiedRest(scenario,time,dt));if(!ok)++current.failures;
        const Truth tr=truth(scenario,time);const auto mid=truth(scenario,time-dt/2);true_path+=length(mid.v)*dt;
        if((scenario==3||scenario==4||scenario==7)&&k%aid_every==0){
            V p=tr.p+V{.2*aid_gaussian(aid_rng),.2*aid_gaussian(aid_rng),.2*aid_gaussian(aid_rng)};
            const bool outage=scenario==4&&std::fmod(time,7200)>=3600&&std::fmod(time,7200)<3660;
            if(scenario==4&&(k/aid_every)%1000==0)p=p+V{100,-100,100};
            if(!outage)nav.updatePosition(iv(p),{.04,.04,.04},time);
        }
        if(scenario>=5&&k%mag_every==0){
            const V ideal=transposeTimes(matrix(tr.rpy),{0,20,-40});
            V measured={1.002*ideal.x+.001*ideal.y-.0005*ideal.z+.1,
                        .001*ideal.x+.998*ideal.y+.0007*ideal.z-.08,
                        -.0005*ideal.x+.0007*ideal.y+1.001*ideal.z+.06};
            measured=mag_systematics?measured+V{.005,-.004,.003}*(temperature-25):ideal;
            const bool disturbance=std::fmod(time,3600)>=1800&&std::fmod(time,3600)<1810;
            if(disturbance)measured=measured+V{30,-20,25};
            const double quantum=100.0/6842;
            for(int j=0;j<3;++j)measured[j]=quantum*std::round((measured[j]+.4*mag_gaussian(mag_rng))/quantum);
            if(nav.updateMagnetometer(iv(measured),{.16,.16,.16},time))++current.mag_accepted;else ++current.mag_rejected;
        }
        if(scenario>=5&&k%baro_every==0){
            // Standard-atmosphere inverse pair, plus weather pressure drift.
            const double p0=1013.25,scale=44330.0,exponent=5.255;
            const double weather_hpa=-.04*time/3600+.05*sin(2*pi*time/21600);
            double pressure=p0*std::pow(1-tr.p.z/scale,exponent)+weather_hpa+.018*baro_gaussian(baro_rng);
            pressure=std::round(pressure*4096)/4096;
            const double altitude=scale*(1-std::pow(pressure/p0,1/exponent));
            if(nav.updateBarometer(altitude,.15*.15,time))++current.baro_accepted;else ++current.baro_rejected;
        }
        const Vector3D af={float(accel.x),float(accel.y),float(accel.z)},gf={float(gyro.x),float(gyro.y),float(gyro.z)};
        execute_data_filtration(&baseline_q,af,gf,float(dt));baseline=execute_navigation(af,gf,baseline_q,float(dt));
        current.add(vv(nav.state().position),nav.state().orientation,tr,nav.state().travelled_distance_m,true_path,&nav);
        old.add({baseline.position_m.x,baseline.position_m.y,baseline.position_m.z},{baseline_q.w,baseline_q.x,baseline_q.y,baseline_q.z},tr,baseline.total_distance_m,true_path,nullptr);
        const uint64_t phase=k%(7200*aid_every),start=3600*aid_every,end=3660*aid_every;
        const bool outage_boundary=scenario==4&&(phase==start-1||phase==start||phase==end-1||phase==end);
        if(k%log_every==0||k==samples||outage_boundary){
            const auto sigma=nav.positionSigma();series<<names[scenario]<<','<<seed<<','<<time<<','<<tr.p.x<<','<<tr.p.y<<','<<tr.p.z<<','<<nav.state().position.x<<','<<nav.state().position.y<<','<<nav.state().position.z<<','<<baseline.position_m.x<<','<<baseline.position_m.y<<','<<baseline.position_m.z<<','<<current.final_error<<','<<old.final_error<<','<<sigma.x<<','<<sigma.y<<','<<sigma.z<<','<<current.final_tilt<<','<<current.final_yaw<<','<<nav.state().stationary<<'\n';
        }
    }
    row(summary,names[scenario],seed,"new_eskf",hours,dt,current,&nav,mag_systematics);row(summary,names[scenario],seed,"cached_baseline",hours,dt,old,nullptr,mag_systematics);summary.flush();series.flush();
    std::cout<<names[scenario]<<" seed="<<seed<<" new final="<<current.final_error<<" m rms="<<sqrt(current.sum_sq_error/current.count)<<" m old final="<<old.final_error<<" m faults="<<nav.state().fault<<std::endl;
}
}
int main(int argc,char**argv){
    double hours=8,dt=.02;int seeds=3,seed_start=1,only=-1,mag_systematics=1;std::string out="simulation/results";
    for(int i=1;i<argc;++i){const std::string arg=argv[i];if(i+1>=argc)throw std::runtime_error("Every option needs a value");const std::string value=argv[++i];if(arg=="--hours")hours=std::stod(value);else if(arg=="--dt")dt=std::stod(value);else if(arg=="--seeds")seeds=std::stoi(value);else if(arg=="--seed-start")seed_start=std::stoi(value);else if(arg=="--scenario")only=std::stoi(value);else if(arg=="--output")out=value;else if(arg=="--mag-systematics")mag_systematics=std::stoi(value);else throw std::runtime_error("Unknown option: "+arg);}
    if(hours<=0||dt<=0||dt>.02||seeds<1||only>8||only< -1||std::abs(.04/dt-std::round(.04/dt))>1e-8||std::abs(.1/dt-std::round(.1/dt))>1e-8)throw std::runtime_error("Need hours>0, 0<dt<=.02 dividing .04 and .1 seconds, seeds>=1, scenario 0..8");
    if(mag_systematics!=0&&mag_systematics!=1)throw std::runtime_error("--mag-systematics must be 0 or 1");
    if(seed_start<1||seed_start>1000000||seeds>1000000-seed_start)throw std::runtime_error("Seed range must be positive and below one million");
    validateTruth();std::filesystem::create_directories(out);std::ofstream summary(out+"/summary.csv"),series(out+"/timeseries.csv");if(!summary||!series)throw std::runtime_error("Cannot open output CSVs");summary<<std::setprecision(12);series<<std::setprecision(12);
    summary<<"scenario,seed,estimator,hours,dt_s,samples,final_position_error_m,max_position_error_m,rms_position_error_m,final_tilt_error_deg,rms_tilt_error_deg,final_yaw_error_deg,rms_yaw_error_deg,truth_path_m,estimated_path_m,path_error_m,coverage_xyz_3sigma,failure_samples,fault_bits,accepted_updates,rejected_updates,final_covariance_cholesky_ok,final_horizontal_error_m,final_vertical_error_m,rms_horizontal_error_m,rms_vertical_error_m,mag_accepted,mag_rejected,baro_accepted,baro_rejected,sensor_profile,mag_systematics\n";
    series<<"scenario,seed,time_s,true_x,true_y,true_z,new_x,new_y,new_z,baseline_x,baseline_y,baseline_z,new_position_error_m,baseline_position_error_m,sigma_x,sigma_y,sigma_z,new_tilt_deg,new_yaw_deg,stationary\n";
    for(int scenario=0;scenario<9;++scenario)if(only<0||only==scenario)for(int seed=seed_start;seed<seed_start+seeds;++seed)run(scenario,seed,hours,dt,summary,series,mag_systematics);
}
