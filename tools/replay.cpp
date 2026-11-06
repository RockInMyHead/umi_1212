// Replays timestamped IMU and synchronous aiding through the actual embedded core.
#include "navigation.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>

namespace {
std::vector<std::string> split(const std::string& s){std::vector<std::string> out;std::stringstream in(s);std::string v;while(std::getline(in,v,','))out.push_back(v);if(!s.empty()&&s.back()==',')out.push_back("");return out;}
double value(const std::string& s){size_t pos;double v=std::stod(s,&pos);if(pos!=s.size()||!std::isfinite(v))throw std::runtime_error("Nonfinite or malformed value: "+s);return v;}
bool flag(const std::string& s){if(s=="0")return false;if(s=="1")return true;throw std::runtime_error("Flags must be 0 or 1");}
imu::Vec3 vector(const std::vector<std::string>& v,int i){return {value(v[i]),value(v[i+1]),value(v[i+2])};}
}
int main(int argc,char** argv){try{
    if(argc!=4){std::cerr<<"Usage: replay initial_state.csv samples.csv estimates.csv\nSee docs/REPLAY.md. Inputs must already be sensor-calibrated.\n";return 2;}
    std::ifstream initial(argv[1]),samples(argv[2]);if(!initial||!samples)throw std::runtime_error("Cannot open input");
    std::string line;std::getline(initial,line);std::getline(initial,line);auto s=split(line);
    if(s.size()!=23&&s.size()!=26)throw std::runtime_error("Initial state needs 23 or26 columns");
    imu::Config config;config.gravity=value(s[19]);config.earth_rate_nav=vector(s,20);
    if(s.size()==26)config.magnetic_field_nav=vector(s,23);
    imu::Navigator nav(config);
    if(!nav.initialize(value(s[0]),{value(s[1]),value(s[2]),value(s[3]),value(s[4])},vector(s,5),vector(s,8),vector(s,11),vector(s,14)))throw std::runtime_error("Invalid initial state");
    // This utility uses documented core priors; edit Config for characterized noise.
    if(value(s[17])!=1||value(s[18])!=1)throw std::runtime_error("Initial attitude/origin must be declared valid (columns17,18 = 1)");
    std::ofstream out(argv[3]);if(!out)throw std::runtime_error("Cannot open output");
    out<<std::setprecision(15)<<"time_s,px,py,pz,vx,vy,vz,qw,qx,qy,qz,sigma_px,sigma_py,sigma_pz,quiet,stationary,fault,accepted,rejected,path_m,barometer_bias_m\n";
    const std::string base="time_s,ax,ay,az,gx,gy,gz,temperature_c,verified_stationary,saturated,px,py,pz,position_sigma,vx,vy,vz,velocity_sigma";
    if(!std::getline(samples,line)||(line!=base&&line!=base+",mx,my,mz,mag_sigma,baro_altitude,baro_sigma"))throw std::runtime_error("Unexpected samples header");
    const size_t columns=line==base?18:24;
    size_t rows=0;
    while(std::getline(samples,line)){
        if(line.empty())continue;auto v=split(line);if(v.size()!=columns)throw std::runtime_error("Unexpected column count at sample "+std::to_string(rows+1));
        imu::Sample sample{value(v[0]),vector(v,1),vector(v,4),value(v[7]),flag(v[9])};
        if(!nav.predict(sample,flag(v[8])))throw std::runtime_error("Estimator fault at "+v[0]+" bits="+std::to_string(nav.state().fault));
        for(int i:{10,14}){
            bool any=false,all=true;for(int j=i;j<i+4;++j){any|=!v[j].empty();all&=!v[j].empty();}
            if(any&&!all)throw std::runtime_error("Partial aiding vector at "+v[0]);
            if(all){double sigma=value(v[i+3]);if(sigma<=0)throw std::runtime_error("Aiding sigma must be positive");imu::Vec3 var{sigma*sigma,sigma*sigma,sigma*sigma};if(i==10)nav.updatePosition(vector(v,i),var,sample.time_s);else nav.updateVelocity(vector(v,i),var,sample.time_s);}
        }
        if(columns==24){
            bool any=false,all=true;for(int j=18;j<22;++j){any|=!v[j].empty();all&=!v[j].empty();}
            if(any&&!all)throw std::runtime_error("Partial magnetometer observation");
            if(all){double sigma=value(v[21]);if(sigma<=0)throw std::runtime_error("Mag sigma must be positive");nav.updateMagnetometer(vector(v,18),{sigma*sigma,sigma*sigma,sigma*sigma},sample.time_s);}
            if(v[22].empty()!=v[23].empty())throw std::runtime_error("Partial barometer observation");
            if(!v[22].empty()){double sigma=value(v[23]);if(sigma<=0)throw std::runtime_error("Barometer sigma must be positive");nav.updateBarometer(value(v[22]),sigma*sigma,sample.time_s);}
        }
        const auto& x=nav.state();auto p=nav.positionSigma();
        if(x.fault)throw std::runtime_error("Estimator fault after aiding at "+v[0]+" bits="+std::to_string(x.fault));
        out<<x.time_s<<','<<x.position.x<<','<<x.position.y<<','<<x.position.z<<','<<x.velocity.x<<','<<x.velocity.y<<','<<x.velocity.z<<','<<x.orientation.w<<','<<x.orientation.x<<','<<x.orientation.y<<','<<x.orientation.z<<','<<p.x<<','<<p.y<<','<<p.z<<','<<x.quiet<<','<<x.stationary<<','<<x.fault<<','<<x.accepted_updates<<','<<x.rejected_updates<<','<<x.travelled_distance_m<<','<<x.barometer_bias_m<<'\n';++rows;
    }
    if(!rows)throw std::runtime_error("No samples");
    std::cout<<rows<<" samples replayed; "<<nav.state().rejected_updates<<" aiding observations rejected.\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
