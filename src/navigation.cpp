#include "navigation.h"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <limits>

namespace imu {
namespace {
bool finite(Vec3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }
bool finite(Quat q) { return std::isfinite(q.w)&&std::isfinite(q.x)&&std::isfinite(q.y)&&std::isfinite(q.z); }
double square(double x) { return x*x; }
Vec3 isotropic(double x) { return {x,x,x}; }
void skew(Vec3 v,double s[3][3]) {
    s[0][0]=0;s[0][1]=-v.z;s[0][2]=v.y;
    s[1][0]=v.z;s[1][1]=0;s[1][2]=-v.x;
    s[2][0]=-v.y;s[2][1]=v.x;s[2][2]=0;
}
void rotationMatrix(Quat q,double r[3][3]) {
    for(int j=0;j<3;++j) { Vec3 e{};e[j]=1;Vec3 c=rotate(q,e);for(int i=0;i<3;++i)r[i][j]=c[i]; }
}
void rightJacobian(Vec3 v,double j[3][3]) {
    double s[3][3],a=norm(v);skew(v,s);
    double c1=a<1e-6?0.5-a*a/24:(1-std::cos(a))/(a*a);
    double c2=a<1e-6?1.0/6-a*a/120:(a-std::sin(a))/(a*a*a);
    for(int r=0;r<3;++r)for(int c=0;c<3;++c){double s2=0;for(int k=0;k<3;++k)s2+=s[r][k]*s[k][c];j[r][c]=(r==c?1:0)-c1*s[r][c]+c2*s2;}
}
Vec3 calibrated(Vec3 raw,const double m[3][3],Vec3 bias,Vec3 slope,double dt) {
    Vec3 v=raw-bias-slope*dt,out{};
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)out[i]+=m[i][j]*v[j];
    return out;
}
bool validMatrix(const double m[3][3]) {
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)if(!std::isfinite(m[i][j]))return false;
    double d=m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
    return std::isfinite(d)&&std::abs(d)>1e-9;
}
void magneticMatrix(const State& s,double m[3][3]) {
    const auto& e=s.magnetic_soft_iron;
    m[0][0]=1+e[0];m[1][1]=1+e[1];m[2][2]=1+e[2];
    m[0][1]=m[1][0]=e[3];m[0][2]=m[2][0]=e[4];m[1][2]=m[2][1]=e[5];
}
Vec3 matVec(const double m[3][3],Vec3 v) {
    Vec3 out{};for(int i=0;i<3;++i)for(int j=0;j<3;++j)out[i]+=m[i][j]*v[j];return out;
}
}
Vec3 operator+(Vec3 a,Vec3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
Vec3 operator-(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
Vec3 operator*(Vec3 a,double s){return {a.x*s,a.y*s,a.z*s};}
Vec3 operator/(Vec3 a,double s){return a*(1/s);}
double dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
double norm(Vec3 a){return std::sqrt(dot(a,a));}
Vec3 cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
Quat multiply(Quat a,Quat b){return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};}
Quat conjugate(Quat q){return {q.w,-q.x,-q.y,-q.z};}
Quat normalized(Quat q){double n=std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);if(!(n>1e-15)||!std::isfinite(n))return {std::numeric_limits<double>::quiet_NaN(),0,0,0};return {q.w/n,q.x/n,q.y/n,q.z/n};}
Quat expRotation(Vec3 v){double a=norm(v),s=a<1e-8?0.5-a*a/48:std::sin(a/2)/a;return {std::cos(a/2),s*v.x,s*v.y,s*v.z};}
Vec3 rotate(Quat q,Vec3 v){Vec3 u{q.x,q.y,q.z};return v+cross(u,cross(u,v)+v*q.w)*2;}
Quat gravityAlignment(Vec3 a){
    if(!finite(a)||norm(a)<1e-9)return normalized({0,0,0,0});
    // ZYX Euler convention with zero initial yaw, defined except vertical pitch.
    double roll=std::atan2(a.y,a.z),pitch=std::atan2(-a.x,std::hypot(a.y,a.z));
    return normalized(multiply(expRotation({0,pitch,0}),expRotation({roll,0,0})));
}
Navigator::Navigator(Config config):config_(config){
    const double nonnegative[]={config.accel_noise,config.gyro_noise,config.accel_bias_walk,config.gyro_bias_walk,config.position_initial_sigma,config.velocity_initial_sigma,config.tilt_initial_sigma,config.yaw_initial_sigma,config.accel_bias_initial_sigma,config.gyro_bias_initial_sigma,config.barometer_bias_initial_sigma,config.barometer_bias_walk,config.magnetic_bias_initial_sigma,config.magnetic_bias_walk,config.magnetic_scale_initial_sigma,config.magnetic_scale_walk};
    for(double v:nonnegative)config_valid_ &= std::isfinite(v)&&v>=0;
    const double positive[]={config.gravity,config.max_dt,config.innovation_gate_3d,config.innovation_gate_1d,config.magnetic_relative_norm_tolerance,config.quiet_accel_rms,config.quiet_gyro_rms,config.quiet_gravity_error,config.quiet_window_s,config.quiet_dwell_s,config.stationary_update_s,config.zero_velocity_sigma,config.static_accel_sigma,config.static_gyro_sigma};
    for(double v:positive)config_valid_ &= std::isfinite(v)&&v>0;
    const auto& c=config.calibration;
    config_valid_ &= finite(config.earth_rate_nav)&&finite(config.magnetic_field_nav)&&finite(c.accel_offset)&&finite(c.gyro_offset)&&finite(c.accel_temp_slope)&&finite(c.gyro_temp_slope)&&std::isfinite(c.reference_temperature_c)&&validMatrix(c.accel_matrix)&&validMatrix(c.gyro_matrix);
}
bool Navigator::initialize(double t,Quat q,Vec3 p,Vec3 v,Vec3 ba,Vec3 bg){
    state_={};std::memset(P_,0,sizeof(P_));quiet_elapsed_=quiet_dwell_=stationary_elapsed_=0;
    acc_mean_=gyro_mean_=acc_m2_=gyro_m2_=static_acc_sum_=static_gyro_sum_={};quiet_count_=static_count_=0;
    q=normalized(q);
    if(!config_valid_||!std::isfinite(t)||!finite(q)||!finite(p)||!finite(v)||!finite(ba)||!finite(bg)){state_.fault=InvalidInput;return false;}
    state_.time_s=t;state_.orientation=q;state_.position=p;state_.velocity=v;state_.accel_bias=ba;state_.gyro_bias=bg;state_.initialized=true;
    for(int i=0;i<3;++i){P_[i][i]=square(config_.position_initial_sigma);P_[i+3][i+3]=square(config_.velocity_initial_sigma);P_[i+9][i+9]=square(config_.accel_bias_initial_sigma);P_[i+12][i+12]=square(config_.gyro_bias_initial_sigma);}
    P_[15][15]=square(config_.barometer_bias_initial_sigma);
    for(int i=16;i<19;++i)P_[i][i]=square(config_.magnetic_bias_initial_sigma);
    for(int i=19;i<25;++i)P_[i][i]=square(config_.magnetic_scale_initial_sigma);
    double r[3][3];rotationMatrix(q,r);
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)for(int k=0;k<3;++k)P_[i+6][j+6]+=r[k][i]*r[k][j]*square(k==2?config_.yaw_initial_sigma:config_.tilt_initial_sigma);
    return true;
}
void Navigator::propagateCovariance(Vec3 f,Vec3 omega,Quat midpoint,double dt){
    // Exact frozen-rate attitude blocks avoid artificial covariance inflation during
    // long rotations. Translation retains a small-step local approximation.
    double r[3][3],sf[3][3],se[3][3],ar[3][3],jr[3][3];
    rotationMatrix(midpoint,r);skew(f,sf);skew(config_.earth_rate_nav,se);
    rotationMatrix(expRotation(omega*(-dt)),ar);rightJacobian(omega*dt,jr);
    std::memset(scratch_a_,0,sizeof(scratch_a_));
    for(int i=0;i<N;++i)scratch_a_[i][i]=1;
    for(int i=0;i<3;++i){
        scratch_a_[i][i+3]=dt;
        for(int j=0;j<3;++j){
            double a=0;for(int k=0;k<3;++k)a-=r[i][k]*sf[k][j];
            scratch_a_[i+3][j+6]=a*dt;scratch_a_[i][j+6]=a*dt*dt/2;
            scratch_a_[i+3][j+9]=-r[i][j]*dt;scratch_a_[i][j+9]=-r[i][j]*dt*dt/2;
            scratch_a_[i+6][j+6]=ar[i][j];
            scratch_a_[i+6][j+12]=-jr[i][j]*dt;
            scratch_a_[i+3][j+3]-=2*se[i][j]*dt;
            scratch_a_[i][j+3]-=se[i][j]*dt*dt;
        }
    }
    // Compress the transition's nonzero rows; at most 11 entries in a position row.
    constexpr int max_row_entries=11;
    int count[N]={},indices[N][max_row_entries];double values[N][max_row_entries];
    for(int i=0;i<N;++i)for(int j=0;j<N;++j)if(scratch_a_[i][j]!=0){
        if(count[i]>=max_row_entries){state_.fault|=NumericalFailure;return;}
        int k=count[i]++;indices[i][k]=j;values[i][k]=scratch_a_[i][j];
    }
    std::memset(scratch_b_,0,sizeof(scratch_b_));
    for(int i=0;i<N;++i)for(int k=0;k<count[i];++k){int row=indices[i][k];double a=values[i][k];for(int j=0;j<N;++j)scratch_b_[i][j]+=a*P_[row][j];}
    for(int i=0;i<N;++i)for(int j=0;j<=i;++j){double a=0;for(int k=0;k<count[j];++k)a+=scratch_b_[i][indices[j][k]]*values[j][k];P_[i][j]=P_[j][i]=a;}
    // Isotropic continuous white acceleration: exact integrated p/v noise block.
    const double qa=square(config_.accel_noise),qg=square(config_.gyro_noise);
    for(int i=0;i<3;++i){P_[i][i]+=qa*dt*dt*dt/3;P_[i][i+3]+=qa*dt*dt/2;P_[i+3][i]+=qa*dt*dt/2;P_[i+3][i+3]+=qa*dt;P_[i+6][i+6]+=qg*dt;P_[i+9][i+9]+=square(config_.accel_bias_walk)*dt;P_[i+12][i+12]+=square(config_.gyro_bias_walk)*dt;}
    P_[15][15]+=square(config_.barometer_bias_walk)*dt;
    for(int i=16;i<19;++i)P_[i][i]+=square(config_.magnetic_bias_walk)*dt;
    for(int i=19;i<25;++i)P_[i][i]+=square(config_.magnetic_scale_walk)*dt;
}
void Navigator::updateQuiet(Vec3 a,Vec3 w,double dt){
    // Diagnostic only. Quiet IMU signals cannot establish zero translational velocity.
    if(std::abs(norm(a)-config_.gravity)>config_.quiet_gravity_error*2||norm(w)>config_.quiet_gyro_rms*3){state_.quiet=false;quiet_dwell_=0;}
    ++quiet_count_;quiet_elapsed_+=dt;
    Vec3 da=a-acc_mean_,dw=w-gyro_mean_;
    acc_mean_=acc_mean_+da/double(quiet_count_);gyro_mean_=gyro_mean_+dw/double(quiet_count_);
    for(int i=0;i<3;++i){acc_m2_[i]+=da[i]*(a[i]-acc_mean_[i]);gyro_m2_[i]+=dw[i]*(w[i]-gyro_mean_[i]);}
    if(quiet_elapsed_+1e-9<config_.quiet_window_s)return;
    double av=(acc_m2_.x+acc_m2_.y+acc_m2_.z)/quiet_count_,gv=(gyro_m2_.x+gyro_m2_.y+gyro_m2_.z)/quiet_count_+dot(gyro_mean_,gyro_mean_);
    bool quiet=av<square(config_.quiet_accel_rms)&&gv<square(config_.quiet_gyro_rms)&&std::abs(norm(acc_mean_)-config_.gravity)<config_.quiet_gravity_error;
    quiet_dwell_=quiet?quiet_dwell_+quiet_elapsed_:0;state_.quiet=quiet_dwell_+1e-9>=config_.quiet_dwell_s;
    quiet_count_=0;quiet_elapsed_=0;acc_mean_=gyro_mean_=acc_m2_=gyro_m2_={};
}
bool Navigator::predict(const Sample& s,bool verified_stationary){
    if(!state_.initialized||state_.fault)return false;
    if(!std::isfinite(s.time_s)||!finite(s.accel)||!finite(s.gyro)||!std::isfinite(s.temperature_c)){state_.fault|=InvalidInput;state_.stationary=false;return false;}
    double dt=s.time_s-state_.time_s;
    if(!(dt>0)||dt>config_.max_dt){state_.fault|=TimeGap;state_.stationary=false;return false;}
    if(s.saturated){state_.fault|=Saturation;state_.stationary=false;return false;}
    const auto& c=config_.calibration;double td=s.temperature_c-c.reference_temperature_c;
    Vec3 a=calibrated(s.accel,c.accel_matrix,c.accel_offset,c.accel_temp_slope,td),w=calibrated(s.gyro,c.gyro_matrix,c.gyro_offset,c.gyro_temp_slope,td);
    Vec3 f=a-state_.accel_bias,omega=w-state_.gyro_bias;
    if(!finite(f)||!finite(omega)){state_.fault|=InvalidInput;return false;}
    updateQuiet(f,omega-rotate(conjugate(state_.orientation),config_.earth_rate_nav),dt);
    state_.stationary=verified_stationary&&state_.quiet;
    Quat mid=normalized(multiply(multiply(expRotation(config_.earth_rate_nav*(-dt/2)),state_.orientation),expRotation(omega*(dt/2))));
    Vec3 inertial_acc=rotate(mid,f)+Vec3{0,0,-config_.gravity};
    Vec3 first=inertial_acc-cross(config_.earth_rate_nav,state_.velocity)*2;
    Vec3 midpoint_velocity=state_.velocity+first*(dt/2);
    Vec3 acceleration=inertial_acc-cross(config_.earth_rate_nav,midpoint_velocity)*2;
    Vec3 displacement=state_.velocity*dt+acceleration*(dt*dt/2);
    state_.position=state_.position+displacement;state_.velocity=state_.velocity+acceleration*dt;
    if(!state_.stationary)state_.travelled_distance_m+=norm(displacement);
    state_.orientation=normalized(multiply(multiply(expRotation(config_.earth_rate_nav*(-dt)),state_.orientation),expRotation(omega*dt)));
    propagateCovariance(f,omega,mid,dt);state_.time_s=s.time_s;
    if(!healthy())return false;
    if(state_.stationary){
        stationary_elapsed_+=dt;++static_count_;static_acc_sum_=static_acc_sum_+a;static_gyro_sum_=static_gyro_sum_+w;
        if(stationary_elapsed_+1e-9>=config_.stationary_update_s){
            Vec3 am=static_acc_sum_/double(static_count_),wm=static_gyro_sum_/double(static_count_);
            double h[3][N]={};for(int i=0;i<3;++i)h[i][i+3]=1;
            // Rest comes from external evidence. Disable innovation rejection only for ZUPT.
            measurement(h,state_.velocity*(-1),isotropic(square(config_.zero_velocity_sigma)),false);
            Vec3 gb=rotate(conjugate(state_.orientation),{0,0,config_.gravity});double sh[3][3];skew(gb,sh);std::memset(h,0,sizeof(h));
            for(int i=0;i<3;++i){h[i][i+9]=1;for(int j=0;j<3;++j)h[i][j+6]=sh[i][j];}
            measurement(h,am-gb-state_.accel_bias,isotropic(square(config_.static_accel_sigma)));
            Vec3 eb=rotate(conjugate(state_.orientation),config_.earth_rate_nav);skew(eb,sh);std::memset(h,0,sizeof(h));
            for(int i=0;i<3;++i){h[i][i+12]=1;for(int j=0;j<3;++j)h[i][j+6]=sh[i][j];}
            measurement(h,wm-eb-state_.gyro_bias,isotropic(square(config_.static_gyro_sigma)));
            stationary_elapsed_=0;static_count_=0;static_acc_sum_=static_gyro_sum_={};
        }
    }else{stationary_elapsed_=0;static_count_=0;static_acc_sum_=static_gyro_sum_={};}
    return healthy();
}
bool Navigator::measurement(const double H[3][N],Vec3 residual,Vec3 variance,bool gate,double gate_limit){
    if(state_.fault||!finite(residual)||!finite(variance)||variance.x<=0||variance.y<=0||variance.z<=0){++state_.rejected_updates;return false;}
    // Gate the JOINT 3D innovation before changing anything. Cholesky avoids inversion.
    double ph[N][3]={},S[3][3]={},L[3][3]={},y[3]={};
    for(int i=0;i<N;++i)for(int a=0;a<3;++a)for(int j=0;j<N;++j)ph[i][a]+=P_[i][j]*H[a][j];
    for(int a=0;a<3;++a)for(int b=0;b<3;++b){S[a][b]=a==b?variance[a]:0;for(int i=0;i<N;++i)S[a][b]+=H[a][i]*ph[i][b];}
    for(int i=0;i<3;++i)for(int j=0;j<=i;++j){double v=S[i][j];for(int k=0;k<j;++k)v-=L[i][k]*L[j][k];if(i==j){if(!(v>0)||!std::isfinite(v)){state_.fault|=NumericalFailure;return false;}L[i][j]=std::sqrt(v);}else L[i][j]=v/L[j][j];}
    double nis=0;for(int i=0;i<3;++i){double v=residual[i];for(int j=0;j<i;++j)v-=L[i][j]*y[j];y[i]=v/L[i][i];nis+=y[i]*y[i];}
    if(!std::isfinite(nis)||(gate&&nis>(gate_limit>0?gate_limit:config_.innovation_gate_3d))){++state_.rejected_updates;return false;}
    double dx[N]={};
    // Sequential scalar Joseph updates are equivalent to a joint update for diagonal R.
    for(int a=0;a<3;++a){
        double pht[N]={},k[N],s=variance[a],innovation=residual[a];
        for(int i=0;i<N;++i){innovation-=H[a][i]*dx[i];for(int j=0;j<N;++j)pht[i]+=P_[i][j]*H[a][j];}
        for(int i=0;i<N;++i)s+=H[a][i]*pht[i];
        if(!(s>0)||!std::isfinite(s)){state_.fault|=NumericalFailure;return false;}
        for(int i=0;i<N;++i){k[i]=pht[i]/s;dx[i]+=k[i]*innovation;}
        for(int i=0;i<N;++i)for(int j=0;j<=i;++j){double value=P_[i][j]-k[i]*pht[j]-pht[i]*k[j]+k[i]*s*k[j];P_[i][j]=P_[j][i]=value;}
    }
    inject(dx);++state_.accepted_updates;return healthy();
}
void Navigator::inject(const double dx[N]){
    state_.barometer_bias_m+=dx[15];
    for(int i=0;i<6;++i)state_.magnetic_soft_iron[i]+=dx[i+19];
    for(int i=0;i<3;++i){state_.position[i]+=dx[i];state_.velocity[i]+=dx[i+3];state_.accel_bias[i]+=dx[i+9];state_.gyro_bias[i]+=dx[i+12];state_.magnetic_bias_uT[i]+=dx[i+16];}
    Vec3 d{dx[6],dx[7],dx[8]};state_.orientation=normalized(multiply(state_.orientation,expRotation(d)));
    // Exact SO(3) right Jacobian resets covariance to the newly injected tangent frame.
    double j[3][3];rightJacobian(d,j);
    std::memcpy(scratch_a_,P_,sizeof(P_));
    for(int r=0;r<3;++r)for(int c=0;c<N;++c){scratch_a_[r+6][c]=0;for(int k=0;k<3;++k)scratch_a_[r+6][c]+=j[r][k]*P_[k+6][c];}
    std::memcpy(P_,scratch_a_,sizeof(P_));
    for(int r=0;r<N;++r)for(int c=0;c<3;++c){P_[r][c+6]=0;for(int k=0;k<3;++k)P_[r][c+6]+=scratch_a_[r][k+6]*j[c][k];}
    for(int i=0;i<N;++i)for(int k=0;k<i;++k)P_[i][k]=P_[k][i]=(P_[i][k]+P_[k][i])/2;
}
bool Navigator::healthy(){
    if(!finite(state_.position)||!finite(state_.velocity)||!finite(state_.orientation)||!finite(state_.accel_bias)||!finite(state_.gyro_bias)||!finite(state_.magnetic_bias_uT)||!std::isfinite(state_.travelled_distance_m)||!std::isfinite(state_.barometer_bias_m))state_.fault|=NumericalFailure;
    for(int i=0;i<N;++i){if(P_[i][i]<0||!std::isfinite(P_[i][i]))state_.fault|=NumericalFailure;for(int j=0;j<i;++j)if(!std::isfinite(P_[i][j]))state_.fault|=NumericalFailure;}
    // Do not silently clip an invalid calibration estimate without covariance handling.
    double m[3][3];magneticMatrix(state_,m);
    for(double e:state_.magnetic_soft_iron)if(!std::isfinite(e))state_.fault|=NumericalFailure;
    const double minor=m[0][0]*m[1][1]-m[0][1]*m[0][1];
    const double det=m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
    if(!(m[0][0]>1e-6)||!(minor>1e-8)||!(det>1e-10))state_.fault|=NumericalFailure;
    if(state_.fault)state_.stationary=false;
    return !state_.fault;
}
bool Navigator::externalUpdate(Vec3 measured,Vec3 variance,double t,int offset){
    if(!state_.initialized||state_.fault||!finite(measured)||!std::isfinite(t)||std::abs(t-state_.time_s)>1e-6){++state_.rejected_updates;return false;}
    double h[3][N]={};for(int i=0;i<3;++i)h[i][i+offset]=1;
    return measurement(h,measured-(offset==0?state_.position:state_.velocity),variance);
}
bool Navigator::updatePosition(Vec3 m,Vec3 v,double t){return externalUpdate(m,v,t,0);}
bool Navigator::updateVelocity(Vec3 m,Vec3 v,double t){return externalUpdate(m,v,t,3);}
bool Navigator::updateMagnetometer(Vec3 m,Vec3 v,double t){
    const double reference=norm(config_.magnetic_field_nav);
    if(!state_.initialized||state_.fault||!finite(m)||!std::isfinite(t)||std::abs(t-state_.time_s)>1e-6||reference<1e-6||std::abs(norm(m-state_.magnetic_bias_uT)-reference)>config_.magnetic_relative_norm_tolerance*reference){++state_.rejected_updates;return false;}
    Vec3 expected=rotate(conjugate(state_.orientation),config_.magnetic_field_nav);
    double s[3][3],h[3][N]={},scale[3][3];skew(expected,s);magneticMatrix(state_,scale);
    for(int i=0;i<3;++i){h[i][i+16]=1;for(int j=0;j<3;++j)for(int k=0;k<3;++k)h[i][j+6]+=scale[i][k]*s[k][j];}
    h[0][19]=expected.x;h[1][20]=expected.y;h[2][21]=expected.z;
    h[0][22]=expected.y;h[1][22]=expected.x;
    h[0][23]=expected.z;h[2][23]=expected.x;
    h[1][24]=expected.z;h[2][24]=expected.y;
    return measurement(h,m-matVec(scale,expected)-state_.magnetic_bias_uT,v);
}
bool Navigator::updateBarometer(double altitude,double variance,double t){
    if(!state_.initialized||state_.fault||!std::isfinite(altitude)||!std::isfinite(t)||std::abs(t-state_.time_s)>1e-6){++state_.rejected_updates;return false;}
    // Two zero-Jacobian rows pad a scalar observation for the shared update kernel.
    // Their residuals are zero and carry no information about the state.
    double h[3][N]={};h[0][2]=1;h[0][15]=1;
    return measurement(h,{altitude-state_.position.z-state_.barometer_bias_m,0,0},{variance,1,1},true,config_.innovation_gate_1d);
}
Vec3 Navigator::positionSigma()const{return {std::sqrt(P_[0][0]),std::sqrt(P_[1][1]),std::sqrt(P_[2][2])};}
Vec3 Navigator::velocitySigma()const{return {std::sqrt(P_[3][3]),std::sqrt(P_[4][4]),std::sqrt(P_[5][5])};}
} // namespace imu
