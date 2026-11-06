#pragma once
#include <cstdint>

namespace imu {
constexpr int N = 25; // p,v,attitude,ba,bg,baro bias,mag bias,6 symmetric mag scale terms
struct Vec3 { double x=0,y=0,z=0; double& operator[](int i) { return i==0?x:(i==1?y:z); } double operator[](int i) const { return i==0?x:(i==1?y:z); } };
Vec3 operator+(Vec3 a,Vec3 b); Vec3 operator-(Vec3 a,Vec3 b);
Vec3 operator*(Vec3 a,double s); Vec3 operator/(Vec3 a,double s);
double dot(Vec3 a,Vec3 b); double norm(Vec3 a); Vec3 cross(Vec3 a,Vec3 b);
struct Quat { double w=1,x=0,y=0,z=0; };
Quat multiply(Quat a,Quat b); Quat conjugate(Quat q); Quat normalized(Quat q);
Quat expRotation(Vec3 radians); Vec3 rotate(Quat q,Vec3 v);
// Align measured gravity to +Z with yaw=0. Does NOT estimate accelerometer bias.
Quat gravityAlignment(Vec3 specific_force);
struct Calibration {
    double accel_matrix[3][3]={{1,0,0},{0,1,0},{0,0,1}};
    double gyro_matrix[3][3]={{1,0,0},{0,1,0},{0,0,1}};
    Vec3 accel_offset{},gyro_offset{},accel_temp_slope{},gyro_temp_slope{};
    double reference_temperature_c=25;
};
struct Config {
    double gravity=9.80665;
    // Fixed local ENU frame. Set from surveyed latitude/heading, or leave zero.
    Vec3 earth_rate_nav{};
    // Calibrated geomagnetic field in the navigation frame, microtesla.
    // Zero disables magnetic aiding. Requires known declination/frame convention.
    Vec3 magnetic_field_nav{};
    double magnetic_relative_norm_tolerance=0.20;
    // Persistent residual hard-iron/slow thermal uncertainty, NOT white sensor noise.
    double magnetic_bias_initial_sigma=1.0,magnetic_bias_walk=0.002; // uT, uT/sqrt(s)
    double magnetic_scale_initial_sigma=0.005,magnetic_scale_walk=0.000001; // fractional, /sqrt(s)
    // Continuous-time diffusion amplitudes: sample std = density/sqrt(dt).
    // Nominal engineering priors, NOT an MPU6050 performance guarantee.
    double accel_noise=0.004,gyro_noise=0.00040;
    double accel_bias_walk=0.0002,gyro_bias_walk=0.000002;
    double max_dt=0.05;
    double position_initial_sigma=0.1,velocity_initial_sigma=0.05;
    double tilt_initial_sigma=0.03,yaw_initial_sigma=0.5;
    double accel_bias_initial_sigma=0.10,gyro_bias_initial_sigma=0.005;
    double barometer_bias_initial_sigma=2.0,barometer_bias_walk=0.02;
    double innovation_gate_3d=25.0;
    double innovation_gate_1d=16.0;
    double quiet_accel_rms=0.15,quiet_gyro_rms=0.025,quiet_gravity_error=0.3;
    double quiet_window_s=0.5,quiet_dwell_s=1.0;
    double stationary_update_s=0.1;
    double zero_velocity_sigma=0.02,static_accel_sigma=0.08,static_gyro_sigma=0.002;
    Calibration calibration{};
};
struct Sample {
    double time_s=0; Vec3 accel{},gyro{}; double temperature_c=25;
    bool saturated=false;
};
enum Fault : uint32_t { NoFault=0, InvalidInput=1, TimeGap=2, Saturation=4, NumericalFailure=8 };
struct State {
    Vec3 position{},velocity{},accel_bias{},gyro_bias{};
    Vec3 magnetic_bias_uT{};
    // Residual symmetric soft-iron matrix S: xx,yy,zz,xy,xz,yz; h=(I+S)R'B+b.
    double magnetic_soft_iron[6]{};
    Quat orientation{}; // Hamilton scalar-first, body -> fixed local z-up frame.
    double time_s=0,travelled_distance_m=0;
    double barometer_bias_m=0;
    bool quiet=false,stationary=false,initialized=false;
    uint32_t fault=0,accepted_updates=0,rejected_updates=0;
};
class Navigator {
public:
    explicit Navigator(Config config=Config{});
    bool initialize(double time_s,Quat orientation=Quat{},Vec3 position=Vec3{},Vec3 velocity=Vec3{},Vec3 accel_bias=Vec3{},Vec3 gyro_bias=Vec3{});
    // Samples are interval averages for (previous time, time_s]. No fabricated dt.
    // true means independently VERIFIED translational and rotational rest.
    bool predict(const Sample& sample,bool verified_stationary=false);
    // Independent, synchronous, diagonal-covariance measurements in the same frame.
    // Delayed measurements are rejected; replay/buffer them at the application layer.
    bool updatePosition(Vec3 measured,Vec3 variance,double time_s);
    bool updateVelocity(Vec3 measured,Vec3 variance,double time_s);
    // Input must be hard/soft-iron calibrated and rotated into the IMU body frame.
    bool updateMagnetometer(Vec3 field_uT,Vec3 variance_uT2,double time_s);
    // Relative altitude in metres, positive up, sharing the initialization origin.
    // Weather/pressure-origin error is a separate, generally unobservable bias.
    bool updateBarometer(double relative_altitude_m,double variance_m2,double time_s);
    const State& state() const { return state_; }
    double covariance(int i,int j) const { return P_[i][j]; }
    Vec3 positionSigma() const;
    Vec3 velocitySigma() const;
    const Config& config() const { return config_; }
private:
    Config config_; State state_{}; double P_[N][N]{};
    // Per-instance workspaces keep embedded task stack bounded. Not reentrant.
    double scratch_a_[N][N]{},scratch_b_[N][N]{};
    double quiet_elapsed_=0,quiet_dwell_=0,stationary_elapsed_=0;
    Vec3 acc_mean_{},gyro_mean_{},acc_m2_{},gyro_m2_{};
    unsigned quiet_count_=0;
    Vec3 static_acc_sum_{},static_gyro_sum_{}; unsigned static_count_=0;
    bool config_valid_=true;
    void propagateCovariance(Vec3 f,Vec3 omega,Quat midpoint,double dt);
    bool measurement(const double H[3][N],Vec3 residual,Vec3 variance,bool gate=true,double gate_limit=-1);
    void inject(const double dx[N]);
    bool healthy();
    void updateQuiet(Vec3 accel,Vec3 gyro,double dt);
    bool externalUpdate(Vec3 measured,Vec3 variance,double time_s,int offset);
};
} // namespace imu
