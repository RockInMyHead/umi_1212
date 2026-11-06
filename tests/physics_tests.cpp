#include "navigation.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
int checks = 0;
void require(bool condition, const char* message) {
    ++checks;
    if (!condition) {
        std::cerr << "PHYSICS TEST FAILED: " << message << '\n';
        std::exit(1);
    }
}
double attitudeError(imu::Quat a, imu::Quat b) {
    imu::Quat d = imu::multiply(imu::conjugate(a), b);
    return 2 * std::atan2(std::sqrt(d.x*d.x+d.y*d.y+d.z*d.z), std::abs(d.w));
}
imu::Config deterministicConfig() {
    imu::Config c;
    c.accel_noise = c.gyro_noise = c.accel_bias_walk = c.gyro_bias_walk = 0;
    c.barometer_bias_walk = c.barometer_bias_initial_sigma = 0;
    c.magnetic_bias_walk = c.magnetic_bias_initial_sigma = 0;
    c.magnetic_scale_walk = c.magnetic_scale_initial_sigma = 0;
    c.position_initial_sigma = c.velocity_initial_sigma = 0;
    c.accel_bias_initial_sigma = c.gyro_bias_initial_sigma = 0;
    c.quiet_window_s = c.quiet_dwell_s = c.stationary_update_s = 0.1;
    return c;
}
void gravityAndJacobianChecks() {
    const imu::Quat q = imu::expRotation({0.37, -0.28, 0.61});
    const imu::Vec3 up{0, 0, 9.80665}, force{1.3, -2.7, 9.4};
    const imu::Vec3 earth{0, 5.16e-5, 5.16e-5};
    const imu::Vec3 measuredGravity = imu::rotate(imu::conjugate(q), up);
    require(imu::norm(imu::rotate(q, measuredGravity)-up) < 1e-12,
            "body/nav gravity round trip");
    require(imu::norm(imu::rotate(imu::gravityAlignment(measuredGravity), measuredGravity)-up) < 1e-12,
            "gravity initialization aligns an arbitrary tilt");
    const double eps = 1e-6;
    for (int axis = 0; axis < 3; ++axis) {
        imu::Vec3 e{}; e[axis] = 1;
        const imu::Quat qp = imu::multiply(q, imu::expRotation(e*eps));
        const imu::Quat qm = imu::multiply(q, imu::expRotation(e*(-eps)));
        const imu::Vec3 forwardDerivative =
            (imu::rotate(qp, force)-imu::rotate(qm, force))/(2*eps);
        const imu::Vec3 forwardExpected = imu::rotate(q, imu::cross(e, force));
        require(imu::norm(forwardDerivative-forwardExpected) < 2e-9,
                "right attitude force derivative is -R skew(force)");
        for (imu::Vec3 reference : {up, earth, imu::Vec3{20,5,-40}}) {
            const imu::Vec3 inverseDerivative =
                (imu::rotate(imu::conjugate(qp), reference)
                 -imu::rotate(imu::conjugate(qm), reference))/(2*eps);
            const imu::Vec3 inverseExpected =
                imu::cross(imu::rotate(imu::conjugate(q), reference), e);
            require(imu::norm(inverseDerivative-inverseExpected) < 5e-10*std::max(1.0,imu::norm(reference)),
                    "gravity/Earth/magnetic observation derivative has positive skew sign");
        }
    }
}
void staticCorrectionChecks() {
    imu::Config c = deterministicConfig();
    c.tilt_initial_sigma = 0.1; c.yaw_initial_sigma = 0;
    // Keep radial uncertainty above second-order gravity residuals while the
    // initial tilt is still nonzero; an unrealistically tiny R would gate it.
    c.static_accel_sigma = 0.01;
    c.zero_velocity_sigma = 1e6; // Isolate the direct gravity correction.
    imu::Navigator tilt(c);
    require(tilt.initialize(0, imu::expRotation({0.02, -0.03, 0})), "tilt initialization");
    for (int i = 1; i <= 200; ++i) {
        imu::Sample s; s.time_s = i*0.01; s.accel = {0, 0, c.gravity};
        require(tilt.predict(s, true), "static tilted propagation");
    }
    const imu::Vec3 estimatedUp = imu::rotate(tilt.state().orientation, {0, 0, 1});
    if (std::hypot(estimatedUp.x, estimatedUp.y) >= 1e-5)
        std::cerr << "tilt residual " << estimatedUp.x << ", " << estimatedUp.y
                  << " accepted " << tilt.state().accepted_updates
                  << " rejected " << tilt.state().rejected_updates << '\n';
    require(std::hypot(estimatedUp.x, estimatedUp.y) < 1e-5,
            "stationary gravity update corrects tilt with the right sign");

    c.tilt_initial_sigma = c.yaw_initial_sigma = 0;
    c.static_accel_sigma = 0.001;
    c.accel_bias_initial_sigma = 0.1;
    c.static_gyro_sigma = 0.00001;
    c.gyro_bias_initial_sigma = 0.01;
    imu::Navigator bias(c);
    require(bias.initialize(0), "bias initialization");
    const imu::Vec3 ba{0.03, -0.04, 0.06}, bg{0, 0, 0.001};
    for (int i = 1; i <= 300; ++i) {
        imu::Sample s; s.time_s = i*0.01; s.accel = ba+imu::Vec3{0,0,c.gravity}; s.gyro = bg;
        require(bias.predict(s, true), "static biased propagation");
    }
    require(imu::norm(bias.state().accel_bias-ba) < 1e-5,
            "static residual accelerometer bias has correct sign");
    require(imu::norm(bias.state().gyro_bias-bg) < 1e-7,
            "static residual gyro bias has correct sign");
}
void earthEquilibriumCheck() {
    imu::Config c = deterministicConfig();
    c.earth_rate_nav = {0, 5.1563039657e-5, 5.1563039657e-5};
    const imu::Quat q = imu::expRotation({0.31, -0.42, 1.16});
    imu::Navigator nav(c);
    require(nav.initialize(0, q), "Earth equilibrium initialization");
    for (int i = 1; i <= 10000; ++i) {
        imu::Sample s; s.time_s = i*0.01;
        s.accel = imu::rotate(imu::conjugate(q), {0,0,c.gravity});
        s.gyro = imu::rotate(imu::conjugate(q), c.earth_rate_nav);
        require(nav.predict(s), "Earth equilibrium propagation");
    }
    require(attitudeError(q, nav.state().orientation) < 1e-10,
            "fixed sensor stays fixed with the body-frame Earth signal");
    require(imu::norm(nav.state().position) < 1e-6,
            "Earth equilibrium has no gravity leakage");
}
void eightHourSpinCovarianceCheck() {
    // This isolates covariance transport: exact synthetic spin, no stochastic
    // sensor errors or uncertain biases. It is not an MPU6050 accuracy claim.
    imu::Config c = deterministicConfig();
    c.max_dt = 0.051;
    c.tilt_initial_sigma = 0.03; c.yaw_initial_sigma = 0.5;
    imu::Navigator nav(c);
    require(nav.initialize(0), "constant-spin initialization");
    constexpr int count = 576000;
    constexpr double dt = 0.05;
    for (int i = 1; i <= count; ++i) {
        imu::Sample s; s.time_s = i*dt;
        s.accel = {0,0,c.gravity}; s.gyro = {0,0,1};
        if (!nav.predict(s)) {
            std::cerr << "spin failed at simulated time " << s.time_s
                      << " fault " << nav.state().fault << '\n';
            require(false, "eight-hour constant-spin propagation");
        }
    }
    require(std::abs(nav.state().time_s-28800) < 1e-9, "spin reaches eight hours");
    require(std::abs(nav.covariance(6,6)-0.0009) < 1e-9,
            "spin must not manufacture roll covariance");
    require(std::abs(nav.covariance(7,7)-0.0009) < 1e-9,
            "spin must not manufacture pitch covariance");
    require(std::abs(nav.covariance(8,8)-0.25) < 1e-9,
            "spin preserves yaw covariance without observations");
    require(imu::norm(nav.state().position) < 1e-8, "yaw spin has no displacement");
    const imu::Quat expected = imu::expRotation({0,0,28800});
    require(attitudeError(expected, nav.state().orientation) < 1e-8,
            "eight-hour quaternion integration remains accurate");
}
void magneticChecks() {
    imu::Config c = deterministicConfig();
    c.magnetic_field_nav = {20,5,-40};
    c.tilt_initial_sigma = 0;
    c.yaw_initial_sigma = 0.3;
    imu::Navigator heading(c);
    require(heading.initialize(0, imu::expRotation({0,0,0.1})), "magnetic yaw initialization");
    const double initialError = attitudeError({}, heading.state().orientation);
    for (int i=1; i<=100; ++i) {
        imu::Sample s; s.time_s=i*0.01; s.accel={0,0,c.gravity};
        require(heading.predict(s), "magnetic yaw propagation");
        require(heading.updateMagnetometer(c.magnetic_field_nav, {0.25,0.25,0.25}, s.time_s),
                "valid magnetic observation accepted");
    }
    require(attitudeError({}, heading.state().orientation)<initialError/100,
            "magnetic observation corrects yaw with positive Jacobian sign");

    c.tilt_initial_sigma=c.yaw_initial_sigma=0.001;
    imu::Navigator disturbed(c);
    require(disturbed.initialize(0), "magnetic disturbance initialization");
    const double pBefore=disturbed.covariance(8,8);
    require(!disturbed.updateMagnetometer(c.magnetic_field_nav*1.5,{0.25,0.25,0.25},0),
            "magnetic magnitude disturbance rejected");
    const imu::Vec3 rotated=imu::rotate(imu::expRotation({0,0,1.5707963267948966}),c.magnetic_field_nav);
    require(std::abs(imu::norm(rotated)-imu::norm(c.magnetic_field_nav))<1e-10,
            "directional disturbance preserves field magnitude");
    require(!disturbed.updateMagnetometer(rotated,{0.25,0.25,0.25},0),
            "joint innovation rejects same-norm directional disturbance when attitude is known");
    require(attitudeError({},disturbed.state().orientation)==0 && disturbed.covariance(8,8)==pBefore,
            "magnetic rejection preserves state and covariance");

    c.magnetic_field_nav={0,0,45}; c.tilt_initial_sigma=0; c.yaw_initial_sigma=0.3;
    imu::Navigator vertical(c);
    require(vertical.initialize(0,imu::expRotation({0,0,0.4})), "vertical field initialization");
    require(vertical.updateMagnetometer(c.magnetic_field_nav,{0.25,0.25,0.25},0),
            "matching vertical magnetic field accepted");
    require(std::abs(vertical.covariance(8,8)-0.09)<1e-12,
            "vertical magnetic field supplies no yaw information");
    require(std::abs(attitudeError({},vertical.state().orientation)-0.4)<1e-12,
            "vertical field cannot resolve yaw error");
}
void barometricGaugeChecks() {
    imu::Config c=deterministicConfig();
    c.position_initial_sigma=2; c.barometer_bias_initial_sigma=2;
    imu::Navigator gauge(c);
    require(gauge.initialize(0), "barometer gauge initialization");
    require(gauge.updateBarometer(2,0.04,0), "barometer sum observation accepted");
    const double expected=8.0/8.04;
    require(std::abs(gauge.state().position.z-expected)<1e-12,
            "barometer height correction follows prior uncertainty");
    require(std::abs(gauge.state().barometer_bias_m-expected)<1e-12,
            "barometer positive offset correction has correct sign");
    const double differenceVariance=gauge.covariance(2,2)+gauge.covariance(15,15)
                                    -2*gauge.covariance(2,15);
    require(std::abs(differenceVariance-8)<1e-11,
            "barometer retains unobservable height-minus-offset uncertainty");
    const double sumVariance=gauge.covariance(2,2)+gauge.covariance(15,15)
                              +2*gauge.covariance(2,15);
    require(sumVariance<0.04 && sumVariance>0,
            "barometer constrains observed height-plus-offset combination");
    const double heightBefore=gauge.state().position.z, biasBefore=gauge.state().barometer_bias_m;
    const double varianceBefore=gauge.covariance(2,2);
    require(!gauge.updateBarometer(100,0.04,0), "barometer jump rejected");
    require(gauge.state().position.z==heightBefore && gauge.state().barometer_bias_m==biasBefore
            && gauge.covariance(2,2)==varianceBefore,
            "barometer rejection preserves state and covariance");

    c.position_initial_sigma=0;
    imu::Navigator knownHeight(c);
    require(knownHeight.initialize(0), "known-height barometer initialization");
    require(knownHeight.updateBarometer(2,0.04,0), "known-height barometer correction");
    require(knownHeight.state().position.z==0,
            "barometer respects explicitly known initial height");
    require(std::abs(knownHeight.state().barometer_bias_m-8.0/4.04)<1e-12,
            "independent known height allows offset estimation");
}
void magneticBiasGaugeChecks() {
    // Fixed pose: yaw and a transverse hard-iron residual are not separately
    // observed. Repetition reduces white noise, not persistent bias uncertainty.
    imu::Config c=deterministicConfig();
    c.magnetic_field_nav={20,0,-40};
    c.tilt_initial_sigma=0; c.yaw_initial_sigma=0.1;
    c.magnetic_bias_initial_sigma=1;
    imu::Navigator gauge(c);
    require(gauge.initialize(0),"magnetic bias gauge initialization");
    constexpr int observations=400;
    for(int i=0;i<observations;++i)
        require(gauge.updateMagnetometer(c.magnetic_field_nav,{0.16,0.16,0.16},0),
                "fixed-pose magnetic gauge observation");
    const double yawPrior=0.01, biasPrior=1, horizontalFieldSquared=400;
    const double effectiveWhiteVariance=0.16/observations;
    const double expected=yawPrior*(biasPrior+effectiveWhiteVariance)
                         /(horizontalFieldSquared*yawPrior+biasPrior+effectiveWhiteVariance);
    require(std::abs(gauge.covariance(8,8)-expected)<1e-10,
            "fixed-pose yaw variance matches persistent-bias analytic posterior");
    require(gauge.covariance(8,8)>0.0019,
            "repeated magnetic readings do not erase unobservable yaw-bias ambiguity");
    require(std::abs(gauge.covariance(8,17))>0.03,
            "magnetic yaw and residual bias retain the required cross covariance");

    imu::Navigator offset(c);
    require(offset.initialize(0),"fixed unknown magnetic offset initialization");
    const imu::Vec3 measured=c.magnetic_field_nav+imu::Vec3{0.1,-0.2,0.05};
    for(int i=0;i<observations;++i)
        require(offset.updateMagnetometer(measured,{0.16,0.16,0.16},0),
                "fixed unknown magnetic offset observation");
    const imu::Vec3 fitted=imu::rotate(imu::conjugate(offset.state().orientation),c.magnetic_field_nav)
                          +offset.state().magnetic_bias_uT;
    require(imu::norm(fitted-measured)<0.002,
            "magnetic posterior fits the observed sum of attitude and residual bias");
    require(offset.covariance(8,8)>0.001,
            "unknown fixed offset must not create precise one-pose heading");
}
imu::Vec3 applyResidualScale(const double s[6],imu::Vec3 v) {
    return {(1+s[0])*v.x+s[3]*v.y+s[4]*v.z,
            s[3]*v.x+(1+s[1])*v.y+s[5]*v.z,
            s[4]*v.x+s[5]*v.y+(1+s[2])*v.z};
}
void magneticScaleJacobianAndGaugeChecks() {
    const double scale[6]={0.002,-0.003,0.004,0.001,-0.002,0.0015};
    const imu::Quat q=imu::expRotation({0.37,-0.28,0.61});
    const imu::Vec3 field{20,5,-40};
    const imu::Vec3 body=imu::rotate(imu::conjugate(q),field);
    const imu::Vec3 columns[6]={{body.x,0,0},{0,body.y,0},{0,0,body.z},
                              {body.y,body.x,0},{body.z,0,body.x},{0,body.z,body.y}};
    constexpr double eps=1e-6;
    for(int term=0;term<6;++term) {
        double plus[6],minus[6];
        for(int j=0;j<6;++j)plus[j]=minus[j]=scale[j];
        plus[term]+=eps;minus[term]-=eps;
        const imu::Vec3 derivative=(applyResidualScale(plus,body)-applyResidualScale(minus,body))/(2*eps);
        require(imu::norm(derivative-columns[term])<3e-8,
                "symmetric residual-scale Jacobian column agrees with finite differences");
    }
    for(int axis=0;axis<3;++axis) {
        imu::Vec3 e{};e[axis]=1;
        const imu::Quat qp=imu::multiply(q,imu::expRotation(e*eps));
        const imu::Quat qm=imu::multiply(q,imu::expRotation(e*(-eps)));
        const imu::Vec3 derivative=(applyResidualScale(scale,imu::rotate(imu::conjugate(qp),field))
                                 -applyResidualScale(scale,imu::rotate(imu::conjugate(qm),field)))/(2*eps);
        const imu::Vec3 expected=applyResidualScale(scale,imu::cross(body,e));
        require(imu::norm(derivative-expected)<3e-8,
                "magnetic attitude Jacobian includes the residual-scale matrix");
    }
    // Black-box checks of each implemented scale column. A basis-axis field
    // makes the chosen coefficient independently observable with known attitude.
    const int inputAxes[6]={0,1,2,0,0,1},outputAxes[6]={0,1,2,1,2,2};
    for(int term=0;term<6;++term) {
        imu::Config c=deterministicConfig();
        c.tilt_initial_sigma=c.yaw_initial_sigma=0;
        c.magnetic_scale_initial_sigma=0.005;
        c.magnetic_field_nav[inputAxes[term]]=40;
        imu::Navigator nav(c);require(nav.initialize(0),"scale-column initialization");
        imu::Vec3 measured=c.magnetic_field_nav;measured[outputAxes[term]]+=40*0.002;
        require(nav.updateMagnetometer(measured,{0.0001,0.0001,0.0001},0),
                "scale-column observation accepted");
        const double expected=0.002*0.04/0.0401;
        require(std::abs(nav.state().magnetic_soft_iron[term]-expected)<1e-10,
                "implemented residual-scale coefficient has correct axis, sign and gain");
    }
    imu::Config c=deterministicConfig();
    c.magnetic_field_nav={20,0,-40};c.tilt_initial_sigma=0;c.yaw_initial_sigma=0.1;
    c.magnetic_scale_initial_sigma=0.005;
    imu::Navigator gauge(c);require(gauge.initialize(0),"magnetic scale gauge initialization");
    constexpr int observations=400;
    for(int i=0;i<observations;++i)
        require(gauge.updateMagnetometer(c.magnetic_field_nav,{0.16,0.16,0.16},0),
                "fixed-pose scale gauge observation");
    // For B=(20,0,-40), Sxy and Syz jointly imitate transverse field error.
    const double residualFieldVariance=0.005*0.005*(20*20+40*40);
    const double whiteVariance=0.16/observations;
    const double expected=0.01*(residualFieldVariance+whiteVariance)
                          /(400*0.01+residualFieldVariance+whiteVariance);
    require(std::abs(gauge.covariance(8,8)-expected)<1e-10,
            "fixed-pose yaw covariance preserves the analytic scale ambiguity");
    require(gauge.covariance(8,8)>0.00012,
            "uncertain soft iron must not average away as white measurement noise");
    require(std::abs(gauge.covariance(20,20)-0.005*0.005)<1e-15,
            "one-pose field with zero Y component leaves Syy unobserved");
}
void eightHourWeatherAmbiguityCheck() {
    // Two physically different local-model worlds have identical measurements:
    // A: fixed height, pressure-reference drift equivalent to 30 m over 8 h.
    // B: 30 m constant-velocity climb, no pressure-reference drift.
    // Their unknown initial vertical velocities differ by 1.04 mm/s. No rest or
    // external height assertion is supplied. Filtering cannot identify the world.
    imu::Config c=deterministicConfig();
    c.max_dt=0.051; c.tilt_initial_sigma=c.yaw_initial_sigma=0;
    c.position_initial_sigma=0.1; c.velocity_initial_sigma=0.05;
    c.barometer_bias_initial_sigma=2; c.barometer_bias_walk=0.02;
    imu::Navigator nav(c);
    require(nav.initialize(0), "weather ambiguity initialization");
    constexpr int count=576000;
    for (int i=1; i<=count; ++i) {
        imu::Sample s; s.time_s=i*0.05; s.accel={0,0,c.gravity};
        if(!nav.predict(s)) require(false,"weather ambiguity propagation");
        if(i%20==0 && !nav.updateBarometer(30*s.time_s/28800,0.25,s.time_s))
            require(false,"smooth weather/climb observation accepted");
    }
    const double errorA=std::abs(nav.state().position.z);
    const double errorB=std::abs(nav.state().position.z-30);
    require(std::abs(nav.state().time_s-28800)<1e-9,"weather ambiguity reaches eight hours");
    require(std::max(errorA,errorB)>=15-1e-9,
            "identical measurements cannot guarantee small height error in both worlds");
    std::cout << "Eight-hour identical-stream pressure example: fixed-height error=" << errorA
              << " m; slow-climb error=" << errorB << " m.\n";
}
}

int main() {
    gravityAndJacobianChecks();
    staticCorrectionChecks();
    earthEquilibriumCheck();
    eightHourSpinCovarianceCheck();
    magneticChecks();
    magneticBiasGaugeChecks();
    magneticScaleJacobianAndGaugeChecks();
    barometricGaugeChecks();
    eightHourWeatherAmbiguityCheck();
    std::cout << "Physics tests passed: " << checks
              << " checks, including eight simulated hours of constant spin.\n";
}
