#include "algorithm_core.h"
#include <math.h>
#include <string.h>
namespace {
constexpr float GRAVITY = 9.80665f;
constexpr int WINDOW_SIZE = 16;
constexpr float VARIANCE_LIMIT = 0.008f;
constexpr float ACC_MEAN_LIMIT = 0.25f;
constexpr float GYRO_MEAN_LIMIT = 0.05f;
constexpr float ACC_DEAD_ZONE = 0.08f;
Vector3D acc_window[WINDOW_SIZE] = {};
Vector3D gyro_window[WINDOW_SIZE] = {};
int window_index = 0;
bool window_ready = false;
Vector3D previous_acceleration = {};
Vector3D velocity = {};
Vector3D position = {};
float total_distance = 0.0f;
float clamp_value(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
float norm(Vector3D v) { return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z); }
bool detect_stationary() {
    if (!window_ready) return false;
    Vector3D am = {}, gm = {};
    for (int i = 0; i < WINDOW_SIZE; ++i) {
        am.x += acc_window[i].x; am.y += acc_window[i].y; am.z += acc_window[i].z;
        gm.x += gyro_window[i].x; gm.y += gyro_window[i].y; gm.z += gyro_window[i].z;
    }
    const float s = 1.0f / WINDOW_SIZE;
    am.x *= s; am.y *= s; am.z *= s; gm.x *= s; gm.y *= s; gm.z *= s;
    float variance = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; ++i) {
        const float x = acc_window[i].x-am.x, y = acc_window[i].y-am.y, z = acc_window[i].z-am.z;
        variance += x*x + y*y + z*z;
    }
    variance *= s;
    return variance < VARIANCE_LIMIT && norm(am) < ACC_MEAN_LIMIT && norm(gm) < GYRO_MEAN_LIMIT;
}
}
Vector3D rotate_to_global_frame(Vector3D v, Quaternion q) {
    Vector3D r;
    const float ww=q.w*q.w, xx=q.x*q.x, yy=q.y*q.y, zz=q.z*q.z;
    const float xy=q.x*q.y, xz=q.x*q.z, yz=q.y*q.z;
    const float wx=q.w*q.x, wy=q.w*q.y, wz=q.w*q.z;
    r.x=v.x*(ww+xx-yy-zz)+v.y*2.0f*(xy-wz)+v.z*2.0f*(xz+wy);
    r.y=v.x*2.0f*(xy+wz)+v.y*(ww-xx+yy-zz)+v.z*2.0f*(yz-wx);
    r.z=v.x*2.0f*(xz-wy)+v.y*2.0f*(yz+wx)+v.z*(ww-xx-yy+zz);
    return r;
}
void reset_navigation() {
    memset(acc_window, 0, sizeof(acc_window)); memset(gyro_window, 0, sizeof(gyro_window));
    window_index=0; window_ready=false; previous_acceleration={}; velocity={}; position={}; total_distance=0.0f;
}

NavigationState execute_navigation(Vector3D acc, Vector3D gyro, Quaternion q, float dt) {
    NavigationState out = {};
    out.position_m = position; out.total_distance_m = total_distance;
    if (!isfinite(dt) || dt < 0.0005f || dt > 0.1f) return out;
    Vector3D a = rotate_to_global_frame(acc, q); a.z -= GRAVITY;
    acc_window[window_index]=a; gyro_window[window_index]=gyro;
    window_index=(window_index+1)%WINDOW_SIZE; if (window_index==0) window_ready=true;
    out.stationary=detect_stationary();
    if (out.stationary) { velocity={}; previous_acceleration={}; }
    else {
        if (fabsf(a.x) < ACC_DEAD_ZONE) a.x = 0.0f;
        if (fabsf(a.y) < ACC_DEAD_ZONE) a.y = 0.0f;
        if (fabsf(a.z) < ACC_DEAD_ZONE) a.z = 0.0f;
        const Vector3D old_v=velocity;
        velocity.x+=0.5f*(previous_acceleration.x+a.x)*dt;
        velocity.y+=0.5f*(previous_acceleration.y+a.y)*dt;
        velocity.z+=0.5f*(previous_acceleration.z+a.z)*dt;
        Vector3D d={0.5f*(old_v.x+velocity.x)*dt,0.5f*(old_v.y+velocity.y)*dt,0.5f*(old_v.z+velocity.z)*dt};
        position.x+=d.x; position.y+=d.y; position.z+=d.z;
        out.step_distance_m=norm(d); total_distance+=out.step_distance_m;
        if (out.step_distance_m>0.000001f) {
            out.alpha_deg=acosf(clamp_value(d.x/out.step_distance_m,-1,1))*57.2957795f;
            out.beta_deg=acosf(clamp_value(d.y/out.step_distance_m,-1,1))*57.2957795f;
            out.gamma_deg=acosf(clamp_value(d.z/out.step_distance_m,-1,1))*57.2957795f;
        }
        previous_acceleration=a;
    }
    out.position_m=position; out.total_distance_m=total_distance;
    return out;
}
