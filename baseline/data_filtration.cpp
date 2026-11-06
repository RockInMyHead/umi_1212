#include "algorithm_core.h"
#include <math.h>

namespace {
constexpr float TWO_KP=0.2f, TWO_KI=0.01f, INTEGRAL_LIMIT=0.25f;
float ix=0, iy=0, iz=0;
float limit(float v) { return v>INTEGRAL_LIMIT?INTEGRAL_LIMIT:(v < -INTEGRAL_LIMIT?-INTEGRAL_LIMIT:v); }
}

void reset_data_filtration() { ix=iy=iz=0.0f; }
void execute_data_filtration(Quaternion *q, Vector3D acc, Vector3D gyro, float dt) {
    if (!q || !isfinite(dt) || dt<=0 || dt>0.1f) return;
    const float an=sqrtf(acc.x*acc.x+acc.y*acc.y+acc.z*acc.z);
    if (an>8.0f && an<11.5f) {
        acc.x/=an; acc.y/=an; acc.z/=an;
        Vector3D g={2*(q->x*q->z+q->w*q->y),2*(q->y*q->z-q->w*q->x),q->w*q->w-q->x*q->x-q->y*q->y+q->z*q->z};
        Vector3D e={acc.y*g.z-acc.z*g.y,acc.z*g.x-acc.x*g.z,acc.x*g.y-acc.y*g.x};
        // The integral term must be time based, otherwise changing loop frequency changes the filter.
        ix=limit(ix+e.x*TWO_KI*dt); iy=limit(iy+e.y*TWO_KI*dt); iz=limit(iz+e.z*TWO_KI*dt);
        gyro.x+=TWO_KP*e.x+ix; gyro.y+=TWO_KP*e.y+iy; gyro.z+=TWO_KP*e.z+iz;
    }
    const float h=0.5f*dt; const Quaternion o=*q;
    q->w+=(-o.x*gyro.x-o.y*gyro.y-o.z*gyro.z)*h;
    q->x+=( o.w*gyro.x+o.y*gyro.z-o.z*gyro.y)*h;
    q->y+=( o.w*gyro.y-o.x*gyro.z+o.z*gyro.x)*h;
    q->z+=( o.w*gyro.z+o.x*gyro.y-o.y*gyro.x)*h;
    const float n=sqrtf(q->w*q->w+q->x*q->x+q->y*q->y+q->z*q->z);
    if (n>0.000001f && isfinite(n)) { q->w/=n; q->x/=n; q->y/=n; q->z/=n; }
    else { *q={1,0,0,0}; reset_data_filtration(); }
}
