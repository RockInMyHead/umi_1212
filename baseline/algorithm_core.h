#ifndef ALGORITHM_CORE_H
#define ALGORITHM_CORE_H
#include <stdbool.h>

typedef struct { float x, y, z; } Vector3D;
typedef struct { float w, x, y, z; } Quaternion;
typedef struct {
    Vector3D position_m;
    float step_distance_m;
    float total_distance_m;
    float alpha_deg, beta_deg, gamma_deg;
    bool stationary;
} NavigationState;
NavigationState execute_navigation(Vector3D acc_local_mps2, Vector3D gyro_local_radps,
                                   Quaternion orientation, float dt_s);
Vector3D rotate_to_global_frame(Vector3D local_vec, Quaternion q);
void reset_navigation(void);
void execute_data_filtration(Quaternion *q, Vector3D acc_local_mps2,
                             Vector3D gyro_local_radps, float dt_s);
void reset_data_filtration(void);
#endif
