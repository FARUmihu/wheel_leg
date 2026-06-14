#ifndef LEG_KINEMATICS_H
#define LEG_KINEMATICS_H

#include <stdint.h>

typedef enum {
    LEG_SIDE_LEFT = 0,
    LEG_SIDE_RIGHT = 1,
} leg_side_t;

typedef enum {
    LEG_KINEMATICS_OK = 0,
    LEG_KINEMATICS_BAD_ARGUMENT = -1,
    LEG_KINEMATICS_UNREACHABLE = -2,
    LEG_KINEMATICS_LIMIT = -3,
} leg_kinematics_status_t;

typedef struct {
    float x_mm;
    float y_mm;
} leg_point_t;

typedef struct {
    float theta1_deg; /* Feetech-side decoupled angle. */
    float theta2_deg; /* DM4310-side decoupled angle. */
} leg_joint_angles_t;

typedef struct {
    uint16_t ft_raw;
    float dm_rad;
} leg_actuator_state_t;

typedef struct {
    leg_point_t foot_d;
    leg_joint_angles_t joint;
} leg_kinematics_result_t;

float leg_kinematics_ft_raw_to_theta1_deg(leg_side_t side, uint16_t raw);
uint16_t leg_kinematics_theta1_deg_to_ft_raw(leg_side_t side, float theta1_deg);

float leg_kinematics_dm_rad_to_theta2_deg(leg_side_t side, float dm_rad);
float leg_kinematics_theta2_deg_to_dm_rad(leg_side_t side, float theta2_deg);

leg_kinematics_status_t leg_kinematics_forward_from_angles(leg_side_t side,
                                                           const leg_joint_angles_t *joint,
                                                           leg_kinematics_result_t *result);

leg_kinematics_status_t leg_kinematics_inverse_to_angles(leg_side_t side,
                                                         const leg_point_t *foot_d,
                                                         leg_kinematics_result_t *result);

leg_kinematics_status_t leg_kinematics_forward_from_actuator(leg_side_t side,
                                                             const leg_actuator_state_t *actuator,
                                                             leg_kinematics_result_t *result);

leg_kinematics_status_t leg_kinematics_fixed_ft_forward(leg_side_t side,
                                                        uint16_t fixed_ft_raw,
                                                        float dm_rad,
                                                        leg_kinematics_result_t *result);

leg_kinematics_status_t leg_kinematics_inverse_to_actuator(leg_side_t side,
                                                           const leg_point_t *foot_d,
                                                           leg_actuator_state_t *actuator,
                                                           leg_kinematics_result_t *result);

#endif /* LEG_KINEMATICS_H */
