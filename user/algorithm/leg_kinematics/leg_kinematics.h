#ifndef LEG_KINEMATICS_H
#define LEG_KINEMATICS_H

#include <stdint.h>

typedef enum {
    LEG_SIDE_LEFT = 0,
    LEG_SIDE_RIGHT = 1,
} leg_side_t;

typedef enum {
    LEG_KINEMATICS_OK = 0,
    LEG_KINEMATICS_NOT_IMPLEMENTED = -1,
    LEG_KINEMATICS_BAD_ARGUMENT = -2,
} leg_kinematics_status_t;

typedef struct {
    uint16_t ft_raw;  /* Feetech native position count. */
    float dm_rad;     /* DM4310 motor position in rad. */
} leg_actuator_state_t;

typedef struct {
    uint16_t fixed_ft_raw; /* Fixed Feetech position count used as geometry input. */
    float dm_rad;          /* DM4310 motor position in rad. */
} leg_fixed_ft_state_t;

typedef struct {
    float leg_length; /* Positive means the leg gets longer. */
    float leg_angle;  /* Positive means the leg swings forward. */
} leg_pose_t;

leg_kinematics_status_t leg_kinematics_full_forward(leg_side_t side,
                                                    const leg_actuator_state_t *actuator,
                                                    leg_pose_t *pose);

leg_kinematics_status_t leg_kinematics_full_inverse(leg_side_t side,
                                                    const leg_pose_t *pose,
                                                    leg_actuator_state_t *actuator);

leg_kinematics_status_t leg_kinematics_fixed_ft_forward(leg_side_t side,
                                                        const leg_fixed_ft_state_t *actuator,
                                                        leg_pose_t *pose);

#endif /* LEG_KINEMATICS_H */
