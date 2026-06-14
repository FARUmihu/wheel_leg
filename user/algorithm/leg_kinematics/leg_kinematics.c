#include "leg_kinematics.h"

static int leg_side_is_valid(leg_side_t side)
{
    return (side == LEG_SIDE_LEFT) || (side == LEG_SIDE_RIGHT);
}

leg_kinematics_status_t leg_kinematics_full_forward(leg_side_t side,
                                                    const leg_actuator_state_t *actuator,
                                                    leg_pose_t *pose)
{
    if ((actuator == 0) || (pose == 0) || !leg_side_is_valid(side)) {
        return LEG_KINEMATICS_BAD_ARGUMENT;
    }

    return LEG_KINEMATICS_NOT_IMPLEMENTED;
}

leg_kinematics_status_t leg_kinematics_full_inverse(leg_side_t side,
                                                    const leg_pose_t *pose,
                                                    leg_actuator_state_t *actuator)
{
    if ((pose == 0) || (actuator == 0) || !leg_side_is_valid(side)) {
        return LEG_KINEMATICS_BAD_ARGUMENT;
    }

    return LEG_KINEMATICS_NOT_IMPLEMENTED;
}

leg_kinematics_status_t leg_kinematics_fixed_ft_forward(leg_side_t side,
                                                        const leg_fixed_ft_state_t *actuator,
                                                        leg_pose_t *pose)
{
    if ((actuator == 0) || (pose == 0) || !leg_side_is_valid(side)) {
        return LEG_KINEMATICS_BAD_ARGUMENT;
    }

    return LEG_KINEMATICS_NOT_IMPLEMENTED;
}
