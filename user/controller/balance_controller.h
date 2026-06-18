#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float wheel_pos_kp;
    float wheel_vel_kd;
    float output_limit;
    float integral_limit;
    float wheel_pos_limit;
    float wheel_pos_leak_rate;
    float deadband_deg;
    float min_output;
    float output_slew_rate;
    float integral;
    float wheel_position;
    float output;
} balance_controller_t;

void balance_controller_init(balance_controller_t *ctl);
void balance_controller_config(balance_controller_t *ctl,
                               float kp, float ki, float kd,
                               float output_limit,
                               float integral_limit);
void balance_controller_reset(balance_controller_t *ctl);
void balance_controller_set_response(balance_controller_t *ctl,
                                     float deadband_deg,
                                     float min_output,
                                     float output_slew_rate);
void balance_controller_set_wheel_response(balance_controller_t *ctl,
                                           float wheel_pos_kp,
                                           float wheel_vel_kd,
                                           float wheel_pos_limit,
                                           float wheel_pos_leak_rate);
float balance_controller_update(balance_controller_t *ctl,
                                float target_pitch_deg,
                                float pitch_deg,
                                float pitch_rate_rad_s,
                                float wheel_velocity_rad_s,
                                float dt_s);

#endif /* BALANCE_CONTROLLER_H */
