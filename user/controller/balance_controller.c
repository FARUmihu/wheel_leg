#include "balance_controller.h"

#define BALANCE_NONLINEAR_START_DEG 2.0f
#define BALANCE_NONLINEAR_SLOPE     0.12f
#define BALANCE_NONLINEAR_MAX_GAIN  0.65f

static float balance_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float balance_clampf(float x, float limit)
{
    float abs_limit = balance_absf(limit);

    if (x > abs_limit) {
        return abs_limit;
    }
    if (x < -abs_limit) {
        return -abs_limit;
    }
    return x;
}

static float balance_apply_deadband(float x, float deadband)
{
    float abs_deadband = balance_absf(deadband);

    if (x > abs_deadband) {
        return x - abs_deadband;
    }
    if (x < -abs_deadband) {
        return x + abs_deadband;
    }
    return 0.0f;
}

static float balance_shape_tilt_error(float tilt_error)
{
    float abs_error = balance_absf(tilt_error);

    if (abs_error <= BALANCE_NONLINEAR_START_DEG) {
        return tilt_error;
    }

    float gain = (abs_error - BALANCE_NONLINEAR_START_DEG) *
                 BALANCE_NONLINEAR_SLOPE;
    if (gain > BALANCE_NONLINEAR_MAX_GAIN) {
        gain = BALANCE_NONLINEAR_MAX_GAIN;
    }

    return tilt_error * (1.0f + gain);
}

void balance_controller_init(balance_controller_t *ctl)
{
    if (ctl == 0) {
        return;
    }

    ctl->kp = 0.0f;
    ctl->ki = 0.0f;
    ctl->kd = 0.0f;
    ctl->output_limit = 0.0f;
    ctl->integral_limit = 0.0f;
    ctl->deadband_deg = 0.0f;
    ctl->min_output = 0.0f;
    ctl->output_slew_rate = 0.0f;
    ctl->integral = 0.0f;
    ctl->output = 0.0f;
}

void balance_controller_config(balance_controller_t *ctl,
                               float kp, float ki, float kd,
                               float output_limit,
                               float integral_limit)
{
    if (ctl == 0) {
        return;
    }

    ctl->kp = kp;
    ctl->ki = ki;
    ctl->kd = kd;
    ctl->output_limit = balance_absf(output_limit);
    ctl->integral_limit = balance_absf(integral_limit);
    balance_controller_reset(ctl);
}

void balance_controller_set_response(balance_controller_t *ctl,
                                     float deadband_deg,
                                     float min_output,
                                     float output_slew_rate)
{
    if (ctl == 0) {
        return;
    }

    ctl->deadband_deg = balance_absf(deadband_deg);
    ctl->min_output = balance_absf(min_output);
    ctl->output_slew_rate = balance_absf(output_slew_rate);
}

void balance_controller_reset(balance_controller_t *ctl)
{
    if (ctl == 0) {
        return;
    }

    ctl->integral = 0.0f;
    ctl->output = 0.0f;
}

float balance_controller_update(balance_controller_t *ctl,
                                float target_pitch_deg,
                                float pitch_deg,
                                float pitch_rate_rad_s,
                                float dt_s)
{
    if ((ctl == 0) || (dt_s <= 0.0f)) {
        return 0.0f;
    }

    float tilt_error = balance_apply_deadband(pitch_deg - target_pitch_deg,
                                              ctl->deadband_deg);
    float proportional_error = balance_shape_tilt_error(tilt_error);

    ctl->integral += tilt_error * dt_s;
    ctl->integral = balance_clampf(ctl->integral, ctl->integral_limit);
    float target_output = ctl->kp * proportional_error +
                          ctl->ki * ctl->integral +
                          ctl->kd * pitch_rate_rad_s;
    target_output = balance_clampf(target_output, ctl->output_limit);
    if ((tilt_error != 0.0f) && (target_output != 0.0f)) {
        float min_output = ctl->min_output;

        if (min_output > ctl->output_limit) {
            min_output = ctl->output_limit;
        }
        if (balance_absf(target_output) < min_output) {
            target_output = (target_output > 0.0f) ? min_output : -min_output;
        }
    }

    if (ctl->output_slew_rate > 0.0f) {
        float delta_limit = ctl->output_slew_rate * dt_s;
        ctl->output += balance_clampf(target_output - ctl->output, delta_limit);
    } else {
        ctl->output = target_output;
    }
    ctl->output = balance_clampf(ctl->output, ctl->output_limit);

    return ctl->output;
}
