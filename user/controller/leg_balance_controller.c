#include "leg_balance_controller.h"

static float leg_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float leg_clampf(float x, float limit)
{
    float abs_limit = leg_absf(limit);

    if (x > abs_limit) {
        return abs_limit;
    }
    if (x < -abs_limit) {
        return -abs_limit;
    }
    return x;
}

void leg_balance_controller_init(leg_balance_controller_t *ctl)
{
    if (ctl == 0) {
        return;
    }

    ctl->kp = 0.0f;
    ctl->kd = 0.0f;
    ctl->bias = 0.0f;
    ctl->output_limit = 0.0f;
    ctl->slew_rate = 0.0f;
    ctl->output = 0.0f;
}

void leg_balance_controller_config(leg_balance_controller_t *ctl,
                                   float kp, float kd,
                                   float bias,
                                   float output_limit,
                                   float slew_rate)
{
    if (ctl == 0) {
        return;
    }

    ctl->kp = kp;
    ctl->kd = kd;
    ctl->bias = bias;
    ctl->output_limit = leg_absf(output_limit);
    ctl->slew_rate = leg_absf(slew_rate);
    leg_balance_controller_reset(ctl);
}

void leg_balance_controller_reset(leg_balance_controller_t *ctl)
{
    if (ctl == 0) {
        return;
    }

    ctl->output = 0.0f;
}

float leg_balance_controller_update(leg_balance_controller_t *ctl,
                                    float pitch_deg,
                                    float pitch_rate_rad_s,
                                    float dt_s)
{
    if ((ctl == 0) || (dt_s <= 0.0f)) {
        return 0.0f;
    }

    float target = ctl->bias +
                   ctl->kp * pitch_deg +
                   ctl->kd * pitch_rate_rad_s;
    target = leg_clampf(target, ctl->output_limit);

    if (ctl->slew_rate > 0.0f) {
        float delta_limit = ctl->slew_rate * dt_s;
        ctl->output += leg_clampf(target - ctl->output, delta_limit);
    } else {
        ctl->output = target;
    }
    ctl->output = leg_clampf(ctl->output, ctl->output_limit);

    return ctl->output;
}
