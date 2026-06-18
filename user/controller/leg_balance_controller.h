#ifndef LEG_BALANCE_CONTROLLER_H
#define LEG_BALANCE_CONTROLLER_H

typedef struct {
    float kp;
    float kd;
    float bias;
    float output_limit;
    float slew_rate;
    float output;
} leg_balance_controller_t;

void leg_balance_controller_init(leg_balance_controller_t *ctl);
void leg_balance_controller_config(leg_balance_controller_t *ctl,
                                   float kp, float kd,
                                   float bias,
                                   float output_limit,
                                   float slew_rate);
void leg_balance_controller_reset(leg_balance_controller_t *ctl);
float leg_balance_controller_update(leg_balance_controller_t *ctl,
                                    float pitch_deg,
                                    float pitch_rate_rad_s,
                                    float dt_s);

#endif /* LEG_BALANCE_CONTROLLER_H */
