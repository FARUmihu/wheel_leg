#include "app.h"

#include "bsp_can.h"
#include "dm_imu.h"
#include "dm_motor.h"
#include "feetech_servo.h"
#include "../algorithm/leg_kinematics/leg_kinematics.h"
#include "../controller/balance_controller.h"
#include "../controller/leg_balance_controller.h"
#include "stm32f4xx_hal.h"

dm_motor_t g_dm_motors[4];
volatile app_status_t g_app_status;
volatile app_cmd_t g_app_cmd;
volatile app_obs_t g_app_obs;

static uint32_t s_imu_request_ms;
static uint8_t s_joint_hold_divider;
static float s_body_pitch_zero;
static float s_body_roll_zero;
static float s_body_yaw_zero;
static uint8_t s_body_zeroed;
static balance_controller_t s_balance_ctrl;
static leg_balance_controller_t s_leg_balance_ctrl;
static uint8_t s_wheel_send_divider;
static uint8_t s_wheel_enable_frames;
static uint8_t s_wheel_zero_divider;
static uint16_t s_wheel_zero_ticks;
static uint8_t s_leg_assist_divider;
static uint8_t s_leg_assist_active;
static float s_leg_assist_left_base_rad;
static float s_leg_assist_right_base_rad;
static float s_leg_assist_left_base_kp;
static float s_leg_assist_left_base_kd;
static float s_leg_assist_right_base_kp;
static float s_leg_assist_right_base_kd;
static uint8_t s_tune_stage;
static uint32_t s_tune_stage_ms;
volatile float g_balance_dbg_pitch_raw;
volatile float g_balance_dbg_pitch_pred;
volatile float g_balance_dbg_pitch_rate;
volatile float g_balance_dbg_pitch_age_s;
volatile float g_balance_dbg_wheel_vel;
volatile float g_balance_dbg_torque;
volatile uint32_t g_balance_dbg_ticks;

typedef struct {
    uint32_t tick;
    float pitch_raw;
    float pitch_pred;
    float pitch_rate;
    float wheel_vel;
    float torque;
} app_balance_dbg_sample_t;

#define APP_BALANCE_DBG_LOG_COUNT 256U
#define APP_BALANCE_DBG_LOG_DIVIDER 2U

volatile app_balance_dbg_sample_t g_balance_dbg_log[APP_BALANCE_DBG_LOG_COUNT];
volatile uint16_t g_balance_dbg_log_idx;
static uint8_t s_balance_dbg_log_divider;

#define APP_INITIAL_LEFT_X_MM      (-36.0f)
#define APP_INITIAL_LEFT_Y_MM      (-80.0f)
#define APP_INITIAL_RIGHT_X_MM      36.0f
#define APP_INITIAL_RIGHT_Y_MM     (-80.0f)
#define APP_INITIAL_FT_SPEED       200U
#define APP_INITIAL_FT_ACC           0U
#define APP_INITIAL_LEFT_DM_KP    60.0f
#define APP_INITIAL_LEFT_DM_KD     0.60f
#define APP_INITIAL_LEFT_DM_T     -4.0f
#define APP_INITIAL_RIGHT_DM_KP   60.0f
#define APP_INITIAL_RIGHT_DM_KD    0.60f
#define APP_INITIAL_RIGHT_DM_T     4.0f
#define APP_JOINT_HOLD_DIVIDER     2U
#define APP_WHEEL_SEND_DIVIDER     5U
#define APP_WHEEL_ENABLE_FRAMES   20U
#define APP_LEG_ASSIST_DIVIDER    40U
#define APP_LEG_ASSIST_DT_S       (APP_CONTROL_DT_S * (float)APP_LEG_ASSIST_DIVIDER)
#define APP_LEG_ASSIST_SLEW_RAD_S  0.10f
#define APP_LEG_ASSIST_LEFT_SIGN    1.0f
#define APP_LEG_ASSIST_RIGHT_SIGN  (-1.0f)
#define APP_LEG_ASSIST_LEFT_DM_KP  80.0f
#define APP_LEG_ASSIST_LEFT_DM_KD   0.80f
#define APP_LEG_ASSIST_RIGHT_DM_KP  90.0f
#define APP_LEG_ASSIST_RIGHT_DM_KD  0.90f
#define APP_CONTROL_DT_S           0.0005f
#define APP_BALANCE_DT_S           (APP_CONTROL_DT_S * (float)APP_WHEEL_SEND_DIVIDER)
#define APP_BALANCE_INTEGRAL_LIMIT  1.5f
#define APP_BALANCE_WHEEL_POS_KP    0.0f
#define APP_BALANCE_WHEEL_VEL_KD  (-0.010f)
#define APP_BALANCE_WHEEL_POS_LIMIT  18.0f
#define APP_BALANCE_WHEEL_POS_LEAK    0.15f
#define APP_BALANCE_PITCH_PREDICT_MAX_S  0.035f
#define APP_BALANCE_DEADBAND_DEG    0.0f
#define APP_BALANCE_MIN_TORQUE      0.04f
#define APP_BALANCE_TORQUE_SLEW   300.0f
#define APP_BALANCE_WHEEL_MIT_KD    0.03f
#define APP_BALANCE_TILT_STOP_DEG  30.0f
#define APP_BALANCE_TORQUE_SIGN     1.0f
#define APP_BALANCE_WHEEL_STATE_SIGN 1.0f
#define APP_WHEEL_ZERO_TICKS      400U
#define APP_DM_BIT(idx)            ((uint8_t)(1U << (idx)))
#define APP_DM_JOINT_MASK          ((uint8_t)(APP_DM_BIT(APP_DM_LEFT_JOINT) | APP_DM_BIT(APP_DM_RIGHT_JOINT)))
#define APP_DM_WHEEL_MASK          ((uint8_t)(APP_DM_BIT(APP_DM_LEFT_WHEEL) | APP_DM_BIT(APP_DM_RIGHT_WHEEL)))

#define APP_TUNE_AUTOSTART          1U
#define APP_TUNE_USE_LEG_ASSIST     0U
#define APP_TUNE_POSE_DELAY_MS   1500U
#define APP_TUNE_START_DELAY_MS  4500U
#define APP_TUNE_RUN_MS             0U
#define APP_TUNE_WHEEL_KP          0.350f
#define APP_TUNE_WHEEL_KI          0.001f
#define APP_TUNE_WHEEL_KD          0.580f
#define APP_TUNE_WHEEL_MAX         3.6f
#define APP_TUNE_LEG_KP           (-0.008f)
#define APP_TUNE_LEG_KD           (-0.0015f)
#define APP_TUNE_LEG_BIAS         (-0.035f)
#define APP_TUNE_LEG_MAX           0.10f

static void app_run_joint_hold(void);
static void app_run_balance(void);
static void app_run_wheel_zero_stop(void);
static void app_run_leg_assist(float pitch, float pitch_rate);
static void app_leg_assist_restore(void);
static void app_tune_autostart_task(void);

typedef uint8_t (*app_debug_u8_void_fn_t)(void);
typedef uint8_t (*app_debug_balance_start_fn_t)(float, float, float, float);
typedef uint8_t (*app_debug_leg_balance_start_fn_t)(float, float, float, float,
                                                    float, float, float, float);
typedef void (*app_debug_void_fn_t)(void);

static volatile app_debug_u8_void_fn_t s_debug_initial_pose_solve;
static volatile app_debug_u8_void_fn_t s_debug_initial_pose_apply;
static volatile app_debug_balance_start_fn_t s_debug_balance_start;
static volatile app_debug_leg_balance_start_fn_t s_debug_balance_leg_assist_start;
static volatile app_debug_void_fn_t s_debug_balance_stop;
static volatile app_debug_void_fn_t s_debug_leg_hold_stop;

_Static_assert(APP_DM_LEFT_JOINT == DM_MOTOR_LEFT_JOINT_IDX, "DM index mismatch");
_Static_assert(APP_DM_RIGHT_JOINT == DM_MOTOR_RIGHT_JOINT_IDX, "DM index mismatch");
_Static_assert(APP_DM_LEFT_WHEEL == DM_MOTOR_LEFT_WHEEL_IDX, "DM index mismatch");
_Static_assert(APP_DM_RIGHT_WHEEL == DM_MOTOR_RIGHT_WHEEL_IDX, "DM index mismatch");

static void app_keep_debug_symbols(void)
{
    s_debug_initial_pose_solve = app_initial_pose_solve;
    s_debug_initial_pose_apply = app_initial_pose_apply;
    s_debug_balance_start = app_balance_start;
    s_debug_balance_leg_assist_start = app_balance_leg_assist_start;
    s_debug_balance_stop = app_balance_stop;
    s_debug_leg_hold_stop = app_leg_hold_stop;
}

static void app_cmd_set_defaults(void)
{
    g_app_cmd.mode = APP_MODE_IDLE;
    g_app_cmd.dm_send_mask = 0U;
    g_app_cmd.dm_enable_mask = 0U;
    g_app_cmd.ft_write_mask = 0U;
    g_app_cmd.ft_read_mask = 0U;
    g_app_cmd.imu_request_once = 0U;

    for (uint8_t i = 0; i < APP_DM_COUNT; i++) {
        g_app_cmd.dm[i].p = 0.0f;
        g_app_cmd.dm[i].v = 0.0f;
        g_app_cmd.dm[i].kp = 0.0f;
        g_app_cmd.dm[i].kd = 0.0f;
        g_app_cmd.dm[i].t = 0.0f;
    }

    for (uint8_t i = 0; i < APP_FT_COUNT; i++) {
        g_app_cmd.ft[i].pos = FEETECH_POS_MID;
        g_app_cmd.ft[i].speed = 0U;
        g_app_cmd.ft[i].acc = 0U;
    }
}

static uint16_t app_ft_raw_from_servo(app_ft_index_t idx)
{
    int16_t raw = g_ft_servos[idx].pos;

    if (raw < 0) {
        return 0U;
    }

    return (uint16_t)raw;
}

static float app_wrap_deg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static float app_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float app_clampf(float x, float min_value, float max_value)
{
    if (x < min_value) {
        return min_value;
    }
    if (x > max_value) {
        return max_value;
    }
    return x;
}

static float app_predict_pitch_deg(float pitch_deg, float pitch_rate_rad_s)
{
    if (g_imu.last_euler_update_ms == 0U) {
        return pitch_deg;
    }

    float age_s = (float)(HAL_GetTick() - g_imu.last_euler_update_ms) * 0.001f;
    age_s = app_clampf(age_s, 0.0f, APP_BALANCE_PITCH_PREDICT_MAX_S);
    g_balance_dbg_pitch_age_s = age_s;
    return app_wrap_deg(pitch_deg + pitch_rate_rad_s * 57.29578f * age_s);
}

static void app_set_dm_cmd(app_dm_index_t idx,
                           float p, float v, float kp, float kd, float t)
{
    g_app_cmd.dm[idx].p = p;
    g_app_cmd.dm[idx].v = v;
    g_app_cmd.dm[idx].kp = kp;
    g_app_cmd.dm[idx].kd = kd;
    g_app_cmd.dm[idx].t = t;
}

static void app_obs_update_leg(app_leg_index_t leg_idx,
                               leg_side_t side,
                               app_ft_index_t ft_idx,
                               app_dm_index_t dm_idx)
{
    uint16_t ft_raw = app_ft_raw_from_servo(ft_idx);
    float dm_rad = g_dm_motors[dm_idx].pos;
    leg_actuator_state_t actuator;
    leg_kinematics_result_t result;

    actuator.ft_raw = ft_raw;
    actuator.dm_rad = dm_rad;

    leg_kinematics_status_t st =
        leg_kinematics_forward_from_actuator(side, &actuator, &result);

    g_app_obs.leg[leg_idx].ft_raw = ft_raw;
    g_app_obs.leg[leg_idx].dm_rad = dm_rad;
    g_app_obs.leg[leg_idx].theta1_deg = result.joint.theta1_deg;
    g_app_obs.leg[leg_idx].theta2_deg = result.joint.theta2_deg;
    g_app_obs.leg[leg_idx].foot_d_x_mm = result.foot_d.x_mm;
    g_app_obs.leg[leg_idx].foot_d_y_mm = result.foot_d.y_mm;
    g_app_obs.leg[leg_idx].usable = (st == LEG_KINEMATICS_OK) ? 1U : 0U;
}

static void app_obs_update(void)
{
    for (uint8_t i = 0; i < APP_DM_COUNT; i++) {
        g_app_obs.dm[i].err = g_dm_motors[i].err;
        g_app_obs.dm[i].pos = g_dm_motors[i].pos;
        g_app_obs.dm[i].vel = g_dm_motors[i].vel;
        g_app_obs.dm[i].vel_filtered = g_dm_motors[i].vel_filtered;
        g_app_obs.dm[i].torque = g_dm_motors[i].torque;
        g_app_obs.dm[i].temp_mos = g_dm_motors[i].temp_mos;
        g_app_obs.dm[i].temp_rotor = g_dm_motors[i].temp_rotor;
    }

    for (uint8_t i = 0; i < APP_FT_COUNT; i++) {
        g_app_obs.ft[i].pos = g_ft_servos[i].pos;
        g_app_obs.ft[i].speed = g_ft_servos[i].speed;
        g_app_obs.ft[i].load = g_ft_servos[i].load;
        g_app_obs.ft[i].voltage = g_ft_servos[i].voltage;
        g_app_obs.ft[i].temp = g_ft_servos[i].temp;
    }

    g_app_obs.imu.pitch = g_imu.pitch;
    g_app_obs.imu.roll = g_imu.roll;
    g_app_obs.imu.yaw = g_imu.yaw;
    for (uint8_t i = 0; i < 3U; i++) {
        g_app_obs.imu.gyro[i] = g_imu.gyro[i];
        g_app_obs.imu.accel[i] = g_imu.accel[i];
    }
    g_app_obs.imu.rx_count = g_imu.rx_count;
    g_app_obs.imu.can_rx_count = g_imu.can_rx_count;
    g_app_obs.imu.tx_count = g_imu.tx_count;
    g_app_obs.imu.tx_error_count = g_imu.tx_error_count;
    g_app_obs.imu.last_update_ms = g_imu.last_update_ms;
    g_app_obs.imu.last_can_id = g_imu.last_can_id;
    g_app_obs.imu.online = imu_is_online(100U);
    g_app_obs.imu.last_type = g_imu.last_type;
    g_app_obs.imu.last_tx_status = g_imu.last_tx_status;

    g_app_obs.body.pitch = app_wrap_deg(g_app_obs.imu.pitch - s_body_pitch_zero);
    g_app_obs.body.roll = app_wrap_deg(g_app_obs.imu.roll - s_body_roll_zero);
    g_app_obs.body.yaw = app_wrap_deg(g_app_obs.imu.yaw - s_body_yaw_zero);
    for (uint8_t i = 0; i < 3U; i++) {
        g_app_obs.body.gyro[i] = g_app_obs.imu.gyro[i];
        g_app_obs.body.accel[i] = g_app_obs.imu.accel[i];
    }
    g_app_obs.body.online = g_app_obs.imu.online;
    g_app_obs.body.zeroed = s_body_zeroed;

    app_obs_update_leg(APP_LEG_LEFT, LEG_SIDE_LEFT,
                       APP_FT_LEFT, APP_DM_LEFT_JOINT);
    app_obs_update_leg(APP_LEG_RIGHT, LEG_SIDE_RIGHT,
                       APP_FT_RIGHT, APP_DM_RIGHT_JOINT);
}

static void app_send_dm_mask(uint8_t send_mask)
{
    for (uint8_t i = 0; i < APP_DM_COUNT; i++) {
        uint8_t bit = (uint8_t)(1U << i);

        if ((send_mask & bit) == 0U) {
            continue;
        }

        float p = g_app_cmd.dm[i].p;
        float v = g_app_cmd.dm[i].v;
        float kp = g_app_cmd.dm[i].kp;
        float kd = g_app_cmd.dm[i].kd;
        float t = g_app_cmd.dm[i].t;

        dm_motor_send_mit(&g_dm_motors[i], p, v, kp, kd, t);
    }
}

static void app_run_dm_manual(void)
{
    app_send_dm_mask(g_app_cmd.dm_send_mask);
}

static void app_run_ft_manual(void)
{
    uint8_t write_mask = g_app_cmd.ft_write_mask;
    uint8_t read_mask = g_app_cmd.ft_read_mask;

    for (uint8_t i = 0; i < APP_FT_COUNT; i++) {
        uint8_t bit = (uint8_t)(1U << i);

        if ((write_mask & bit) != 0U) {
            uint16_t pos = g_app_cmd.ft[i].pos;
            uint16_t speed = g_app_cmd.ft[i].speed;
            uint8_t acc = g_app_cmd.ft[i].acc;

            feetech_servo_set_pos(g_ft_servos[i].id, pos, speed, acc);
            g_app_cmd.ft_write_mask = (uint8_t)(g_app_cmd.ft_write_mask & (uint8_t)(~bit));
        }

        if ((read_mask & bit) != 0U) {
            feetech_servo_update(i);
            g_app_cmd.ft_read_mask = (uint8_t)(g_app_cmd.ft_read_mask & (uint8_t)(~bit));
        }
    }
}

static void app_run_imu_read(void)
{
    if (g_app_cmd.imu_request_once == 0U) {
        return;
    }

    imu_request_data();
    g_app_cmd.imu_request_once = 0U;
}

static void app_run_imu_periodic(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - s_imu_request_ms) < 20U) {
        return;
    }

    imu_request_data();
    s_imu_request_ms = now;
}

void app_init(void)
{
    dm_motor_init(&g_dm_motors[DM_MOTOR_LEFT_JOINT_IDX], &hcan2,
                  DM_MOTOR_LEFT_JOINT_ID,
                  DM4310_V_MAX, DM4310_V_MIN, DM4310_T_MAX, DM4310_T_MIN);
    dm_motor_init(&g_dm_motors[DM_MOTOR_RIGHT_JOINT_IDX], &hcan2,
                  DM_MOTOR_RIGHT_JOINT_ID,
                  DM4310_V_MAX, DM4310_V_MIN, DM4310_T_MAX, DM4310_T_MIN);
    dm_motor_init(&g_dm_motors[DM_MOTOR_LEFT_WHEEL_IDX], &hcan2,
                  DM_MOTOR_LEFT_WHEEL_ID,
                  DM3510_V_MAX, DM3510_V_MIN, DM3510_T_MAX, DM3510_T_MIN);
    dm_motor_init(&g_dm_motors[DM_MOTOR_RIGHT_WHEEL_IDX], &hcan2,
                  DM_MOTOR_RIGHT_WHEEL_ID,
                  DM3510_V_MAX, DM3510_V_MIN, DM3510_T_MAX, DM3510_T_MIN);

    bsp_can_init();
    feetech_servo_init();
    imu_init();

    app_cmd_set_defaults();
    app_obs_update();

    g_app_status.control_ticks = 0;
    g_app_status.background_ticks = 0;
    g_app_status.initialized = 1;
    s_imu_request_ms = HAL_GetTick();
    s_joint_hold_divider = 0U;
    s_wheel_send_divider = 0U;
    s_wheel_enable_frames = 0U;
    s_wheel_zero_divider = 0U;
    s_wheel_zero_ticks = 0U;
    s_leg_assist_divider = 0U;
    s_leg_assist_active = 0U;
    s_leg_assist_left_base_rad = 0.0f;
    s_leg_assist_right_base_rad = 0.0f;
    s_leg_assist_left_base_kp = 0.0f;
    s_leg_assist_left_base_kd = 0.0f;
    s_leg_assist_right_base_kp = 0.0f;
    s_leg_assist_right_base_kd = 0.0f;
    s_tune_stage = 0U;
    s_tune_stage_ms = HAL_GetTick();
    balance_controller_init(&s_balance_ctrl);
    leg_balance_controller_init(&s_leg_balance_ctrl);
    app_keep_debug_symbols();
}

void app_control_2khz(void)
{
    if (g_app_status.initialized == 0U) {
        return;
    }

    app_run_wheel_zero_stop();

    if (g_app_cmd.mode == APP_MODE_DM_MANUAL) {
        app_run_dm_manual();
    } else if (g_app_cmd.mode == APP_MODE_LEG_HOLD) {
        app_run_joint_hold();
    } else if ((g_app_cmd.mode == APP_MODE_BALANCE_TEST) ||
               (g_app_cmd.mode == APP_MODE_BALANCE_LEG_ASSIST)) {
        app_run_balance();
    }

    g_app_status.control_ticks++;
}

static void app_run_joint_hold(void)
{
    uint8_t joint_mask = (uint8_t)(g_app_cmd.dm_send_mask & APP_DM_JOINT_MASK);

    if (joint_mask == 0U) {
        return;
    }

    s_joint_hold_divider++;
    if (s_joint_hold_divider < APP_JOINT_HOLD_DIVIDER) {
        return;
    }
    s_joint_hold_divider = 0U;

    app_send_dm_mask(joint_mask);
}

static void app_balance_send_wheels(float forward_torque)
{
    float t = app_clampf(forward_torque * APP_BALANCE_TORQUE_SIGN,
                         DM3510_T_MIN,
                         DM3510_T_MAX);

    app_set_dm_cmd(APP_DM_LEFT_WHEEL, 0.0f, 0.0f, 0.0f,
                   APP_BALANCE_WHEEL_MIT_KD, t);
    app_set_dm_cmd(APP_DM_RIGHT_WHEEL, 0.0f, 0.0f, 0.0f,
                   APP_BALANCE_WHEEL_MIT_KD, -t);

    app_send_dm_mask(APP_DM_WHEEL_MASK);
}

static void app_run_wheel_zero_stop(void)
{
    if (s_wheel_zero_ticks == 0U) {
        return;
    }

    s_wheel_zero_ticks--;
    s_wheel_zero_divider++;
    if (s_wheel_zero_divider >= APP_WHEEL_SEND_DIVIDER) {
        s_wheel_zero_divider = 0U;
        app_balance_send_wheels(0.0f);
    }

    if (s_wheel_zero_ticks == 0U) {
        dm_motor_disable(&g_dm_motors[APP_DM_LEFT_WHEEL]);
        dm_motor_disable(&g_dm_motors[APP_DM_RIGHT_WHEEL]);
    }
}

static void app_run_balance(void)
{
    app_run_joint_hold();

    s_wheel_send_divider++;
    if (s_wheel_send_divider < APP_WHEEL_SEND_DIVIDER) {
        return;
    }
    s_wheel_send_divider = 0U;

    if ((imu_is_online(100U) == 0U) || (s_body_zeroed == 0U)) {
        balance_controller_reset(&s_balance_ctrl);
        app_balance_send_wheels(0.0f);
        return;
    }

    if (s_wheel_enable_frames > 0U) {
        dm_motor_enable(&g_dm_motors[APP_DM_LEFT_WHEEL]);
        dm_motor_enable(&g_dm_motors[APP_DM_RIGHT_WHEEL]);
        app_balance_send_wheels(0.0f);
        s_wheel_enable_frames--;
        return;
    }

    float pitch = app_wrap_deg(g_imu.pitch - s_body_pitch_zero);
    float pitch_rate = g_imu.gyro[1];
    float pitch_pred = app_predict_pitch_deg(pitch, pitch_rate);

    if (app_absf(pitch_pred) > APP_BALANCE_TILT_STOP_DEG) {
        app_balance_stop();
        return;
    }

    if (g_app_cmd.mode == APP_MODE_BALANCE_LEG_ASSIST) {
        app_run_leg_assist(pitch_pred, pitch_rate);
    }

    float wheel_forward_vel =
        APP_BALANCE_WHEEL_STATE_SIGN *
        0.5f * (g_dm_motors[APP_DM_LEFT_WHEEL].vel_filtered -
                g_dm_motors[APP_DM_RIGHT_WHEEL].vel_filtered);
    float torque = balance_controller_update(&s_balance_ctrl,
                                             0.0f,
                                             pitch_pred,
                                             pitch_rate,
                                             wheel_forward_vel,
                                             APP_BALANCE_DT_S);
    torque = app_clampf(torque, -s_balance_ctrl.output_limit,
                        s_balance_ctrl.output_limit);
    g_balance_dbg_pitch_raw = pitch;
    g_balance_dbg_pitch_pred = pitch_pred;
    g_balance_dbg_pitch_rate = pitch_rate;
    g_balance_dbg_wheel_vel = wheel_forward_vel;
    g_balance_dbg_torque = torque;
    g_balance_dbg_ticks++;
    s_balance_dbg_log_divider++;
    if (s_balance_dbg_log_divider >= APP_BALANCE_DBG_LOG_DIVIDER) {
        uint16_t idx = g_balance_dbg_log_idx;

        s_balance_dbg_log_divider = 0U;
        g_balance_dbg_log[idx].tick = g_app_status.control_ticks;
        g_balance_dbg_log[idx].pitch_raw = pitch;
        g_balance_dbg_log[idx].pitch_pred = pitch_pred;
        g_balance_dbg_log[idx].pitch_rate = pitch_rate;
        g_balance_dbg_log[idx].wheel_vel = wheel_forward_vel;
        g_balance_dbg_log[idx].torque = torque;
        idx++;
        if (idx >= APP_BALANCE_DBG_LOG_COUNT) {
            idx = 0U;
        }
        g_balance_dbg_log_idx = idx;
    }
    app_balance_send_wheels(torque);
}

static void app_run_leg_assist(float pitch, float pitch_rate)
{
    if (s_leg_assist_active == 0U) {
        return;
    }

    s_leg_assist_divider++;
    if (s_leg_assist_divider < APP_LEG_ASSIST_DIVIDER) {
        return;
    }
    s_leg_assist_divider = 0U;

    float offset =
        leg_balance_controller_update(&s_leg_balance_ctrl,
                                      pitch,
                                      pitch_rate,
                                      APP_LEG_ASSIST_DT_S);

    g_app_cmd.dm[APP_DM_LEFT_JOINT].p =
        s_leg_assist_left_base_rad + APP_LEG_ASSIST_LEFT_SIGN * offset;
    g_app_cmd.dm[APP_DM_RIGHT_JOINT].p =
        s_leg_assist_right_base_rad + APP_LEG_ASSIST_RIGHT_SIGN * offset;
}

static void app_leg_assist_restore(void)
{
    if (s_leg_assist_active == 0U) {
        return;
    }

    g_app_cmd.dm[APP_DM_LEFT_JOINT].p = s_leg_assist_left_base_rad;
    g_app_cmd.dm[APP_DM_RIGHT_JOINT].p = s_leg_assist_right_base_rad;
    g_app_cmd.dm[APP_DM_LEFT_JOINT].kp = s_leg_assist_left_base_kp;
    g_app_cmd.dm[APP_DM_LEFT_JOINT].kd = s_leg_assist_left_base_kd;
    g_app_cmd.dm[APP_DM_RIGHT_JOINT].kp = s_leg_assist_right_base_kp;
    g_app_cmd.dm[APP_DM_RIGHT_JOINT].kd = s_leg_assist_right_base_kd;
    s_leg_assist_active = 0U;
    s_leg_assist_divider = 0U;
    leg_balance_controller_reset(&s_leg_balance_ctrl);
}

void app_background(void)
{
    if (g_app_status.initialized == 0U) {
        return;
    }

    switch (g_app_cmd.mode) {
        case APP_MODE_FT_MANUAL:
            app_run_ft_manual();
            break;
        case APP_MODE_IMU_READ:
            app_run_imu_read();
            break;
        default:
            break;
    }

    app_run_imu_periodic();
    app_obs_update();
    app_tune_autostart_task();
    g_app_status.background_ticks++;
}

static void app_tune_autostart_task(void)
{
    if (APP_TUNE_AUTOSTART == 0U) {
        return;
    }

    uint32_t now = HAL_GetTick();

    if (s_tune_stage == 0U) {
        if ((now - s_tune_stage_ms) < APP_TUNE_POSE_DELAY_MS) {
            return;
        }
        if (app_initial_pose_apply() != 0U) {
            s_tune_stage = 1U;
            s_tune_stage_ms = now;
        }
        return;
    }

    if (s_tune_stage == 1U) {
        if (((now - s_tune_stage_ms) < APP_TUNE_START_DELAY_MS) ||
            (g_app_obs.body.online == 0U)) {
            return;
        }
        if (app_imu_zero_current() == 0U) {
            return;
        }

        uint8_t ok;
        if (APP_TUNE_USE_LEG_ASSIST != 0U) {
            ok = app_balance_leg_assist_start(APP_TUNE_WHEEL_KP,
                                              APP_TUNE_WHEEL_KI,
                                              APP_TUNE_WHEEL_KD,
                                              APP_TUNE_WHEEL_MAX,
                                              APP_TUNE_LEG_KP,
                                              APP_TUNE_LEG_KD,
                                              APP_TUNE_LEG_BIAS,
                                              APP_TUNE_LEG_MAX);
        } else {
            ok = app_balance_start(APP_TUNE_WHEEL_KP,
                                   APP_TUNE_WHEEL_KI,
                                   APP_TUNE_WHEEL_KD,
                                   APP_TUNE_WHEEL_MAX);
        }

        if (ok != 0U) {
            s_tune_stage = 2U;
            s_tune_stage_ms = now;
        }
        return;
    }

    if ((s_tune_stage == 2U) && (APP_TUNE_RUN_MS > 0U) &&
        ((now - s_tune_stage_ms) >= APP_TUNE_RUN_MS)) {
        app_balance_stop();
        s_tune_stage = 3U;
    }
}

uint8_t app_imu_zero_current(void)
{
    if (g_app_obs.imu.online == 0U) {
        return 0U;
    }

    s_body_pitch_zero = g_app_obs.imu.pitch;
    s_body_roll_zero = g_app_obs.imu.roll;
    s_body_yaw_zero = g_app_obs.imu.yaw;
    s_body_zeroed = 1U;
    app_obs_update();
    return 1U;
}

void app_imu_zero_clear(void)
{
    s_body_pitch_zero = 0.0f;
    s_body_roll_zero = 0.0f;
    s_body_yaw_zero = 0.0f;
    s_body_zeroed = 0U;
    app_obs_update();
}

uint8_t app_leg_pose_solve_to_cmd(float left_x_mm, float left_y_mm,
                                  float right_x_mm, float right_y_mm)
{
    leg_point_t left = {left_x_mm, left_y_mm};
    leg_point_t right = {right_x_mm, right_y_mm};
    leg_actuator_state_t left_actuator;
    leg_actuator_state_t right_actuator;
    leg_kinematics_result_t left_result;
    leg_kinematics_result_t right_result;

    leg_kinematics_status_t left_status =
        leg_kinematics_inverse_to_actuator(LEG_SIDE_LEFT,
                                           &left,
                                           &left_actuator,
                                           &left_result);
    leg_kinematics_status_t right_status =
        leg_kinematics_inverse_to_actuator(LEG_SIDE_RIGHT,
                                           &right,
                                           &right_actuator,
                                           &right_result);

    if ((left_status != LEG_KINEMATICS_OK) ||
        (right_status != LEG_KINEMATICS_OK)) {
        return 0U;
    }

    g_app_cmd.ft[APP_FT_LEFT].pos = left_actuator.ft_raw;
    g_app_cmd.ft[APP_FT_LEFT].speed = APP_INITIAL_FT_SPEED;
    g_app_cmd.ft[APP_FT_LEFT].acc = APP_INITIAL_FT_ACC;
    g_app_cmd.ft[APP_FT_RIGHT].pos = right_actuator.ft_raw;
    g_app_cmd.ft[APP_FT_RIGHT].speed = APP_INITIAL_FT_SPEED;
    g_app_cmd.ft[APP_FT_RIGHT].acc = APP_INITIAL_FT_ACC;

    app_set_dm_cmd(APP_DM_LEFT_JOINT,
                   left_actuator.dm_rad, 0.0f,
                   APP_INITIAL_LEFT_DM_KP,
                   APP_INITIAL_LEFT_DM_KD,
                   APP_INITIAL_LEFT_DM_T);
    app_set_dm_cmd(APP_DM_RIGHT_JOINT,
                   right_actuator.dm_rad, 0.0f,
                   APP_INITIAL_RIGHT_DM_KP,
                   APP_INITIAL_RIGHT_DM_KD,
                   APP_INITIAL_RIGHT_DM_T);

    return 1U;
}

uint8_t app_leg_pose_apply(float left_x_mm, float left_y_mm,
                           float right_x_mm, float right_y_mm)
{
    if (app_leg_pose_solve_to_cmd(left_x_mm, left_y_mm,
                                  right_x_mm, right_y_mm) == 0U) {
        return 0U;
    }

    feetech_servo_enable(FEETECH_ID_LEFT);
    feetech_servo_enable(FEETECH_ID_RIGHT);
    feetech_servo_sync_set_pos(g_app_cmd.ft[APP_FT_LEFT].pos,
                               g_app_cmd.ft[APP_FT_RIGHT].pos,
                               g_app_cmd.ft[APP_FT_LEFT].speed,
                               g_app_cmd.ft[APP_FT_LEFT].acc);

    dm_motor_enable(&g_dm_motors[APP_DM_LEFT_JOINT]);
    dm_motor_enable(&g_dm_motors[APP_DM_RIGHT_JOINT]);
    g_app_cmd.dm_send_mask = (uint8_t)(g_app_cmd.dm_send_mask | APP_DM_JOINT_MASK);
    g_app_cmd.mode = APP_MODE_LEG_HOLD;
    return 1U;
}

uint8_t app_initial_pose_solve(void)
{
    return app_leg_pose_solve_to_cmd(APP_INITIAL_LEFT_X_MM,
                                     APP_INITIAL_LEFT_Y_MM,
                                     APP_INITIAL_RIGHT_X_MM,
                                     APP_INITIAL_RIGHT_Y_MM);
}

uint8_t app_initial_pose_apply(void)
{
    return app_leg_pose_apply(APP_INITIAL_LEFT_X_MM,
                              APP_INITIAL_LEFT_Y_MM,
                              APP_INITIAL_RIGHT_X_MM,
                              APP_INITIAL_RIGHT_Y_MM);
}

uint8_t app_balance_start(float pitch_kp, float pitch_ki,
                          float pitch_kd, float max_torque)
{
    if (g_app_obs.body.online == 0U) {
        return 0U;
    }

    if (s_body_zeroed == 0U) {
        if (app_imu_zero_current() == 0U) {
            return 0U;
        }
    }

    balance_controller_config(&s_balance_ctrl,
                              pitch_kp, pitch_ki, pitch_kd,
                              app_clampf(max_torque, 0.0f, DM3510_T_MAX),
                              APP_BALANCE_INTEGRAL_LIMIT);
    balance_controller_set_response(&s_balance_ctrl,
                                    APP_BALANCE_DEADBAND_DEG,
                                    APP_BALANCE_MIN_TORQUE,
                                    APP_BALANCE_TORQUE_SLEW);
    balance_controller_set_wheel_response(&s_balance_ctrl,
                                          APP_BALANCE_WHEEL_POS_KP,
                                          APP_BALANCE_WHEEL_VEL_KD,
                                          APP_BALANCE_WHEEL_POS_LIMIT,
                                          APP_BALANCE_WHEEL_POS_LEAK);
    s_wheel_send_divider = 0U;
    s_wheel_enable_frames = APP_WHEEL_ENABLE_FRAMES;
    s_wheel_zero_ticks = 0U;
    s_wheel_zero_divider = 0U;
    s_leg_assist_active = 0U;

    app_set_dm_cmd(APP_DM_LEFT_WHEEL, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    app_set_dm_cmd(APP_DM_RIGHT_WHEEL, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    dm_motor_enable(&g_dm_motors[APP_DM_LEFT_WHEEL]);
    dm_motor_enable(&g_dm_motors[APP_DM_RIGHT_WHEEL]);
    g_app_cmd.dm_send_mask = (uint8_t)(g_app_cmd.dm_send_mask | APP_DM_WHEEL_MASK);
    app_balance_send_wheels(0.0f);
    g_app_cmd.mode = APP_MODE_BALANCE_TEST;
    return 1U;
}

uint8_t app_balance_leg_assist_start(float pitch_kp, float pitch_ki,
                                     float pitch_kd, float max_torque,
                                     float leg_kp, float leg_kd,
                                     float leg_bias_rad,
                                     float leg_max_offset_rad)
{
    if (app_balance_start(pitch_kp, pitch_ki, pitch_kd, max_torque) == 0U) {
        return 0U;
    }

    s_leg_assist_left_base_rad = g_app_cmd.dm[APP_DM_LEFT_JOINT].p;
    s_leg_assist_right_base_rad = g_app_cmd.dm[APP_DM_RIGHT_JOINT].p;
    s_leg_assist_left_base_kp = g_app_cmd.dm[APP_DM_LEFT_JOINT].kp;
    s_leg_assist_left_base_kd = g_app_cmd.dm[APP_DM_LEFT_JOINT].kd;
    s_leg_assist_right_base_kp = g_app_cmd.dm[APP_DM_RIGHT_JOINT].kp;
    s_leg_assist_right_base_kd = g_app_cmd.dm[APP_DM_RIGHT_JOINT].kd;
    s_leg_assist_divider = 0U;
    s_leg_assist_active = 1U;
    g_app_cmd.dm[APP_DM_LEFT_JOINT].kp = APP_LEG_ASSIST_LEFT_DM_KP;
    g_app_cmd.dm[APP_DM_LEFT_JOINT].kd = APP_LEG_ASSIST_LEFT_DM_KD;
    g_app_cmd.dm[APP_DM_RIGHT_JOINT].kp = APP_LEG_ASSIST_RIGHT_DM_KP;
    g_app_cmd.dm[APP_DM_RIGHT_JOINT].kd = APP_LEG_ASSIST_RIGHT_DM_KD;
    leg_balance_controller_config(&s_leg_balance_ctrl,
                                  leg_kp,
                                  leg_kd,
                                  leg_bias_rad,
                                  leg_max_offset_rad,
                                  APP_LEG_ASSIST_SLEW_RAD_S);
    g_app_cmd.mode = APP_MODE_BALANCE_LEG_ASSIST;
    return 1U;
}

void app_balance_stop(void)
{
    s_wheel_enable_frames = 0U;
    app_leg_assist_restore();
    app_balance_send_wheels(0.0f);
    s_wheel_zero_ticks = APP_WHEEL_ZERO_TICKS;
    s_wheel_zero_divider = 0U;
    g_app_cmd.dm_send_mask = (uint8_t)(g_app_cmd.dm_send_mask & (uint8_t)(~APP_DM_WHEEL_MASK));
    app_set_dm_cmd(APP_DM_LEFT_WHEEL, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    app_set_dm_cmd(APP_DM_RIGHT_WHEEL, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    balance_controller_reset(&s_balance_ctrl);

    if ((g_app_cmd.mode == APP_MODE_BALANCE_TEST) ||
        (g_app_cmd.mode == APP_MODE_BALANCE_LEG_ASSIST)) {
        g_app_cmd.mode = ((g_app_cmd.dm_send_mask & APP_DM_JOINT_MASK) != 0U) ?
                         APP_MODE_LEG_HOLD : APP_MODE_IDLE;
    }
}

void app_leg_hold_stop(void)
{
    g_app_cmd.dm_send_mask = (uint8_t)(g_app_cmd.dm_send_mask & (uint8_t)(~APP_DM_JOINT_MASK));
    dm_motor_disable(&g_dm_motors[APP_DM_LEFT_JOINT]);
    dm_motor_disable(&g_dm_motors[APP_DM_RIGHT_JOINT]);
    feetech_servo_disable(FEETECH_ID_LEFT);
    feetech_servo_disable(FEETECH_ID_RIGHT);

    if (g_app_cmd.mode == APP_MODE_LEG_HOLD) {
        g_app_cmd.mode = APP_MODE_IDLE;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        app_control_2khz();
    }
}
