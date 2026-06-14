#include "app.h"

#include "bsp_can.h"
#include "dm_imu.h"
#include "dm_motor.h"
#include "feetech_servo.h"
#include "../algorithm/leg_kinematics/leg_kinematics.h"
#include "stm32f4xx_hal.h"

dm_motor_t g_dm_motors[4];
volatile app_status_t g_app_status;
volatile app_cmd_t g_app_cmd;
volatile app_obs_t g_app_obs;

_Static_assert(APP_DM_LEFT_JOINT == DM_MOTOR_LEFT_JOINT_IDX, "DM index mismatch");
_Static_assert(APP_DM_RIGHT_JOINT == DM_MOTOR_RIGHT_JOINT_IDX, "DM index mismatch");
_Static_assert(APP_DM_LEFT_WHEEL == DM_MOTOR_LEFT_WHEEL_IDX, "DM index mismatch");
_Static_assert(APP_DM_RIGHT_WHEEL == DM_MOTOR_RIGHT_WHEEL_IDX, "DM index mismatch");

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

    app_obs_update_leg(APP_LEG_LEFT, LEG_SIDE_LEFT,
                       APP_FT_LEFT, APP_DM_LEFT_JOINT);
    app_obs_update_leg(APP_LEG_RIGHT, LEG_SIDE_RIGHT,
                       APP_FT_RIGHT, APP_DM_RIGHT_JOINT);
}

static void app_run_dm_manual(void)
{
    uint8_t send_mask = g_app_cmd.dm_send_mask;

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
}

void app_control_2khz(void)
{
    if (g_app_status.initialized == 0U) {
        return;
    }

    if (g_app_cmd.mode == APP_MODE_DM_MANUAL) {
        app_run_dm_manual();
    }

    g_app_status.control_ticks++;
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

    app_obs_update();
    g_app_status.background_ticks++;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        app_control_2khz();
    }
}
