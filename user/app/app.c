#include "app.h"

#include "bsp_can.h"
#include "dm_imu.h"
#include "dm_motor.h"
#include "feetech_servo.h"
#include "stm32f4xx_hal.h"

dm_motor_t g_dm_motors[4];
volatile app_status_t g_app_status;

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

    g_app_status.control_ticks = 0;
    g_app_status.background_ticks = 0;
    g_app_status.initialized = 1;
}

void app_control_2khz(void)
{
    if (g_app_status.initialized == 0U) {
        return;
    }

    g_app_status.control_ticks++;
}

void app_background(void)
{
    if (g_app_status.initialized == 0U) {
        return;
    }

    g_app_status.background_ticks++;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        app_control_2khz();
    }
}
