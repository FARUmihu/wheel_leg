#ifndef DM_IMU_H
#define DM_IMU_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ── CAN ID 配置 ─────────────────────────────────────────────────
 * IMU_CAN_ID : IMU 自身的 CAN ID（出厂默认 0x01，可通过上位机修改）
 * IMU_MST_ID : STM32 的 CAN ID（IMU 向此 ID 回复数据）
 * ────────────────────────────────────────────────────────────── */
#define IMU_CAN_ID  0x01
#define IMU_MST_ID  0x11

/* ── 数据量程（来自 DM-IMU 手册）────────────────────────────── */
#define IMU_ACCEL_MAX   235.2f    /* m/s²  */
#define IMU_ACCEL_MIN  -235.2f
#define IMU_GYRO_MAX    34.88f   /* rad/s */
#define IMU_GYRO_MIN   -34.88f
#define IMU_PITCH_MAX   90.0f    /* °     */
#define IMU_PITCH_MIN  -90.0f
#define IMU_ROLL_MAX    180.0f
#define IMU_ROLL_MIN   -180.0f
#define IMU_YAW_MAX     180.0f
#define IMU_YAW_MIN    -180.0f

/* ── IMU 数据结构体 ──────────────────────────────────────────── */
typedef struct {
    float pitch;      /* 俯仰角 (°) */
    float roll;       /* 横滚角 (°) */
    float yaw;        /* 偏航角 (°) */
    float gyro[3];    /* 角速度 xyz (rad/s) */
    float accel[3];   /* 加速度 xyz (m/s²)  */
    uint32_t rx_count;
    uint32_t can_rx_count;
    uint32_t tx_count;
    uint32_t tx_error_count;
    uint32_t last_update_ms;
    uint32_t last_can_id;
    uint8_t last_type;
    uint8_t last_len;
    uint8_t last_tx_status;
    uint8_t last_raw[8];
} dm_imu_t;

extern dm_imu_t g_imu;
extern volatile uint16_t g_imu_can_id;

/* ── 接口 ────────────────────────────────────────────────────── */

/**
 * @brief  初始化 IMU，配置为主动推送模式
 * @note   须在 bsp_can_init() 之后调用；
 *         主动模式下 IMU 按固定频率自动推送数据，无需轮询。
 */
void imu_init(void);

/**
 * @brief  请求 IMU 立即推送一次欧拉角 + 陀螺仪数据
 * @note   数据异步到达，调用后在下一次 CAN RX 回调中更新 g_imu；
 *         主动模式下通常不需要调用此函数。
 */
void imu_request_data(void);
uint8_t imu_is_online(uint32_t timeout_ms);

#endif /* DM_IMU_H */
