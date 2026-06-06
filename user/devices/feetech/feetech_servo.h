#ifndef FEETECH_SERVO_H
#define FEETECH_SERVO_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ── USART 句柄 ──────────────────────────────────────────────────
 * CubeMX 配置好 USART 后改为实际句柄名（如 huart1）
 * 飞特 SCSCL 系列波特率：1 Mbps，8N1
 * ────────────────────────────────────────────────────────────── */
#define FEETECH_HUART  huart1   /* CubeMX 配置好 USART 后改为实际句柄名 */

/* ── 舵机 ID ─────────────────────────────────────────────────── */
#define FEETECH_ID_LEFT   1
#define FEETECH_ID_RIGHT  2

/* ── 位置量程（步，0~1023 对应 0°~300°，512 为中位）────────── */
#define FEETECH_POS_MIN  0
#define FEETECH_POS_MAX  1023
#define FEETECH_POS_MID  512

/* ── 舵机状态 ────────────────────────────────────────────────── */
typedef struct {
    uint8_t  id;
    int16_t  pos;      /* 当前位置（步）*/
    int16_t  speed;    /* 当前速度（步/秒，带符号）*/
    int16_t  load;     /* 当前负载（带符号）*/
    uint8_t  voltage;  /* 电压（0.1V）*/
    uint8_t  temp;     /* 温度（°C）*/
} feetech_servo_t;

/* [0] 左腿  [1] 右腿 */
extern feetech_servo_t g_ft_servos[2];

/* ── 接口 ────────────────────────────────────────────────────── */

/** 模块初始化，须在 MX_USARTx_UART_Init() 之后调用 */
void feetech_servo_init(void);

/** 使能/失能扭矩 */
void feetech_servo_enable(uint8_t id);
void feetech_servo_disable(uint8_t id);

/**
 * @brief  控制单个舵机的位置和速度
 * @param  id    舵机 ID
 * @param  pos   目标位置（0~1023）
 * @param  speed 运动速度（步/秒，0=最大速度）
 * @param  acc   加速度（0~254，0=最大加速度）
 */
void feetech_servo_set_pos(uint8_t id, uint16_t pos, uint16_t speed, uint8_t acc);

/**
 * @brief  同步控制两个舵机（同一帧发出，同时启动）
 */
void feetech_servo_sync_set_pos(uint16_t pos_l, uint16_t pos_r,
                                 uint16_t speed, uint8_t acc);

/**
 * @brief  读取舵机反馈，更新 g_ft_servos[idx]
 * @note   阻塞约 2~5 ms，勿在 2kHz ISR 中调用
 * @retval 0 成功，-1 通信错误
 */
int feetech_servo_update(uint8_t idx);

#endif /* FEETECH_SERVO_H */
