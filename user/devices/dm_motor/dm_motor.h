#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "bsp_can.h"

/* ================================================================
 * MIT 模式通用参数（DM4310 / DM3510 共用）
 * ================================================================ */
#define DM_P_MAX     12.5f
#define DM_P_MIN    -12.5f
#define DM_KP_MAX   500.0f
#define DM_KP_MIN     0.0f
#define DM_KD_MAX     5.0f
#define DM_KD_MIN     0.0f
#define DM_EMA_ALPHA  0.2f   /* 速度 EMA 滤波系数 */

/* ================================================================
 * DM4310 关节电机量程
 *   额定转速 ±30 rad/s，峰值力矩 ±10 N·m
 * ================================================================ */
#define DM4310_V_MAX   30.0f
#define DM4310_V_MIN  -30.0f
#define DM4310_T_MAX   10.0f
#define DM4310_T_MIN  -10.0f

/* ================================================================
 * DM3510 轮毂电机量程
 *   TODO: 以下数值需对照 DM3510 手册核实后修改
 * ================================================================ */
#define DM3510_V_MAX   45.0f
#define DM3510_V_MIN  -45.0f
#define DM3510_T_MAX    4.0f
#define DM3510_T_MIN   -4.0f

/* ================================================================
 * 电机逻辑索引与 CAN ID
 * ================================================================ */
#define DM_MOTOR_LEFT_JOINT_IDX    0
#define DM_MOTOR_RIGHT_JOINT_IDX   1
#define DM_MOTOR_LEFT_WHEEL_IDX    2
#define DM_MOTOR_RIGHT_WHEEL_IDX   3

#define DM_MOTOR_LEFT_JOINT_ID     1
#define DM_MOTOR_LEFT_WHEEL_ID     2
#define DM_MOTOR_RIGHT_JOINT_ID    4
#define DM_MOTOR_RIGHT_WHEEL_ID    5

/* ================================================================
 * 电机实例结构体（DM4310 / DM3510 共用）
 * ================================================================ */
typedef struct {
    /* 配置（初始化时填写）*/
    CAN_HandleTypeDef *hcan;   /* 所在 CAN 总线 */
    uint16_t           id;     /* 电机 CAN ID   */
    float v_max, v_min;        /* 该型号速度量程 */
    float t_max, t_min;        /* 该型号力矩量程 */
    /* 反馈数据（由 RX 回调更新）*/
    uint8_t err;               /* 错误码 */
    float   pos;               /* 位置 (rad)    */
    float   vel;               /* 速度 (rad/s)  */
    float   vel_filtered;      /* EMA 滤波速度  */
    float   torque;            /* 力矩 (N·m)    */
    int8_t  temp_mos;          /* MOS 温度 (°C) */
    int8_t  temp_rotor;        /* 线圈温度 (°C) */
} dm_motor_t;

/* ================================================================
 * 全局电机数组
 *   [0] 左腿关节  DM4310   [1] 右腿关节  DM4310
 *   [2] 左腿轮毂  DM3510   [3] 右腿轮毂  DM3510
 * ================================================================ */
extern dm_motor_t g_dm_motors[4];
extern volatile uint32_t g_dm_rx_count;

/* ================================================================
 * 接口函数
 * ================================================================ */

/**
 * @brief  初始化一个电机实例，绑定总线、ID 和量程
 * @note   在 main 或 app 初始化阶段调用，调用前须先执行 bsp_can_init()
 */
void dm_motor_init(dm_motor_t *m, CAN_HandleTypeDef *hcan, uint16_t id,
                   float v_max, float v_min, float t_max, float t_min);

/**
 * @brief  使能电机（发送 0xFC 指令帧）
 * @note   DM4310 / DM3510 通用
 */
void dm_motor_enable(dm_motor_t *m);

/**
 * @brief  失能电机（发送 0xFD 指令帧）
 * @note   DM4310 / DM3510 通用
 */
void dm_motor_disable(dm_motor_t *m);

/**
 * @brief  将当前物理位置设为零点（发送 0xFE 指令帧）
 * @note   DM4310 专用：掉电保存，每次上电执行一次
 *         DM3510 轮毂电机通常不需要设零点
 */
void dm_motor_set_zero(dm_motor_t *m);

/**
 * @brief  MIT 模式控制帧
 * @param  p_des  期望位置 (rad)，DM4310 关节使用；DM3510 轮控时可置 0
 * @param  v_des  期望速度 (rad/s)
 * @param  kp     位置刚度增益
 * @param  kd     速度阻尼增益
 * @param  t_ff   前馈力矩 (N·m)
 * @retval 0 发送成功，1 发送失败（邮箱满）
 * @note   DM4310 / DM3510 帧格式相同，量程由实例内 v_max/t_max 决定
 */
uint8_t dm_motor_send_mit(dm_motor_t *m, float p_des, float v_des,
                          float kp, float kd, float t_ff);

#endif /* DM_MOTOR_H */
