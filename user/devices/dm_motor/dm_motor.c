#include "dm_motor.h"

extern dm_motor_t g_dm_motors[4];

volatile uint32_t g_dm_rx_count;
volatile uint32_t g_dm_rx_last_id;
volatile uint8_t g_dm_rx_last_len;

/* ── 定点 ↔ 浮点 转换 ──────────────────────────────────────────── */

static uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    if (x > x_max) x = x_max;
    if (x < x_min) x = x_min;
    return (uint16_t)((x - x_min) * (float)((1 << bits) - 1) / (x_max - x_min));
}

static float uint_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    return (float)x * (x_max - x_min) / (float)((1 << bits) - 1) + x_min;
}

/* ── 初始化 ─────────────────────────────────────────────────────── */

void dm_motor_init(dm_motor_t *m, CAN_HandleTypeDef *hcan, uint16_t id,
                   float v_max, float v_min, float t_max, float t_min)
{
    m->hcan         = hcan;
    m->id           = id;
    m->v_max        = v_max;
    m->v_min        = v_min;
    m->t_max        = t_max;
    m->t_min        = t_min;
    m->err          = 0;
    m->pos          = 0.0f;
    m->vel          = 0.0f;
    m->vel_filtered = 0.0f;
    m->torque       = 0.0f;
    m->temp_mos     = 0;
    m->temp_rotor   = 0;
}

/* ── 特殊指令帧（DM4310 / DM3510 通用）────────────────────────── */

void dm_motor_enable(dm_motor_t *m)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    bsp_can_send(m->hcan, m->id, data, 8);
}

void dm_motor_disable(dm_motor_t *m)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    bsp_can_send(m->hcan, m->id, data, 8);
}

/**
 * @note DM4310 关节专用：将当前物理位置保存为零点，掉电不丢失。
 *       DM3510 轮毂电机无需调用此函数。
 */
void dm_motor_set_zero(dm_motor_t *m)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    bsp_can_send(m->hcan, m->id, data, 8);
}

/* ── MIT 模式控制帧（DM4310 / DM3510 帧格式相同）─────────────── */

/**
 * @note 帧格式（8 字节）：
 *   [0-1]  p_des  16bit
 *   [2]    v_des  高 8bit（共 12bit）
 *   [3]    v_des 低 4bit | kp 高 4bit
 *   [4]    kp 低 8bit
 *   [5]    kd 高 8bit（共 12bit）
 *   [6]    kd 低 4bit | t_ff 高 4bit
 *   [7]    t_ff 低 8bit
 *
 * DM4310 关节：p_des 有意义，用于位置/阻抗控制。
 * DM3510 轮毂：纯速度/力矩控制时 p_des 置 0，kp 置 0。
 */
uint8_t dm_motor_send_mit(dm_motor_t *m, float p_des, float v_des,
                          float kp, float kd, float t_ff)
{
    uint8_t data[8];

    uint16_t p_int  = float_to_uint(p_des, DM_P_MIN,  DM_P_MAX,  16);
    uint16_t v_int  = float_to_uint(v_des, m->v_min,  m->v_max,  12);
    uint16_t kp_int = float_to_uint(kp,    DM_KP_MIN, DM_KP_MAX, 12);
    uint16_t kd_int = float_to_uint(kd,    DM_KD_MIN, DM_KD_MAX, 12);
    uint16_t t_int  = float_to_uint(t_ff,  m->t_min,  m->t_max,  12);

    data[0] =  p_int >> 8;
    data[1] =  p_int & 0xFF;
    data[2] =  v_int >> 4;
    data[3] = (uint8_t)((v_int  & 0x0F) << 4) | (uint8_t)(kp_int >> 8);
    data[4] =  kp_int & 0xFF;
    data[5] =  kd_int >> 4;
    data[6] = (uint8_t)((kd_int & 0x0F) << 4) | (uint8_t)(t_int >> 8);
    data[7] =  t_int & 0xFF;

    return bsp_can_send(m->hcan, m->id, data, 8);
}

/* ── 反馈解码（内部，由 RX 回调调用）──────────────────────────── */

/**
 * @note 反馈帧格式（8 字节）：
 *   [0]    ERR(高4bit) | 保留(低4bit)
 *   [1-2]  pos   16bit
 *   [3]    vel 高 8bit（共 12bit）
 *   [4]    vel 低 4bit | t 高 4bit
 *   [5]    t 低 8bit（共 12bit）
 *   [6]    temp_mos   (int8)
 *   [7]    temp_rotor (int8)
 *
 * DM4310 / DM3510 反馈帧格式相同，量程由实例内 v_max/t_max 决定。
 */
static void dm_motor_decode(dm_motor_t *m, const uint8_t *data)
{
    m->err = data[0] >> 4;

    uint16_t p_int = (uint16_t)(data[1] << 8) | data[2];
    m->pos = uint_to_float(p_int, DM_P_MIN, DM_P_MAX, 16);

    uint16_t v_int = (uint16_t)(data[3] << 4) | (data[4] >> 4);
    m->vel = uint_to_float(v_int, m->v_min, m->v_max, 12);
    m->vel_filtered = DM_EMA_ALPHA * m->vel + (1.0f - DM_EMA_ALPHA) * m->vel_filtered;

    uint16_t t_int = (uint16_t)((data[4] & 0x0F) << 8) | data[5];
    m->torque = uint_to_float(t_int, m->t_min, m->t_max, 12);

    m->temp_mos   = (int8_t)data[6];
    m->temp_rotor = (int8_t)data[7];
}

/* ── CAN RX 弱函数覆盖 ─────────────────────────────────────────── */

static void dm_rx_dispatch(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t len)
{
    uint16_t feedback_id = (uint16_t)(data[0] & 0x0FU);

    g_dm_rx_count++;
    g_dm_rx_last_id = id;
    g_dm_rx_last_len = len;

    for (int i = 0; i < 4; i++) {
        if ((g_dm_motors[i].hcan == hcan) &&
            ((g_dm_motors[i].id == (uint16_t)id) || (g_dm_motors[i].id == feedback_id))) {
            dm_motor_decode(&g_dm_motors[i], data);
            return;
        }
    }
}

/* CAN1 归 IMU 使用，此处不覆盖 bsp_can1_rx_callback */

void bsp_can2_rx_callback(uint32_t id, uint8_t *data, uint8_t len)
{
    dm_rx_dispatch(&hcan2, id, data, len);
}
