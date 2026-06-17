#include "dm_imu.h"
#include "bsp_can.h"

dm_imu_t g_imu;
volatile uint16_t g_imu_can_id = IMU_CAN_ID;

/* ── 内部工具 ────────────────────────────────────────────────── */

static float uint_to_float(uint16_t x, float x_min, float x_max)
{
    return (float)x * (x_max - x_min) / 65535.0f + x_min;
}

/* 向 IMU 发送配置/请求指令
 * 帧格式：{0xCC, reg_id, ac, 0xDD, data[3:0]} */
static void imu_send_cmd(uint8_t reg_id, uint8_t ac, uint32_t data)
{
    uint8_t buf[8] = {0xCC, reg_id, ac, 0xDD, 0, 0, 0, 0};
    buf[4] = (uint8_t)(data & 0xFF);
    buf[5] = (uint8_t)(data >> 8);
    buf[6] = (uint8_t)(data >> 16);
    buf[7] = (uint8_t)(data >> 24);

    g_imu.last_tx_status = bsp_can_send(&hcan1, g_imu_can_id, buf, 8);
    g_imu.tx_count++;
    if (g_imu.last_tx_status != 0U) {
        g_imu.tx_error_count++;
    }
}

/* ── 数据解码（内部，由 RX 回调调用）────────────────────────── */

static void imu_decode_accel(const uint8_t *d)
{
    g_imu.accel[0] = uint_to_float((uint16_t)(d[3] << 8 | d[2]), IMU_ACCEL_MIN, IMU_ACCEL_MAX);
    g_imu.accel[1] = uint_to_float((uint16_t)(d[5] << 8 | d[4]), IMU_ACCEL_MIN, IMU_ACCEL_MAX);
    g_imu.accel[2] = uint_to_float((uint16_t)(d[7] << 8 | d[6]), IMU_ACCEL_MIN, IMU_ACCEL_MAX);
}

static void imu_decode_gyro(const uint8_t *d)
{
    g_imu.gyro[0] = uint_to_float((uint16_t)(d[3] << 8 | d[2]), IMU_GYRO_MIN, IMU_GYRO_MAX);
    g_imu.gyro[1] = uint_to_float((uint16_t)(d[5] << 8 | d[4]), IMU_GYRO_MIN, IMU_GYRO_MAX);
    g_imu.gyro[2] = uint_to_float((uint16_t)(d[7] << 8 | d[6]), IMU_GYRO_MIN, IMU_GYRO_MAX);
}

static void imu_decode_euler(const uint8_t *d)
{
    g_imu.pitch = uint_to_float((uint16_t)(d[3] << 8 | d[2]), IMU_PITCH_MIN, IMU_PITCH_MAX);
    g_imu.yaw   = uint_to_float((uint16_t)(d[5] << 8 | d[4]), IMU_YAW_MIN,   IMU_YAW_MAX);
    g_imu.roll  = uint_to_float((uint16_t)(d[7] << 8 | d[6]), IMU_ROLL_MIN,  IMU_ROLL_MAX);
}

/* ── CAN1 RX 回调（覆盖 bsp_can 弱函数）────────────────────── */

void bsp_can1_rx_callback(uint32_t id, uint8_t *data, uint8_t len)
{
    g_imu.can_rx_count++;
    g_imu.last_can_id = id;
    g_imu.last_len = len;
    for (uint8_t i = 0U; (i < len) && (i < 8U); i++) {
        g_imu.last_raw[i] = data[i];
    }

    if (id != IMU_MST_ID) return;
    if (len < 8U) return;

    switch (data[0]) {
        case 1: imu_decode_accel(data); break;
        case 2: imu_decode_gyro(data);  break;
        case 3: imu_decode_euler(data); break;
        default: break;
    }

    if ((data[0] >= 1U) && (data[0] <= 3U)) {
        g_imu.rx_count++;
        g_imu.last_update_ms = HAL_GetTick();
        g_imu.last_type = data[0];
    }
}

/* ── 接口实现 ────────────────────────────────────────────────── */

void imu_init(void)
{
    g_imu.rx_count = 0U;
    g_imu.can_rx_count = 0U;
    g_imu.tx_count = 0U;
    g_imu.tx_error_count = 0U;
    g_imu.last_update_ms = 0U;
    g_imu.last_can_id = 0U;
    g_imu.last_type = 0U;
    g_imu.last_len = 0U;
    g_imu.last_tx_status = 0U;
    for (uint8_t i = 0U; i < 8U; i++) {
        g_imu.last_raw[i] = 0U;
    }

    /* 设置主动推送模式，IMU 自动按内部频率推送数据 */
    imu_send_cmd(11, 1, 1);   /* CHANGE_ACTIVE = 11, write, value = 1 */
}

void imu_request_data(void)
{
    imu_send_cmd(1, 0, 0);   /* ACCEL_DATA = 1, read */
    /* 主动请求一次欧拉角和陀螺仪数据（请求模式下使用）*/
    imu_send_cmd(3, 0, 0);   /* EULER_DATA = 3, read */
    imu_send_cmd(2, 0, 0);   /* GYRO_DATA  = 2, read */
}

uint8_t imu_is_online(uint32_t timeout_ms)
{
    if (g_imu.rx_count == 0U) {
        return 0U;
    }

    return ((HAL_GetTick() - g_imu.last_update_ms) <= timeout_ms) ? 1U : 0U;
}
