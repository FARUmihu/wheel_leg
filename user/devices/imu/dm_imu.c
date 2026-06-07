#include "dm_imu.h"
#include "bsp_can.h"

dm_imu_t g_imu;

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
    bsp_can_send(&hcan1, IMU_CAN_ID, buf, 8);
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
    if (id != IMU_MST_ID) return;

    switch (data[0]) {
        case 1: imu_decode_accel(data); break;
        case 2: imu_decode_gyro(data);  break;
        case 3: imu_decode_euler(data); break;
        default: break;
    }
}

/* ── 接口实现 ────────────────────────────────────────────────── */

void imu_init(void)
{
    /* 设置主动推送模式，IMU 自动按内部频率推送数据 */
    imu_send_cmd(11, 1, 1);   /* CHANGE_ACTIVE = 11, write, value = 1 */
}

void imu_request_data(void)
{
    /* 主动请求一次欧拉角和陀螺仪数据（请求模式下使用）*/
    imu_send_cmd(3, 0, 0);   /* EULER_DATA = 3, read */
    imu_send_cmd(2, 0, 0);   /* GYRO_DATA  = 2, read */
}
