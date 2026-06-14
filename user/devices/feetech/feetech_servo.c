#include "feetech_servo.h"

extern UART_HandleTypeDef huart2;   /* CubeMX 生成，配置好后与 FEETECH_HUART 保持一致 */

feetech_servo_t g_ft_servos[2];

/* ── 协议常量 ────────────────────────────────────────────────── */
#define INST_WRITE       0x03
#define INST_READ        0x02
#define INST_SYNC_WRITE  0x83
#define ADDR_TORQUE      40    /* 扭矩使能寄存器 */
#define ADDR_GOAL_POS    42    /* 目标位置、运行时间、运行速度连续写入起始地址 */
#define ADDR_POS_L       56    /* 当前位置反馈起始地址 */
#define FB_LEN           8     /* 一次读取反馈的字节数（位置/速度/负载/电压/温度）*/

/* ── 内部工具 ────────────────────────────────────────────────── */

static void uart_send(uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&FEETECH_HUART, buf, len, 10);
}

/* 带超时的逐字节接收，接收不到返回 0 */
static int uart_recv(uint8_t *buf, uint16_t len)
{
    return (HAL_UART_Receive(&FEETECH_HUART, buf, len, 10) == HAL_OK) ? len : 0;
}

/* 清空接收缓冲区 */
static void uart_flush(void)
{
    uint8_t tmp;
    while (HAL_UART_Receive(&FEETECH_HUART, &tmp, 1, 1) == HAL_OK) {}
}

/* 构造写指令包并发送 */
static void send_write(uint8_t id, uint8_t addr,
                       const uint8_t *payload, uint8_t payload_len)
{
    uint8_t buf[32];
    uint8_t idx = 0;
    uint8_t checksum;
    uint8_t msg_len = (uint8_t)(payload_len + 3);

    buf[idx++] = 0xFF;
    buf[idx++] = 0xFF;
    buf[idx++] = id;
    buf[idx++] = msg_len;
    buf[idx++] = INST_WRITE;
    buf[idx++] = addr;

    checksum = (uint8_t)(id + msg_len + INST_WRITE + addr);
    for (uint8_t i = 0; i < payload_len; i++) {
        buf[idx++] = payload[i];
        checksum = (uint8_t)(checksum + payload[i]);
    }
    buf[idx++] = (uint8_t)(~checksum);

    uart_flush();
    uart_send(buf, idx);
}

/* ── 接口实现 ────────────────────────────────────────────────── */

void feetech_servo_init(void)
{
    g_ft_servos[0].id = FEETECH_ID_LEFT;
    g_ft_servos[1].id = FEETECH_ID_RIGHT;
}

void feetech_servo_enable(uint8_t id)
{
    uint8_t val = 1;
    send_write(id, ADDR_TORQUE, &val, 1);
}

void feetech_servo_disable(uint8_t id)
{
    uint8_t val = 0;
    send_write(id, ADDR_TORQUE, &val, 1);
}

void feetech_servo_set_pos(uint8_t id, uint16_t pos, uint16_t speed, uint8_t acc)
{
    (void)acc;

    /* 从 ADDR_GOAL_POS 连续写 6 字节：pos_H | pos_L | time_H | time_L | spd_H | spd_L */
    uint8_t payload[6];
    payload[0] = (uint8_t)(pos   >> 8);
    payload[1] = (uint8_t)(pos   & 0xFF);
    payload[2] = 0;               /* time = 0 */
    payload[3] = 0;
    payload[4] = (uint8_t)(speed >> 8);
    payload[5] = (uint8_t)(speed & 0xFF);

    send_write(id, ADDR_GOAL_POS, payload, sizeof(payload));
}

void feetech_servo_sync_set_pos(uint16_t pos_l, uint16_t pos_r,
                                 uint16_t speed, uint8_t acc)
{
    /* SYNC_WRITE 帧：0xFF 0xFF 0xFE | mesLen | INST | addr | data_len | [ID payload]... | cs */
    const uint8_t data_len = 6;   /* 每个舵机的数据字节数 */
    const uint8_t n        = 2;   /* 舵机数量 */
    uint8_t msg_len = (uint8_t)((data_len + 1) * n + 4);

    uint8_t buf[32];
    uint8_t idx = 0;
    uint8_t checksum;

    buf[idx++] = 0xFF;
    buf[idx++] = 0xFF;
    buf[idx++] = 0xFE;            /* 广播 ID */
    buf[idx++] = msg_len;
    buf[idx++] = INST_SYNC_WRITE;
    buf[idx++] = ADDR_GOAL_POS;
    buf[idx++] = data_len;

    checksum = (uint8_t)(0xFE + msg_len + INST_SYNC_WRITE + ADDR_GOAL_POS + data_len);

    uint16_t positions[2] = {pos_l, pos_r};
    uint8_t  ids[2]       = {FEETECH_ID_LEFT, FEETECH_ID_RIGHT};

    for (uint8_t i = 0; i < n; i++) {
        buf[idx++] = ids[i];
        buf[idx++] = (uint8_t)(positions[i] >> 8);
        buf[idx++] = (uint8_t)(positions[i] & 0xFF);
        buf[idx++] = 0;           /* time = 0 */
        buf[idx++] = 0;
        buf[idx++] = (uint8_t)(speed >> 8);
        buf[idx++] = (uint8_t)(speed & 0xFF);

        checksum = (uint8_t)(checksum + ids[i]
                   + (positions[i] >> 8) + (positions[i] & 0xFF)
                   + (speed >> 8) + (speed & 0xFF));
    }
    buf[idx++] = (uint8_t)(~checksum);

    uart_flush();
    uart_send(buf, idx);
}

int feetech_servo_update(uint8_t idx)
{
    feetech_servo_t *s = &g_ft_servos[idx];

    /* 发送 READ 请求：读取从 ADDR_POS_L 开始的 FB_LEN 字节 */
    uint8_t req[8];
    uint8_t msg_len = 4;
    uint8_t checksum = (uint8_t)(~(uint8_t)(s->id + msg_len + INST_READ + ADDR_POS_L + FB_LEN));

    req[0] = 0xFF;
    req[1] = 0xFF;
    req[2] = s->id;
    req[3] = msg_len;
    req[4] = INST_READ;
    req[5] = ADDR_POS_L;
    req[6] = FB_LEN;
    req[7] = checksum;

    uart_flush();
    uart_send(req, sizeof(req));

    /* 接收响应：0xFF 0xFF ID (FB_LEN+2) status data[FB_LEN] cs */
    uint8_t rsp[16];
    uint8_t rsp_len = (uint8_t)(FB_LEN + 6);
    if (uart_recv(rsp, rsp_len) != rsp_len) return -1;
    if (rsp[0] != 0xFF || rsp[1] != 0xFF)  return -1;
    if (rsp[2] != s->id)                    return -1;

    uint8_t *d = &rsp[5];   /* 数据起始 */

    s->pos     = (int16_t)((d[0] << 8) | d[1]);
    s->speed   = (int16_t)((d[2] << 8) | d[3]);
    if (s->speed & (1 << 15)) s->speed = -(s->speed & ~(1 << 15));
    s->load    = (int16_t)((d[4] << 8) | d[5]);
    if (s->load  & (1 << 10)) s->load  = -(s->load  & ~(1 << 10));
    s->voltage = d[6];
    s->temp    = d[7];

    return 0;
}
