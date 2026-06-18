#include "remote_esp32.h"

#define REMOTE_FRAME_HEAD_0 0xAAU
#define REMOTE_FRAME_HEAD_1 0x55U
#define REMOTE_CMD_CONTROL  0x01U
#define REMOTE_CONTROL_LEN  0x04U
#define REMOTE_MAX_PAYLOAD  16U

typedef enum {
    REMOTE_RX_HEAD_0 = 0,
    REMOTE_RX_HEAD_1,
    REMOTE_RX_CMD,
    REMOTE_RX_SEQ,
    REMOTE_RX_LEN,
    REMOTE_RX_PAYLOAD,
    REMOTE_RX_CRC_LO,
    REMOTE_RX_CRC_HI,
} remote_rx_state_t;

volatile remote_esp32_state_t g_remote_esp32;

static UART_HandleTypeDef *s_remote_huart;
static uint8_t s_remote_rx_byte;
static remote_rx_state_t s_rx_state;
static uint8_t s_rx_cmd;
static uint8_t s_rx_seq;
static uint8_t s_rx_len;
static uint8_t s_rx_payload[REMOTE_MAX_PAYLOAD];
static uint8_t s_rx_payload_idx;
static uint8_t s_rx_crc_lo;

static uint16_t remote_crc16_modbus_update(uint16_t crc, uint8_t data)
{
    crc ^= data;
    for (uint8_t i = 0U; i < 8U; i++) {
        if ((crc & 0x0001U) != 0U) {
            crc = (uint16_t)((crc >> 1) ^ 0xA001U);
        } else {
            crc >>= 1;
        }
    }

    return crc;
}

static uint16_t remote_frame_crc(void)
{
    uint16_t crc = 0xFFFFU;

    crc = remote_crc16_modbus_update(crc, s_rx_cmd);
    crc = remote_crc16_modbus_update(crc, s_rx_seq);
    crc = remote_crc16_modbus_update(crc, s_rx_len);
    for (uint8_t i = 0U; i < s_rx_len; i++) {
        crc = remote_crc16_modbus_update(crc, s_rx_payload[i]);
    }

    return crc;
}

static int16_t remote_read_i16_le(const uint8_t *data)
{
    uint16_t raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return (int16_t)raw;
}

static void remote_accept_frame(void)
{
    if ((s_rx_cmd != REMOTE_CMD_CONTROL) || (s_rx_len != REMOTE_CONTROL_LEN)) {
        g_remote_esp32.drop_count++;
        return;
    }

    g_remote_esp32.vx_mm_s = remote_read_i16_le(&s_rx_payload[0]);
    g_remote_esp32.wz_mrad_s = remote_read_i16_le(&s_rx_payload[2]);
    g_remote_esp32.seq = s_rx_seq;
    g_remote_esp32.online = 1U;
    g_remote_esp32.last_update_ms = HAL_GetTick();
    g_remote_esp32.rx_frame_count++;
}

static void remote_reset_parser(void)
{
    s_rx_state = REMOTE_RX_HEAD_0;
    s_rx_cmd = 0U;
    s_rx_seq = 0U;
    s_rx_len = 0U;
    s_rx_payload_idx = 0U;
    s_rx_crc_lo = 0U;
}

void remote_esp32_init(UART_HandleTypeDef *huart)
{
    s_remote_huart = huart;
    g_remote_esp32.vx_mm_s = 0;
    g_remote_esp32.wz_mrad_s = 0;
    g_remote_esp32.seq = 0U;
    g_remote_esp32.online = 0U;
    g_remote_esp32.last_update_ms = 0U;
    g_remote_esp32.rx_frame_count = 0U;
    g_remote_esp32.crc_error_count = 0U;
    g_remote_esp32.drop_count = 0U;
    remote_reset_parser();

    if (s_remote_huart != 0) {
        (void)HAL_UART_Receive_IT(s_remote_huart, &s_remote_rx_byte, 1U);
    }
}

void remote_esp32_background(void)
{
    if (remote_esp32_is_online(500U) == 0U) {
        g_remote_esp32.online = 0U;
        g_remote_esp32.vx_mm_s = 0;
        g_remote_esp32.wz_mrad_s = 0;
    }
}

void remote_esp32_rx_byte(uint8_t byte)
{
    switch (s_rx_state) {
        case REMOTE_RX_HEAD_0:
            if (byte == REMOTE_FRAME_HEAD_0) {
                s_rx_state = REMOTE_RX_HEAD_1;
            }
            break;

        case REMOTE_RX_HEAD_1:
            s_rx_state = (byte == REMOTE_FRAME_HEAD_1) ? REMOTE_RX_CMD : REMOTE_RX_HEAD_0;
            break;

        case REMOTE_RX_CMD:
            s_rx_cmd = byte;
            s_rx_state = REMOTE_RX_SEQ;
            break;

        case REMOTE_RX_SEQ:
            s_rx_seq = byte;
            s_rx_state = REMOTE_RX_LEN;
            break;

        case REMOTE_RX_LEN:
            s_rx_len = byte;
            s_rx_payload_idx = 0U;
            if (s_rx_len > REMOTE_MAX_PAYLOAD) {
                g_remote_esp32.drop_count++;
                remote_reset_parser();
            } else if (s_rx_len == 0U) {
                s_rx_state = REMOTE_RX_CRC_LO;
            } else {
                s_rx_state = REMOTE_RX_PAYLOAD;
            }
            break;

        case REMOTE_RX_PAYLOAD:
            s_rx_payload[s_rx_payload_idx++] = byte;
            if (s_rx_payload_idx >= s_rx_len) {
                s_rx_state = REMOTE_RX_CRC_LO;
            }
            break;

        case REMOTE_RX_CRC_LO:
            s_rx_crc_lo = byte;
            s_rx_state = REMOTE_RX_CRC_HI;
            break;

        case REMOTE_RX_CRC_HI: {
            uint16_t rx_crc = (uint16_t)s_rx_crc_lo | ((uint16_t)byte << 8);
            if (rx_crc == remote_frame_crc()) {
                remote_accept_frame();
            } else {
                g_remote_esp32.crc_error_count++;
            }
            remote_reset_parser();
            break;
        }

        default:
            remote_reset_parser();
            break;
    }
}

uint8_t remote_esp32_is_online(uint32_t timeout_ms)
{
    if (g_remote_esp32.last_update_ms == 0U) {
        return 0U;
    }

    return ((HAL_GetTick() - g_remote_esp32.last_update_ms) <= timeout_ms) ? 1U : 0U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((s_remote_huart != 0) && (huart->Instance == s_remote_huart->Instance)) {
        remote_esp32_rx_byte(s_remote_rx_byte);
        (void)HAL_UART_Receive_IT(s_remote_huart, &s_remote_rx_byte, 1U);
    }
}
