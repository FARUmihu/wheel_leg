#ifndef REMOTE_ESP32_H
#define REMOTE_ESP32_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    int16_t vx_mm_s;
    int16_t wz_mrad_s;
    uint8_t seq;
    uint8_t online;
    uint32_t last_update_ms;
    uint32_t rx_frame_count;
    uint32_t crc_error_count;
    uint32_t drop_count;
} remote_esp32_state_t;

extern volatile remote_esp32_state_t g_remote_esp32;

void remote_esp32_init(UART_HandleTypeDef *huart);
void remote_esp32_background(void);
void remote_esp32_rx_byte(uint8_t byte);
uint8_t remote_esp32_is_online(uint32_t timeout_ms);

#endif /* REMOTE_ESP32_H */
