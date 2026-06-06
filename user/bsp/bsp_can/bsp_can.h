#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx_hal.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void bsp_can_init(void);
uint8_t bsp_can_send(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint8_t len);

/* 弱函数，device 层按需覆盖 */
void bsp_can1_rx_callback(uint32_t id, uint8_t *data, uint8_t len);
void bsp_can2_rx_callback(uint32_t id, uint8_t *data, uint8_t len);

#endif /* BSP_CAN_H */
