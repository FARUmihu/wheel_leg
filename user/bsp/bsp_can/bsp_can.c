#include "bsp_can.h"

void bsp_can_init(void)
{
    CAN_FilterTypeDef filter;

    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;  /* CAN2 使用 bank 14~27 */

    /* CAN1: bank 0, FIFO0 */
    filter.FilterBank           = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan1);

    /* CAN2: bank 14, FIFO0；MspInit 未使能 CAN2 中断，此处补上 */
    filter.FilterBank           = 14;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan2, &filter);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
}


uint8_t bsp_can_send(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    tx_header.StdId              = id;
    tx_header.IDE                = CAN_ID_STD;
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.DLC                = len;
    tx_header.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox) == HAL_OK) ? 0 : 1;
}

__weak void bsp_can1_rx_callback(uint32_t id, uint8_t *data, uint8_t len) {}
__weak void bsp_can2_rx_callback(uint32_t id, uint8_t *data, uint8_t len) {}

static void can_rx_dispatch(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];

    while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) == HAL_OK)
    {
        if      (hcan->Instance == CAN1) bsp_can1_rx_callback(rx_header.StdId, data, (uint8_t)rx_header.DLC);
        else if (hcan->Instance == CAN2) bsp_can2_rx_callback(rx_header.StdId, data, (uint8_t)rx_header.DLC);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    can_rx_dispatch(hcan);
}
