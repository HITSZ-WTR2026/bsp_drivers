/**
 * @file    can_common.c
 * @author  syhanjin
 * @date    2025-10-08
 */

#include "can_common.h"
#ifdef __cplusplus
extern "C" {
#endif

static CAN_CommonCallbackMapItem_t map[CAN_COMMON_FILTER_NUM];
static size_t map_size = 0;
// 只有 FDCAN 才会有 default callback
#ifdef CAN_COMMON_USE_FDCAN
static CAN_CommonRxCallback_t default_callback = NULL;
#endif

void CAN_Common_SendMessage(CAN_CommonDevice_t* device, const CAN_CommonTxHeader_t* header, const uint8_t* data)
{
#ifdef CAN_COMMON_USE_FDCAN
    if (device->type == CAN_COMMON_FDCAN)
    {
        const FDCAN_TxHeaderTypeDef fdcan_header = CAN_Common_TxHeader_to_fdcan(header);
        HAL_FDCAN_AddMessageToTxFifoQ(device->handle, &fdcan_header, data);
    }
#endif
#ifdef CAN_COMMON_USE_CAN
    if (device->type == CAN_COMMON_CAN)
    {
        uint32_t mailbox;
        const CAN_TxHeaderTypeDef can_header = CAN_Common_TxHeader_to_can(header);
        HAL_CAN_AddTxMessage(device->handle, &can_header, data, &mailbox);
    }
#endif
}

size_t CAN_Common_AddCallback(CAN_CommonDevice_t* device, const uint8_t filter_index,
                              const CAN_CommonRxLocation_t location, const CAN_CommonRxCallback_t callback)
{
    if (map_size < CAN_COMMON_FILTER_NUM)
    {
        map[map_size].device       = device;
        map[map_size].location     = location;
        map[map_size].callback     = callback;
        map[map_size].filter_index = filter_index;
        map_size++;
        return map_size - 1;
    }
    return -1;
}

void CAN_Common_RemoveCallback(const size_t index)
{
    if (index < map_size)
    {
        map[index] = map[map_size - 1];
        map_size--;
    }
}

#ifdef CAN_COMMON_USE_FDCAN
void CAN_Common_SetDefaultCallback(const CAN_CommonRxCallback_t callback) { default_callback = callback; }

static void fdcan_run_callbacks(FDCAN_HandleTypeDef* hfdcan, const CAN_CommonRxLocation_t location,
                                const FDCAN_RxHeaderTypeDef* header, const uint8_t* data)
{
    const CAN_CommonRxHeader_t common_header = CAN_Common_RxHeader_from_fdcan(header);
    // run callbacks
    if (common_header.is_filter_miss)
    {
        if (default_callback != NULL)
            default_callback(location, &common_header, data);
    }
    else
    {
        for (size_t i = 0; i < map_size; i++)
        {
            if (map[i].device->handle == hfdcan && map[i].location == location &&
                map[i].filter_index == header->FilterIndex)
                map[i].callback(location, &common_header, data);
        }
    }
}

void FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs == FDCAN_IT_RX_FIFO0_NEW_MESSAGE || RxFifo0ITs == FDCAN_IT_RX_FIFO0_FULL ||
        RxFifo0ITs == FDCAN_IT_RX_FIFO0_WATERMARK)
        do
        { // 提取所有消息并顺序处理
            FDCAN_RxHeaderTypeDef header;
            uint8_t data[64];
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data);
            fdcan_run_callbacks(hfdcan, CAN_COMMON_RX_FIFO0, &header, data);
        }
        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0);
}

void FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
{
    if (RxFifo1ITs == FDCAN_IT_RX_FIFO1_NEW_MESSAGE || RxFifo1ITs == FDCAN_IT_RX_FIFO1_FULL ||
        RxFifo1ITs == FDCAN_IT_RX_FIFO1_WATERMARK)
        do
        { // 提取所有消息并顺序处理
            FDCAN_RxHeaderTypeDef header;
            uint8_t data[64];
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &header, data);
            fdcan_run_callbacks(hfdcan, CAN_COMMON_RX_FIFO1, &header, data);
        }
        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0);
}
#endif

#ifdef CAN_COMMON_USE_CAN
static void can_run_callbacks(CAN_HandleTypeDef* hcan, const CAN_CommonRxLocation_t location,
                              const CAN_RxHeaderTypeDef* header, const uint8_t* data)
{
    const CAN_CommonRxHeader_t common_header = CAN_Common_RxHeader_from_can(header);
    // run callbacks
    for (size_t i = 0; i < map_size; i++)
    {
        if (map[i].device->handle == hcan && map[i].location == location &&
            map[i].filter_index == header->FilterMatchIndex)
            map[i].callback(location, &common_header, data);
    }
}

void CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    do
    {
        CAN_RxHeaderTypeDef header;
        uint8_t data[8];
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data);
        can_run_callbacks(hcan, CAN_COMMON_RX_FIFO0, &header, data);
    }
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0);
}

void CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    do
    {
        CAN_RxHeaderTypeDef header;
        uint8_t data[8];
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &header, data);
        can_run_callbacks(hcan, CAN_COMMON_RX_FIFO1, &header, data);
    }
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0);
}
#endif

#ifdef __cplusplus
}
#endif
