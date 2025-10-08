/**
 * @file    can_common.h
 * @author  syhanjin
 * @date    2025-10-08
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#ifndef CAN_COMMON_H
#define CAN_COMMON_H

#include <stdbool.h>
#include "main.h"

#define CAN_COMMON_FILTER_NUM (16)

#ifdef FDCAN1
#define CAN_COMMON_USE_FDCAN
#include "fdcan.h"
#endif
#ifdef CAN1
#define CAN_COMMON_USE_CAN
#include "can.h"
#endif


#ifdef __cplusplus
extern "C" {
#endif
typedef struct
{
    uint32_t id;      ///< CAN identifier：标准 ID 或 扩展 ID
    bool is_extended; ///< 是否为扩展 ID
    bool is_remote;   ///< 是否为远程帧
    uint8_t dlc;      ///< 数据长度码

    /* CAN FD 专有部分 */
#ifdef CAN_COMMON_USE_FDCAN
    bool is_fd;             ///< 是否为 CAN FD 帧
    bool brs;               ///< bit rate switch
    bool store_tx_event;    ///< 是否存储发送事件
    uint8_t massage_marker; ///< FDCAN 消息标识，FDCAN 才有效
#endif

    /* bxCAN 专有部分 */
#ifdef CAN_COMMON_USE_CAN
    FunctionalState transmit_time; ///< 是否启用发送时间戳
#endif
} CAN_CommonTxHeader_t;

typedef struct
{
    uint32_t id;      ///< CAN identifier：标准 ID 或 扩展 ID
    bool is_extended; ///< 是否为扩展 ID
    bool is_remote;   ///< 是否为远程帧
    uint8_t dlc;      ///< 数据长度码

    uint16_t timestamp; ///< 接收时间戳 (FDCAN: RxTimestamp, bxCAN: Timestamp)

    uint8_t filter_index; ///< 匹配的滤波器索引 (FDCAN: 0~127/63, CAN: 0~255)

    /* CAN FD 专有部分 */
#ifdef CAN_COMMON_USE_FDCAN
    bool is_fd;          ///< 是否为 CAN FD 帧
    bool brs;            ///< bit rate switch
    bool is_filter_miss; ///< 是否未匹配过滤器
#endif
} CAN_CommonRxHeader_t;

typedef enum
{
#ifdef CAN_COMMON_USE_CAN
    CAN_COMMON_CAN,
#endif
#ifdef CAN_COMMON_USE_FDCAN
    CAN_COMMON_FDCAN,
#endif
} CAN_CommonType_t;

typedef struct
{
    void* handle;          ///< CAN 句柄
    CAN_CommonType_t type; ///< CAN 类型
} CAN_CommonDevice_t;

typedef enum
{
    CAN_COMMON_RX_FIFO0,
    CAN_COMMON_RX_FIFO1,
#ifdef CAN_COMMON_USE_FDCAN
    CAN_COMMON_RX_BUFFER,
#endif
} CAN_CommonRxLocation_t;

typedef void (*CAN_CommonRxCallback_t)(CAN_CommonRxLocation_t location, const CAN_CommonRxHeader_t* header,
                                       const uint8_t* data);

typedef struct
{
    uint8_t filter_index;
    CAN_CommonDevice_t* device; ///< CAN 设备
    CAN_CommonRxLocation_t location;
    CAN_CommonRxCallback_t callback;
} CAN_CommonCallbackMapItem_t;

#ifdef CAN_COMMON_USE_FDCAN
static FDCAN_TxHeaderTypeDef CAN_Common_TxHeader_to_fdcan(const CAN_CommonTxHeader_t* header)
{
    return (FDCAN_TxHeaderTypeDef){
        .Identifier         = header->id,
        .IdType             = header->is_extended ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID,
        .TxFrameType        = header->is_remote ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME,
        .DataLength         = header->dlc,
        .BitRateSwitch      = header->brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF,
        .FDFormat           = header->is_fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN,
        .TxEventFifoControl = header->store_tx_event ? FDCAN_STORE_TX_EVENTS : FDCAN_NO_TX_EVENTS,
        .MessageMarker      = header->massage_marker,
    };
}

static CAN_CommonTxHeader_t CAN_Common_TxHeader_from_fdcan(const FDCAN_TxHeaderTypeDef* header)
{
    return (CAN_CommonTxHeader_t){
        .id             = header->Identifier,
        .is_extended    = header->IdType == FDCAN_EXTENDED_ID,
        .is_remote      = header->TxFrameType == FDCAN_REMOTE_FRAME,
        .dlc            = header->DataLength,
        .brs            = header->BitRateSwitch == FDCAN_BRS_ON,
        .is_fd          = header->FDFormat == FDCAN_FD_CAN,
        .store_tx_event = header->TxEventFifoControl == FDCAN_STORE_TX_EVENTS,
        .massage_marker = header->MessageMarker,
    };
}

static CAN_CommonRxHeader_t CAN_Common_RxHeader_from_fdcan(const FDCAN_RxHeaderTypeDef* header)
{
    return (CAN_CommonRxHeader_t){
        .id             = header->Identifier,
        .is_extended    = header->IdType == FDCAN_EXTENDED_ID,
        .is_remote      = header->RxFrameType == FDCAN_REMOTE_FRAME,
        .dlc            = header->DataLength,
        .brs            = header->BitRateSwitch == FDCAN_BRS_ON,
        .is_fd          = header->FDFormat == FDCAN_FD_CAN,
        .timestamp      = header->RxTimestamp,
        .filter_index   = header->FilterIndex,
        .is_filter_miss = header->IsFilterMatchingFrame,
    };
}

void FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs);
void FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs);
#endif

#ifdef CAN_COMMON_USE_CAN
static CAN_TxHeaderTypeDef CAN_Common_TxHeader_to_can(const CAN_CommonTxHeader_t* header)
{
    if (header->is_extended)
    {
        return (CAN_TxHeaderTypeDef){
            .ExtId              = header->id,
            .IDE                = CAN_ID_EXT,
            .RTR                = header->is_remote ? CAN_RTR_REMOTE : CAN_RTR_DATA,
            .DLC                = header->dlc,
            .TransmitGlobalTime = header->transmit_time,
        };
    }
    else
    {
        return (CAN_TxHeaderTypeDef){
            .StdId              = header->id,
            .IDE                = CAN_ID_STD,
            .RTR                = header->is_remote ? CAN_RTR_REMOTE : CAN_RTR_DATA,
            .DLC                = header->dlc,
            .TransmitGlobalTime = header->transmit_time,
        };
    }
}

static CAN_CommonTxHeader_t CAN_Common_TxHeader_from_can(const CAN_TxHeaderTypeDef* header)
{
    return (CAN_CommonTxHeader_t){
        .id            = header->IDE == CAN_ID_EXT ? header->ExtId : header->StdId,
        .is_extended   = header->IDE == CAN_ID_EXT,
        .is_remote     = header->RTR == CAN_RTR_REMOTE,
        .dlc           = header->DLC,
        .transmit_time = header->TransmitGlobalTime,
    };
}

static CAN_CommonRxHeader_t CAN_Common_RxHeader_from_can(const CAN_RxHeaderTypeDef* header)
{
    return (CAN_CommonRxHeader_t){
        .id           = header->IDE == CAN_ID_EXT ? header->ExtId : header->StdId,
        .is_extended  = header->IDE == CAN_ID_EXT,
        .is_remote    = header->RTR == CAN_RTR_REMOTE,
        .dlc          = header->DLC,
        .timestamp    = header->Timestamp,
        .filter_index = header->FilterMatchIndex,
    };
}
void CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan);
void CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan);
#endif

#ifdef __cplusplus
}
#endif

#endif // CAN_COMMON_H
