/*
 * @Description: Task of DM-Bot control
 * @Author: qianwan
 * @Date: 2023-12-25 11:44:40
 * @LastEditTime: 2024-02-02 12:28:16
 * @LastEditors: qianwan
 */

#include "CarMsgs.h"
#include "ZDT.hpp"
#include "fdcan.h"
#include "om.h"
#include "tx_api.h"
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

static ZDT::cZDT zdt[2];


static uint32_t *can_mail_box = nullptr;

static uint8_t can_data2_0[8];
static uint8_t can_data2_1[8];

void CANFilterConfig();

static void MotorReInit();

TX_THREAD MotorThread;
uint8_t MotorThreadStack[3072] = {0};
TX_SEMAPHORE MotorSemaphore1;
TX_SEMAPHORE MotorSemaphore2;

uint8_t  rx_data_1[8];
uint8_t  rx_data_2[8];

FDCAN_RxHeaderTypeDef RxHeader1;
FDCAN_RxHeaderTypeDef RxHeader2;

uint8_t fdb_11;
uint8_t fdb_12;
float v1;

uint8_t fdb_21;
uint8_t fdb_22;
float v2;
Msg_Remoter_t rmt_msg;

/*Close-loop control Motors*/

[[noreturn]] void MotorThreadFun(ULONG initial_input) {
    /*Creat Wheel Topic*/
    om_suber_t *rm_suber = om_subscribe(om_find_topic("REMOTER", UINT32_MAX));
    //
    // Msg_Motor_Ctr_t msg_zdt{};
    // uint8_t zdt_enable_last[4] = {0, 0};

    zdt[0].SetID(1); //FDCAN 1
    zdt[1].SetID(1); //FDCAN 2
    tx_thread_sleep(5000);
    CANFilterConfig();
    FDCAN_TxHeaderTypeDef tx_header2_0 = {
        .Identifier = 0x01,
        .IdType = FDCAN_EXTENDED_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0x00
    };

    FDCAN_TxHeaderTypeDef tx_header2_1 = {
        .Identifier = 0x01,
        .IdType = FDCAN_EXTENDED_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0x00
    };

    // MotorReInit();

    tx_thread_sleep(100);

    for (;;) {

        om_suber_export(rm_suber, &rmt_msg, false);

        int16_t vx = rmt_msg.ch_1*1000;
        int16_t vy = rmt_msg.ch_0*1000;

        zdt[0].TorqueControl(tx_header2_0.Identifier, tx_header2_0.DataLength, can_data2_0,
                      65535,vx+vy);
        zdt[1].TorqueControl(tx_header2_1.Identifier, tx_header2_1.DataLength, can_data2_1,
                      65535,-vx+vy);

        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header2_0, can_data2_0);
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header2_1, can_data2_1);
        tx_semaphore_get(&MotorSemaphore1,10);
        tx_semaphore_get(&MotorSemaphore2,10);
        fdb_11 = zdt[0].CommonFDBHandel(RxHeader1.Identifier,rx_data_1);
        fdb_21 = zdt[1].CommonFDBHandel(RxHeader2.Identifier,rx_data_2);


        zdt[0].ReadVel(tx_header2_0.Identifier, tx_header2_0.DataLength, can_data2_0);
        zdt[1].ReadVel(tx_header2_1.Identifier, tx_header2_1.DataLength, can_data2_1);
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header2_0, can_data2_0);
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header2_1, can_data2_1);
        tx_semaphore_get(&MotorSemaphore1,10);
        tx_semaphore_get(&MotorSemaphore2,10);
        fdb_11 = zdt[0].ReadVelFDB(RxHeader1.Identifier,rx_data_1);
        fdb_12 = zdt[1].ReadVelFDB(RxHeader2.Identifier,rx_data_2);

        v1 = zdt[0].GetVelocity();
        v2 = zdt[1].GetVelocity();

        tx_thread_sleep(1);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {

    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader1, rx_data_1);
    tx_semaphore_put(&MotorSemaphore1);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {

    HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1, &RxHeader2, rx_data_2);
    tx_semaphore_put(&MotorSemaphore2);
}

void CANFilterConfig(void) {
    FDCAN_FilterTypeDef Filter;
    Filter.IdType = FDCAN_EXTENDED_ID;
    Filter.FilterIndex = 0;
    Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    Filter.FilterType = FDCAN_FILTER_MASK;
    Filter.FilterID1 = 0x0000;
    Filter.FilterID2 = 0x0000;


    HAL_FDCAN_ConfigFilter(&hfdcan1, &Filter);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    Filter.FilterIndex = 0;
    Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &Filter);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);
}
