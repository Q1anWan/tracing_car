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
#include "core.h"
#include "arm_math.h"
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

#define K_CURRENT 10000.0f
/*Close-loop control Motors*/

[[noreturn]] void MotorThreadFun(ULONG initial_input) {
    /*Creat Wheel Topic*/
    om_suber_t *wheel_ctl_suber = om_subscribe(om_find_topic("MOTOR_CTL", UINT32_MAX));
    om_topic_t *wheel_fdb = om_find_topic("MOTOR_FDB", UINT32_MAX);
    //
    // Msg_Motor_Ctr_t msg_zdt{};
    // uint8_t zdt_enable_last[4] = {0, 0};
    Msg_Motor_Fdb_t wheel_fdb_data{};
    Msg_Motor_Ctr_t wheel_ctr_data{};
    wheel_ctr_data.torque[0]=0;
    wheel_ctr_data.torque[1]=0;

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

        om_suber_export(wheel_ctl_suber, &wheel_ctr_data, false);

        int16_t current[2];

        current[0] = static_cast<int16_t>((wheel_ctr_data.torque[0] * K_CURRENT));
        current[1] = static_cast<int16_t>((wheel_ctr_data.torque[1] * K_CURRENT));

        if (current[0]>4000) {
            current[0] = 4000;
        } else if (current[0]<-4000) {
            current[0] = -4000;
        }
        if (current[1]>4000) {
            current[1] = 4000;
        } else if (current[1]<-4000) {
            current[1] = -4000;
        }


        zdt[0].TorqueControl(tx_header2_0.Identifier, tx_header2_0.DataLength, can_data2_0,
                      65535,current[0]);
        zdt[1].TorqueControl(tx_header2_1.Identifier, tx_header2_1.DataLength, can_data2_1,
                      65535,current[1]);

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
        wheel_fdb_data.vel[0] = zdt[0].GetVelocity()*2*PI*WHEEL_R/60.0f;
        wheel_fdb_data.vel[1] = zdt[1].GetVelocity()*2*PI*WHEEL_R/60.0f;

        om_publish(wheel_fdb, &wheel_fdb_data, sizeof(wheel_fdb_data), false, false);
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
