#include "usart.h"           // STM32 HAL USART 驱动头文件（包含 huart3 实例）
#include "DL_H723.h"         // 板级支持包头文件（定义缓存控制函数等）
#include "CarMsgs.h"         // 项目通信数据结构定义（如需要定义速度/控制帧结构）
#include "app_threadx.h"     // ThreadX RTOS 的线程与信号量接口
#include "om.h"              // Open Message 通信中间件接口（如未使用可忽略）
#include "mavlink.h"

// =============================
// 通信线程资源
// =============================
TX_THREAD SerialCommThread;             // ThreadX 线程控制块
uint8_t SerialCommThreadStack[2048];    // 线程栈空间 2KB
TX_SEMAPHORE SerialCommSem;             // 信号量，用于串口接收通知线程处理

// =============================
// 全局变量，用于供其他模块（如 LQR 控制）访问小车速度信息
// =============================
int16_t maixcam_vx = 0;     // 从 MaixCAM 接收到的线速度（单位 mm/s）
int16_t maixcam_wz = 0;     // 从 MaixCAM 接收到的角速度（单位 mrad/s）
uint8_t uart_rx_buf[128];  // 串口接收缓冲区（DMA 直接写入）
uint16_t maixcam_len;
// =============================
// 串口通信处理线程函数
// 功能：等待信号量，解析串口数据帧，提取速度信息
// [[noreturn]] 表示该函数不会返回
// =============================
[[noreturn]] void SerialCommThreadFun(ULONG input) {
    // 启动 DMA + IDLE 模式接收
    mavlink_status_t status;
    /*选择一个Mavlink通道*/
    int chan = MAVLINK_COMM_0;
    /*创建一个Mavlink消息结构体*/
    mavlink_message_t msg;
    Msg_Control_Vector_t MAIXCAM_vector={.vel = 0,.w=0};
    om_topic_t *ctl_topic = om_find_topic("MAIXCAM_CTL", UINT32_MAX);
    while (1) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart_rx_buf, sizeof(uart_rx_buf));
        // 等待串口数据接收完成信号（由中断回调发出）
        if (tx_semaphore_get(&SerialCommSem, 1000) == TX_NO_INSTANCE) {
            MAIXCAM_vector.w = 0;
            MAIXCAM_vector.vel = 0;
            MAIXCAM_vector.timestamp = tx_time_get();
            MAIXCAM_vector.over_time = true;
            om_publish(ctl_topic, &MAIXCAM_vector, sizeof(MAIXCAM_vector), false, false);
            tx_semaphore_get(&SerialCommSem, TX_WAIT_FOREVER);
        }
        // 清除 CPU 数据缓存，确保读取的是 DMA 写入后的最新数据
        SCB_InvalidateDCache_by_Addr((uint32_t *) uart_rx_buf, sizeof(uart_rx_buf));
        mavlink_maixcam_lane_feedback_t maixcam_msg;

        /*收到新数据*/
        for (ULONG i = 0; i < maixcam_len; i++) {
            /*解包*/
            /*MavlinkV2出现错误包后，再次接收二个正常包后恢复正常解析，但第一个正常包将丢失，第二个可被正确解析*/
            if (mavlink_parse_char(chan, uart_rx_buf[i], &msg, &status)) {
                /*解析包成功 处理数据*/
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK: {
                        mavlink_msg_maixcam_lane_feedback_decode(&msg, &maixcam_msg);
                        MAIXCAM_vector.vel = maixcam_msg.vx;
                        MAIXCAM_vector.w = maixcam_msg.wz;
                        MAIXCAM_vector.timestamp = tx_time_get();
                        MAIXCAM_vector.over_time = false;
                        /*Do someting with new message*/
                        /*...*/
                        break;
                    }
                }
            }
        }
        if (tx_time_get() - MAIXCAM_vector.timestamp > 1000) {
            MAIXCAM_vector.over_time = true;
        }

        om_publish(ctl_topic, &MAIXCAM_vector, sizeof(MAIXCAM_vector), false, false);
    }
}

