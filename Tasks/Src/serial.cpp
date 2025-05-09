#include "usart.h"           // STM32 HAL USART 驱动头文件（包含 huart3 实例）
#include "DL_H723.h"         // 板级支持包头文件（定义缓存控制函数等）
#include "CarMsgs.h"         // 项目通信数据结构定义（如需要定义速度/控制帧结构）
#include "app_threadx.h"     // ThreadX RTOS 的线程与信号量接口
#include "om.h"              // Open Message 通信中间件接口（如未使用可忽略）

// =============================
// 串口通信帧格式定义
// =============================
#define UART_FRAME_LEN 7          // 总帧长度为 7 字节（固定格式）
#define UART_FRAME_HEAD 0xA5      // 帧头标志，用于判断包起始
#define UART_FRAME_TAIL 0x5A      // 帧尾标志，用于判断包结尾

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
uint8_t uart_rx_buf[UART_FRAME_LEN];  // 串口接收缓冲区（DMA 直接写入）

// =============================
// 串口通信处理线程函数
// 功能：等待信号量，解析串口数据帧，提取速度信息
// [[noreturn]] 表示该函数不会返回
// =============================
[[noreturn]] void SerialCommThreadFun(ULONG input) {
    // 启动 DMA + IDLE 模式接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_rx_buf, UART_FRAME_LEN);

    // 禁用 DMA 半中断（只保留 IDLE 中断）
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);

    while (1) {
        // 等待串口数据接收完成信号（由中断回调发出）
        if (tx_semaphore_get(&SerialCommSem, TX_WAIT_FOREVER) == TX_SUCCESS) {

            // 清除 CPU 数据缓存，确保读取的是 DMA 写入后的最新数据
            SCB_InvalidateDCache_by_Addr((uint32_t *) uart_rx_buf, UART_FRAME_LEN);

            // 判断帧头和帧尾是否正确，确保包合法
            if (uart_rx_buf[0] == UART_FRAME_HEAD && uart_rx_buf[6] == UART_FRAME_TAIL) {

                // 校验中间数据是否正确（使用 XOR 校验）
                uint8_t checksum = uart_rx_buf[1] ^ uart_rx_buf[2] ^ uart_rx_buf[3] ^ uart_rx_buf[4];
                if (checksum == uart_rx_buf[5]) {
                    // 提取 vx 和 wz，两字节合并为 int16_t（小端格式）
                    maixcam_vx = (int16_t)(uart_rx_buf[2] << 8 | uart_rx_buf[1]);
                    maixcam_wz = (int16_t)(uart_rx_buf[4] << 8 | uart_rx_buf[3]);
                }
            }

            // 重新启动 DMA + IDLE 模式继续接收下一帧
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_rx_buf, UART_FRAME_LEN);
        }

        // 稍作休眠，释放 CPU 控制权（虽然信号量阻塞已足够）
        tx_thread_sleep(1);
    }
}

// =============================
// 串口接收事件回调函数（版本名带 _serial）
// 由 HAL 在 UART 空闲中断触发时调用
// =============================
void HAL_UARTEx_RxEventCallback_serial(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 判断是否为 USART3 串口，并且数据长度正确
    if (huart == &huart3 && Size == UART_FRAME_LEN)
    {
        // 通知串口处理线程可以解析数据
        tx_semaphore_put(&SerialCommSem);
    }
}
