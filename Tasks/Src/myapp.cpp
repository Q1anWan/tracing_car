#include "main.h"
#include "app_threadx.h"
#include "om.h"
#include "CarMsgs.h"

TX_THREAD my_thread;
TX_THREAD my_thread2;
uint8_t my_thread_stack[1024];
uint8_t my_thread_stack2[1024];

TX_SEMAPHORE my_sem;

/*OneMessage pool*/
TX_BYTE_POOL MsgPool;
UCHAR Msg_PoolBuf[4096] = {0};

/*EKF pool*/
TX_BYTE_POOL MathPool;
UCHAR Math_PoolBuf[14336] = {0};

extern TX_THREAD SerialCommThread;
extern uint8_t SerialCommThreadStack[2048];

extern void SerialCommThreadFun(ULONG initial_input);  // 声明函数


extern TX_THREAD RemoterThread;
extern uint8_t RemoterThreadStack[2048];
extern TX_SEMAPHORE RemoterThreadSem;

extern void RemoterThreadFun(ULONG initial_input);


extern TX_THREAD MotorThread;
extern uint8_t MotorThreadStack[3072];

extern void MotorThreadFun(ULONG initial_input);

extern TX_THREAD IMUThread;
extern uint8_t IMUThreadStack[4096];

extern void IMUThreadFun(ULONG initial_input);

extern TX_THREAD IMUTemThread;
extern uint8_t IMUTemThreadStack[1024];
extern void IMUTemThreadFun(ULONG initial_input);

extern TX_THREAD LQRThread;
extern uint8_t LQRThreadStack[4096];
extern void LQRThreadFun(ULONG initial_input);

void my_thread_entry(ULONG thread_input) {
    /* Enter into a forever loop. */
    LL_GPIO_SetOutputPin(EN5V_GPIO_Port,EN5V_Pin);
    LL_GPIO_SetOutputPin(OUT1_GPIO_Port, OUT1_Pin);
    LL_GPIO_SetOutputPin(OUT2_GPIO_Port, OUT2_Pin);
}

void my_thread_entry2(ULONG thread_input) {
    /* Enter into a forever loop. */
    while (1) {
        /* Increment thread counter. */
        if (tx_semaphore_get(&my_sem, TX_WAIT_FOREVER) == TX_SUCCESS) {
            HAL_GPIO_TogglePin(OUT1_GPIO_Port, OUT1_Pin);
            HAL_GPIO_TogglePin(OUT2_GPIO_Port, OUT2_Pin);
        }
        /* Sleep for 1 tick. */
        // tx_thread_sleep(500);
    }
}

extern "C" void myapp_start() {
    /*Communication pool in ccram*/
    tx_byte_pool_create(
        &MsgPool,
        (CHAR *) "Msg_Pool",
        Msg_PoolBuf,
        sizeof(Msg_PoolBuf));
    om_init();
    om_config_topic(nullptr, "CA", "REMOTER", sizeof(Msg_Remoter_t));
    om_config_topic(nullptr, "CA", "INS", sizeof(Msg_INS_t));
    om_config_topic(nullptr, "CA", "MOTOR_FDB", sizeof(Msg_Motor_Fdb_t));
    om_config_topic(nullptr, "CA", "MOTOR_CTL", sizeof(Msg_Motor_Ctr_t));
    tx_byte_pool_create(
        &MathPool,
        (CHAR *) "Math_Pool",
        Math_PoolBuf,
        sizeof(Math_PoolBuf));


    tx_semaphore_create(&my_sem, "My Semaphore", 0);
    tx_semaphore_create(&RemoterThreadSem, "RemoterSemaphore", 0);

    /* Create my_thread! */
    tx_thread_create(&my_thread, "My Thread1",
                     my_thread_entry, 0x1234, my_thread_stack, sizeof(my_thread_stack),
                     3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);

    // tx_thread_create(&my_thread2, "My Thread2",
    //                  my_thread_entry2, 0x1234, my_thread_stack2, sizeof(my_thread_stack2),
    //                  3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&RemoterThread, "Remoter",
                     RemoterThreadFun, 0x0000, RemoterThreadStack, sizeof(RemoterThreadStack),
                     2, 2, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&MotorThread, "Motor",
                     MotorThreadFun, 0x0000, MotorThreadStack, sizeof(MotorThreadStack),
                     2, 2, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&IMUThread, "IMU", IMUThreadFun, 0x0000, IMUThreadStack, sizeof(IMUThreadStack), 2, 2,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&IMUTemThread, "IMU_Temp", IMUTemThreadFun, 0x0000, IMUTemThreadStack, sizeof(IMUTemThreadStack), 4, 4,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&LQRThread,"LQR", LQRThreadFun, 0x0000, LQRThreadStack, sizeof(LQRThreadStack), 3, 3,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&SerialCommThread, "SerialComm",SerialCommThreadFun, 0x0000,SerialCommThreadStack, sizeof(SerialCommThreadStack),3, 3,
                 TX_NO_TIME_SLICE, TX_AUTO_START);

}
