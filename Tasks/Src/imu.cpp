#include "spi.h"
#include "usart.h"

#include "DL_H723.h"
#include "DWT.hpp"

#include "libspi-i-hal-1.0.hpp"
#include "libbmi088-1.0.hpp"
#include "libpid-i-2.1.hpp"

#include "om.h"
#include "CarMsgs.h"

#include "Filter.hpp"
#include "QuaternionEKF.h"

#include "core.h"
#include "CRC.h"
#include "Flash.hpp"
#include "adc.h"

using namespace SPI;
using namespace BMI088;
using namespace IMUA;


uint8_t BMI088_Config(cBMI088 &bmi088);

static cIMUA *imu_handle = nullptr;

TX_THREAD IMUThread;
uint8_t IMUThreadStack[4096] = {0};

__PACKED_STRUCT imu_cal_t {
    float gyro[3];
    float accel_k;
    uint8_t crc;
};

float YAW_T;
float GRAVITY_FIXED = 9.7887f;
Msg_INS_t msg_ins{};

[[noreturn]] void IMUThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    /* INS Topic */
    om_topic_t *ins_topic = om_find_topic("INS", UINT32_MAX);

    cDWT dwt;
    int16_t accel[3];
    int16_t gyro[3];
    float accel_f_norm[3];
    float gyro_f[3];
    float gyro_offset[3] = {0};
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float last_yaw = 0.0f;
    float multi_yaw = 0.0f;

    cFilterBTW2_1000Hz_100Hz filter_accel[3];
    cFilterBTW2_1000Hz_100Hz filter_gyro[3];
    // Msg_INS_t msg_ins{};
    imu_cal_t imu_cal{};

    Flash::cFlashCore::flash_memcpy((uint8_t *) &imu_cal, (uint8_t *) FLASH_STORAGE_BASE_ADDRESS, sizeof(imu_cal_t));
    if (imu_cal.crc == CRC8Maxim((uint8_t *) &imu_cal, sizeof(imu_cal_t) - 1)) {
        gyro_offset[0] = imu_cal.gyro[0];
        gyro_offset[1] = imu_cal.gyro[1];
        gyro_offset[2] = imu_cal.gyro[2];
    } else {
        gyro_offset[0] = 0.0f;
        gyro_offset[1] = 0.0f;
        gyro_offset[2] = 0.0f;
        imu_cal.gyro[0] = 0.0f;
        imu_cal.gyro[1] = 0.0f;
        imu_cal.gyro[2] = 0.0f;
        imu_cal.accel_k = 1.0f;
    }
    GRAVITY_FIXED = GRAVITY_FIXED * imu_cal.accel_k;

    cSPI spi_accel(&hspi2, SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, UINT32_MAX);
    cSPI spi_gyro(&hspi2, SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, UINT32_MAX);
    cBMI088 bmi088(&spi_accel, &spi_gyro);

    imu_handle = &bmi088;
    BMI088_Config(bmi088);

    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
    /* EXTI interrupt init*/
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    tx_thread_sleep(120);
    if (!LL_GPIO_IsInputPinSet(KEY_GPIO_Port, KEY_Pin)) {
        if (!LL_TIM_IsEnabledCounter(TIM12)) {
            LL_TIM_CC_EnableChannel(TIM12, LL_TIM_CHANNEL_CH2);
            LL_TIM_EnableAllOutputs(TIM12);
            LL_TIM_EnableCounter(TIM12);
        }

        uint16_t cnt[2];
        cnt[0] = 19;
        cnt[1] = 0;
        LL_TIM_OC_SetCompareCH2(TIM12, cnt[0]);
        tx_thread_sleep(300);
        LL_TIM_OC_SetCompareCH2(TIM12, 0);

        //Wait for IMU Heat
        while (bmi088.GetTem() < 49.0f) {
            tx_thread_sleep(100);
        }
        tx_thread_sleep(5000);

        //Pre-set
        gyro_offset[0] = 0.0f;
        gyro_offset[1] = 0.0f;
        gyro_offset[2] = 0.0f;
        accel_f_norm[0] = 1.0f / LSB_ACC_16B_12G;

        /*Calibrate Gyro for 100 s*/
        for (uint16_t i = 0; i < 10000; i++) {
            bmi088.GetGyro((uint8_t *) gyro);
            bmi088.GetAccel((uint8_t *) accel);

            gyro_offset[0] = 0.99f * gyro_offset[0] - 0.01f * static_cast<float>(gyro[0]);
            gyro_offset[1] = 0.99f * gyro_offset[1] - 0.01f * static_cast<float>(gyro[1]);
            gyro_offset[2] = 0.99f * gyro_offset[2] - 0.01f * static_cast<float>(gyro[2]);

            accel_f_norm[0] = 0.99f * accel_f_norm[0] + 0.01f * sqrtf(static_cast<float>(accel[0] * accel[0] +
                                                                                         accel[1] * accel[1] +
                                                                                         accel[2] * accel[2]));

            if (++cnt[1] == 100) {
                LL_TIM_OC_SetCompareCH2(TIM12, cnt[0]);
                cnt[0] = cnt[0] ? 0 : 19;
                cnt[1] = 0;
            }
            tx_thread_sleep(10);
        }

        LL_TIM_OC_SetCompareCH2(TIM12, 19);
        __disable_interrupts();
        FLASH_EraseInitTypeDef EraseInitStruct{.TypeErase=FLASH_TYPEERASE_SECTORS, .Banks=FLASH_BANK_1, .Sector=FLASH_STORAGE_SECTOR, .NbSectors=1, .VoltageRange=FLASH_VOLTAGE_RANGE_3};
        HAL_FLASH_Unlock();
        /*Erase Flash*/
        uint32_t error_msg;
        uint8_t write_buf[32];
        imu_cal.gyro[0] = gyro_offset[0];
        imu_cal.gyro[1] = gyro_offset[1];
        imu_cal.gyro[2] = gyro_offset[2];
        imu_cal.accel_k = 1.0f / (accel_f_norm[0] * static_cast<float>(LSB_ACC_16B_12G));
        imu_cal.crc = CRC8Maxim((uint8_t *) &imu_cal, sizeof(imu_cal_t) - 1);
        memcpy(write_buf, &imu_cal, sizeof(imu_cal_t));
        if (HAL_FLASHEx_Erase(&EraseInitStruct, &error_msg) != HAL_OK) {
            HAL_FLASH_Lock();
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, FLASH_STORAGE_BASE_ADDRESS, (uint32_t) write_buf) !=
            HAL_OK) {
            HAL_FLASH_Lock();
        }
        HAL_FLASH_Lock();
        NVIC_SystemReset();
    }


    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1);
    dwt.update();
    for (;;) {
        tx_thread_sleep(1);

        bmi088.GetAccel((uint8_t *) accel);
        bmi088.GetGyro((uint8_t *) gyro);
        accel_f_norm[0] =
                static_cast<float>(accel[0]) * static_cast<float>(LSB_ACC_16B_12G) * GRAVITY_FIXED;
        accel_f_norm[1] =
                static_cast<float>(accel[1]) * static_cast<float>(LSB_ACC_16B_12G) * GRAVITY_FIXED;
        accel_f_norm[2] =
                static_cast<float>(accel[2]) * static_cast<float>(LSB_ACC_16B_12G) * GRAVITY_FIXED;

        /*100Hz LowPass BWT 2-Order*/
        /*Watch out! Orientation R-F-U*/
        accel_f_norm[0] = filter_accel[0].Update(accel_f_norm[0]);
        accel_f_norm[1] = filter_accel[1].Update(accel_f_norm[1]);
        accel_f_norm[2] = filter_accel[2].Update(accel_f_norm[2]);

        gyro_f[0] = filter_gyro[0].Update((static_cast<float> (gyro[0]) + imu_cal.gyro[0]) * static_cast<float> (LSB_GYRO_16B_1000_R));
        gyro_f[1] = filter_gyro[1].Update((static_cast<float> (gyro[1]) + imu_cal.gyro[1]) * static_cast<float> (LSB_GYRO_16B_1000_R));
        gyro_f[2] = filter_gyro[2].Update((static_cast<float> (gyro[2]) + imu_cal.gyro[2]) * static_cast<float> (LSB_GYRO_16B_1000_R));

        IMU_QuaternionEKF_Update(quaternion, gyro_f[0], gyro_f[1], gyro_f[2], accel_f_norm[0], accel_f_norm[1],
                                 accel_f_norm[2],
                                 dwt.dt_sec());

        /*Message*/
        msg_ins.timestamp = tx_time_get();
        memcpy(msg_ins.quaternion, quaternion, sizeof(quaternion));

        msg_ins.accel[0] = accel_f_norm[0];
        msg_ins.accel[1] = accel_f_norm[1];
        msg_ins.accel[2] = accel_f_norm[2];
        msg_ins.gyro[0] = gyro_f[0];
        msg_ins.gyro[1] = gyro_f[1];
        msg_ins.gyro[2] = gyro_f[2];

        msg_ins.euler[0] = atan2f(2.0f * (quaternion[0] * quaternion[1] +
                                          quaternion[2] * quaternion[3]),
                                  2.0f * (quaternion[0] * quaternion[0] +
                                          quaternion[3] * quaternion[3]) - 1.0f);
        msg_ins.euler[1] =
                asinf(-2.0f * (quaternion[1] * quaternion[3] -
                               quaternion[0] * quaternion[2]));

        float yaw_tmp = atan2f(2.0f * (quaternion[0] * quaternion[3] +
                                       quaternion[1] * quaternion[2]),
                               2.0f * (quaternion[0] * quaternion[0] +
                                       quaternion[1] * quaternion[1]) - 1.0f);
        float yaw_diff = yaw_tmp - last_yaw;
        if (yaw_diff > PI) {
            multi_yaw += yaw_diff - 2 * PI;
        } else if (yaw_diff < -PI) {
            multi_yaw += yaw_diff + 2 * PI;
        } else {
            multi_yaw += yaw_diff;
        }
        msg_ins.euler[2] = multi_yaw;
        YAW_T = multi_yaw;
        last_yaw = yaw_tmp;

        om_publish(ins_topic, &msg_ins, sizeof(msg_ins), true, false);
    }
}

TX_THREAD IMUTemThread;
uint8_t IMUTemThreadStack[1024] = {0};
float tmp_last;
float battery_voltage = 24.0f;
[[noreturn]] void IMUTemThreadFun(ULONG initial_input) {
    UNUSED(initial_input);

    // Msg_DBG_t dbg = {};
    // om_topic_t *dbg_topic = om_find_topic("DBG", UINT32_MAX);
    uint32_t buzzer_value=0;
    cDWT dwt;
    PID::PID_Inc_f pid(8.0f, 0.5f, 3.0f, 0.0f, 0.32, 49, 0, false, 0, true, 1.5f);
    pid.SetRef(50.0f);

    LL_TIM_EnableAllOutputs(TIM12);
    LL_TIM_CC_EnableChannel(TIM12, LL_TIM_CHANNEL_CH2);
    LL_TIM_EnableCounter(TIM12);
    LL_TIM_OC_SetCompareCH2(TIM12, 99);

    LL_TIM_EnableAllOutputs(TIM3);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableCounter(TIM3);

    HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
    /*Assert*/
    while (imu_handle == nullptr) {
        LL_TIM_OC_SetCompareCH4(TIM3, 0);
        tx_thread_sleep(100);
    }
    LL_TIM_OC_SetCompareCH2(TIM12, 0);

    tmp_last = imu_handle->GetTem();
    LL_TIM_OC_SetCompareCH4(TIM3, 99);
    tx_thread_sleep(2000);

    if (tmp_last == imu_handle->GetTem()) {
        //error in temp
        LL_TIM_OC_SetCompareCH4(TIM3, 0);
        tx_thread_suspend(&IMUTemThread);
    }

    while (imu_handle->GetTem() < 47.5f) {
        battery_voltage = 0.9f * battery_voltage + HAL_ADC_GetValue(&hadc1) * 0.000055389404296875f;
        tmp_last = imu_handle->GetTem();
        HAL_ADC_Start(&hadc1);
        tx_thread_sleep(320);
    }

    dwt.update();
    for (;;) {
        tx_thread_sleep(320);
        battery_voltage = 0.9f * battery_voltage + HAL_ADC_GetValue(&hadc1) * 0.000055389404296875f;

        if (battery_voltage > 8.0f) {
            if (battery_voltage < 11.4) {

                if (buzzer_value == 0) {
                    buzzer_value = 49;
                } else {
                    buzzer_value = 0;
                }
            }
            else {
                buzzer_value = 0;
            }
            LL_TIM_OC_SetCompareCH2(TIM12, buzzer_value);
        }

        tmp_last = imu_handle->GetTem();
        float k_pid = (24.0f/ battery_voltage);
        k_pid = k_pid*k_pid*pid.Calculate(imu_handle->GetTem(), dwt.dt_sec());
        if (k_pid > 199) {
            k_pid = 199;
        }
        LL_TIM_OC_SetCompareCH4(TIM3, static_cast<uint32_t>(k_pid));
        HAL_ADC_Start(&hadc1);
    }
}

void EXTI15_10_IRQHandler() {
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10)) {
        static uint16_t cnt;
        if (cnt++ == 320) {
            cnt = 0;
            imu_handle->UpdateTem();
        }
        imu_handle->UpdateAccel();
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
    }
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12)) {
        imu_handle->UpdateGyro();
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
    }
}

uint8_t BMI088_Config(cBMI088 &bmi088) {
    /*Begin ACC SPI Communication*/
    bmi088.ReadReg(0x00, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(1);
    /*Reset Sensor*/
    bmi088.WriteReg(0x14, 0xB6, BMI088_CS::CS_GYRO);
    tx_thread_sleep(100);
    bmi088.WriteReg(0x7E, 0xB6, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(100);
    /*Begin ACC SPI Communication*/
    bmi088.ReadReg(0x00, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(1);
    bmi088.ReadReg(0x00, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(1);

    /*Check Chip Connection*/
    while (bmi088.ReadReg(0x00, BMI088_CS::CS_GYRO) != 0x0F) {
        return 0x01;
        //ERROR
    }
    /*Check Chip Connection*/
    while (bmi088.ReadReg(0x00, BMI088_CS::CS_ACCEL) != 0x1E) {
        return 0x02;
        //ERROR
    }
    tx_thread_sleep(10);

    /*Start to config IMU*/
    /*Enable accelerometer */
    bmi088.WriteReg(0x7D, 0x04, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(10);
    /*Normal Mode*/
    bmi088.WriteReg(0x7C, 0x00, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(10);
    /*Accel range +-12G */
    bmi088.WriteReg(0x41, 0x02, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(5);
    /*Accel ODR 1.6KHz BW 280Hz*/
    bmi088.WriteReg(0x40, 0xAC, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(5);
    /*Accel INT1 PP AL*/
    bmi088.WriteReg(0x53, 0x08, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(5);
    /*Accel DRY pin to INT1*/
    bmi088.WriteReg(0x58, 0x04, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(5);

    /*Gyro range 1000dps*/
    bmi088.WriteReg(0x0F, 0x01, BMI088_CS::CS_GYRO);
    tx_thread_sleep(5);
    /*Gyro ODR 1KHz BW 116Hz*/
    bmi088.WriteReg(0x10, 0x02, BMI088_CS::CS_GYRO);
    tx_thread_sleep(5);
    /*Gyro Normal Mode*/
    bmi088.WriteReg(0x11, 0x00, BMI088_CS::CS_GYRO);
    tx_thread_sleep(5);
    /*Gyro INT DRY*/
    bmi088.WriteReg(0x15, 0x80, BMI088_CS::CS_GYRO);
    tx_thread_sleep(5);
    /*Gyro INT3 PP AL*/
    bmi088.WriteReg(0x16, 0x0C, BMI088_CS::CS_GYRO);
    tx_thread_sleep(5);
    /*Gyro DRY pin to INT3*/
    bmi088.WriteReg(0x18, 0x01, BMI088_CS::CS_GYRO);
    tx_thread_sleep(5);

    /*Enable IDLE*/
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);
    NVIC_EnableIRQ(EXTI4_IRQn);//ACC
    NVIC_EnableIRQ(EXTI9_5_IRQn);//GYRO
    return 0;
}
