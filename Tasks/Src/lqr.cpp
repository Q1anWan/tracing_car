#include "core.h"
#include "app_threadx.h"
#include "CarMsgs.h"
#include "om.h"
#include "KF_VelFusion.h"
#include "Filter.hpp"

#define t  0.001f
#define t2 0.000001f
#define t3 0.000000001f
#define t4 0.000000000001f
#define t5 0.000000000000001f

class cVelFusionKF {
protected:
    const float qq = 5.0f; //10
    const float rv = 0.1f;
    const float ra = 60.0f; //25.0f

    const float A_Init[9] = {1, t, t2 / 2, 0, 1, t, 0, 0, 1};
    const float Q_Init[9] = {
        t5 / 20 * qq, t4 / 8 * qq, t3 / 6 * qq, t4 / 8 * qq, t3 / 3 * qq, t2 / 2 * qq, t3 / 6 * qq,
        t2 / 2 * qq, t * qq
    };
    const float H_Init[6] = {0, 1, 0, 0, 0, 1};
    const float P_Init[9] = {10, 0, 0, 0, 10, 0, 0, 0, 10};
    const float R_Init[4] = {rv, 0, 0, ra};

public:
    VelFusionKF_t KF;

    cVelFusionKF() {
        VelFusionKF_Init(&this->KF, 3, 2); //Inertia odome 3 State 2 observation
        memcpy(this->KF.P_data, P_Init, sizeof(P_Init));
        memcpy(this->KF.A_data, A_Init, sizeof(A_Init));
        memcpy(this->KF.Q_data, Q_Init, sizeof(Q_Init));
        memcpy(this->KF.H_data, H_Init, sizeof(H_Init));
        memcpy(this->KF.R_data, R_Init, sizeof(R_Init));
    }

    void ResetKF(void) { VelFusionKF_Reset(&this->KF); }

    void UpdateKalman(float Velocity, float AccelerationX) {
        this->KF.MeasuredVector[0] = Velocity;
        this->KF.MeasuredVector[1] = AccelerationX;
        VelFusionKF_Update(&this->KF);
    }

    float GetXhat() {
        return this->KF.xhat.pData[0];
    }

    float GetVhat() {
        return this->KF.xhat.pData[1];
    }
};

float ref_x[4];
float fdb_x[4];
float ut[2];
TX_THREAD LQRThread;
uint8_t LQRThreadStack[4096] = {0};

Msg_Motor_Fdb_t motor_fdb;

[[noreturn]] void LQRThreadFun(ULONG initial_input) {
    om_suber_t *ins_suber = om_subscribe(om_find_topic("INS", UINT32_MAX));
    om_suber_t *motor_fdb_suber = om_subscribe(om_find_topic("MOTOR_FDB", UINT32_MAX));
    om_suber_t *rmt_suber = om_subscribe(om_find_topic("REMOTER", UINT32_MAX));
    om_topic_t *motor_control_topic = om_find_topic("MOTOR_CTL", UINT32_MAX);


    Msg_Motor_Ctr_t motor_control;
    Msg_INS_t ins;
    Msg_Remoter_t rmt;

    cVelFusionKF kf;
    cFilterBTW2_1000Hz_33Hz filter_control[4];

    float K[8] = {-1.963278, -0.750100, 1.442505, 0.299184, 1.963278, 0.750100, 1.442505, 0.299184};
    // float ref_x[4];
    // float fdb_x[4];
    // float ut[2];
    float multi_yaw = 0.0f;
    float yaw_last = 0.0f;
    for (;;) {
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(motor_fdb_suber, &motor_fdb, false);
        om_suber_export(rmt_suber, &rmt, false);

        //KF
        float vel_temp = 0.5f * (motor_fdb.vel[0] - motor_fdb.vel[1]) * WHEEL_R;

        float q_inv[4] = {ins.quaternion[0], -ins.quaternion[1], -ins.quaternion[2], -ins.quaternion[3]};
        float a_body[4] = {0, ins.accel[0], ins.accel[1], ins.accel[2]};
        float a_world[4] = {0};
        float tmp[4] = {0};
        arm_quaternion_product_f32(ins.quaternion, a_body, tmp, 1);
        arm_quaternion_product_f32(tmp, q_inv, a_world, 1);

        //Head direction
        float a = sqrtf(a_world[1] * a_world[1] + a_world[2] * a_world[2]) *
                  arm_cos_f32(atan2f(a_world[2], a_world[1]) - ins.euler[2]);

        kf.UpdateKalman(vel_temp, a);


        float yaw_offset = ins.euler[2] - yaw_last;
        if (yaw_offset > PI) {
            multi_yaw += yaw_offset - 2 * PI;
        } else if (yaw_offset < -PI) {
            multi_yaw += yaw_offset + 2 * PI;
        } else {
            multi_yaw += yaw_offset;
        }
        yaw_last = ins.euler[2];

        fdb_x[0] = kf.GetXhat();
        fdb_x[1] = kf.GetVhat();
        fdb_x[2] = multi_yaw;
        fdb_x[3] = ins.gyro[2];

        if (rmt.switch_left == 2) {
            ref_x[1] = filter_control[0].Update(rmt.ch_3);
            ref_x[0] = ref_x[0] + ref_x[1] * 0.001f; //s = v*t
            //相距太大作下限制幅度
            if (ref_x[0] - fdb_x[0] > 0.2f) {
                ref_x[0] = fdb_x[0] + 0.2f;
            } else if (ref_x[0] - fdb_x[0] < -0.2f) {
                ref_x[0] = fdb_x[0] - 0.2f;
            }

            ref_x[2] -= filter_control[1].Update(rmt.ch_0) * 0.01f;
            ref_x[3] = 0;

            ut[0] = K[0] * (fdb_x[0] - ref_x[0]) + K[1] * (fdb_x[1] - ref_x[1]) +
                    K[2] * (fdb_x[2] - ref_x[2]) + K[3] * (fdb_x[3] - ref_x[3]);
            ut[1] = K[4] * (fdb_x[0] - ref_x[0]) + K[5] * (fdb_x[1] - ref_x[1]) +
                    K[6] * (fdb_x[2] - ref_x[2]) + K[7] * (fdb_x[3] - ref_x[3]);

            motor_control.torque[0] = ut[0];
            motor_control.torque[1] = ut[1];
        } else {
            ref_x[0] = fdb_x[0];
            ref_x[1] = fdb_x[1];
            ref_x[2] = fdb_x[2];
            ref_x[3] = fdb_x[3];

            multi_yaw = ins.euler[2];
            yaw_last = ins.euler[2];

            motor_control.torque[0] = 0;
            motor_control.torque[1] = 0;

            filter_control[0].CleanBuf();
            filter_control[1].CleanBuf();
        }

        om_publish(motor_control_topic, &motor_control, sizeof(motor_control), true, false);
        tx_thread_sleep(1);
    }
}
