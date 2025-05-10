#include "CarMsgs.h"
#include "om.h"
#include "tx_api.h"
#include "core.h"
#include "arm_math.h"
#include "Filter.hpp"

TX_THREAD ScheThread;
uint8_t ScheThreadStack[1024] = {0};
Msg_Remoter_t remoter;
Msg_Control_Vector_t control_vector;
[[noreturn]] void ScheThreadFun(ULONG input) {

    Msg_Control_Vector_t MAIXCAM_vector;


    om_suber_t *rmt_suber = om_subscribe(om_find_topic("REMOTER", UINT32_MAX));
    om_suber_t *cam_vec_suber = om_subscribe(om_find_topic("MAIXCAM_CTL", UINT32_MAX));
    om_topic_t *control_vector_topic = om_find_topic("ROBOT_CTL", UINT32_MAX);

    for (;;) {
        om_suber_export(rmt_suber, &remoter, false);
        om_suber_export(cam_vec_suber, &MAIXCAM_vector, false);

        if (remoter.online==true) {
            if (remoter.switch_left==1) {
                control_vector.vel = 0;
                control_vector.w = 0;
                control_vector.over_time = true;
            }
            else if (remoter.switch_left==2) {
                control_vector.vel = remoter.ch_3*1.2f;
                control_vector.w = -remoter.ch_0*3.0f;
                control_vector.over_time = false;
            }
            else {
                if (MAIXCAM_vector.over_time==false) {
                    control_vector.vel = MAIXCAM_vector.vel;
                    control_vector.w = MAIXCAM_vector.w;
                    control_vector.over_time = false;
                }
                else {
                    control_vector.vel = 0;
                    control_vector.w = 0;
                    control_vector.over_time = true;
                }
            }
        }
        else {
            control_vector.vel = 0;
            control_vector.w = 0;
            control_vector.over_time = true;
        }
        control_vector.timestamp = tx_time_get();
        om_publish(control_vector_topic, &control_vector, sizeof(control_vector), true, false);
        tx_thread_sleep(2);
    }
}