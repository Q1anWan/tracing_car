#pragma once
#ifndef ZDT_H
#define ZDT_H

#include "cstdint"

namespace ZDT {
    // ZDT state

    enum class ZDTState : uint16_t {
        NORMAL = 0x00,
        NOT_SATISFIED = 0x01,
        IDLE = 0x02
    };

    enum ZDTErrorCode : uint8_t {
        NORMAL = 0x00,
        UNSATISFIED_ID,
        UNSATISFIED_Command,
        ERROR_CHECK,
        FDB_COMMAND_MISSING,
        ERROR_MSG,
        FDB_ERROR_POINTER,
    };

    class cZDT {
    protected:
        uint16_t _id;
        ZDTState _state = ZDTState::IDLE;
        uint16_t _encoder;
        uint64_t _last_update_time = 0;
        uint16_t _last_command;
        float _velocity;
    public:

        cZDT(uint8_t ID = 0x01, uint8_t check_sum = 0x6B) :
                _id(ID) {
        }

        virtual uint8_t GetCheck(uint8_t* data, uint8_t len) {
            return 0x6B;
        }

        void SetID(uint16_t id) { _id = id; }

        uint8_t CommonFDBHandel(uint32_t exID, uint8_t* data);
        uint8_t Enable(uint32_t &exID, uint32_t &datalen, uint8_t *pbuf, uint8_t enable_flag, bool syn_flag = false);

//        uint8_t SetPID(uint32_t &exID0, uint32_t &exID1, uint32_t &exID2, uint32_t &datalen0, uint32_t &datalen1,
//                       uint32_t &datalen2, uint8_t *pbuf0, uint8_t *pbuf1, uint8_t *pbuf2,
//                       bool save_flag, uint32_t kp_trape, uint32_t kp_dirct, uint32_t kp_v, uint32_t ki_v,
//                       uint8_t check_sum = 0x6B);
//        uint8_t SetPIDFDB(uint8_t *pdata, uint8_t check_sum = 0x6B);

        // uint8_t
        // SetZeroPst(uint32_t &exID, uint32_t &datalen, uint8_t *pbuf, bool storage = true, uint8_t check_sum = 0x6B);
        //
        // uint8_t ZeroTrigger(uint32_t &exID, uint32_t &datalen, uint8_t *pbuf, bool move_mode, bool syn_flag = false,
        //                     uint8_t check_sum = 0x6B);
        //
        // uint8_t
        // TpzPst(uint32_t &exID0, uint32_t &exID1, uint32_t &datalen0, uint32_t &datalen1, uint8_t *pbuf0, uint8_t *pbuf1,
        //        bool dir, uint16_t accel,
        //        uint16_t deaccel, float maxspeed, float pst, bool opposite_mode, bool syn_flag = false,
        //        uint8_t check_sum = 0x6B);
        //
        // uint8_t TpzPstFDB(uint8_t *pdata, uint8_t check_sum = 0x6B);

        uint8_t TorqueControl(uint32_t &exID, uint32_t &datalen, uint8_t *pbuf, uint16_t current_ramp, int16_t current, bool syn_flag = false);

        uint8_t ReadVel(uint32_t &exID, uint32_t &datalen, uint8_t *pbuf);
        uint8_t ReadVelFDB(uint32_t exID, uint8_t *pdata);

        // uint8_t ReadEncoder(uint32_t &exID, uint32_t &datalen, uint8_t *pbuf, uint8_t check_sum = 0x6B);
        //
        // uint8_t ReadEncoderFDB(uint8_t *pdata, uint8_t check_sum = 0x6B);

        // uint8_t ClearPstZero(uint32_t &exID, uint32_t &datalen, uint8_t *pbuf, uint8_t check_sum = 0x6B);
        //
        // uint8_t ClearPstZeroFDB(uint8_t *pdata, uint8_t check_sum = 0x6B);
        //
        // uint8_t ClearOverBlock(uint32_t &exID, uint32_t &datalen, uint8_t *pbuf, uint8_t check_sum = 0x6B);
        //
        // uint8_t ClearOverBlockFDB(uint8_t *pdata, uint8_t check_sum = 0x6B);

        uint8_t TimeUpdate(uint64_t new_time);

        uint64_t GetTime();

        void SetState(ZDTState state) {
            _state = state;
        };

        ZDTState GetState();

        float GetVelocity() {
            return _velocity;
        };
    };
}


#endif