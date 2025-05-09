// MESSAGE MAIXCAM_LANE_FEEDBACK support class

#pragma once

namespace mavlink {
namespace mavlink {
namespace msg {

/**
 * @brief MAIXCAM_LANE_FEEDBACK message
 *
 * Tracking speed and angular velocity output from MaixCAM vision.
 */
struct MAIXCAM_LANE_FEEDBACK : mavlink::Message {
    static constexpr msgid_t MSG_ID = 100;
    static constexpr size_t LENGTH = 12;
    static constexpr size_t MIN_LENGTH = 12;
    static constexpr uint8_t CRC_EXTRA = 154;
    static constexpr auto NAME = "MAIXCAM_LANE_FEEDBACK";


    float vx; /*< [m/s] Line-tracking linear velocity (forward) */
    float wz; /*< [rad/s] Angular velocity around Z axis (yaw) */
    uint32_t timestamp; /*< [ms] Timestamp in milliseconds (system time) */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  vx: " << vx << std::endl;
        ss << "  wz: " << wz << std::endl;
        ss << "  timestamp: " << timestamp << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << vx;                            // offset: 0
        map << wz;                            // offset: 4
        map << timestamp;                     // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> vx;                            // offset: 0
        map >> wz;                            // offset: 4
        map >> timestamp;                     // offset: 8
    }
};

} // namespace msg
} // namespace mavlink
} // namespace mavlink
