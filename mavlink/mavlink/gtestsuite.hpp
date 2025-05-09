/** @file
 *	@brief MAVLink comm testsuite protocol generated from mavlink.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "mavlink.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(mavlink, MAIXCAM_LANE_FEEDBACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mavlink::msg::MAIXCAM_LANE_FEEDBACK packet_in{};
    packet_in.vx = 17.0;
    packet_in.wz = 45.0;
    packet_in.timestamp = 963497880;

    mavlink::mavlink::msg::MAIXCAM_LANE_FEEDBACK packet1{};
    mavlink::mavlink::msg::MAIXCAM_LANE_FEEDBACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.wz, packet2.wz);
    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
}

#ifdef TEST_INTEROP
TEST(mavlink_interop, MAIXCAM_LANE_FEEDBACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_maixcam_lane_feedback_t packet_c {
         17.0, 45.0, 963497880
    };

    mavlink::mavlink::msg::MAIXCAM_LANE_FEEDBACK packet_in{};
    packet_in.vx = 17.0;
    packet_in.wz = 45.0;
    packet_in.timestamp = 963497880;

    mavlink::mavlink::msg::MAIXCAM_LANE_FEEDBACK packet2{};

    mavlink_msg_maixcam_lane_feedback_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.wz, packet2.wz);
    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
