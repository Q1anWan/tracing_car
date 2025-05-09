/** @file
 *	@brief MAVLink comm protocol generated from mavlink.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace mavlink {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (through @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 1> MESSAGE_ENTRIES {{ {100, 154, 12, 12, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 2;


// ENUM DEFINITIONS


/** @brief System ID for MaixCAM to MCU MAVLink messages. */
enum class MAIXCAM_SYSTEM_ID
{
    SYS_ID_MCU=1, /* MCU that receives data for LQR control. | */
    SYS_ID_MAIXCAM=10, /* MaixCAM device sending tracking data. | */
};

//! MAIXCAM_SYSTEM_ID ENUM_END
constexpr auto MAIXCAM_SYSTEM_ID_ENUM_END = 11;


} // namespace mavlink
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_maixcam_lane_feedback.hpp"

// base include

