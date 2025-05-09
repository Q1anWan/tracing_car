/** @file
 *  @brief MAVLink comm protocol generated from mavlink.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_MAVLINK_H
#define MAVLINK_MAVLINK_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_MAVLINK.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_MAVLINK_XML_HASH -6538925925454141697

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{100, 154, 12, 12, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_MAVLINK

// ENUM DEFINITIONS


/** @brief System ID for MaixCAM to MCU MAVLink messages. */
#ifndef HAVE_ENUM_MAIXCAM_SYSTEM_ID
#define HAVE_ENUM_MAIXCAM_SYSTEM_ID
typedef enum MAIXCAM_SYSTEM_ID
{
   SYS_ID_MCU=1, /* MCU that receives data for LQR control. | */
   SYS_ID_MAIXCAM=10, /* MaixCAM device sending tracking data. | */
   MAIXCAM_SYSTEM_ID_ENUM_END=11, /*  | */
} MAIXCAM_SYSTEM_ID;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_maixcam_lane_feedback.h"

// base include



#if MAVLINK_MAVLINK_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_MAIXCAM_LANE_FEEDBACK}
# define MAVLINK_MESSAGE_NAMES {{ "MAIXCAM_LANE_FEEDBACK", 100 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_MAVLINK_H
