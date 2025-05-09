#pragma once
// MESSAGE MAIXCAM_LANE_FEEDBACK PACKING

#define MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK 100


typedef struct __mavlink_maixcam_lane_feedback_t {
 float vx; /*< [m/s] Line-tracking linear velocity (forward)*/
 float wz; /*< [rad/s] Angular velocity around Z axis (yaw)*/
 uint32_t timestamp; /*< [ms] Timestamp in milliseconds (system time)*/
} mavlink_maixcam_lane_feedback_t;

#define MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN 12
#define MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_MIN_LEN 12
#define MAVLINK_MSG_ID_100_LEN 12
#define MAVLINK_MSG_ID_100_MIN_LEN 12

#define MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_CRC 154
#define MAVLINK_MSG_ID_100_CRC 154



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAIXCAM_LANE_FEEDBACK { \
    100, \
    "MAIXCAM_LANE_FEEDBACK", \
    3, \
    {  { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_maixcam_lane_feedback_t, vx) }, \
         { "wz", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_maixcam_lane_feedback_t, wz) }, \
         { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_maixcam_lane_feedback_t, timestamp) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAIXCAM_LANE_FEEDBACK { \
    "MAIXCAM_LANE_FEEDBACK", \
    3, \
    {  { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_maixcam_lane_feedback_t, vx) }, \
         { "wz", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_maixcam_lane_feedback_t, wz) }, \
         { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_maixcam_lane_feedback_t, timestamp) }, \
         } \
}
#endif

/**
 * @brief Pack a maixcam_lane_feedback message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vx [m/s] Line-tracking linear velocity (forward)
 * @param wz [rad/s] Angular velocity around Z axis (yaw)
 * @param timestamp [ms] Timestamp in milliseconds (system time)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_maixcam_lane_feedback_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float vx, float wz, uint32_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, wz);
    _mav_put_uint32_t(buf, 8, timestamp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN);
#else
    mavlink_maixcam_lane_feedback_t packet;
    packet.vx = vx;
    packet.wz = wz;
    packet.timestamp = timestamp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_CRC);
}

/**
 * @brief Pack a maixcam_lane_feedback message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param vx [m/s] Line-tracking linear velocity (forward)
 * @param wz [rad/s] Angular velocity around Z axis (yaw)
 * @param timestamp [ms] Timestamp in milliseconds (system time)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_maixcam_lane_feedback_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float vx, float wz, uint32_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, wz);
    _mav_put_uint32_t(buf, 8, timestamp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN);
#else
    mavlink_maixcam_lane_feedback_t packet;
    packet.vx = vx;
    packet.wz = wz;
    packet.timestamp = timestamp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN);
#endif
}

/**
 * @brief Pack a maixcam_lane_feedback message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vx [m/s] Line-tracking linear velocity (forward)
 * @param wz [rad/s] Angular velocity around Z axis (yaw)
 * @param timestamp [ms] Timestamp in milliseconds (system time)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_maixcam_lane_feedback_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float vx,float wz,uint32_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, wz);
    _mav_put_uint32_t(buf, 8, timestamp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN);
#else
    mavlink_maixcam_lane_feedback_t packet;
    packet.vx = vx;
    packet.wz = wz;
    packet.timestamp = timestamp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_CRC);
}

/**
 * @brief Encode a maixcam_lane_feedback struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param maixcam_lane_feedback C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_maixcam_lane_feedback_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_maixcam_lane_feedback_t* maixcam_lane_feedback)
{
    return mavlink_msg_maixcam_lane_feedback_pack(system_id, component_id, msg, maixcam_lane_feedback->vx, maixcam_lane_feedback->wz, maixcam_lane_feedback->timestamp);
}

/**
 * @brief Encode a maixcam_lane_feedback struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param maixcam_lane_feedback C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_maixcam_lane_feedback_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_maixcam_lane_feedback_t* maixcam_lane_feedback)
{
    return mavlink_msg_maixcam_lane_feedback_pack_chan(system_id, component_id, chan, msg, maixcam_lane_feedback->vx, maixcam_lane_feedback->wz, maixcam_lane_feedback->timestamp);
}

/**
 * @brief Encode a maixcam_lane_feedback struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param maixcam_lane_feedback C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_maixcam_lane_feedback_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_maixcam_lane_feedback_t* maixcam_lane_feedback)
{
    return mavlink_msg_maixcam_lane_feedback_pack_status(system_id, component_id, _status, msg,  maixcam_lane_feedback->vx, maixcam_lane_feedback->wz, maixcam_lane_feedback->timestamp);
}

/**
 * @brief Send a maixcam_lane_feedback message
 * @param chan MAVLink channel to send the message
 *
 * @param vx [m/s] Line-tracking linear velocity (forward)
 * @param wz [rad/s] Angular velocity around Z axis (yaw)
 * @param timestamp [ms] Timestamp in milliseconds (system time)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_maixcam_lane_feedback_send(mavlink_channel_t chan, float vx, float wz, uint32_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, wz);
    _mav_put_uint32_t(buf, 8, timestamp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK, buf, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_CRC);
#else
    mavlink_maixcam_lane_feedback_t packet;
    packet.vx = vx;
    packet.wz = wz;
    packet.timestamp = timestamp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK, (const char *)&packet, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_CRC);
#endif
}

/**
 * @brief Send a maixcam_lane_feedback message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_maixcam_lane_feedback_send_struct(mavlink_channel_t chan, const mavlink_maixcam_lane_feedback_t* maixcam_lane_feedback)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_maixcam_lane_feedback_send(chan, maixcam_lane_feedback->vx, maixcam_lane_feedback->wz, maixcam_lane_feedback->timestamp);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK, (const char *)maixcam_lane_feedback, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_maixcam_lane_feedback_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float vx, float wz, uint32_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, wz);
    _mav_put_uint32_t(buf, 8, timestamp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK, buf, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_CRC);
#else
    mavlink_maixcam_lane_feedback_t *packet = (mavlink_maixcam_lane_feedback_t *)msgbuf;
    packet->vx = vx;
    packet->wz = wz;
    packet->timestamp = timestamp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK, (const char *)packet, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_CRC);
#endif
}
#endif

#endif

// MESSAGE MAIXCAM_LANE_FEEDBACK UNPACKING


/**
 * @brief Get field vx from maixcam_lane_feedback message
 *
 * @return [m/s] Line-tracking linear velocity (forward)
 */
static inline float mavlink_msg_maixcam_lane_feedback_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field wz from maixcam_lane_feedback message
 *
 * @return [rad/s] Angular velocity around Z axis (yaw)
 */
static inline float mavlink_msg_maixcam_lane_feedback_get_wz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field timestamp from maixcam_lane_feedback message
 *
 * @return [ms] Timestamp in milliseconds (system time)
 */
static inline uint32_t mavlink_msg_maixcam_lane_feedback_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Decode a maixcam_lane_feedback message into a struct
 *
 * @param msg The message to decode
 * @param maixcam_lane_feedback C-struct to decode the message contents into
 */
static inline void mavlink_msg_maixcam_lane_feedback_decode(const mavlink_message_t* msg, mavlink_maixcam_lane_feedback_t* maixcam_lane_feedback)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    maixcam_lane_feedback->vx = mavlink_msg_maixcam_lane_feedback_get_vx(msg);
    maixcam_lane_feedback->wz = mavlink_msg_maixcam_lane_feedback_get_wz(msg);
    maixcam_lane_feedback->timestamp = mavlink_msg_maixcam_lane_feedback_get_timestamp(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN? msg->len : MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN;
        memset(maixcam_lane_feedback, 0, MAVLINK_MSG_ID_MAIXCAM_LANE_FEEDBACK_LEN);
    memcpy(maixcam_lane_feedback, _MAV_PAYLOAD(msg), len);
#endif
}
