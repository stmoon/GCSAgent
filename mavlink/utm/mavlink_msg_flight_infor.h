#pragma once
// MESSAGE FLIGHT_INFOR PACKING

#define MAVLINK_MSG_ID_FLIGHT_INFOR 180

MAVPACKED(
typedef struct __mavlink_flight_infor_t {
 int32_t droneID; /*< [ID] Drone ID */
 int32_t userID; /*< [ID]  User ID */
}) mavlink_flight_infor_t;

#define MAVLINK_MSG_ID_FLIGHT_INFOR_LEN 8
#define MAVLINK_MSG_ID_FLIGHT_INFOR_MIN_LEN 8
#define MAVLINK_MSG_ID_180_LEN 8
#define MAVLINK_MSG_ID_180_MIN_LEN 8

#define MAVLINK_MSG_ID_FLIGHT_INFOR_CRC 251
#define MAVLINK_MSG_ID_180_CRC 251



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FLIGHT_INFOR { \
    180, \
    "FLIGHT_INFOR", \
    2, \
    {  { "droneID", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_flight_infor_t, droneID) }, \
         { "userID", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_flight_infor_t, userID) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FLIGHT_INFOR { \
    "FLIGHT_INFOR", \
    2, \
    {  { "droneID", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_flight_infor_t, droneID) }, \
         { "userID", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_flight_infor_t, userID) }, \
         } \
}
#endif

/**
 * @brief Pack a flight_infor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param droneID [ID] Drone ID 
 * @param userID [ID]  User ID 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flight_infor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t droneID, int32_t userID)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FLIGHT_INFOR_LEN];
    _mav_put_int32_t(buf, 0, droneID);
    _mav_put_int32_t(buf, 4, userID);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLIGHT_INFOR_LEN);
#else
    mavlink_flight_infor_t packet;
    packet.droneID = droneID;
    packet.userID = userID;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLIGHT_INFOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FLIGHT_INFOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLIGHT_INFOR_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_INFOR_LEN, MAVLINK_MSG_ID_FLIGHT_INFOR_CRC);
}

/**
 * @brief Pack a flight_infor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param droneID [ID] Drone ID 
 * @param userID [ID]  User ID 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flight_infor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t droneID,int32_t userID)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FLIGHT_INFOR_LEN];
    _mav_put_int32_t(buf, 0, droneID);
    _mav_put_int32_t(buf, 4, userID);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLIGHT_INFOR_LEN);
#else
    mavlink_flight_infor_t packet;
    packet.droneID = droneID;
    packet.userID = userID;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLIGHT_INFOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FLIGHT_INFOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLIGHT_INFOR_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_INFOR_LEN, MAVLINK_MSG_ID_FLIGHT_INFOR_CRC);
}

/**
 * @brief Encode a flight_infor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param flight_infor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flight_infor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_flight_infor_t* flight_infor)
{
    return mavlink_msg_flight_infor_pack(system_id, component_id, msg, flight_infor->droneID, flight_infor->userID);
}

/**
 * @brief Encode a flight_infor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flight_infor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flight_infor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_flight_infor_t* flight_infor)
{
    return mavlink_msg_flight_infor_pack_chan(system_id, component_id, chan, msg, flight_infor->droneID, flight_infor->userID);
}

/**
 * @brief Send a flight_infor message
 * @param chan MAVLink channel to send the message
 *
 * @param droneID [ID] Drone ID 
 * @param userID [ID]  User ID 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flight_infor_send(mavlink_channel_t chan, int32_t droneID, int32_t userID)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FLIGHT_INFOR_LEN];
    _mav_put_int32_t(buf, 0, droneID);
    _mav_put_int32_t(buf, 4, userID);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLIGHT_INFOR, buf, MAVLINK_MSG_ID_FLIGHT_INFOR_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_INFOR_LEN, MAVLINK_MSG_ID_FLIGHT_INFOR_CRC);
#else
    mavlink_flight_infor_t packet;
    packet.droneID = droneID;
    packet.userID = userID;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLIGHT_INFOR, (const char *)&packet, MAVLINK_MSG_ID_FLIGHT_INFOR_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_INFOR_LEN, MAVLINK_MSG_ID_FLIGHT_INFOR_CRC);
#endif
}

/**
 * @brief Send a flight_infor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_flight_infor_send_struct(mavlink_channel_t chan, const mavlink_flight_infor_t* flight_infor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_flight_infor_send(chan, flight_infor->droneID, flight_infor->userID);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLIGHT_INFOR, (const char *)flight_infor, MAVLINK_MSG_ID_FLIGHT_INFOR_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_INFOR_LEN, MAVLINK_MSG_ID_FLIGHT_INFOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_FLIGHT_INFOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_flight_infor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t droneID, int32_t userID)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, droneID);
    _mav_put_int32_t(buf, 4, userID);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLIGHT_INFOR, buf, MAVLINK_MSG_ID_FLIGHT_INFOR_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_INFOR_LEN, MAVLINK_MSG_ID_FLIGHT_INFOR_CRC);
#else
    mavlink_flight_infor_t *packet = (mavlink_flight_infor_t *)msgbuf;
    packet->droneID = droneID;
    packet->userID = userID;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLIGHT_INFOR, (const char *)packet, MAVLINK_MSG_ID_FLIGHT_INFOR_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_INFOR_LEN, MAVLINK_MSG_ID_FLIGHT_INFOR_CRC);
#endif
}
#endif

#endif

// MESSAGE FLIGHT_INFOR UNPACKING


/**
 * @brief Get field droneID from flight_infor message
 *
 * @return [ID] Drone ID 
 */
static inline int32_t mavlink_msg_flight_infor_get_droneID(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field userID from flight_infor message
 *
 * @return [ID]  User ID 
 */
static inline int32_t mavlink_msg_flight_infor_get_userID(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Decode a flight_infor message into a struct
 *
 * @param msg The message to decode
 * @param flight_infor C-struct to decode the message contents into
 */
static inline void mavlink_msg_flight_infor_decode(const mavlink_message_t* msg, mavlink_flight_infor_t* flight_infor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    flight_infor->droneID = mavlink_msg_flight_infor_get_droneID(msg);
    flight_infor->userID = mavlink_msg_flight_infor_get_userID(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FLIGHT_INFOR_LEN? msg->len : MAVLINK_MSG_ID_FLIGHT_INFOR_LEN;
        memset(flight_infor, 0, MAVLINK_MSG_ID_FLIGHT_INFOR_LEN);
    memcpy(flight_infor, _MAV_PAYLOAD(msg), len);
#endif
}
