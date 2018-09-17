#pragma once
// MESSAGE DRONE_INFOR PACKING

#define MAVLINK_MSG_ID_DRONE_INFOR 181

MAVPACKED(
typedef struct __mavlink_drone_infor_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 int32_t lat; /*< [degE7] Latitude, expressed*/
 int32_t lon; /*< [degE7] Longitude, expressed*/
 int32_t alt; /*< [mm] Altitude (AMSL). */
 int16_t vx; /*< [cm/s] Ground X Speed (Latitude, positive north)*/
 int16_t vy; /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
 int16_t vz; /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
 uint16_t hdg; /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. */
}) mavlink_drone_infor_t;

#define MAVLINK_MSG_ID_DRONE_INFOR_LEN 24
#define MAVLINK_MSG_ID_DRONE_INFOR_MIN_LEN 24
#define MAVLINK_MSG_ID_181_LEN 24
#define MAVLINK_MSG_ID_181_MIN_LEN 24

#define MAVLINK_MSG_ID_DRONE_INFOR_CRC 133
#define MAVLINK_MSG_ID_181_CRC 133



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DRONE_INFOR { \
    181, \
    "DRONE_INFOR", \
    8, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_infor_t, time_boot_ms) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_drone_infor_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_drone_infor_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_drone_infor_t, alt) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_drone_infor_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_drone_infor_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_drone_infor_t, vz) }, \
         { "hdg", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_drone_infor_t, hdg) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DRONE_INFOR { \
    "DRONE_INFOR", \
    8, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drone_infor_t, time_boot_ms) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_drone_infor_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_drone_infor_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_drone_infor_t, alt) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_drone_infor_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_drone_infor_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_drone_infor_t, vz) }, \
         { "hdg", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_drone_infor_t, hdg) }, \
         } \
}
#endif

/**
 * @brief Pack a drone_infor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (AMSL). 
 * @param vx [cm/s] Ground X Speed (Latitude, positive north)
 * @param vy [cm/s] Ground Y Speed (Longitude, positive east)
 * @param vz [cm/s] Ground Z Speed (Altitude, positive down)
 * @param hdg [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_infor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_INFOR_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_int16_t(buf, 16, vx);
    _mav_put_int16_t(buf, 18, vy);
    _mav_put_int16_t(buf, 20, vz);
    _mav_put_uint16_t(buf, 22, hdg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_INFOR_LEN);
#else
    mavlink_drone_infor_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_INFOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_INFOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DRONE_INFOR_MIN_LEN, MAVLINK_MSG_ID_DRONE_INFOR_LEN, MAVLINK_MSG_ID_DRONE_INFOR_CRC);
}

/**
 * @brief Pack a drone_infor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (AMSL). 
 * @param vx [cm/s] Ground X Speed (Latitude, positive north)
 * @param vy [cm/s] Ground Y Speed (Longitude, positive east)
 * @param vz [cm/s] Ground Z Speed (Altitude, positive down)
 * @param hdg [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drone_infor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,int32_t lat,int32_t lon,int32_t alt,int16_t vx,int16_t vy,int16_t vz,uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_INFOR_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_int16_t(buf, 16, vx);
    _mav_put_int16_t(buf, 18, vy);
    _mav_put_int16_t(buf, 20, vz);
    _mav_put_uint16_t(buf, 22, hdg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRONE_INFOR_LEN);
#else
    mavlink_drone_infor_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRONE_INFOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRONE_INFOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DRONE_INFOR_MIN_LEN, MAVLINK_MSG_ID_DRONE_INFOR_LEN, MAVLINK_MSG_ID_DRONE_INFOR_CRC);
}

/**
 * @brief Encode a drone_infor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param drone_infor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_infor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_drone_infor_t* drone_infor)
{
    return mavlink_msg_drone_infor_pack(system_id, component_id, msg, drone_infor->time_boot_ms, drone_infor->lat, drone_infor->lon, drone_infor->alt, drone_infor->vx, drone_infor->vy, drone_infor->vz, drone_infor->hdg);
}

/**
 * @brief Encode a drone_infor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param drone_infor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drone_infor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_drone_infor_t* drone_infor)
{
    return mavlink_msg_drone_infor_pack_chan(system_id, component_id, chan, msg, drone_infor->time_boot_ms, drone_infor->lat, drone_infor->lon, drone_infor->alt, drone_infor->vx, drone_infor->vy, drone_infor->vz, drone_infor->hdg);
}

/**
 * @brief Send a drone_infor message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (AMSL). 
 * @param vx [cm/s] Ground X Speed (Latitude, positive north)
 * @param vy [cm/s] Ground Y Speed (Longitude, positive east)
 * @param vz [cm/s] Ground Z Speed (Altitude, positive down)
 * @param hdg [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_drone_infor_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRONE_INFOR_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_int16_t(buf, 16, vx);
    _mav_put_int16_t(buf, 18, vy);
    _mav_put_int16_t(buf, 20, vz);
    _mav_put_uint16_t(buf, 22, hdg);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_INFOR, buf, MAVLINK_MSG_ID_DRONE_INFOR_MIN_LEN, MAVLINK_MSG_ID_DRONE_INFOR_LEN, MAVLINK_MSG_ID_DRONE_INFOR_CRC);
#else
    mavlink_drone_infor_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.hdg = hdg;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_INFOR, (const char *)&packet, MAVLINK_MSG_ID_DRONE_INFOR_MIN_LEN, MAVLINK_MSG_ID_DRONE_INFOR_LEN, MAVLINK_MSG_ID_DRONE_INFOR_CRC);
#endif
}

/**
 * @brief Send a drone_infor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_drone_infor_send_struct(mavlink_channel_t chan, const mavlink_drone_infor_t* drone_infor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_drone_infor_send(chan, drone_infor->time_boot_ms, drone_infor->lat, drone_infor->lon, drone_infor->alt, drone_infor->vx, drone_infor->vy, drone_infor->vz, drone_infor->hdg);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_INFOR, (const char *)drone_infor, MAVLINK_MSG_ID_DRONE_INFOR_MIN_LEN, MAVLINK_MSG_ID_DRONE_INFOR_LEN, MAVLINK_MSG_ID_DRONE_INFOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_DRONE_INFOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_drone_infor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_int16_t(buf, 16, vx);
    _mav_put_int16_t(buf, 18, vy);
    _mav_put_int16_t(buf, 20, vz);
    _mav_put_uint16_t(buf, 22, hdg);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_INFOR, buf, MAVLINK_MSG_ID_DRONE_INFOR_MIN_LEN, MAVLINK_MSG_ID_DRONE_INFOR_LEN, MAVLINK_MSG_ID_DRONE_INFOR_CRC);
#else
    mavlink_drone_infor_t *packet = (mavlink_drone_infor_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->hdg = hdg;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRONE_INFOR, (const char *)packet, MAVLINK_MSG_ID_DRONE_INFOR_MIN_LEN, MAVLINK_MSG_ID_DRONE_INFOR_LEN, MAVLINK_MSG_ID_DRONE_INFOR_CRC);
#endif
}
#endif

#endif

// MESSAGE DRONE_INFOR UNPACKING


/**
 * @brief Get field time_boot_ms from drone_infor message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_drone_infor_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field lat from drone_infor message
 *
 * @return [degE7] Latitude, expressed
 */
static inline int32_t mavlink_msg_drone_infor_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon from drone_infor message
 *
 * @return [degE7] Longitude, expressed
 */
static inline int32_t mavlink_msg_drone_infor_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from drone_infor message
 *
 * @return [mm] Altitude (AMSL). 
 */
static inline int32_t mavlink_msg_drone_infor_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field vx from drone_infor message
 *
 * @return [cm/s] Ground X Speed (Latitude, positive north)
 */
static inline int16_t mavlink_msg_drone_infor_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field vy from drone_infor message
 *
 * @return [cm/s] Ground Y Speed (Longitude, positive east)
 */
static inline int16_t mavlink_msg_drone_infor_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field vz from drone_infor message
 *
 * @return [cm/s] Ground Z Speed (Altitude, positive down)
 */
static inline int16_t mavlink_msg_drone_infor_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field hdg from drone_infor message
 *
 * @return [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. 
 */
static inline uint16_t mavlink_msg_drone_infor_get_hdg(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Decode a drone_infor message into a struct
 *
 * @param msg The message to decode
 * @param drone_infor C-struct to decode the message contents into
 */
static inline void mavlink_msg_drone_infor_decode(const mavlink_message_t* msg, mavlink_drone_infor_t* drone_infor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    drone_infor->time_boot_ms = mavlink_msg_drone_infor_get_time_boot_ms(msg);
    drone_infor->lat = mavlink_msg_drone_infor_get_lat(msg);
    drone_infor->lon = mavlink_msg_drone_infor_get_lon(msg);
    drone_infor->alt = mavlink_msg_drone_infor_get_alt(msg);
    drone_infor->vx = mavlink_msg_drone_infor_get_vx(msg);
    drone_infor->vy = mavlink_msg_drone_infor_get_vy(msg);
    drone_infor->vz = mavlink_msg_drone_infor_get_vz(msg);
    drone_infor->hdg = mavlink_msg_drone_infor_get_hdg(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DRONE_INFOR_LEN? msg->len : MAVLINK_MSG_ID_DRONE_INFOR_LEN;
        memset(drone_infor, 0, MAVLINK_MSG_ID_DRONE_INFOR_LEN);
    memcpy(drone_infor, _MAV_PAYLOAD(msg), len);
#endif
}
