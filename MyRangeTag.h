// Range DW1000 tag stuff for OMower
// $Id$

#ifndef _MY_RANGE_TAG_H
#define _MY_RANGE_TAG_H

// Maximum tag devices
#define MAX_TAGS_AROUND 10

// Delay in milliseconds to wait until packet is sending by someone
#define DELAY_PACKET 20
// Delay in milliseconds between broadcasts
#define DELAY_BROADCAST 1000
// Delay in milliseconds between requesting ranges for static tags
#define DELAY_STATIC_RANGE 500
// Delay in milliseconds between requesting ranges for moving tags
#define DELAY_MOVING_RANGE 30
// Random delay range added to DELAY_PACKET
#define DELAY_RANDOM 10

// How much milliseconds to wait before drop tag from database
#define DROP_TIMEOUT 20000

// How much time to wait for anything received before we reset DW1000
#define TIMEOUT_RECEIVE 2000

// reply delay time (must be same for both sides (in microseconds)
#define DELAY_REPLY_TIME 4000

// Maximum time (in milliseconds) to use tag distance for trianguation
#define TAG_THRESHOLD 400

// Distance correction (in centimeters)
#define DISTANCE_CORRECTION 21

// Modbus slave ID
#define MODBUS_SLAVE 32

// Tags data (note, RSI field stored without negative sign - less is better!)
struct tags_t {
  uint16_t tagId;
  // distance to the tag in cm
  uint16_t room;
  int8_t type;
  uint16_t distance;
  uint32_t lastSeen;
  int32_t X, Y, Z;
  uint8_t rsi;
  int8_t degree;
};

// Tags data to send
struct tagsSend_t {
  uint16_t tagId;
  uint16_t distance;
};

// Data packet
struct pkt_t {
  uint16_t targId;
  uint8_t pktType;
  uint16_t myId;
  union {
    struct {
      int8_t myType;
      uint16_t myRoom;
      int32_t myX;
      int32_t myY;
      int32_t myZ;
      int8_t myDegree;
    };
    struct {
      byte stamp1[5];
      byte stamp2[5];
      byte stamp3[5];
    };
    uint16_t distance;
  } payload;
  struct tagsSend_t tags[MAX_TAGS_AROUND];
  uint16_t crc;
};

// Types of tags
#define T_STATIC 0
#define T_MOVING 1

// Packet types (numbers are same as in DW1000Ranging)
#define POLL 0                                                                                                
#define POLL_ACK 1                                                                                            
#define RANGE 2                                                                                               
#define RANGE_REPORT 3                                                                                        
#define RANGE_FAILED 255                                                                                      
#define BLINK 4
// special packet, setting only room
#define SET_ROOM 252
#define BROADCAST 253
// special packet, containing id and coordinates of the tag
#define SET_PARAM 254

#endif
