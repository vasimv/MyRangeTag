// Range tag for OMower by vasimv (c) 2017
// $Id$
// Code for range calculation was taken from DW1000 arduino library, Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
// Code for triangulation by convict@education.lu

// Use LCD 16x2 to print debug info
//#define LCD_DEBUG

#ifdef LCD_DEBUG
#include <SPI.h>
#include <LiquidCrystal.h>
#endif
#include <SPI.h>
#include <DW1000.h>
#include <EEPROM.h>
#include <stdint.h>
#include <string.h>
#include "MyRangeTag.h"

// Print debug info to the serial console
// #define DEBUG

// Print extended debug to the console (note, distance ranging may not work properly because delay)!!!
// #define DEBUG_BROKE

#ifdef LCD_DEBUG
LiquidCrystal lcd(3, 4, 5, 6, 7, 8);
#endif

// DWM1000 pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin
const uint8_t PIN_LED = 14;

// Variables from EEPROM
uint16_t myId = 0x0010;
// Room number
uint16_t myRoom = 1;
// Tag type (T_STATIC, T_MOVING - last one will collect data at higher frequency)
uint8_t myType = T_MOVING;
// Coordinates of the tag and degree of north pole (-179..180)
int32_t myX = 0;
int32_t myY = 0;
int32_t myZ = 0;
int8_t myDegree = 0;

// someone sending range request, do not try to send our packets while
boolean someoneRanging = false;
// Time to force free someoneRanging status (in case some of packets were lost)
uint32_t tFreeStatus = 0;
// we're ranging currently
volatile boolean weRanging = false;
// Tag we send ranging request to
uint8_t rangeTag = 0;
// How much tags we have in the database
uint8_t numTags = 0;
// We're sending packet now
volatile boolean weSending = false;
// Received packet in the buffer
volatile boolean somethingReceived = false;
// Fail to receive packet
volatile boolean failReceive = false;
// Send finished flag
volatile boolean sendFinished = false;

// Array of tags data
struct tags_t tags[MAX_TAGS_AROUND];

struct pkt_t recvPkt;
struct pkt_t sendPkt;

// Find tag in tags database by id (returns NULL if not found)
struct tags_t *findTag(uint16_t id);
// Send packet
void sendPacket(byte *data, uint16_t len);
// Send broadcast with my own data
void sendBroadcast();
// Returns true if radio channel is busy
boolean radioBusy();
// Drop too old records from tags database
void dropOld();
// Send fail packet
void sendFail();
// Send poll packet
void sendPoll(uint16_t id);
// Send poll-ack packet
void sendPollAck(uint16_t id);
// Send range request
void sendRangeReq(uint16_t id);
// Send range report
void sendRangeReport(uint16_t id, uint16_t distance);

// Timestamps
DW1000Time timePollSent;
DW1000Time timePollReceived;
DW1000Time timePollAckSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
DW1000Time timeRangeReceived;
// Computed range time
DW1000Time timeComputedRange;

/*++++++++++++++++++begin receive routines+++++++++++++*/
#define aa .0710843   /*linear function to transform raw_1 to real half-byte*/
#define bb -6.9802983 
#define cycle  4.3904  /*=1.28E-4 * 343 * 100 result in cm*/

/*++++++++++++++begin positioning routines++++++++++++*/

#define POINTS  3
#define SOLUTIONS 2
#define TOLERANCE 2500 /*this is the square of the maximum distance of 
                          the two-circle intersection solution point to the 
                          third circle*/

inline double sqr(double x) {
  return x*x;
}

int calcPosition() {
  int32_t x[POINTS] = {2,10,2}; /*make these variables global<===============beacon coordinates==============*/
  int32_t y[POINTS] = {1,5,11}; /*beacon-positions in cm*/
  uint16_t d[POINTS] = {4,8,6};
  uint8_t i, j, k, m, n, err;
  int32_t diff, max;
  double B,u;
  double xr[SOLUTIONS];
  double yr[SOLUTIONS];
  double A,v,w,sq_d,delta,temp2,t1,t2;

  // Find suitable tags for triangulation
  j = 0;
  for (i = 0; i < numTags; i++) {
    if ((tags[i].room == myRoom) && (tags[i].distance != 0)
        && (tags[i].type == T_STATIC) && ((millis() - tags[i].lastSeen) < TAG_THRESHOLD)) {
      x[j] = tags[i].X;
      y[j] = tags[i].Y;
      // z[j] = tags[i].Z;
      d[j] = tags[i].distance;
      j++;
      if (j >= POINTS)
        break;
    }
  }
  // Didn't found enough tags
  if (j < POINTS)
    return 1;
  
  /*positioning initialization*/
  max=0;
  for( i=0;i<2;i++)   /*find the two beacons which maximize the y[i]-y[j]*/
  {
        for(j=i+1;j<3;j++)
        {
              diff=abs(y[i]-y[j]);
              if(diff>max)
              {
                    max=diff;
                    m=i;
                    n=j;
                    k=3-i-j;  
               }
         }
  }
     
  diff=y[m]-y[n];
  B=(x[m]-x[n])/diff;
  u=1+sqr(B);

  err=0;  /*no error yet*/
   
  /*now the calculations*/
  A=((sqr(d[n])-sqr(d[m])+sqr(x[m])-sqr(x[n]))/diff+y[m]+y[n])/2;
  
  v=2*(y[n]*B-x[n]-A*B);
  w=-sqr(d[n])+sqr(x[n])+sqr(y[n])+sqr(A)-2*y[n]*A;
  delta=-4*u*w+sqr(v);
  if(delta<0) {err=7; } else
  {
    sq_d=sqrt(delta);
    xr[0]=(sq_d-v)/u/2;
    xr[1]=-(v+sq_d)/u/2;
    yr[0]=A-B*xr[0];
    yr[1]=A-B*xr[1];
  
  
    t1=abs(sqr(x[k]-xr[0])+sqr(y[k]-yr[0])-sqr(d[k])); /*test the third circle*/
        t2=abs(sqr(x[k]-xr[1])+sqr(y[k]-yr[1])-sqr(d[k]));
    if (t2<t1) 
    {
      /*exchange the solution, index0 will point the true solution*/
                  temp2=xr[0];
      xr[0]=xr[1];
      xr[1]=temp2;
      temp2=yr[0];
      yr[0]=yr[1];
      yr[1]=temp2;
            temp2=t2;
      t2=t1;
      t1=temp2;
                                                
    }
    if (t1>TOLERANCE)  err=8;
                                                   
  }
  #ifdef DEBUG
  Serial.print(xr[0]);
  Serial.print(' ');
  Serial.println(yr[0]);
  Serial.print(xr[1]);
  Serial.print(' ');
  Serial.println(yr[1]);
  Serial.println(err);
  #endif
  // Update my coordinates
  if ((myType == T_MOVING) && (err == 0)) {
    myX = xr[0];
    myY = yr[0];
    // myZ = zr[0];
  }
  return err;
}

// Returns true if radio channel is busy
boolean radioBusy() {
  if (weSending)
    return true;
  DW1000.readSystemEventStatusRegister();
  return DW1000.isReceiving();
}

void sendPacketStart() {
  weSending = true;
  DW1000.newTransmit();
  DW1000.setDefaults();  
}

void sendPacketEnd(byte *data, uint16_t len) {
  DW1000.setData(data, len);
  DW1000.startTransmit();  
}

void sendPacket(byte *data, uint16_t len) {
  sendPacketStart();
  sendPacketEnd(data, len);
}

// Send fail packet
void sendFail() {
  // Create packet to send
  sendPkt.targId = 0xFFFF;
  sendPkt.myId = myId;
  sendPkt.payload.myRoom = myRoom;
  sendPkt.pktType = RANGE_FAILED;
  sendPacket((byte *) &sendPkt, sizeof(sendPkt));
}

// Send poll packet
void sendPoll(uint16_t id) {
  // Create packet to send
  sendPkt.targId = id;
  sendPkt.myId = myId;
  sendPkt.pktType = POLL;
  sendPacket((byte *) &sendPkt, sizeof(sendPkt));
}

// Send poll-ack packet
void sendPollAck(uint16_t id) {
  // Create packet and send with a delay
  sendPkt.targId = id;
  sendPkt.myId = myId;
  sendPkt.pktType = POLL_ACK;
  sendPacketStart();
  DW1000Time deltaTime = DW1000Time(DELAY_REPLY_TIME, DW1000Time::MICROSECONDS);
  DW1000.setDelay(deltaTime);
  sendPacketEnd((byte *) &sendPkt, sizeof(sendPkt));
}

void sendRangeReq(uint16_t id) {
  // Create packet and send with a delay
  sendPkt.targId = id;
  sendPkt.myId = myId;
  sendPkt.pktType = RANGE;
  sendPacketStart();
  DW1000Time deltaTime = DW1000Time(DELAY_REPLY_TIME, DW1000Time::MICROSECONDS);
  timeRangeSent = DW1000.setDelay(deltaTime);
  // DW1000.getSystemTimestamp(timeRangeSent);
  timePollSent.getTimestamp(sendPkt.payload.stamp1);
  timePollAckReceived.getTimestamp(sendPkt.payload.stamp2);
  timeRangeSent.getTimestamp(sendPkt.payload.stamp3);
  sendPacketEnd((byte *) &sendPkt, sizeof(sendPkt));
}

void sendRangeReport(uint16_t id, uint16_t distance) {
  // Create packet with range report
  sendPkt.targId = id;
  sendPkt.myId = myId;
  sendPkt.pktType = RANGE_REPORT;
  sendPkt.payload.distance = distance;
  sendPacket((byte *) &sendPkt, sizeof(sendPkt));
}

void computeRangeAsymmetric() {
  // asymmetric two-way ranging (more computation intense, less error prone)
  DW1000Time round1 = (timePollAckReceived - timePollSent).wrap();
  DW1000Time reply1 = (timePollAckSent - timePollReceived).wrap();
  DW1000Time round2 = (timeRangeReceived - timePollAckSent).wrap();
  DW1000Time reply2 = (timeRangeSent - timePollAckReceived).wrap();
  #ifdef DEBUG_BROKE
  Serial.print(F("round1 "));
  Serial.print(round1.getAsMicroSeconds());
  Serial.print(F(" reply1 "));
  Serial.print(reply1.getAsMicroSeconds());
  Serial.print(F(" round2 "));
  Serial.print(round2.getAsMicroSeconds());
  Serial.print(F(" reply2 "));
  Serial.println(reply2.getAsMicroSeconds());
  #endif

  DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
  // set tof timestamp
  timeComputedRange.setTimestamp(tof);
}

// Find tag in tags database by id (returns NULL if not found)
struct tags_t *findTag(uint16_t id) {
  for (uint8_t i = 0; i < numTags; i++)
    if (tags[i].tagId == id)
      return &tags[i];
  return NULL;
}


// Drop too old records from tags database
void dropOld() {
  for (uint8_t i = 0; i < numTags; i++) {
    if ((tags[i].lastSeen + DROP_TIMEOUT) < millis()) {
      // Drop old tag
      #ifdef DEBUG
      Serial.print(F("Dropping tag #"));
      Serial.print(i);
      Serial.print(F(" ("));
      Serial.print(tags[i].lastSeen + DROP_TIMEOUT);
      Serial.print(F(" < "));
      Serial.print(millis());
      Serial.println(F(")"));
      #endif
      if (i != (numTags - 1))
        memmove((void *) &tags[i], (void *) &tags[i + 1], sizeof(struct tags_t) * (MAX_TAGS_AROUND - 1));
      numTags--;
      break;
    }
  }
}

// send broadcast with my data
void sendBroadcast() {
  // Create packet to send
  sendPkt.targId = 0xFFFF;
  sendPkt.pktType = BROADCAST;
  sendPkt.myId = myId;
  sendPkt.payload.myRoom = myRoom;
  sendPkt.payload.myX = myX;
  sendPkt.payload.myY = myY;
  sendPkt.payload.myZ = myZ;
  sendPkt.payload.myDegree = myDegree;
  sendPkt.payload.myType = myType;
  for (uint8_t i = 0; i < MAX_TAGS_AROUND; i++) {
    sendPkt.tags[i].tagId = tags[i].tagId;
    sendPkt.tags[i].distance = tags[i].distance;
  }
  sendPkt.crc = 0;
  sendPacket((byte *) &sendPkt, sizeof(sendPkt));
}

// Send BLINK packet
void sendBlink(uint16_t id) {
  // Create packet to send
  sendPkt.targId = id;
  sendPkt.pktType = BLINK;
  sendPkt.myId = myId;
  sendPkt.payload.myType = myType;
  sendPkt.payload.myRoom = myRoom;
  sendPkt.payload.myX = myX;
  sendPkt.payload.myY = myY;
  sendPkt.payload.myZ = myZ;
  sendPkt.payload.myDegree = myDegree;
  sendPkt.payload.myType = myType;
  sendPkt.crc = 0;
  sendPacket((byte *) &sendPkt, sizeof(sendPkt));  
}

// Send SET_ROOM packet
void sendSetRoom(uint16_t id, uint16_t room) {
  // Create packet to send
  sendPkt.targId = id;
  sendPkt.pktType = SET_ROOM;
  sendPkt.myId = myId;
  sendPkt.payload.myRoom = room;
  sendPkt.crc = 0;
  sendPacket((byte *) &sendPkt, sizeof(sendPkt));  
}

// Send SET_PARAM packet
void sendParam(uint16_t id, uint16_t newId, uint16_t room, int8_t type, int32_t X, int32_t Y, int32_t Z, int8_t degree) {
  // Create packet to send
  sendPkt.targId = id;
  sendPkt.pktType = SET_PARAM;
  sendPkt.myId = newId;
  sendPkt.payload.myType = type;
  sendPkt.payload.myRoom = room;
  sendPkt.payload.myX = X;
  sendPkt.payload.myY = Y;
  sendPkt.payload.myZ = Z;
  sendPkt.payload.myDegree = degree;
  sendPkt.crc = 0;
  sendPacket((byte *) &sendPkt, sizeof(sendPkt));    
}

// Interrupt - Packet send complete, remember timestamps
void intSent() {
  weSending = false;
  sendFinished = true;
  switch (sendPkt.pktType) {
    case POLL:
      DW1000.getTransmitTimestamp(timePollSent);
      break;
    case RANGE:
      DW1000.getTransmitTimestamp(timeRangeSent);
      break;
    case POLL_ACK:
      DW1000.getTransmitTimestamp(timePollAckSent);
      break;
    default:
      break;
  }
}

// Interrupt - Packet receive completed
void intReceived() {
  weSending = false;
  somethingReceived = true;
}

// Interrupt - Packet receive failed
void intFailed() {
  weSending = false;
  failReceive = true;
}

// Calculate CRC of EEPROM
uint32_t crcEEPROM(uint16_t addr, uint16_t len) {
  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  uint32_t crc = ~0L;

  for (int index = addr ; index < (addr + len)  ; ++index) {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}

// Read variables from EEPROM (returns true if success)
boolean loadEEPROM() {
  uint32_t crc;

  // Check for signature
  if (EEPROM.read(0) != 0xA6) {
    #ifdef DEBUG
    Serial.println(F("No signature in EEPROM"));
    #endif
    return false;
  }
  // Read CRC from EEPROM
  EEPROM.get(1, crc);
  // Check CRC
  if (crc != crcEEPROM(5, 23))
    return false;
  // Now we sure that data is correct, read stuff
  EEPROM.get(5, myId);
  EEPROM.get(7, myRoom);
  EEPROM.get(9, myType);
  EEPROM.get(10, myX);
  EEPROM.get(14, myY);
  EEPROM.get(18, myZ);
  EEPROM.get(22, myDegree);
  return true;
}

// Write variables in EEPROM
void saveEEPROM() {
  EEPROM.write(0, 0xA6);
  
  EEPROM.put(5, myId);
  EEPROM.put(7, myRoom);
  EEPROM.put(9, myType);
  EEPROM.put(10, myX);
  EEPROM.put(14, myY);
  EEPROM.put(18, myZ);
  EEPROM.put(22, myDegree);
  EEPROM.put(1, crcEEPROM(5, 23));
}

uint32_t lastReceive = 0;

// Most useful mode - 850 kbit/s, 16 MHz, 512 preamble (gives 70+ meters at 6200 MHz with DWM1000's antenna)
static constexpr byte MODE_850_SHORT[] = {DW1000Class::TRX_RATE_850KBPS, DW1000Class::TX_PULSE_FREQ_16MHZ, DW1000Class::TX_PREAMBLE_LEN_512};

void resetDW1000() {
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  #ifdef DEBUG
  Serial.println(F("Resetting DW1000"));
  #endif
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.enableMode(MODE_850_SHORT);
  DW1000.setNetworkId(11);
  DW1000.setDeviceAddress(myId);
  DW1000.commitConfiguration();
  DW1000.attachSentHandler(intSent);
  DW1000.attachReceivedHandler(intReceived);
  DW1000.attachReceiveFailedHandler(intFailed);
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

void setup() {
  Serial.begin(115200);
  memset((void *) tags, 0, sizeof(tags));
  
  EEPROM.begin();
  if (!loadEEPROM())
    Serial.println(F("Can't load config from EEPROM, using defaults"));

  saveEEPROM();
  EEPROM.end();

  #ifdef LCD_DEBUG
  lcd.begin(16,2);
  lcd.print(myId);
  lcd.setCursor(0, 1);
  #endif

  #ifdef DEBUG
  Serial.print(F("Packet size "));
  Serial.println(sizeof(sendPkt));
  Serial.print(F("myId "));
  Serial.println(myId);
  Serial.print(F("myType "));
  Serial.print(myType);
  Serial.print(F(" myX "));
  Serial.print(myX);
  Serial.print(F(", myY "));
  Serial.print(myY);
  Serial.print(F(", myZ "));
  Serial.print(myZ);
  Serial.print(F(", myDegree "));
  Serial.println(myDegree);
  delay(500);
  #endif
  resetDW1000();
  lastReceive = millis();
}

// Modbus frame buffer (to send coordinates)
byte bufModbusOut[32];
uint16_t cntOut = 0;

// Incoming modbus buffer
byte bufModbusIn[32];
uint16_t cntIn = 0;
uint32_t lastModbus;

// Calculate CRC for modbus frame                                                                             
uint16_t modbusCrc(byte *buf, int len) {                                                                   
  uint32_t tmp, tmp2;                                                                                         
  uint8_t Flag;                                                                                               
  uint16_t i, j;                                                                                            
                                                                                                              
  tmp = 0xFFFF;                                                                                             
  for (i = 0; i < len; i++) {                                                                               
    tmp = tmp ^ buf[i];                                                                                   
    for (j = 1; j <= 8; j++) {                                                                            
      Flag = tmp & 0x0001;                                                                              
      tmp >>=1;                                                                                         
      if (Flag)                                                                                         
        tmp ^= 0xA001;                                                                                
    }                                                                                                     
  }                                                                                                         
  tmp2 = tmp >> 8;                                                                                          
  tmp = (tmp << 8) | tmp2;                                                                                  
  tmp &= 0xFFFF;                                                                                            
  return (uint16_t) tmp;                                                                                    
} // uint16_t modbusCrc(byte *buf, int len)                                                                   
                                                                                                              
// Add CRC to output modbus packet                                                                            
void addModbusCrc() {                                                                                         
  uint16_t tCrc;                                                                                              
                                                                                                              
  tCrc = modbusCrc(bufModbusOut, cntOut);                                                                     
  bufModbusOut[cntOut] = (tCrc >> 8) & 0xff;                                                                  
  cntOut++;                                                                                                   
  bufModbusOut[cntOut] = tCrc & 0xff;                                                                         
  cntOut++;                                                                                                   
} // void addModbusCrc()

void putModbus16(uint16_t val) {
  bufModbusOut[cntOut] = (val >> 8) & 0xff;
  cntOut++;
  bufModbusOut[cntOut] = val & 0xff;
  cntOut++;
}

void putModbus32(uint32_t val) {
  bufModbusOut[cntOut] = (val >> 24) & 0xff;
  cntOut++;
  bufModbusOut[cntOut] = (val >> 16) & 0xff;
  cntOut++;
  bufModbusOut[cntOut] = (val >> 8) & 0xff;
  cntOut++;
  bufModbusOut[cntOut] = val & 0xff;
  cntOut++;
}

// Send coordinates by modbus (read holding registers frame)
void sendCoords() {
  cntOut = 0;
  bufModbusOut[cntOut] = MODBUS_SLAVE;
  cntOut++;
  bufModbusOut[cntOut] = 0x03;
  cntOut++;
  bufModbusOut[cntOut] = 20;
  cntOut++;
  putModbus16(myId);
  putModbus16(myRoom);
  putModbus16(myType);
  putModbus32((uint32_t) myX);
  putModbus32((uint32_t) myY);
  putModbus32((uint32_t) myZ);
  putModbus16((uint16_t) ((int16_t) myDegree));
  addModbusCrc();
  Serial.write(bufModbusOut, cntOut);
}

// Retrieve 16-bit value from modbus frame
uint16_t getModbus16(uint16_t addr) {
  return bufModbusIn[addr + 1] | (bufModbusIn[addr] << 8);
}

// Retrieve 32-bit value from modbus frame
uint32_t getModbus32(uint16_t addr) {
  return bufModbusIn[addr + 3] | (bufModbusIn[addr + 2] << 8) | (bufModbusIn[addr + 1] << 16) | (bufModbusIn[addr] << 24);
}

// Send modbus ACK/ERR
void sendModbusAck(uint8_t code, boolean ok) {
  cntOut = 0;
  bufModbusOut[cntOut] = MODBUS_SLAVE;
  cntOut++;
  bufModbusOut[cntOut] = code;
  cntOut++;
  if (!ok) {
    bufModbusOut[cntOut] = 1;
    cntOut++;
  }
  addModbusCrc();
  // MODBUS requires delay before sending frame
  delay(2);
  Serial.write(bufModbusOut, cntOut);
}

// Process incoming modbus frame
// Understands only write multiple holding (0x10), always 10 16-bit values
// First holding register address - tag Id
// if id in content is set to 0 - set only room
// if room is set to 0 - just send blink command
void processModbus() {
  // Wrong slave ID
  if (bufModbusIn[0] != MODBUS_SLAVE)
    return;
  // Wrong function code or incoming length
  if ((bufModbusIn[1] != 0x10) || (bufModbusIn[6] != 20)) {
    sendModbusAck(bufModbusIn[1] | 0x80, false);
    return;
  }
  // Check CRC
  if (getModbus16(27) != modbusCrc(bufModbusIn, 27)) {
    sendModbusAck(bufModbusIn[1] | 0x80, false);
    return;
  }
  sendModbusAck(bufModbusIn[1], true);
  
  uint16_t targId = getModbus16(2);
  uint16_t newId = getModbus16(7);
  uint16_t newRoom = getModbus16(9);
  if ((newId == 0) && (newRoom == 0)) {
    // send BLINK packet
    sendBlink(targId);
    return;
  }
  if (newId == 0) {
    // Send SET_ROOM packet
    sendSetRoom(targId, newRoom);
    return;
  }
  
  // Send SET_PARAM packet
  sendParam(targId, newId, newRoom, (int8_t) getModbus16(11), (int32_t) getModbus32(13),
            (int32_t) getModbus32(17), (int32_t) getModbus32(21), (int8_t) getModbus16(25));
}

uint32_t tNextBroadcast = 0;
uint32_t tNextRange = 0;
uint32_t tBlinkLED = 0;

#ifdef DEBUG
uint32_t tNextReport = 0;
#endif

// Switch to next tag for range request
void switchToNext() {
  rangeTag++;
  if (rangeTag >= numTags)
    rangeTag = 0;
  #ifdef DEBUG
  Serial.print(F("Next tag is #"));
  Serial.println(rangeTag);
  #endif
  if (myType == T_STATIC)
    tNextRange = millis() + DELAY_STATIC_RANGE + random(1, DELAY_RANDOM);
  else
    tNextRange = millis() + DELAY_MOVING_RANGE + random(1, DELAY_RANDOM);
}

void loop() {
  struct tags_t *tag;
  uint16_t distance;

  if (sendFinished) {
    sendFinished = false;
  }

  if (somethingReceived) {
    DW1000.getData((byte *) &recvPkt, sizeof(recvPkt));
    somethingReceived = false;
    lastReceive = millis();
    #ifdef DEBUG_BROKE
    Serial.print(F("New packet: "));
    Serial.print(recvPkt.myId);
    Serial.print(F(" type: "));
    Serial.print(recvPkt.pktType);
    Serial.print(F(" for: "));
    Serial.println(recvPkt.targId);
    #endif
    switch (recvPkt.pktType) {
      case BROADCAST:
        // Broadcast received, add tag to the database if in same room
        if (recvPkt.payload.myRoom == myRoom) {
          tag = findTag(recvPkt.myId);
          // If tag is found - just update, try to add in if not
          if (!tag && (numTags < MAX_TAGS_AROUND)) {
            tag = &tags[numTags];
            numTags++;
            tag->distance = 0;
          }
          // Update tag database
          if (tag) {
            tag->tagId = recvPkt.myId;
            tag->lastSeen = millis();
            tag->X = recvPkt.payload.myX;
            tag->Y = recvPkt.payload.myY;
            tag->Z = recvPkt.payload.myZ;
            tag->degree = recvPkt.payload.myDegree;
            tag->room = recvPkt.payload.myRoom;
            tag->type = recvPkt.payload.myType;
            #ifdef DEBUG
            Serial.print(F("Updated tag "));
            Serial.println(tag->tagId);
            #endif
          }
        }
        break;
      case POLL:
        // Poll received, ignore if not for us
        if (recvPkt.targId != myId) {
          someoneRanging = true;
          tFreeStatus = millis() + DELAY_MOVING_RANGE;
          break;
        }
        DW1000.getReceiveTimestamp(timePollReceived);
        weRanging = true;
        tFreeStatus = millis() + DELAY_MOVING_RANGE;
        sendPollAck(recvPkt.myId);
        #ifdef DEBUG_BROKE
        Serial.print(F("Sending poll-ack to "));
        Serial.println(recvPkt.myId);
        #endif
        break;
      case POLL_ACK:
        // Poll ACK received, ignore if not for us
        if (recvPkt.targId != myId) {
          someoneRanging = true;
          tFreeStatus = millis() + DELAY_MOVING_RANGE;
          break;
        }
        DW1000.getReceiveTimestamp(timePollAckReceived);
        weRanging = true;
        tFreeStatus = millis() + DELAY_MOVING_RANGE;
        sendRangeReq(recvPkt.myId);
        #ifdef DEBUG_BROKE
        Serial.print(F("Sending range to "));
        Serial.println(recvPkt.myId);
        #endif
        break;
      case RANGE:
        // Range request received, ignore if not for us
        if (recvPkt.targId != myId) {
          someoneRanging = true;
          tFreeStatus = millis() + DELAY_MOVING_RANGE;
          break;
        }
        DW1000.getReceiveTimestamp(timeRangeReceived);
        timePollSent.setTimestamp(recvPkt.payload.stamp1);
        timePollAckReceived.setTimestamp(recvPkt.payload.stamp2);
        timeRangeSent.setTimestamp(recvPkt.payload.stamp3);
        computeRangeAsymmetric();
        #ifdef DEBUG_BROKE
        Serial.print(F("Calculated "));
        Serial.println(timeComputedRange.getAsMeters());
        #endif
        distance = (uint16_t) (timeComputedRange.getAsMeters() * 100.0);
        sendRangeReport(recvPkt.myId, distance);
        #ifdef DEBUG
        Serial.print(F("Sending range report to "));
        Serial.print(recvPkt.myId);
        Serial.print(F(" distance: "));
        Serial.println(distance);
        #endif
        weRanging = false;
        break;
      case RANGE_REPORT:
        // Range report received, ignore if not for us
        someoneRanging = false;
        if (recvPkt.targId != myId)
          break;
        weRanging = false;
        #ifdef DEBUG
        Serial.print(F("Range report from "));
        Serial.print(recvPkt.myId);
        Serial.print(F(" distance "));
        Serial.println(recvPkt.payload.distance);
        #endif
        // Update distance to the tag
        tag = findTag(recvPkt.myId);
        if (tag) {
          // Check if reported range is not too big
          if ((tag->distance == 0) || (tag->distance > (recvPkt.payload.distance / 10))) {
            if (recvPkt.payload.distance < DISTANCE_CORRECTION)
              tag->distance = 1;
            else
              tag->distance = recvPkt.payload.distance - DISTANCE_CORRECTION;
          }
          tag->rsi = (uint8_t) abs(DW1000.getReceivePower());
          tag->lastSeen = millis();
        }
        // We've done with all range tags around, calculate our position
        if (rangeTag == 0) {
          if (calcPosition() == 0)
            sendCoords();
          #ifdef LCD_DEBUG
          lcd.setCursor(0,0);
          lcd.print(myX);
          lcd.print(' ');
          lcd.setCursor(9,0);
          lcd.print(myY);
          lcd.print(' ');
          #endif
        }
        break;
      case RANGE_FAILED:
        // Range protocol failed
        someoneRanging = false;
        weRanging = false;
        break;
      case BLINK:
        // Blink message received, ignore if not for us
        if (recvPkt.targId == myId)
          tBlinkLED = millis() + 10000;
        break;
      case SET_ROOM:
        // Set room number from payload and save in EEPROM
        if (recvPkt.targId != myId)
          break;
        myRoom = recvPkt.payload.myRoom;
        #ifdef DEBUG
        Serial.print(F("Saving room number into EEPROM: "));
        Serial.println(myRoom);
        #endif
        saveEEPROM();
        break;
      case SET_PARAM:
        // Set parameters and save in EEPROM
        if (recvPkt.targId != myId)
          break;
        myId = recvPkt.myId;
        myX = recvPkt.payload.myX;
        myY = recvPkt.payload.myY;
        myZ = recvPkt.payload.myZ;
        myDegree = recvPkt.payload.myDegree;
        myType = recvPkt.payload.myType;
        myRoom = recvPkt.payload.myRoom;
        #ifdef DEBUG
        Serial.print(F("Saving config into EEPROM: id "));
        Serial.println(myId);
        #endif
        saveEEPROM();
        break;
      default:
        break;
    }

    return;
  }

  // Failed receive, just reset ranging status
  if (failReceive) {
    failReceive = false;
    #ifdef DEBUG
    Serial.println(F("Receive failed (bad CRC or something)"));
    #endif
    // Send "failed" status if we were ranging, ignore otherwise
    if (weRanging) {
      weRanging = false;
      if (!radioBusy())
        sendFail();
      return;
    }
  }

  // Watchdog timeout
  if ((lastReceive + TIMEOUT_RECEIVE) < millis()) {
    lastReceive = millis();
    resetDW1000();
    return;
  }

  // Free someoneRanging status
  if ((someoneRanging || weRanging) && (tFreeStatus <= millis())) {
    someoneRanging = false;
    weRanging = false;
  }

  // Send broadcast if radio is free and no one is ranging
  if (tNextBroadcast <= millis()) {
    // Drop old tags from the database
    dropOld();
    
    if (someoneRanging || weRanging || radioBusy())
      tNextBroadcast = millis() + DELAY_PACKET + random(1, DELAY_RANDOM);
    else {
      // send broadcast
      #ifdef DEBUG
      Serial.println(F("Sending broadcast"));
      #endif
      
      sendBroadcast();
      tNextBroadcast = millis() + DELAY_BROADCAST + random(1, DELAY_RANDOM);      
    }
  }

  // Start ranging procedure if radio is free and no one else is ranging
  if ((numTags > 0) && (tNextRange <= millis()) && !weRanging) {
    if (someoneRanging || weRanging || radioBusy()) {
      if (weRanging)
        tNextRange = millis() + (myType == T_STATIC) ? DELAY_STATIC_RANGE : DELAY_MOVING_RANGE + random(1, DELAY_RANDOM);
      else
        tNextRange = millis() + DELAY_PACKET + random(1, DELAY_RANDOM);
    } else {
      // Start ranging procedure
      switchToNext();
      weRanging = true;
      #ifdef DEBUG
      Serial.print(F("Sending poll request to "));
      Serial.println(tags[rangeTag].tagId);
      #endif
      tFreeStatus = millis() + DELAY_MOVING_RANGE;
      sendPoll(tags[rangeTag].tagId);
    }
  }

  if (Serial.available()) {
    lastModbus = millis();
    while (Serial.available() && (cntIn < sizeof(bufModbusIn))) {
      bufModbusIn[cntIn] = Serial.read();
      cntIn++;
    }
    // We've got full frame, processing
    if (cntIn <= 29) {
      processModbus();
      cntIn = 0;
    }
  } else {
    // Discard modbus buffer if too long pause
    if ((cntIn > 0) && ((millis() - lastModbus) > 20))
      cntIn = 0;
  }

  #ifdef DEBUG
  if (tNextReport <= millis()) {
    tNextReport = millis() + 1000;
    Serial.print("\nDW1000 status: ");
    DW1000.readSystemEventStatusRegister();
    for (int i = 0; i < LEN_SYS_STATUS; i++) {
      Serial.print(' ');
      Serial.print(DW1000._sysstatus[i]);
    }
    Serial.println();
    Serial.print(F("weSending = "));
    Serial.print(weSending);
    Serial.print(F(" weRanging = "));
    Serial.print(weRanging);
    Serial.print(F(" someoneRanging = "));
    Serial.println(someoneRanging);

    for (int i = 0; i < numTags; i++) {
      Serial.print(F("Tag #"));
      Serial.print(i);
      Serial.print(F(": Id "));
      Serial.print(tags[i].tagId);
      Serial.print(F(", X "));
      Serial.print(tags[i].X);
      Serial.print(F(", Y "));
      Serial.print(tags[i].Y);
      Serial.print(F(", distance "));
      Serial.println(tags[i].distance);
    }
    Serial.println();
  }
  #endif

  // Blink led if we were asked for
  if (tBlinkLED > millis()) {
    if ((millis() / 100) % 2 == 0)
      digitalWrite(PIN_LED, HIGH);
    else
      digitalWrite(PIN_LED, LOW);
  } else
    digitalWrite(PIN_LED, LOW);
}
