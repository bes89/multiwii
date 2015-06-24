#ifndef HOTTV4_H_
#define HOTTV4_H_

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"

#if defined(HOTTV4_TELEMETRY)

typedef enum {
  HoTTv4NotificationErrorCalibration     = 0x01,
  HoTTv4NotificationErrorReceiver        = 0x02,
  HoTTv4NotificationErrorDataBus         = 0x03,
  HoTTv4NotificationErrorNavigation      = 0x04,
  HoTTv4NotificationErrorError           = 0x05,
  HoTTv4NotificationErrorCompass         = 0x06,
  HoTTv4NotificationErrorSensor          = 0x07,
  HoTTv4NotificationErrorGPS             = 0x08,
  HoTTv4NotificationErrorMotor           = 0x09,
  
  HoTTv4NotificationMaxTemperature       = 0x0A,
  HoTTv4NotificationAltitudeReached      = 0x0B,
  HoTTv4NotificationWaypointReached      = 0x0C,
  HoTTv4NotificationNextWaypoint         = 0x0D,
  HoTTv4NotificationLanding              = 0x0E,
  HoTTv4NotificationGPSFix               = 0x0F,
  HoTTv4NotificationUndervoltage         = 0x10,
  HoTTv4NotificationGPSHold              = 0x11,
  HoTTv4NotificationGPSHome              = 0x12,
  HoTTv4NotificationGPSOff               = 0x13,
  HoTTv4NotificationBeep                 = 0x14,
  HoTTv4NotificationMicrocopter          = 0x15,
  HoTTv4NotificationCapacity             = 0x16,
  HoTTv4NotificationCareFreeOff          = 0x17,
  HoTTv4NotificationCalibrating          = 0x18,
  HoTTv4NotificationMaxRange             = 0x19,
  HoTTv4NotificationMaxAltitude          = 0x1A,
  
  HoTTv4Notification20Meter              = 0x25,
  HoTTv4NotificationMicrocopterOff       = 0x26,
  HoTTv4NotificationAltitudeOn           = 0x27,
  HoTTv4NotificationAltitudeOff          = 0x28,
  HoTTv4Notification100Meter             = 0x29,
  HoTTv4NotificationCareFreeOn           = 0x2E,
  HoTTv4NotificationDown                 = 0x2F,
  HoTTv4NotificationUp                   = 0x30,
  HoTTv4NotificationHold                 = 0x31,
  HoTTv4NotificationGPSOn                = 0x32,
  HoTTv4NotificationFollowing            = 0x33,
  HoTTv4NotificationStarting             = 0x34,
  HoTTv4NotificationReceiver             = 0x35,
} HoTTv4Notification;
static int32_t referenceAltitude = 0;
static int32_t referenceHeading = 0;
static uint8_t minutes = 0;
static uint16_t milliseconds = 0;
static uint8_t hottV4SerialAvailable();
static void hottV4EnableReceiverMode();
static void hottV4EnableTransmitterMode();
static void hottV4SerialWrite(uint8_t data);
static uint8_t hottV4SerialRead();
static void hottV4LoopUntilRegistersReady();
static void hottV4SendBinary(uint8_t *data);
static void hottV4UpdateDirection(uint8_t *data);
static void hottV4TriggerNotification(uint8_t *data, uint8_t notification);
static void hottv4UpdateBattery(uint8_t *data);
static void hottv4UpdateAlt(uint8_t *data, uint8_t lowByteIndex);
static void hottv4UpdateFlightTime(uint8_t *data);
void hottv4Init();
void hottv4Setup();
static void hottV4SendEAMTelemetry();
static void updatePosition(uint8_t *data, uint32_t value, uint8_t index);
static void hottV4SendGPSTelemetry();
static void hottV4SendVarioTelemetry();
#define HOTTV4_TEXT_PAGE_PID 0
#define HOTTV4_TEXT_PAGE_DEBUG 1

// Defines which controller values can be changed for ROLL, PITCH, YAW, etc.
typedef enum {
  HoTTv4ControllerValueP = 1 << 0,
  HoTTv4ControllerValuePID = 1 << 1,
} HoTTv4ControllerValue;

// Defines structure of Preamble text, the corresponding PID index, which PID values can be changed.
typedef struct {
  const char *label;
  uint8_t pidIndex;
  HoTTv4ControllerValue controllerValue;
} HoTTv4TextModeData;
static HoTTv4TextModeData settings[] = { 
                                         {"ROLL :", 0, HoTTv4ControllerValuePID}, 
                                         {"PITCH:", 1, HoTTv4ControllerValuePID},
                                         {"YAW  :", 2, HoTTv4ControllerValuePID},
                                         {"ALT  :", PIDALT, HoTTv4ControllerValuePID},
                                         {"POS  :", PIDPOS, HoTTv4ControllerValuePID},
                                         {"LEVEL:", PIDLEVEL, HoTTv4ControllerValuePID},
                                         {"MAG  :", PIDMAG, HoTTv4ControllerValueP},
                                       };
static uint8_t hottV4Constrain(uint8_t val, uint8_t maxVal);
static uint8_t hottV4SendChar(char c, uint8_t inverted);
static uint16_t hottV4SendWord(const char *w, uint8_t inverted);
static uint16_t hottV4SendTextline(const char *line);
static uint16_t hottV4SendFormattedPValue(int8_t p, int8_t inverted);
static uint16_t hottV4SendFormattedIValue(int8_t i, int8_t inverted);
static uint16_t hottV4SendFormattedDValue(int8_t d, int8_t inverted);
static uint16_t hottV4SendFormattedPIDTextline(void* data, int8_t selectedCol);
static uint16_t hottV4SendPIDSettings(int8_t selectedRow, int8_t selectedCol);
static uint16_t hottV4SendDebugInfos(int8_t selectedRow, int8_t selectedCol);
static void hottV4SendTextFrame(int8_t row, int8_t col, uint16_t (*payloadFunc) (int8_t, int8_t));
static void hottV4Esc();
static void hottV4UpdatePIDValueBy(int8_t row, int8_t col, int8_t val);
static uint8_t isInEditingMode(uint8_t page, uint8_t col);
static uint8_t canSelectLine(uint8_t page, uint8_t row);
static uint8_t nextCol(uint8_t page, uint8_t currentCol, uint8_t controllerValue);
static void hottV4HandleTextMode();
uint8_t canSendTelemetry();
uint8_t hottV4Hook(uint8_t serialData);

#endif

#endif /* HOTTV4_H_ */