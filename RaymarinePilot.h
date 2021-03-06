#include "N2kMsg.h"
#include "N2kTypes.h"
#include <stdint.h>
#include <NMEA2000.h>
#include <N2kMessages.h>

#ifndef RAYMARINEPILOT_H
#define RAYMARINEPILOT_H

enum RaymarinePilotModes {PILOT_MODE_STANDBY = 1, PILOT_MODE_AUTO = 2, PILOT_MODE_WIND = 3, PILOT_MODE_TRACK = 4};

enum key_codes {KEY_PLUS_1 = 0x07f8, KEY_PLUS_10 = 0x08f7, KEY_MINUS_1 = 0x05fa, KEY_MINUS_10 = 0x06f9, KEY_MINUS_1_MINUS_10 = 0x21de, KEY_PLUS_1_PLUS_10 = 0x22dd, KEY_TACK_PORTSIDE = KEY_MINUS_1_MINUS_10, KEY_TACK_STARBORD = KEY_PLUS_1_PLUS_10};

class RaymarinePilot {
  public: 
    static double Heading, Variation;

    static bool alarmWaypoint;

    static uint8_t PilotMode;
    static int PilotSourceAddress;
  
    static void SetEvoPilotMode(tN2kMsg &N2kMsg, RaymarinePilotModes mode);
    static void SetEvoPilotWind(tN2kMsg &N2kMsg, double targetWindDirection);
    static void SetEvoPilotCourse(tN2kMsg &N2kMsg, double heading, int change);
    static inline void SetEvoPilotCourse(tN2kMsg &N2kMsg, double heading) {
      return SetEvoPilotCourse(N2kMsg, heading, 0);
    }
    static void TurnToWaypointMode(tN2kMsg &N2kMsg);
    static void TurnToWaypoint(tN2kMsg &N2kMsg);
    static void KeyCommand(tN2kMsg &N2kMsg, uint16_t command);
    
    static void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

    static bool ParseN2kPGN65288(const tN2kMsg &N2kMsg, unsigned char &AlarmState,  unsigned char &AlarmCode, unsigned char &AlarmGroup);
    static inline bool ParseN2kAlarm(const tN2kMsg &N2kMsg, unsigned char &AlarmState, unsigned char &AlarmCode, unsigned char &AlarmGroup) {
      return ParseN2kPGN65288(N2kMsg, AlarmState, AlarmCode, AlarmGroup);
    }

    static bool ParseN2kPGN65379(const tN2kMsg &N2kMsg, unsigned char &Mode, unsigned char &Submode);
    static inline bool ParseN2kPilotState(const tN2kMsg &N2kMsg, unsigned char &Mode, unsigned char &Submode) {
      return ParseN2kPGN65379(N2kMsg, Mode, Submode);
    }

    static bool ParseN2kPGN65345(const tN2kMsg &N2kMsg, double &WindAngle, double &RollingAverageWindAngle);
    static inline bool ParseN2kPilotWindAngle(const tN2kMsg &N2kMsg, double &WindAngle, double &RollingAverageWindAngle) {
      return ParseN2kPGN65345(N2kMsg, WindAngle, RollingAverageWindAngle);
    }

    static bool ParseN2kPGN65360(const tN2kMsg &N2kMsg, double &HeadingTrue, double &HeadingMagnetic);
    static inline bool ParseN2kPilotLockedHeading(const tN2kMsg &N2kMsg, double &HeadingTrue, double &HeadingMagnetic) {
      return ParseN2kPGN65360(N2kMsg, HeadingTrue, HeadingMagnetic);
    }   
    
};

#endif
