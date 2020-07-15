
// Demo: NMEA2000 library. Actisense NGT-1 compatible gateway.
//   Sends all bus data to serial in Actisense format.
//   Send all data received from serial in Actisense format to the N2kBus, setting
//   data source to the gateway itself.
#define USE_N2K_CAN 1

#define N2k_SPI_CS_PIN 7 //53  // Pin for SPI Can Select
#define N2k_CAN_INT_PIN 27 // maple: pin 27
#define USE_MCP_CAN_CLOCK_SET 8  // possible values 8 for 8Mhz and 16 for 16 Mhz clock

#include <Arduino.h>
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <NMEA2000_CAN.h>

#include "RaymarinePilot.h"
#include "N2kDeviceList.h"

const unsigned long TransmitMessages[] PROGMEM={126208UL,0};
const unsigned long ReceiveMessages[] PROGMEM={127250L,65288L,65379L,0};

tN2kDeviceList *pN2kDeviceList;

byte inByte;
short pilotSourceAddress = -1;


void setup() {
  delay(1000);

  // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
  NMEA2000.SetN2kCANReceiveFrameBufSize(150);
  NMEA2000.SetN2kCANMsgBufSize(8);
  // Set Product information
  NMEA2000.SetProductInformation("00000001", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Evo Pilot Remote",  // Manufacturer's Model ID
                                 "1.0.0.0",  // Manufacturer's Software version code
                                 "1.0.0.0" // Manufacturer's Model version
                                );
  // Set device information
  NMEA2000.SetDeviceInformation(1111, // Unique number. Use e.g. Serial number.
                                132, // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25, // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );
  // Uncomment 3 rows below to see, what device will send to bus
  Serial.begin(115200);
  delay(100);
  
  //NMEA2000.SetForwardStream(&Serial);  // PC output on due programming port
  //NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
  //NMEA2000.SetForwardOwnMessages();

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 51); //N2km_NodeOnly N2km_ListenAndNode
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.ExtendReceiveMessages(ReceiveMessages);

  NMEA2000.SetMsgHandler(RaymarinePilot::HandleNMEA2000Msg);

  
  pN2kDeviceList = new tN2kDeviceList(&NMEA2000);
  //NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  NMEA2000.EnableForward(true); // Disable all msg forwarding to USB (=Serial)
  NMEA2000.Open();

  while(pilotSourceAddress <= 0){
    NMEA2000.ParseMessages(); 
    pilotSourceAddress = getDeviceSourceAddress("EV-1");
  }
  
  Serial.println((String) "Found EV-1 Pilot: " + pilotSourceAddress);
}


void loop() {
  if (Serial.available() > 0) {
    inByte = Serial.read();

    if (inByte == 's') {
      Serial.println("Setting PILOT_MODE_STANDBY");
      
      tN2kMsg N2kMsg;
      RaymarinePilot::SetEvoPilotMode(N2kMsg, pilotSourceAddress, PILOT_MODE_STANDBY);
      NMEA2000.SendMsg(N2kMsg);
    } 
    
    else if (inByte == 'a') {
      Serial.println("Setting PILOT_MODE_AUTO");
      
      tN2kMsg N2kMsg;
      RaymarinePilot::SetEvoPilotMode(N2kMsg, pilotSourceAddress, PILOT_MODE_AUTO);
      NMEA2000.SendMsg(N2kMsg);
    } 
    
    else if (inByte == 'w') {
      Serial.println("Setting PILOT_MODE_WIND");
      
      tN2kMsg N2kMsg;
      RaymarinePilot::SetEvoPilotMode(N2kMsg, pilotSourceAddress, PILOT_MODE_WIND);
      NMEA2000.SendMsg(N2kMsg);
    } 
    
    else if (inByte == 't') {
      Serial.println("Setting PILOT_MODE_TRACK");
      
      tN2kMsg N2kMsg;
      RaymarinePilot::SetEvoPilotMode(N2kMsg, pilotSourceAddress, PILOT_MODE_TRACK);
      NMEA2000.SendMsg(N2kMsg);
    }
    
    else if (inByte == '+') {
      Serial.print("Setting course to ");
      Serial.println((((int) RaymarinePilot::Heading + 10) % 360));
      
      tN2kMsg N2kMsg;
      RaymarinePilot::SetEvoPilotCourse(N2kMsg, pilotSourceAddress, RaymarinePilot::Heading, 10);
      NMEA2000.SendMsg(N2kMsg);
    }
    
    else if (inByte == '-') {
      Serial.print("Setting course to ");
      Serial.println((((int) RaymarinePilot::Heading - 10 + 360) % 360));
      
      tN2kMsg N2kMsg;
      RaymarinePilot::SetEvoPilotCourse(N2kMsg, pilotSourceAddress, RaymarinePilot::Heading, -10);
      NMEA2000.SendMsg(N2kMsg);
    }

    else if(inByte == 'c'){
        Serial.println("Confirm Waypoint");
      
        tN2kMsg N2kMsg1;
        RaymarinePilot::TurnToWaypointMode(N2kMsg1, pilotSourceAddress);
        NMEA2000.SendMsg(N2kMsg1);

        tN2kMsg N2kMsg2;
        RaymarinePilot::TurnToWaypoint(N2kMsg2, pilotSourceAddress);
        NMEA2000.SendMsg(N2kMsg2);
    }

    else if(inByte == '1'){
        Serial.println("Plus 1");
      
        tN2kMsg N2kMsg;
        RaymarinePilot::KeyCommand(N2kMsg, pilotSourceAddress, KEY_PLUS_1);
        NMEA2000.SendMsg(N2kMsg);
    }

    else if(inByte == '2'){
        Serial.println("Plus 10");
      
        tN2kMsg N2kMsg;
        RaymarinePilot::KeyCommand(N2kMsg, pilotSourceAddress, KEY_PLUS_10);
        NMEA2000.SendMsg(N2kMsg);
    }

    else if(inByte == '3'){
        Serial.println("Minus 1");
      
        tN2kMsg N2kMsg;
        RaymarinePilot::KeyCommand(N2kMsg, pilotSourceAddress, KEY_MINUS_1);
        NMEA2000.SendMsg(N2kMsg);
    }

    else if(inByte == '4'){
        Serial.println("Minus 10");
      
        tN2kMsg N2kMsg;
        RaymarinePilot::KeyCommand(N2kMsg, pilotSourceAddress, KEY_MINUS_10);
        NMEA2000.SendMsg(N2kMsg);
    }

    else if(inByte == '5'){
        Serial.println("Minus 1 Minus 10");
      
        tN2kMsg N2kMsg;
        RaymarinePilot::KeyCommand(N2kMsg, pilotSourceAddress, KEY_MINUS_1_MINUS_10);
        NMEA2000.SendMsg(N2kMsg);
    }

    else if(inByte == '6'){
        Serial.println("Plus 1 Plus 10");
      
        tN2kMsg N2kMsg;
        RaymarinePilot::KeyCommand(N2kMsg, pilotSourceAddress, KEY_PLUS_1_PLUS_10);
        NMEA2000.SendMsg(N2kMsg);
    }
  }

  NMEA2000.ParseMessages();  
}

int getDeviceSourceAddress(String model) {  
  if (!pN2kDeviceList->ReadResetIsListUpdated()) return -1;
  for (uint8_t i = 0; i < N2kMaxBusDevices; i++){ 
    const tNMEA2000::tDevice *device=pN2kDeviceList->FindDeviceBySource(i);
    if ( device == 0 ) continue;
    
    String modelVersion = device->GetModelVersion();
    
    if(modelVersion.indexOf(model) >= 0){
      return device->GetSource();
    }
  }

  return -2;
}
