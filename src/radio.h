#ifndef RADIOH
#define RADIOH

#include <Arduino.h>
#include "../../lib/SBUS/src/SBUS.h"




// The size of this struct should not exceed 32 bytes
struct RcData {
  byte axis1; // Aileron (Steering for car)
  byte axis2; // Elevator
  byte axis3; // Throttle
  byte axis4; // Rudder
  bool mode1 = false; // Mode1 (toggle speed limitation)
  bool mode2 = false; // Mode2 (toggle acc. / dec. limitation)
  bool momentary1 = false; // Momentary push button
  byte pot1; // Potentiometer
};


// This struct defines data, which are embedded inside the ACK payload
struct ackPayload {
  float vcc; // vehicle vcc voltage
  float batteryVoltage; // vehicle battery voltage
  bool batteryOk = true; // the vehicle battery voltage is OK!
  byte channel = 1; // the channel number
};



// Function prototypes
ackPayload* getPayloadptr( void );
RcData* getDataptr(void);
void readRadio( void );
void setupRadio( void );

void sendSerialCommands( void );
SBUS* getSBUSptr( void );
void sendSbusCommands( void );


#endif // RADIOH