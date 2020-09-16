#include <arduino.h>
#include "radio.h"
#include "lights.h"
#include "vehicleConfig.h"
#include "rc_board.h"

#include "../../lib/SBUS/src/SBUS.h"

// Hardware configuration: Set up nRF24L01 radio on hardware SPI bus & pins 8 (CE) & 7 (CSN)
#ifdef CONFIG_HAS_NRF24
  RF24 radio(8, 7);
#endif

// SBUS object
#ifdef CONFIG_HAS_SBUS
  #ifndef DEBUG
    SBUS x8r(Serial);

    SBUS* getSBUSptr( void ){return &x8r;}
  #endif
#endif

RcData data;

RcData* getDataptr(void)
{
  return &data;
}
// Radio channels (126 channels are supported)
byte chPointer = 0; // Channel 1 (the first entry of the array) is active by default
const byte NRFchannel[] {
  1, 2
};

// the ID number of the used "radio pipe" must match with the selected ID on the transmitter!
// 10 ID's are available @ the moment
const uint64_t pipeIn[] = {
  0xE9E8F0F0B1LL, 0xE9E8F0F0B2LL, 0xE9E8F0F0B3LL, 0xE9E8F0F0B4LL, 0xE9E8F0F0B5LL,
  0xE9E8F0F0B6LL, 0xE9E8F0F0B7LL, 0xE9E8F0F0B8LL, 0xE9E8F0F0B9LL, 0xE9E8F0F0B0LL
};
const int maxVehicleNumber = (sizeof(pipeIn) / (sizeof(uint64_t)));

ackPayload payload;

ackPayload* getPayloadptr( void )
{
  return &payload;
}

//
// =======================================================================================================
// RADIO SETUP
// =======================================================================================================
//

void setupRadio() {
#ifdef CONFIG_HAS_NRF24

  radio.begin();
  radio.setChannel(NRFchannel[chPointer]);

  // Set Power Amplifier (PA) level to one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  radio.setPALevel(RF24_PA_HIGH); // HIGH

  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(pipeIn[vehicleNumber - 1], true); // Ensure autoACK is enabled
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(5, 5);                  // 5x250us delay (blocking!!), max. 5 retries
  //radio.setCRCLength(RF24_CRC_8);          // Use 8-bit CRC for performance

#ifdef DEBUG
  radio.printDetails();
  delay(3000);
#endif

  radio.openReadingPipe(1, pipeIn[vehicleNumber - 1]);
  radio.startListening();
#endif
}

//
// =======================================================================================================
// READ RADIO DATA
// =======================================================================================================
//

void readRadio() {
#ifdef CONFIG_HAS_NRF24

  static unsigned long lastRecvTime = 0;
  byte pipeNo;

  if (radio.available(&pipeNo)) {
    radio.writeAckPayload(pipeNo, &payload, sizeof(struct ackPayload) );  // prepare the ACK payload
    radio.read(&data, sizeof(struct RcData)); // read the radia data and send out the ACK payload
    hazard = false;
    lastRecvTime = millis();
#ifdef DEBUG
    Serial.print(data.axis1);
    Serial.print("\t");
    Serial.print(data.axis2);
    Serial.print("\t");
    Serial.print(data.axis3);
    Serial.print("\t");
    Serial.print(data.axis4);
    Serial.println("\t");
#endif
  }

  // Switch channel
  if (millis() - lastRecvTime > 500) {
    chPointer ++;
    if (chPointer >= sizeof((*NRFchannel) / sizeof(byte))) chPointer = 0;
    radio.setChannel(NRFchannel[chPointer]);
    payload.channel = NRFchannel[chPointer];
  }

  if (millis() - lastRecvTime > 1000) { // set all analog values to their middle position, if no RC signal is received during 1s!
    data.axis1 = 50; // Aileron (Steering for car)
    data.axis2 = 50; // Elevator
    data.axis3 = 50; // Throttle
    data.axis4 = 50; // Rudder
    hazard = true; // Enable hazard lights
    payload.batteryOk = true; // Clear low battery alert (allows to re-enable the vehicle, if you switch off the transmitter)
#ifdef DEBUG
    Serial.println("No Radio Available - Check Transmitter!");
#endif
  }

  if (millis() - lastRecvTime > 2000) {
    setupRadio(); // re-initialize radio
    lastRecvTime = millis();
  }
#endif
}


//
// =======================================================================================================
// SBUS COMMANDS TO LIGHT- & SOUND CONTROLLER (if not DEBUG, not TXO_momentary1, not TXO_toggle1, not headLights)
// =======================================================================================================
//

void sendSbusCommands() 
{
   vehicleConfig* cfg = getConfigptr();

  static unsigned long lastSbusTime;
  uint16_t channels[16];

  // See: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32

  if (cfg->serialCommands) { // only, if we are in serial command mode
    if (millis() - lastSbusTime > 14) { // Send the data every 14ms
      lastSbusTime = millis();

      // Fill SBUS packet with our channels

      // Proportional channels
      channels[0] = map(data.axis1, 0, 100, 172, 1811);
      channels[1] = map(data.axis2, 0, 100, 172, 1811);
      channels[2] = map(data.axis3, 0, 100, 172, 1811);
      channels[3] = map(data.axis4, 0, 100, 172, 1811);
      channels[4] = map(data.pot1, 0, 100, 172, 1811);

      // Switches etc.
      if (data.mode1) channels[5] = 1811; else channels[5] = 172;
      if (data.mode2) channels[6] = 1811; else channels[6] = 172;
      if (data.momentary1) channels[7] = 1811; else channels[7] = 172;
      if (getHazard()) channels[8] = 1811; else channels[8] = 172;
      if (getLeft()) channels[9] = 1811; else channels[9] = 172;
      if (getRight()) channels[10] = 1811; else channels[10] = 172;

      // write the SBUS packet
#ifdef CONFIG_HAS_SBUS
      x8r.write(&channels[0]);
#endif
    }
  }
}

//
// =======================================================================================================
// SERIAL COMMANDS TO LIGHT- & SOUND CONTROLLER (if not DEBUG, not TXO_momentary1, not TXO_toggle1, not headLights)
// =======================================================================================================
//

void sendSerialCommands()
{
    vehicleConfig* cfg = getConfigptr();

  static unsigned long lastSerialTime;

  // '\n' is used as delimiter (separator of variables) during parsing on the sound controller
  // it is generated by the "println" (print line) command!
  // See: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32

  if (cfg->serialCommands) { // only, if we are in serial command mode
    if (millis() - lastSerialTime > 20) { // Send the data every 20ms
      lastSerialTime = millis();
      Serial.print('<'); // Start marker
      Serial.println(data.axis1);
      Serial.println(data.axis2);
      Serial.println(data.axis3);
      Serial.println(data.axis4);
      Serial.println(data.pot1);
      Serial.println(data.mode1);
      Serial.println(data.mode2);
      Serial.println(data.momentary1);
      Serial.println(getHazard());
      Serial.println(getLeft());
      Serial.println(getRight());
      Serial.print('>'); // End marker
    }
  }
}
