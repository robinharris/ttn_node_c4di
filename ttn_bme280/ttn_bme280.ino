

/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example will send Temperature, Humidity and pressure from a BME280
 * using frequency and encryption settings matching those of
 * the The Things Network. Application will 'sleep' 7x8 seconds (56 seconds)
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 * 
 * Updated for the BME280 sensor
 * DEBUG output is turned on by defining DEBUG at the top.
 * Robin Harris
 * 3rd June 2018
 *
 *******************************************************************************/

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"
#include "i2c.h"
#include <Adafruit_BME280.h>

Adafruit_BME280 bme280;

#include <Arduino.h>

int sleepcycles = 70;  // every sleepcycle will last 8 secs, total sleeptime will be sleepcycles * 8 sec 225 = 30 minutes
bool joined = false;
bool sleeping = false;
#define LedPin 2     // pin 13 LED is not used, because it is connected to the SPI port
#define NO_DEBUG //change to DEBUG to see Serial messages

#ifdef DEBUG
  #define DEBUG_PRINT(x) DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_HEX_PRINT(x,y) Serial.print(x,y)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_HEX_PRINT(x,y)
#endif
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.

static const u1_t DEVEUI[8] = { 0x77, 0x19, 0x56, 0x03, 0x04, 0x56, 0x01, 0x23 };
static const u1_t APPEUI[8] = { 0x36, 0xF2, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t APPKEY[16] = { 0x9F, 0xF6, 0x1F, 0xA3, 0x6A, 0x9A, 0x70, 0x66, 0x66, 0xC9, 0x87, 0x54, 0x24, 0x17, 0xDD, 0xD9 };

void os_getArtEui (u1_t* buf) {
  memcpy(buf, APPEUI, 8);
}

// provide DEVEUI (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
  memcpy(buf, DEVEUI, 8);
}

// provide APPKEY key (16 bytes)
void os_getDevKey (u1_t* buf) {
  memcpy(buf, APPKEY, 16);
}

static osjob_t sendjob;
static osjob_t initjob;

// Pin mapping is hardware specific.
// Pin mapping Doug Larue PCB
const lmic_pinmap lmic_pins = {
.nss = 10,
.rxtx = 0, //LMIC_UNUSED_PIN,
.rst = 0,
.dio = {7},};

void onEvent (ev_t ev) {
  int i,j;
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      DEBUG_PRINTLN("EV_SCAN_TIMEOUT");
      break;
    case EV_BEACON_FOUND:
      DEBUG_PRINTLN("EV_BEACON_FOUND");
      break;
    case EV_BEACON_MISSED:
      DEBUG_PRINTLN("EV_BEACON_MISSED");
      break;
    case EV_BEACON_TRACKED:
      DEBUG_PRINTLN("EV_BEACON_TRACKED");
      break;
    case EV_JOINING:
      DEBUG_PRINTLN("EV_JOINING");
      break;
    case EV_JOINED:
      DEBUG_PRINTLN("EV_JOINED");
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      digitalWrite(LedPin,LOW);
      // after Joining a job with the values will be sent.
      joined = true;
      break;
    case EV_RFU1:
      DEBUG_PRINTLN("EV_RFU1");
      break;
    case EV_JOIN_FAILED:
      DEBUG_PRINTLN("EV_JOIN_FAILED");
      break;
    case EV_REJOIN_FAILED:
      DEBUG_PRINTLN("EV_REJOIN_FAILED");
      // Re-init
      os_setCallback(&initjob, initfunc);
      break;
    case EV_TXCOMPLETE:
      sleeping = true;
      if (LMIC.dataLen){
        // data received in rx slot after tx
        // if any data received, a LED will blink
        // this number of times, with a maximum of 10
        DEBUG_PRINT("Data Received: ");
        // DEBUG_PRINTLN(LMIC.frame[LMIC.dataBeg],HEX);
        i=(LMIC.frame[LMIC.dataBeg]);
        // i (0..255) can be used as data for any other application
        // like controlling a relay, showing a display message etc.
        if (i>10){
          i=10;     // maximum number of BLINKs
        }
        for(j=0;j<i;j++){
          digitalWrite(LedPin,HIGH);
          delay(200);
          digitalWrite(LedPin,LOW);
          delay(400);
        }
      }
      DEBUG_PRINTLN("EV_TXCOMPLETE (includes waiting for RX windows)");
      delay(50);  // delay to complete Serial Output before Sleeping

      // Schedule next transmission
      // next transmission will take place after next wake-up cycle in main loop
      break;
    case EV_LOST_TSYNC:
      DEBUG_PRINTLN("EV_LOST_TSYNC");
      break;
    case EV_RESET:
      DEBUG_PRINTLN("EV_RESET");
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      DEBUG_PRINTLN("EV_RXCOMPLETE");
      break;
    case EV_LINK_DEAD:
      DEBUG_PRINTLN("EV_LINK_DEAD");
      break;
    case EV_LINK_ALIVE:
      DEBUG_PRINTLN("EV_LINK_ALIVE");
      break;
    default:
      DEBUG_PRINTLN("Unknown event");
      break;
  }
}

void do_send(osjob_t* j) {
  byte buffer[3];
  float temperature, pascal, humidity;
  uint16_t t_value, p_value, s_value;
  uint8_t h_value;
  temperature = bme280.readTemperature();
  humidity = bme280.readHumidity();
  pascal = bme280.readPressure();
  pascal=pascal/100;
  DEBUG_PRINT("Pressure: ");
  DEBUG_PRINT(pascal);
  DEBUG_PRINT("Pa\t");
  DEBUG_PRINT("Temperature: ");
  DEBUG_PRINT(temperature);
  DEBUG_PRINT("C\t");
  DEBUG_PRINT("\tHumidity: ");
  DEBUG_PRINT(humidity);
  DEBUG_PRINTLN("%");
  temperature = constrain(temperature,-24,40);  //temp in range -24 to 40 (64 steps)
  pascal=constrain(pascal,970,1034);    //pressure in range 970 to 1034 (64 steps)*/
  t_value=uint16_t((temperature*(100/6.25)+2400/6.25)); //0.0625 degree steps with offset
                                                // no negative values
  DEBUG_PRINT("Coded TEMP: ");
  DEBUG_HEX_PRINT(t_value,HEX);
  p_value=uint16_t((pascal-970)/1); //1 mbar steps, offset 970.
  DEBUG_PRINT("\tCoded Pascal: ");
  DEBUG_HEX_PRINT(p_value,HEX);
  DEBUG_PRINT("\tCoded Humidity: ");
  DEBUG_HEX_PRINT(h_value, HEX);
  DEBUG_PRINTLN("");
  s_value=(p_value<<10) + t_value;  // putting the bits in the right place
  h_value = uint8_t(humidity);
  DEBUG_PRINT("\tCoded sent: ");
  DEBUG_HEX_PRINT(h_value,HEX);
  DEBUG_HEX_PRINT(s_value,HEX);
  DEBUG_PRINTLN("");
  buffer[0]=s_value&0xFF; //lower byte
  buffer[1]=s_value>>8;   //higher byte
  buffer[2]=h_value;
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    DEBUG_PRINTLN("OP_TXRXPEND, not sending");
  }
  else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, (uint8_t*) buffer, 3, 0);
    DEBUG_PRINTLN("Sending: ");
  }
}

// initial job
static void initfunc (osjob_t* j) {
  // reset MAC state
  LMIC_reset();
  // start joining
  LMIC_startJoining();
  // init done - onEvent() callback will be invoked...
}

void setup(){
  Serial.begin(115200);
  delay(250);
  DEBUG_PRINTLN("Starting");
  DEBUG_PRINT("Probe BME280: ");
  boolean status;
  status = bme280.begin();
  if (status) {
    DEBUG_PRINTLN("Sensor found");
  }
  else{
    DEBUG_PRINTLN("Sensor missing");
    while (1) {}
  }
  // if LED is connected to pin 10, it has to be defined before any SPI initialization else
  // it will be used as SS (Slave Select) and controlled by the SPI module
  pinMode(LedPin, OUTPUT);
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  os_setCallback(&initjob, initfunc);
  LMIC_reset();
}

unsigned long time;
void loop(){
  // start OTAA JOIN
  if (joined==false){
    os_runloop_once();
  }
  else{
    do_send(&sendjob);    // Send sensor values
    while(sleeping == false){
      os_runloop_once();
    }
    sleeping = false;
    for (int i=0;i<sleepcycles;i++){
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    //sleep 8 seconds
    }
  }
  digitalWrite(LedPin,((millis()/100) % 2) && (joined==false)); // only blinking when joining and not sleeping
}
