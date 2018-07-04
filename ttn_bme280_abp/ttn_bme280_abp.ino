/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example will send Temperature and Air Pressure
 * using frequency and encryption settings matching those of
 * the The Things Network. Application will 'sleep' 7x8 seconds (56 seconds)
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
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
 *******************************************************************************/

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"

#include <Adafruit_BME280.h>

Adafruit_BME280 bme280;

#include <Arduino.h>

int sleepcycles = 7;  // every sleepcycle will last 8 secs, total sleeptime will be sleepcycles * 8 sec
bool joined = false;
bool sleeping = false;
#define LedPin 2     // pin 13 LED is not used, because it is connected to the SPI port

#define DEBUG //change to DEBUG to see Serial messages

#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_HEX_PRINT(x,y) Serial.print(x,y)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_HEX_PRINT(x,y)
#endif



// LoRaWAN end-device address (DevAddr)
// Copy this from the Device address key on TTN properties for the node
// Note that you have to "scrunch" the hex digits to make a single large value
static const u4_t DEVADDR = 0x260113EF ; // <-- Change this address for every node!

// LoRaWAN NwkSKey, network session key
// Copy this from the Network Session Key entry on TTN properties for the 
// node. Use the msb ordering
static const PROGMEM u1_t NWKSKEY[16] = { 0xED, 0x77, 0x11, 0x55, 0x26, 0x75, 0x98, 0x4F, 
                                          0xC6, 0x63, 0x52, 0x28, 0x74, 0x43, 0xEA, 0xAF };
// LoRaWAN AppSKey, application session key
// Copy this from the App Session Key on TTN properties for the 
// node. Use the msb ordering
static const u1_t PROGMEM APPSKEY[16] = { 0xEF, 0xE8, 0xDB, 0xBA, 0x7F, 0xB8, 0x51, 0xE8, 
                                          0x48, 0xEE, 0xDD, 0x5D, 0x58, 0xD2, 0xAE, 0x50 };



// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;
static osjob_t initjob;

// Pin mapping is hardware specific.
// Pin mapping Doug Larue PCB
const lmic_pinmap lmic_pins = {
.nss = 10,
.rxtx = 0, //LMIC_UNUSED_PIN,
.rst = 0,
.dio = {4, 5, 7},
};

void onEvent (ev_t ev) {
  int i,j;
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      digitalWrite(LedPin,LOW);
      // after Joining a job with the values will be sent.
      joined = true;
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      // Re-init
      os_setCallback(&initjob, initfunc);
      break;
    case EV_TXCOMPLETE:
      sleeping = true;
        if (LMIC.dataLen) {
        // data received in rx slot after tx
        // if any data received, a LED will blink
        // this number of times, with a maximum of 10
        Serial.print(F("Data Received: "));
        Serial.println(LMIC.frame[LMIC.dataBeg],HEX);
        i=(LMIC.frame[LMIC.dataBeg]);
        // i (0..255) can be used as data for any other application
        // like controlling a relay, showing a display message etc.
        if (i>10){
          i=10;     // maximum number of BLINKs
        }
          for(j=0;j<i;j++)
          {
            digitalWrite(LedPin,HIGH);
            delay(200);
            digitalWrite(LedPin,LOW);
            delay(400);
          }
      }
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      delay(50);  // delay to complete Serial Output before Sleeping

      // Schedule next transmission
      // next transmission will take place after next wake-up cycle in main loop
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
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

void setup()
  {
  Serial.begin(115200);
  delay(250);
  Serial.println(F("Starting"));
    Serial.print("Probe BME280: ");
  boolean status;
  status = bme280.begin(0x76);
  if (status) {
    Serial.println("Sensor found");
  }
  else{
    Serial.println("Sensor missing");
    while (1) {}
  }

  // if LED is connected to pin 10, it has to be defined before any SPI initialization else
  // it will be used as SS (Slave Select) and controlled by the SPI module
      pinMode(LedPin, OUTPUT);
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

}

unsigned long time;
void loop() {
   do_send(&sendjob);    // Sent sensor values
      while(sleeping == false)
      {
        os_runloop_once();
      }
      sleeping = false;
      for (int i=0;i<sleepcycles;i++)
      {
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    //sleep 8 seconds
      }
}
