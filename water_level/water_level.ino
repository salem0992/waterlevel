/*******************************************************************************
 * 1Modified by Thomas Laurenson, 2017
 * Specific modifications for use of Dragino LoRaSHield on AU915, sub-band 2
 *onEvent
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the (early prototype version of) The Things Network.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
 *  0.1% in g2).
 *
 * Change DEVADDR to a unique address!
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
const int NUMBER_0_IN_ASCII = 48;

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
static const PROGMEM u1_t NWKSKEY[16] = { 0x9B, 0xE3, 0xFF, 0x70, 0xC7, 0x1B, 0x66, 0xBC, 0x82, 0x47, 0x26, 0xF7, 0xF0, 0xF0, 0x68, 0x48 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
static const u1_t PROGMEM APPSKEY[16] = { 0x46, 0x37, 0x0D, 0xD5, 0xE6, 0xC6, 0x94, 0x85, 0x40, 0xDD, 0x1D, 0x69, 0x91, 0xE1, 0xC6, 0x97 };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
static const u4_t DEVADDR = 0x26041DF3 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t *mydata;
static osjob_t sendjob;
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 20;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

//Arduino PINs to get 
const int temp_sensor_pin = 0; // analog pin used to read temperature sensor
const int trig_pin = 2; //ultrasonic sensor trigger pin
const int echo_pin = 4; //ultrasonic sensor echo pin
const int humid_sensor_pin = 5; // analog pin used to read humidity sensor

//variables for water level
static double voltage;
static double distance;
static double air_speed;

static double val;
static float mv;
static float cel;
  
static double travel_time;
static double temp;
static double humid;
static double sound_speed;
static double humidity;

// establish variables for duration of the ping,
// and the distance result in inches and centimeters:
static long duration, inches, cm;

//get speed of air
//to calculate speed of air: http://www.circuitbasics.com/how-to-set-up-an-ultrasonic-range-finder-on-an-arduino/
//air_speed = 331.4 + (0.606 * temp) + (0.0124 * humid);
//temp: in C degree.
//humid: humidity in % (relative humidity) 
//
double speedOfSound(double temp, double humid)
{
  //to calculate speed of sound with the consideration of tempature and humidity
  //http://www.circuitbasics.com/how-to-set-up-an-ultrasonic-range-finder-on-an-arduino/
  air_speed = 331.4 + (0.606 * temp) + (0.0124 * humid);
  //Serial.println(air_speed);
  return (double) air_speed * 100/1000000; // convert speed from m/s to cm/microseconds
}

// read temperature from LM35 temperature sensor
// ref: http://www.instructables.com/id/ARDUINO-TEMPERATURE-SENSOR-LM35/
double getTemperature()
{
   //get value from temperature analog
   val = analogRead(temp_sensor_pin);
   mv = ( val/1024.0)*5000; 
   cel = mv/10;  //convert to celcious degree
   
   return cel;
}

// get travel time
//ref: http://www.circuitbasics.com/how-to-set-up-an-ultrasonic-range-finder-on-an-arduino/
double getTravelTime()
{
  //setup PIN
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(trig_pin, OUTPUT);
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  pinMode(echo_pin, INPUT);
  duration = pulseIn(echo_pin, HIGH); 

  //reset trig_pin to input so that Lora can receive data later on.
  //not sure why but if don't change this pin mode to input, lora only sends data once
  pinMode(trig_pin, INPUT);
    
  return duration;
}

//calculate humdity
double getHumidity()
{
  voltage = analogRead(humid_sensor_pin);
  humidity = 95 - (double)(95-20)/1023 * voltage;
  //Serial.print("humid: ");
  //Serial.println( humidity );
  return humidity;
}

//calculate distance from the sensor to the water level
double getWaterLevel()
{
   travel_time = getTravelTime();       // reads the value of the ultrasonic sensor, it is in microseconds
   temp = getTemperature();       // reads the value of the temperature sensor
   humid = getHumidity(); // reads the value of the humidity sensor
   
  //copied from https://www.arduino.cc/en/Tutorial/Ping?from=Tutorial.UltrasoundSensor
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  // Then multiply that with the air_speed
  sound_speed = speedOfSound(temp, humid);
  distance = (travel_time / 2) * sound_speed;
  //Serial.println(sound_speed);
  //Serial.println(travel_time);
  //Serial.println(distance);
  return distance;
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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

//convert number to array of characters
// e.g 621 --> ['6', '2', '1']
uint8_t * convertToArray(float number)
{
  //Serial.println("number");
  //Serial.println(number);
  char numberStr[32];
  //int length = sprintf(numberStr, "%4.2f \r\n", number); //bug in arduino when print float to string

  //trick from http://yaab-arduino.blogspot.com.au/2015/12/how-to-sprintf-float-with-arduino.html
  int length = sprintf(numberStr, "%d.%02d", (int)number, (int)(number*100)%100);
  Serial.print("length ");
  Serial.println(length );
  uint8_t *mydata = new uint8_t[ strlen(numberStr)];
  for (int i = 0; i < strlen(numberStr); i++)
  {
     mydata[i] = numberStr[i];
     //  print data to dverify
     Serial.print(mydata[i]);
     Serial.print(" ");
  }

  return mydata;
}

void do_send(osjob_t* j){
    distance = getWaterLevel();
    Serial.println("distance");
    Serial.println(distance);
    mydata = convertToArray (distance);      
     
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        //Serial.println(F("OP_TXRXPEND, not sending"));
        Serial.print("OP_TXRXPEND, not sending; at freq: ");
        Serial.println(LMIC.freq);        
    } else {
        // Prepare upstream data transmission at the next possible time.
       // LMIC.FRAME[0] =
        //[1]++++++++
        
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.print(F("Packet queued for freq: "));
        Serial.println(LMIC.freq);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(9600);
    Serial.println(F("Starting"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly 
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

     // THIS IS WHERE THE AUSTRALIA FREQUENCY MAGIC HAPPENS!
    // The frequency plan is hard-coded
    // But the band (or selected 8 channels) is configured here!
    // This is the same AU915 band as used by TTN
    
    // First, disable channels 0-7
    for (int channel=0; channel<8; ++channel) {
      LMIC_disableChannel(channel);
    }
    // Now, disable channels 16-72 (is there 72 ??)
    for (int channel=16; channel<72; ++channel) {
       LMIC_disableChannel(channel);
    }
    // This means only channels 8-15 are up

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
  //do_send(&sendjob);
  //delay(5000);
  os_runloop_once();
}
