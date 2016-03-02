/*
 /---------------------------------------------------------------/
 * Code for the Aus3D AS5311 Magnetic Encoder I2C Module
 * Chris Barr, 2015
 * 
 * This module is designed to connect to a host device via I2C,
 * and report the current position as observed by the magnetic encoder.
 * 
 * The following I2C commands are implemented:
 * 
 * 0: Responds with 4 bytes reporting distance from encoder. 
 *    Measured in encoder count, distance should be calculated by external controller.
 *    Also sent if no valid command received.
 *    
 * 1: Resets travelled distance to zero
 * 
 * 2: Sets this modules I2C address to the next byte received, 
 *    saves in EEPROM
 *    
 * 3: Sets the brightness of the LED indicator to the next byte received, 
 *    saves in EEPROM
 *    
 * 4: Responds with the current magnetic strength as reported by the encoder.
 *      0: Signal good, in range.
 *      1: Signal weak, edge of range (but probably useable)
 *      2: Signal weak / lost, edge / outside of range (probably not useable)
 * 
 * 
 * Address configuration
 * 
 * 0 = default
 * 1 = trace cut
 * 
 * ADR1 ADR2 |  Mode
 *  0    0   |   X      //Default
 *  0    1   |   Y
 *  1    0   |   Z
 *  1    1   |   E
 /---------------------------------------------------------------/
 */
#include <Wire.h>
#include <EEPROM.h>
#include "FastLED.h"

#define SCHEMA 0x0101

#define PIXEL_PIN   7
#define PIXEL_NUM   2

#define SELECT_PIN  4
#define CLOCK_PIN   5
#define DATA_PIN    6

#define ADDR1_PIN   16
#define ADDR2_PIN   17

#define SERIAL_PERIOD 10
#define TICKS_PER_MM 2048
#define NM_PER_TICK 1950

//#define SERIAL_ENABLED
//#define PARTY_TIME

#define EEPROM_I2C_ADDR 1
#define EEPROM_BRT_ADDR 2

const byte i2c_base_address = 30;
byte i2c_address;

typedef union{
    volatile long val;
    byte bval[4];
}i2cLong;

i2cLong encoderCount;

long count = 0;
long oldCount = 0;
long revolutions = 0;
long offset = 0;
long totalCount = 0;
bool offsetInitialised = false;

float mm = 0;
float oldMm = -999;

bool OCF = false;
bool COF = false;
bool LIN = false;
bool mINC = false;
bool mDEC = false;

unsigned long lastSerialTime = 0;

byte addressOffset;

int ledBrightness = 50;
int ledMode = 0;

CRGB leds[PIXEL_NUM];

void setup() {
  //setup our pins
  pinMode(DATA_PIN, INPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(SELECT_PIN, OUTPUT);
  
  digitalWrite(CLOCK_PIN, HIGH);
  digitalWrite(SELECT_PIN, HIGH);
  
  pinMode(ADDR1_PIN,INPUT_PULLUP);
  pinMode(ADDR2_PIN,INPUT_PULLUP);
  
  addressOffset = digitalRead(ADDR1_PIN) + 2*(digitalRead(ADDR2_PIN));

  short schema;
  EEPROM.get(0, schema);
  if (schema != SCHEMA) {
    reinitialize();
    schema = SCHEMA;
    EEPROM.write(0, schema);
  } else {
    byte tempOffset = EEPROM.read(EEPROM_I2C_ADDR);

    //check that a value has actually been set,
    //otherwise we always over-ride the hardware setting
    if(tempOffset != 99) 
      addressOffset = tempOffset;
    ledBrightness = EEPROM.read(EEPROM_BRT_ADDR);
  }

  FastLED.addLeds<NEOPIXEL, PIXEL_PIN>(leds, PIXEL_NUM);
  FastLED.setBrightness(ledBrightness);

  //signal address
  for(int i = 0; i < addressOffset+1; i++) {
    fill_solid( &(leds[0]), 2, CRGB::White);
    FastLED.show();
    delay(500);
    fill_solid( &(leds[0]), 2, CRGB::Black);
    FastLED.show();
    delay(500);
  }

  i2c_address = i2c_base_address + addressOffset;
  
  /*
  switch(addressOffset) {
    case 0:
      setColour(50,0,0);
      break;
    case 1:
      setColour(0,50,0);
      break;
    case 2:
      setColour(0,0,50);
      break;
    case 3:
      setColour(50,50,50);
      break;
  }
  */
  
  delay(500);
  
  #ifdef SERIAL_ENABLED
    Serial.begin(250000);
    Serial.println("Count:\tDistance:");
  #endif

  Wire.begin(i2c_address);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

}

void loop() {
  
  updateEncoder();
  serial_update();

  if(ledMode == 0) {
    if(mINC == false && mDEC == false) {
      fill_solid( &(leds[0]), 2, CRGB::Green);
      FastLED.show();
    }
  
    if(mINC == true && mDEC == true && LIN == false) {
      fill_solid( &(leds[0]), 2, CRGB::Yellow);
      FastLED.show();
    }
  
    if(mINC == true && mDEC == true && LIN == true) {
      fill_solid( &(leds[0]), 2, CRGB::Red);
      FastLED.show();
    }
  } else if(ledMode == 1) {
    fill_solid( &(leds[0]), 2, CHSV(totalCount/100,255,100+revolutions*5));
    FastLED.show();
  } else if(ledMode == 2) {
    fill_solid( &(leds[0]), 2, CRGB::White);
    FastLED.show();
  }
}

////////////////////////////////////////////////////////////
//----------------------- I2C ----------------------------//
////////////////////////////////////////////////////////////

void requestEvent() {

  Wire.write(encoderCount.bval,4); 

}

void receiveEvent(int numBytes) {

  byte temp = Wire.read();
  byte temp2 = Wire.read();

  switch(temp) {
    case 1: 
      offset = totalCount;
      break;
    case 2:
      setI2cAddress(temp2);
      break;
    case 3:
      setLedBrightness(temp2);
      LEDS.setBrightness(ledBrightness);
      FastLED.show();
      break;
    case 4:
      Wire.write(mINC + (mDEC*2)); 
      break;    
    case 5:
      ledMode = temp2;
  }
  
}

////////////////////////////////////////////////////////////
//--------------------- ENCODER --------------------------//
////////////////////////////////////////////////////////////
void updateEncoder() {
  count = readPosition();

  //check if we've moved from one pole-pair to the next
  if((count-oldCount) > 2048)
    revolutions -= 1;
  if((oldCount - count) > 2048)
    revolutions += 1;

  oldCount = count;

  if(offsetInitialised == false) {
    offset = -count;
    offsetInitialised = true;
  }

  totalCount = (revolutions * 4092) + (count + offset);
  encoderCount.val = totalCount;
  
  mm = (float) (totalCount) /TICKS_PER_MM;
}

int readPosition()
{
  unsigned int position = 0;

  //shift in our data  
  digitalWrite(SELECT_PIN, LOW);
  delayMicroseconds(1);
  byte d1 = shiftIn(DATA_PIN, CLOCK_PIN);
  byte d2 = shiftIn(DATA_PIN, CLOCK_PIN);
  byte d3 = shiftIn(DATA_PIN, CLOCK_PIN);
  digitalWrite(SELECT_PIN, HIGH);

  //get our position variable
  position = d1;
  position = position << 8;

  position |= d2;
  position = position >> 4;


  //check the offset compensation flag: 1 == started up
  if (!(d2 & B00001000))
    OCF = true;

  //check the offset compensation flag: 1 == started up
  if (!(d2 & B00000100))
    COF = true;

  LIN = bitRead(d2,1);

  mINC = bitRead(d2,0);

  mDEC = bitRead(d3,7);

  return position;
}


//read in a byte of data from the digital input of the board.
byte shiftIn(byte data_pin, byte clock_pin)
{
  byte data = 0;

  for (int i=7; i>=0; i--)
  {
    digitalWrite(clock_pin, LOW);
    delayMicroseconds(1);
    digitalWrite(clock_pin, HIGH);
    delayMicroseconds(1);

    byte bit = digitalRead(data_pin);

    data |= (bit << i);

  }

  return data;
}

////////////////////////////////////////////////////////////
//---------------------- EEPROM --------------------------//
////////////////////////////////////////////////////////////

void reinitialize()
{
  setI2cAddress(99);
  setLedBrightness(ledBrightness);
}

void setI2cAddress(byte i2cAddress) {
  EEPROM.put(1, i2cAddress);
}

void setLedBrightness(byte brightness) {
  ledBrightness = brightness;
  EEPROM.put(2, ledBrightness);
}

////////////////////////////////////////////////////////////
//---------------------- SERIAL --------------------------//
////////////////////////////////////////////////////////////
void serial_update() {
  #ifdef SERIAL_ENABLED
    if(Serial.available() > 0) {
      if(Serial.read() == 'r' || Serial.read() == 'R') {
        revolutions = 0;
        offset = -count;
        Serial.println("Resetting offset...");
      }
    }

    if(millis() - lastSerialTime > SERIAL_PERIOD){// && (abs(mm - oldMm) > 0.01)) {
      Serial.print(count);
      Serial.print("\t");
      Serial.print(mm,3);
      Serial.print("\t");
      Serial.print(revolutions);
      Serial.println(":");
      Serial.print((revolutions * 4092) + (count + offset));
      Serial.println();
      lastSerialTime = millis();
      oldMm = mm;
    } else {
      delay(1);  
    }
  #endif
}
