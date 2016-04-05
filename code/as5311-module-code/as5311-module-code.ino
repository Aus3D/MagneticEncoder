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
 * 1: Resets travelled distance to zero
 * 
 * 2: Sets this modules I2C address to the next byte received, 
 *    saves in EEPROM
 *    
 * 3: Set the information that will be sent on the next I2C request.
 *      0: Default, responds with encoder count
 *      1: Responds with magnetic signal strength
 *            0: Signal good, in range.
 *            1: Signal weak, edge of range (but probably useable)
 *            2: Signal weak / lost, edge / outside of range (probably not useable)
 *
 * 4: Clear the settings saved in EEPROM, restorind all defaults
 *      Handy to return to hardware I2C address if required
 *      A power cycle will be required for settings to reset
 *
 * 10: Sets the mode of the LEDs to the next two bytes received, 
 *    saves in EEPROM
 *
 * 11: Sets the brightness of the LEDs to the next two bytes received, 
 *    saves in EEPROM
 *
 * 12: Sets the RGB value for the LEDs to the next three bytes received
 *     LEDs must still be set to RGB mode to display the sent value.
 *
 * 13: Sets the HSV value for the LEDs to the next three bytes received
 *     LEDs must still be set to HSV mode to display the sent value.
 *
 * 14: Set the rate variable for both LEDs to the next two bytes received
 *      Rate is used in some of the different LED modes
 *
 * 15: Set the sleep time for both LEDs to the next two bytes received.
 *      Sleep time determines how the LEDs will behave when there is no axis motion
 *            0: LED always on regardless of axis       (default)
 *            1-255: LED will turn off if there has been no significant
 *                   axis motion for this number of seconds.      
 *
 * 
 * I2C address can either be set over I2C (as shown above), or configured by cutting jumper traces on PCB.
 * If the address has been set by I2C, it can only be overridden by setting a new address over I2C, or by
 * sending the I2C command to reset the EEPROM.
 * 
 * 0 = default
 * 1 = trace cut
 * 
 * ADR1 ADR2 |  Mode
 *  0    0   |   X      //Default
 *  0    1   |   Y
 *  1    0   |   Z
 *  1    1   |   E
 *  
 *  
 *  
 *  LED Modes:
 *  
 *  0     Status indication of magnetic field
 *  1     Solid White
 *  2     Solid Red
 *  3     Solid Green
 *  4     Solid Blue
 *  5     RGB Value
 *  6     HSV Value
 *  7     Party Mode 1
 *  8     Party Mode 2
 *  9     Party Mode 3
 *  10    Reactive Mode 1 (Speed)
 *  11    Reactive Mode 2 (Acceleration)
 *  12    Reactive Mode 3 (Direction)
 *  
 *              
 /---------------------------------------------------------------/
 */
#include <Wire.h>
#include <EEPROM.h>
#include "FastLED.h"

#define SCHEMA 5

#define PIXEL_PIN   7
#define PIXEL_NUM   2

#define SELECT_PIN  4
#define CLOCK_PIN   5
#define DATA_PIN    6

#define ADDR1_PIN   16
#define ADDR2_PIN   17

#define LOOP_TIME_US 20000
#define TICKS_PER_MM 2048

//#define SERIAL_ENABLED

#define EEPROM_I2C_ADDR 1
#define EEPROM_BRT1_ADDR 2
#define EEPROM_BRT2_ADDR 3
#define EEPROM_MODE1_ADDR 4
#define EEPROM_MODE2_ADDR 5
#define EEPROM_RATE1_ADDR 6
#define EEPROM_RATE2_ADDR 7
#define EEPROM_SLP1_ADDR 8
#define EEPROM_SLP2_ADDR 9
#define EEPROM_RGB1_ADDR 10
#define EEPROM_RGB2_ADDR 11
#define EEPROM_RGB3_ADDR 12
#define EEPROM_HSV1_ADDR 13
#define EEPROM_HSV2_ADDR 14
#define EEPROM_HSV3_ADDR 15

#define I2C_MAG_SIG_GOOD 0
#define I2C_MAG_SIG_MID 1
#define I2C_MAG_SIG_BAD 2


const byte i2c_base_address = 30;
byte i2c_address;
int i2c_response_mode = 0;

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
long avgSpeed = 0;

float mm = 0;
float oldMm = -999;
float prevMm = 0;

bool OCF = false;
bool COF = false;
bool LIN = false;
bool mINC = false;
bool mDEC = false;

unsigned long lastLoopTime = 0;

byte addressOffset;

int ledBrightness[] = {50,50};
int ledMode[] = {0,0};
int ledRate[] = {0,0};
int ledSleep[] = {0,0};

byte ledRGB[] = {255,0,0};
byte ledHSV[] = {255,255,255};

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
  i2c_address = i2c_base_address + addressOffset;
  
  FastLED.addLeds<NEOPIXEL, PIXEL_PIN>(leds, PIXEL_NUM);
  FastLED.setBrightness(255);

  if (EEPROM.read(0) != SCHEMA) {
    reinitialize();
    EEPROM.write(0, SCHEMA);
    blinkLeds(1,CRGB::Purple);
  } else {
    eepromLoad();
    blinkLeds(1,CRGB::Green);
  }

  //signal address
  if(i2c_address >= i2c_base_address && i2c_address < i2c_base_address + 4) {
    blinkLeds(i2c_address - i2c_base_address + 1,CRGB::White);
  }
  
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
  updateSerial();
  updateLeds();
  //loopTiming();   //timing seems to prevent module from working at high travel speeds. Better for now just to run as fast as possible, will revisit later.
}

void updateLeds() {
  for(int i = 0; i < PIXEL_NUM; i++) {
    switch (ledMode[i]) {
      case 0:
        if(mINC == false && mDEC == false) { leds[i] = CRGB::Green; }
        if(mINC == true && mDEC == true && LIN == false) { leds[i] = CRGB::Yellow; }
        if(mINC == true && mDEC == true && LIN == true) { leds[i] = CRGB::Red; }
        break;
      case 1:
        leds[i] = CRGB::White;
        break;
      case 2:
        leds[i] = CRGB::Red;
        break;
      case 3:
        leds[i] = CRGB::Green;
        break;
      case 4:
        leds[i] = CRGB::Blue;
        break;
      case 5:
        leds[i].setRGB(ledRGB[0],ledRGB[1],ledRGB[2]);
        break;
      case 6:
        leds[i].setHSV(ledHSV[0],ledHSV[1],ledHSV[2]);
        break;
      case 7:
        leds[i].setHSV(totalCount/10*ledRate[i],255,255);
        break;
    }
    leds[i].nscale8(ledBrightness[i]);
  }
  FastLED.show();
}

void blinkLeds(int times, const CRGB& rgb) {
  
  for(int i = 0; i < times; i++) {  
    for(int j = 0; j < PIXEL_NUM; j++) {
      leds[j] = rgb;
      leds[j].nscale8(20);
      //leds[j].nscale8(ledBrightness[j]);
    }
    FastLED.show();
    delay(500);
    for(int j = 0; j < PIXEL_NUM; j++) {
      leds[j] = CRGB::Black;
      leds[j].nscale8(ledBrightness[j]);
    }
    FastLED.show();
    delay(500);
  }
}


////////////////////////////////////////////////////////////
//----------------------- I2C ----------------------------//
////////////////////////////////////////////////////////////

void requestEvent() {

  switch (i2c_response_mode) {
    case 0:
      Wire.write(encoderCount.bval,4);
      break;
    case 1:
      if(mINC == false && mDEC == false) { Wire.write(I2C_MAG_SIG_GOOD); }
      if(mINC == true && mDEC == true && LIN == false) { Wire.write(I2C_MAG_SIG_MID); }
      if(mINC == true && mDEC == true && LIN == true) { Wire.write(I2C_MAG_SIG_BAD); }
      break;
  }
}

void receiveEvent(int numBytes) {

  byte temp[5] = {0};
  int tempIndex = 0;

  while(Wire.available() > 0) {
    temp[tempIndex] = Wire.read();
    tempIndex++;
  }

  switch(temp[0]) {
    case 1: 
      offset = totalCount;
      break;
    case 2:
      setI2cAddress(temp[1]);
      break;
    case 3:
      i2c_response_mode = temp[1];
      break; 
    case 4:
      eepromClear();
      break; 
    case 10:
      setLedMode(temp[1],temp[2]);
      break;
    case 11:
      setLedBrightness(temp[1],temp[2]);
      break;  
    case 12:
      setLedRGB(temp[1],temp[2],temp[3]);
      break;   
    case 13:
      setLedHSV(temp[1],temp[2],temp[3]);
      break;   
    case 14:
      setLedRate(temp[1],temp[2]);
      break;    
    case 15:
      setLedSleep(temp[1],temp[2]);
      break;   
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

  prevMm = mm;
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

  if (!(d2 & B00001000))
    OCF = true;

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
  eepromClear();
  setI2cAddress(99);
  setLedBrightness(ledBrightness[0],ledBrightness[1]);
  setLedMode(ledMode[0],ledMode[1]);
  setLedRate(ledRate[0],ledRate[1]);
  setLedSleep(ledSleep[0],ledSleep[1]);
  setLedRGB(ledRGB[0],ledRGB[1],ledRGB[2]);
  setLedHSV(ledHSV[0],ledHSV[1],ledHSV[2]);
}

void eepromLoad() {
  byte tempAddress = EEPROM.read(EEPROM_I2C_ADDR);

  //check that a value has actually been set,
  //otherwise we use the hardware setting
  if(tempAddress != 99) 
    i2c_address = tempAddress; 

  ledMode[0] = EEPROM.read(EEPROM_MODE1_ADDR);
  ledMode[1] = EEPROM.read(EEPROM_MODE2_ADDR);
  ledBrightness[0] = EEPROM.read(EEPROM_BRT1_ADDR);
  ledBrightness[1] = EEPROM.read(EEPROM_BRT2_ADDR);
  ledRate[0] = EEPROM.read(EEPROM_RATE1_ADDR);
  ledRate[1] = EEPROM.read(EEPROM_RATE2_ADDR);
  ledSleep[0] = EEPROM.read(EEPROM_SLP1_ADDR);
  ledSleep[1] = EEPROM.read(EEPROM_SLP2_ADDR);
  ledRGB[0] = EEPROM.read(EEPROM_RGB1_ADDR);
  ledRGB[1] = EEPROM.read(EEPROM_RGB2_ADDR);
  ledRGB[2] = EEPROM.read(EEPROM_RGB3_ADDR);
  ledHSV[0] = EEPROM.read(EEPROM_HSV1_ADDR);
  ledHSV[1] = EEPROM.read(EEPROM_HSV2_ADDR);
  ledHSV[2] = EEPROM.read(EEPROM_HSV3_ADDR);
}

void setI2cAddress(byte i2cAddress) {
  EEPROM.put(EEPROM_I2C_ADDR, i2cAddress);
}

void setLedBrightness(byte brightness1, byte brightness2) {
  ledBrightness[0] = brightness1;
  ledBrightness[1] = brightness2;
  EEPROM.put(EEPROM_BRT1_ADDR, ledBrightness[0]);
  EEPROM.put(EEPROM_BRT2_ADDR, ledBrightness[1]);
}

void setLedMode(byte mode1, byte mode2) {
  ledMode[0] = mode1;
  ledMode[1] = mode2;
  EEPROM.put(EEPROM_MODE1_ADDR, ledMode[0]);
  EEPROM.put(EEPROM_MODE2_ADDR, ledMode[1]);
}

void setLedRate(byte rate1, byte rate2) {
  ledRate[0] = rate1;
  ledRate[1] = rate2;
  EEPROM.put(EEPROM_RATE1_ADDR, ledRate[0]);
  EEPROM.put(EEPROM_RATE2_ADDR, ledRate[1]);
}

void setLedSleep(byte sleep1, byte sleep2) {
  ledSleep[0] = sleep1;
  ledSleep[1] = sleep2;
  EEPROM.put(EEPROM_SLP1_ADDR, ledSleep[0]);
  EEPROM.put(EEPROM_SLP2_ADDR, ledSleep[1]);
}

void setLedRGB(byte red, byte green, byte blue) {
  ledRGB[0] = red;
  ledRGB[1] = green;
  ledRGB[2] = blue;
  EEPROM.put(EEPROM_RGB1_ADDR, ledRGB[0]);
  EEPROM.put(EEPROM_RGB2_ADDR, ledRGB[1]);
  EEPROM.put(EEPROM_RGB3_ADDR, ledRGB[2]);
}

void setLedHSV(byte hue, byte sat, byte val) {
  ledHSV[0] = hue;
  ledHSV[1] = sat;
  ledHSV[2] = val;
  EEPROM.put(EEPROM_HSV1_ADDR, ledHSV[0]);
  EEPROM.put(EEPROM_HSV2_ADDR, ledHSV[1]);
  EEPROM.put(EEPROM_HSV3_ADDR, ledHSV[2]);
}

void eepromClear() {
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i,0);
  }
}

////////////////////////////////////////////////////////////
//---------------------- SERIAL --------------------------//
////////////////////////////////////////////////////////////
void updateSerial() {
  #ifdef SERIAL_ENABLED
    if(Serial.available() > 0) {
      if(Serial.read() == 'r' || Serial.read() == 'R') {
        revolutions = 0;
        offset = -count;
        Serial.println("Resetting offset...");
      }
    }

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

  #endif
}

////////////////////////////////////////////////////////////
//------------------------ MISC --------------------------//
////////////////////////////////////////////////////////////


void loopTiming() {
  while(micros() - lastLoopTime < LOOP_TIME_US) {
    delayMicroseconds(20);
  }
  lastLoopTime = micros();
}

long runningAverage(long M) {
  #define LM_SIZE 20
  static long LM[LM_SIZE];      // LastMeasurements
  static byte index = 0;
  static long sum = 0;
  static byte count = 0;

  // keep sum updated to improve speed.
  sum -= LM[index];
  LM[index] = M;
  sum += LM[index];
  index++;
  index = index % LM_SIZE;
  if (count < LM_SIZE) count++;

  return sum / count;
}
