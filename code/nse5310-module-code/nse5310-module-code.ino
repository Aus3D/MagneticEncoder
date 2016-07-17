/*
 /---------------------------------------------------------------/
 * Code for the Aus3D NSE5310 Magnetic Encoder I2C Module
 * Chris Barr, 2016
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
 * 10: Sets the mode of the LEDs based on the next two bytes received, 
 *     byte1 = led (0 or 1), byte 2 = mode (see below)
 *     saves in EEPROM
 *
 * 11: Sets the brightness of the LEDs based on the next two bytes received,
 *    byte1 = led (0 or 1), byte 2 = brightness (0-255)
 *    saves in EEPROM
 *
 * 12: Sets the RGB value for the LEDs to the next three bytes received,
 *     LEDs must still be set to RGB mode to display the sent value.
 *
 * 13: Sets the HSV value for the LEDs to the next three bytes received,
 *     LEDs must still be set to HSV mode to display the sent value.
 *
 * 14: Set the rate variable of the LEDs based on the next two bytes received,
 *      Rate is used in some of the different LED modes
 *
 * 15: Set the sleep time of the LEDs based on the next two bytes received,
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
#include <SoftwareWire.h>
#include <EEPROM.h>
#include "FastLED.h"
#include <avr/wdt.h>

#define SCHEMA 5
#define FIRMWARE_VERSION "1.0.0"

#define PIXEL_PIN   7
#define PIXEL_NUM   2

#define DISABLE_PIN 3

//Encoder Setup
#define ENC_SELECT_PIN  4
#define ENC_CLOCK_PIN   5
#define ENC_DATA_PIN    6
#define ENC_ADDR 0b1000000

SoftwareWire encWire( ENC_DATA_PIN, ENC_CLOCK_PIN, true, true);

//I2C Slave Setup
#define ADDR1_PIN   16
#define ADDR2_PIN   17

//I2C Defines
#define I2C_MAG_SIG_GOOD 0
#define I2C_MAG_SIG_MID 1
#define I2C_MAG_SIG_BAD 2

#define I2C_REPORT_POSITION   0
#define I2C_REPORT_STATUS     1
#define I2C_REPORT_VERSION    2

#define I2C_REQ_REPORT        0
#define I2C_RESET_COUNT       1
#define I2C_SET_ADDR          2
#define I2C_SET_REPORT_MODE   3
#define I2C_CLEAR_EEPROM      4

#define I2C_ENC_LED_PAR_MODE  10
#define I2C_ENC_LED_PAR_BRT   11
#define I2C_ENC_LED_PAR_RATE  12
#define I2C_ENC_LED_PAR_RGB   13
#define I2C_ENC_LED_PAR_HSV   14

//default I2C addresses
#define I2C_ENCODER_PRESET_ADDR_X 30
#define I2C_ENCODER_PRESET_ADDR_Y 31
#define I2C_ENCODER_PRESET_ADDR_Z 32
#define I2C_ENCODER_PRESET_ADDR_E 33
#define I2C_UNALLOCATED_ADDRESS   0

#define MAG_GOOD_RANGE 4

const byte i2c_base_address = 30;
byte i2c_address;
int i2c_response_mode = 0;

//#define SERIAL_ENABLED

//EEPROM Setup
#define EEPROM_USED_SIZE 24 //used to only clear first X bytes on initialisation. Faster than clearing whole EEPROM.
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

typedef union{
    volatile long val;
    byte bval[4];
}i2cLong;

i2cLong encoderCount;

long count = 0;
long oldCount = 0;
long revolutions = 0;
long offset = 0;
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

byte magStrength = I2C_MAG_SIG_BAD;

unsigned long lastLoopTime = 0;

byte addressOffset;

int ledBrightness[] = {20,20};
int ledMode[] = {0,0};
int ledRate[] = {0,0};
int ledSleep[] = {0,0};

byte ledRGB[] = {255,0,0};
byte ledHSV[] = {255,255,255};

CRGB leds[PIXEL_NUM];

// This function is called upon a HARDWARE RESET:
void wdt_first(void) __attribute__((naked)) __attribute__((section(".init3")));
 
// Clear SREG_I on hardware reset.
void wdt_first(void)
{
  MCUSR = 0; // clear reset flags
  wdt_disable();
  // http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
}

void setup() {

  wdt_disable();

  #ifdef SERIAL_ENABLED
    Serial.begin(250000);
    Serial.println("Serial comms initialised");
  #endif

  pinMode(ENC_SELECT_PIN, OUTPUT);
  pinMode(ADDR1_PIN,INPUT_PULLUP);
  pinMode(ADDR2_PIN,INPUT_PULLUP);
  pinMode(DISABLE_PIN,INPUT_PULLUP);

  FastLED.addLeds<NEOPIXEL, PIXEL_PIN>(leds, PIXEL_NUM);
  FastLED.setBrightness(100);

  //check reset pin for permanent disable
  if(digitalRead(DISABLE_PIN) == LOW) {
    //crude debounce
    delay(100);
    if(digitalRead(DISABLE_PIN) == LOW) {
      digitalWrite(ENC_SELECT_PIN, LOW);
      blinkLeds(1,CRGB::White);
      while(true);
    }
  }

  if(initEncoder() == false) {
    #ifdef SERIAL_ENABLED
      Serial.print("Encoder Init Failed!");
    #endif
    while(initEncoder() == false) { blinkLeds(1,CRGB::Red); }
  }
  
  blinkLeds(1,CRGB::Green);

  
  addressOffset = digitalRead(ADDR1_PIN) + 2*(digitalRead(ADDR2_PIN));
  i2c_address = i2c_base_address + addressOffset;

  #ifdef SERIAL_ENABLED
    Serial.print("I2C Address: ");
    Serial.println(i2c_address);
  #endif

  if (EEPROM.read(0) != SCHEMA) {
    reinitialize();
    EEPROM.write(0, SCHEMA);
    eepromLoad();
    blinkLeds(1,CRGB::Purple);
  } else {
    eepromLoad();
    blinkLeds(1,CRGB::Green);
  }

  //signal address
  if(i2c_address >= i2c_base_address && i2c_address < i2c_base_address + 4) {
    switch (i2c_address) {
      case I2C_ENCODER_PRESET_ADDR_X:
        blinkLeds(1,CRGB::Blue);
        break;
      case I2C_ENCODER_PRESET_ADDR_Y:
        blinkLeds(1,CRGB::Yellow);
        break;
      case I2C_ENCODER_PRESET_ADDR_Z:
        blinkLeds(1,CRGB::Green);
        break;
      case I2C_ENCODER_PRESET_ADDR_E:
        blinkLeds(1,CRGB::Magenta);
        break;
    }
    //blinkLeds(i2c_address - i2c_base_address + 1,CRGB::White);
  } else {
    blinkLeds(1,CRGB::White);
  }
  
  delay(50);

  Wire.begin(i2c_address);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

}

void loop() {
  updateEncoder();
  updateLeds();
  watchResetPin();
}

void updateLeds() {
  for(int i = 0; i < PIXEL_NUM; i++) {
    switch (ledMode[i]) {
      case 0:
        if(magStrength == I2C_MAG_SIG_GOOD) { leds[i] = CRGB::Green; }
        else if(magStrength == I2C_MAG_SIG_MID) { leds[i] = CRGB::Yellow; }
        else if(magStrength == I2C_MAG_SIG_BAD) { leds[i] = CRGB::Red; }
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
        leds[i].setHSV(encoderCount.val/10*ledRate[i],255,255);
        break;
      case 8:
        leds[i].setHSV(millis()/1000*ledRate[i],255,255);
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
    case I2C_REPORT_POSITION:
      Wire.write(encoderCount.bval,4);
      break;
    case I2C_REPORT_STATUS:
      Wire.write(magStrength);
      break;
    case I2C_REPORT_VERSION:
      reportVersion();
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
    case I2C_RESET_COUNT: 
      offset = encoderCount.val;
      break;
    case I2C_SET_ADDR:
      setI2cAddress(temp[1]);
      blinkLeds(1,CRGB::White);
      restart();
      break;
    case I2C_SET_REPORT_MODE:
      i2c_response_mode = temp[1];
      break; 
    case I2C_CLEAR_EEPROM:
      eepromClear();
      break; 
    case I2C_ENC_LED_PAR_MODE:
      setLedMode(temp[1],temp[2]);
      break;
    case I2C_ENC_LED_PAR_BRT:
      setLedBrightness(temp[1],temp[2]);
      break;  
    case I2C_ENC_LED_PAR_RGB:
      setLedRGB(temp[1],temp[2],temp[3]);
      break;   
    case I2C_ENC_LED_PAR_HSV:
      setLedHSV(temp[1],temp[2],temp[3]);
      break;   
    case I2C_ENC_LED_PAR_RATE:
      setLedRate(temp[1],temp[2]);
      break;    
  }  
}

void reportVersion() {

   // 1.0.0, Jul 15 2016, 21:07:40
  String versionString = FIRMWARE_VERSION ", " __DATE__ ", " __TIME__ ".";
  //byte versionBytes[versionString.length() + 1];
  //versionString.getBytes(versionBytes,versionString.length() + 1);
  //Wire.write(versionBytes,versionString.length() + 1);
  Wire.write(versionString.c_str());
}

////////////////////////////////////////////////////////////
//--------------------- ENCODER --------------------------//
////////////////////////////////////////////////////////////
bool initEncoder() {

  encWire.begin();

  //read a test byte
  digitalWrite(ENC_SELECT_PIN, LOW);
  encWire.requestFrom(ENC_ADDR,1,true);

  int data = encWire.read();
  
  digitalWrite(ENC_SELECT_PIN, HIGH);

  if(data == -1) {
    return false;
  } else {
    return true;
  }
}

void updateEncoder() {
  count = readPosition();

  //check if we've moved from one pole-pair to the next
  if((count-oldCount) > 2048) {
    revolutions -= 1;
  } else if((oldCount - count) > 2048) {
    revolutions += 1;
  }

  oldCount = count;

  //make the starting position 'zero'
  if(offsetInitialised == false) {
    offset = -count;
    offsetInitialised = true;
  }

  encoderCount.val = (revolutions * 4092) + (count + offset);

}

int readPosition()
{
  unsigned int position = 0;

  //read in our data  
  digitalWrite(ENC_SELECT_PIN, LOW);
  encWire.requestFrom(ENC_ADDR,5,true);

  byte b1 = encWire.read();
  byte b2 = encWire.read();
  byte b3 = encWire.read();
  byte b4 = encWire.read();
  byte b5 = encWire.read();
  
  digitalWrite(ENC_SELECT_PIN, HIGH);

  //get our position variable
  position = b1;
  position = position << 8;

  position |= b2;
  position = position >> 4;

  byte mIncrDecr = bitRead(b2,0);
  byte linAlarm = bitRead(b2,1);
  byte cordicOverflow = bitRead(b2,2);
  byte offsetComp = bitRead(b2,3);

  //determine magnetic signal strength
  if(b4 >= (0x3F-MAG_GOOD_RANGE) && b4 <= (0x3F+MAG_GOOD_RANGE)) { magStrength = I2C_MAG_SIG_GOOD; }
  else if(b4 >= 0x20 && b4 <= 0x5F) { magStrength = I2C_MAG_SIG_MID; }
  else if(b4 < 0x20 || b4 > 0x5F) { magStrength = I2C_MAG_SIG_BAD; }

  return position;
}

////////////////////////////////////////////////////////////
//---------------------- EEPROM --------------------------//
////////////////////////////////////////////////////////////

void reinitialize()
{ 
  #ifdef SERIAL_ENABLED
    Serial.println("Resetting EEPROM");
  #endif
  eepromClear();
  setI2cAddress(I2C_UNALLOCATED_ADDRESS);
  for(int i = 0; i < 2; i++) {
    setLedBrightness(i,ledBrightness[i]);
    setLedMode(i,ledMode[i]);
    setLedRate(i,ledRate[i]);
    setLedSleep(i,ledSleep[i]);
  }

  setLedRGB(ledRGB[0],ledRGB[1],ledRGB[2]);
  setLedHSV(ledHSV[0],ledHSV[1],ledHSV[2]);
}

void eepromLoad() {
  #ifdef SERIAL_ENABLED
    Serial.println("Loading EEPROM");
  #endif
  
  byte tempAddress = EEPROM.read(EEPROM_I2C_ADDR);

  //check that a value has actually been set,
  //otherwise we use the hardware setting
  if(tempAddress != I2C_UNALLOCATED_ADDRESS) 
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

void setLedBrightness(byte led, byte brightness) {
  ledBrightness[constrain(led,0,1)] = brightness;
  EEPROM.put(EEPROM_BRT1_ADDR, ledBrightness[0]);
  EEPROM.put(EEPROM_BRT2_ADDR, ledBrightness[1]);
}

void setLedMode(byte led, byte mode) {
  ledMode[constrain(led,0,1)] = mode;
  EEPROM.put(EEPROM_MODE1_ADDR, ledMode[0]);
  EEPROM.put(EEPROM_MODE2_ADDR, ledMode[1]);
}

void setLedRate(byte led, byte rate) {
  ledRate[constrain(led,0,1)] = rate;
  EEPROM.put(EEPROM_RATE1_ADDR, ledRate[0]);
  EEPROM.put(EEPROM_RATE2_ADDR, ledRate[1]);
}

void setLedSleep(byte led, byte sleep) {
  ledSleep[constrain(led,0,1)] = sleep;
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
  for (int i = 0; i < EEPROM_USED_SIZE; i++) {
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
    //lastSerialTime = millis();
    oldMm = mm;

  #endif
}

////////////////////////////////////////////////////////////
//------------------------ MISC --------------------------//
////////////////////////////////////////////////////////////

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

void watchResetPin() {
  //check reset pin
  if(digitalRead(DISABLE_PIN) == LOW) {
    
    //crude debounce
    delay(100);
    
    if(digitalRead(DISABLE_PIN) == LOW) {
      //blink LEDs, if still shorted after this reset
      blinkLeds(5,CRGB::White);
      if(digitalRead(DISABLE_PIN) == LOW) {
        reinitialize();
        blinkLeds(1,CRGB::Green);
        restart();
      }
    }
  }
}

//enable the watchdog timer and loop infinitely to trigger a reset
void restart() {
  encWire.end();
  delay(10);
  wdt_enable(WDTO_500MS);
  for(;;);
}

