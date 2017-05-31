/*
/---------------------------------------------------------------/
 * Code for the Aus3D Magnetic Encoder I2C Module
 * Chris Barr, 2016
 * 
 * This module is designed to connect to a host device via I2C,
 * and report the current position as observed by the magnetic encoder.
 * 
 * This code is compatible with both AMS AS5311 and NSE-5310 magnetic encoder ICs. The relevant IC must be defined below.
 * 
 * The following I2C commands are implemented:
 *    
 * 1: Resets travelled distance to zero
 * 
 * 2: Sets this modules I2C address to the next byte received, 
 *    saves in EEPROM
 *    Module resets and rejoins bus with new address
 *    
 * 3: Set the information that will be sent on the next I2C request.
 *      0: Default, responds with encoder count
 *      1: Responds with magnetic signal strength
 *            0: Signal good, in range.
 *            1: Signal weak, edge of range (but probably usable)
 *            2: Signal weak / lost, edge / outside of range (probably not usable)
 *      2: Responds with firmware version + compile date / time
 *      3: Responds with raw encoder reading
 *
 * 4: Clear the settings saved in EEPROM, restoring all defaults
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
 *              
 *              
 *  Requires Arduino on Breadboard hardware core from this page:
 *  https://www.arduino.cc/en/Tutorial/ArduinoToBreadboard
 *  
 *  For the ATmega328P with internal 8MHz clock
 /---------------------------------------------------------------/
 */

#define ENCODER_TYPE_AS_5311
//#define ENCODER_TYPE_NSE_5310

#include <Wire.h>

#if defined(ENCODER_TYPE_NSE_5310)
  #include <SoftwareWire.h>
#endif

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

#if defined(ENCODER_TYPE_NSE_5310)
  #define ENC_ADDR 0b1000000
  SoftwareWire encWire( ENC_DATA_PIN, ENC_CLOCK_PIN, true, true);
#endif

//I2C Slave Setup
#define ADDR1_PIN   16
#define ADDR2_PIN   17

//I2C Defines
#define I2C_MAG_SIG_GOOD  0
#define I2C_MAG_SIG_MID   1
#define I2C_MAG_SIG_BAD   2

#define I2C_REPORT_POSITION   0
#define I2C_REPORT_STATUS     1
#define I2C_REPORT_VERSION    2
#define I2C_REPORT_RAW        3

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
#define I2C_READ_BYTES 4

const byte i2c_base_address = I2C_ENCODER_PRESET_ADDR_X;
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

typedef union {
    volatile long val;
    byte bval[4];
}i2cLong;

i2cLong encoderCount;
i2cLong rawCount;

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
int ledMode[]       = {0,0};
int ledRate[]       = {0,0};
int ledSleep[]      = {0,0};

byte ledRGB[] = {255,0,0};
byte ledHSV[] = {255,255,255};

CRGB leds[PIXEL_NUM];

// This function is called upon a HARDWARE RESET:
void wdt_first(void) __attribute__((naked)) __attribute__((section(".init3")));
 
// Clear SREG_I on hardware reset. Make sure watchdog is disabled on new boot.
void wdt_first(void) {
  MCUSR = 0; // clear reset flags
  wdt_disable();
}

void setup() {
  #ifdef SERIAL_ENABLED
    Serial.begin(250000);
    Serial.println("Serial comms initialised");
  #endif

  //Configure pins
  pinMode(ENC_SELECT_PIN, OUTPUT);
  pinMode(ADDR1_PIN,INPUT_PULLUP);
  pinMode(ADDR2_PIN,INPUT_PULLUP);
  pinMode(DISABLE_PIN,INPUT_PULLUP);

  #if defined(ENCODER_TYPE_AS_5311)
    pinMode(ENC_DATA_PIN, INPUT);
    pinMode(ENC_CLOCK_PIN, OUTPUT);

    digitalWrite(ENC_CLOCK_PIN, HIGH);
    digitalWrite(ENC_SELECT_PIN, HIGH);
  #endif

  //Configure LEDs
  FastLED.addLeds<NEOPIXEL, PIXEL_PIN>(leds, PIXEL_NUM);
  FastLED.setBrightness(100);

  //Check reset pin for permanent disable
  if(digitalRead(DISABLE_PIN) == LOW) {
    //crude debounce
    delay(100);
    if(digitalRead(DISABLE_PIN) == LOW) {
      //Set enc_select low to enable encoder I2C with external controller
      digitalWrite(ENC_SELECT_PIN, LOW);
      blinkLeds(1,CRGB::White);
      //Halt startup and never proceed
      while(true);
    }
  }

  //Initialise communication with encoder IC
  if(initEncoder() == false) {
    #ifdef SERIAL_ENABLED
      Serial.print("Encoder Init Failed!");
    #endif

    while(initEncoder() == false) {
     blinkLeds(1,CRGB::Red); 
    }
  } else {
    blinkLeds(1,CRGB::Green);
  }

  //Read address pins as 2-bit number
  addressOffset = digitalRead(ADDR1_PIN) + 2*(digitalRead(ADDR2_PIN));

  //Set I2C address from value
  i2c_address = i2c_base_address + addressOffset;

  #ifdef SERIAL_ENABLED
    Serial.print("I2C Address: ");
    Serial.println(i2c_address);
  #endif

  //Initialise EEPROM if first boot, otherwise read stored values
  if (EEPROM.read(0) != SCHEMA) {
    reinitialize();
    EEPROM.write(0, SCHEMA);
    eepromLoad();
    blinkLeds(1,CRGB::Purple);
  } else {
    eepromLoad();
    blinkLeds(1,CRGB::Green);
  }

  //If a custom I2C address has been saved in EEPROM, 
  //it has now overridden the hardware value

  //Signal I2C address
  if(i2c_address >= I2C_ENCODER_PRESET_ADDR_X && i2c_address <= I2C_ENCODER_PRESET_ADDR_E) {
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
  } else {
    blinkLeds(1,CRGB::White);
  }
  
  delay(50);

  //Join I2C bus as slave
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
        if(magStrength == I2C_MAG_SIG_GOOD) { 
          leds[i] = CRGB::Green; 
        } else if(magStrength == I2C_MAG_SIG_MID) { 
          leds[i] = CRGB::Yellow; 
        } else if(magStrength == I2C_MAG_SIG_BAD) {
          leds[i] = CRGB::Red; 
        }
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
      Wire.write(encoderCount.bval,3);
      break;
    case I2C_REPORT_STATUS:
      Wire.write(magStrength);
      break;
    case I2C_REPORT_VERSION:
      reportVersion();
      break;
    case I2C_REPORT_RAW:
      rawCount.val = count;
      Wire.write(rawCount.bval,4);
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
  // example: 1.0.0, Jul 15 2016, 21:07:40
  String versionString = FIRMWARE_VERSION ", " __DATE__ ", " __TIME__ ".";
  Wire.write(versionString.c_str());
}

////////////////////////////////////////////////////////////
//--------------------- ENCODER --------------------------//
////////////////////////////////////////////////////////////
bool initEncoder() {
  #if defined(ENCODER_TYPE_NSE_5310)
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
  #else
    return true;
  #endif
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
  encoderCount.val = ((encoderCount.val &~((long)3 << 22)) | ((long)magStrength << 22)); //clear the upper two bits of the third byte, insert the two-bit magStrength value
}

int readPosition() {
  unsigned int position = 0;

  #if defined(ENCODER_TYPE_NSE_5310)
    //read in our data  
    digitalWrite(ENC_SELECT_PIN, LOW);
    encWire.requestFrom(ENC_ADDR,I2C_READ_BYTES,true);
  
    byte b[I2C_READ_BYTES];
  
    encWire.readBytes(b,5);
  
    digitalWrite(ENC_SELECT_PIN, HIGH);
  
    //get our position variable
    position = b[0];
    position = position << 8;
  
    position |= b[1];
    position = position >> 4;
  
    byte mIncrDecr      = bitRead(b[1],0);
    byte linAlarm       = bitRead(b[1],1);
    byte cordicOverflow = bitRead(b[1],2);
    byte offsetComp     = bitRead(b[1],3);
  
    //determine magnetic signal strength
    if(b[3] >= (0x3F-MAG_GOOD_RANGE) && b[3] <= (0x3F+MAG_GOOD_RANGE)) { 
      magStrength = I2C_MAG_SIG_GOOD; 
    } else if(b[3] >= 0x20 && b[3] <= 0x5F) { 
      magStrength = I2C_MAG_SIG_MID; 
    } else if(b[3] < 0x20 || b[3] > 0x5F) { 
      magStrength = I2C_MAG_SIG_BAD; 
    }
  #elif defined(ENCODER_TYPE_AS_5311)
    //shift in our data  
    digitalWrite(ENC_SELECT_PIN, LOW);
    delayMicroseconds(1);
    byte d1 = shiftIn(ENC_DATA_PIN, ENC_CLOCK_PIN);
    byte d2 = shiftIn(ENC_DATA_PIN, ENC_CLOCK_PIN);
    byte d3 = shiftIn(ENC_DATA_PIN, ENC_CLOCK_PIN);
    digitalWrite(ENC_SELECT_PIN, HIGH);
  
    //get our position variable
    position = d1;
    position = position << 8;
  
    position |= d2;
    position = position >> 4;
  
    if (!(d2 & B00001000)) {
      OCF = true;
    }
  
    if (!(d2 & B00000100)) {
      COF = true;
    }
  
    LIN = bitRead(d2,1);
    mINC = bitRead(d2,0);
    mDEC = bitRead(d3,7);

    //determine magnetic signal strength
    if(mINC == false && mDEC == false) { magStrength = I2C_MAG_SIG_GOOD;  }
    if(mINC == true && mDEC == true && LIN == false) { magStrength = I2C_MAG_SIG_MID;  }
    if(mINC == true && mDEC == true && LIN == true) { magStrength = I2C_MAG_SIG_BAD;  }
  #endif

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

void reinitialize() {
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
  #if defined(ENCODER_TYPE_NSE_5310)
    encWire.end();
  #endif
  delay(10);
  wdt_enable(WDTO_250MS);
  for(;;);
}

