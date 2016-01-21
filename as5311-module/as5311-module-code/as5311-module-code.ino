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
 /---------------------------------------------------------------/
 */
#include <Wire.h>
#include <EEPROM.h>
#include "FastLED.h"

#define SCHEMA 0x0101

#define ADDR1_PIN 9
#define ADDR2_PIN 10

#define MAGINCN_PIN 14
#define MAGDECN_PIN 15

#define WS2812_PIN 16
#define WS2812_NUM 2

#define EEPROM_I2C_ADDR 1
#define EEPROM_BRT_ADDR 2

const byte i2c_base_address = 30;
byte i2c_address;

typedef union{
    volatile long val = 0;
    byte bval[4];
}i2cLong;

i2cLong encoderCount;
unsigned char temp;

byte magINCn;
byte magDECn;
byte magState;
byte magStatePrev = 3;

byte ledBrightness = 128;

CRGB leds[WS2812_NUM];

void setup() {
   
  delay(500);

  //Set device address, join I2C bus
  //set address select pins to pullup
  pinMode(ADDR1_PIN,INPUT_PULLUP);
  pinMode(ADDR2_PIN,INPUT_PULLUP);


  //set encoder strength indicators to pullup
  pinMode(MAGINCN_PIN,INPUT_PULLUP);
  pinMode(MAGDECN_PIN,INPUT_PULLUP);

  attachInterrupt(0,encoderIsr,CHANGE);
  attachInterrupt(1,encoderIsr,CHANGE);

  delay(500);

  i2c_address = i2c_base_address + (digitalRead(ADDR1_PIN) + digitalRead(ADDR2_PIN)*2);

  short schema;
  EEPROM.get(0, schema);
  if (schema != SCHEMA) {
    reinitialize();
    schema = SCHEMA;
    EEPROM.write(0, schema);
  } else {
    i2c_address = EEPROM.read(EEPROM_I2C_ADDR);
    ledBrightness = EEPROM.read(EEPROM_BRT_ADDR);
  }

  Wire.begin(i2c_address);
  Wire.onRequest(requestEvent);

  FastLED.addLeds<WS2812B, WS2812_PIN, RGB>(leds, WS2812_NUM);

  //signal which i2c address the module identifies as
  for(int i = 0; i < 2; i++) {
    switch(i2c_address) {
      case i2c_base_address:
        fill_solid( &(leds[0]), WS2812_NUM, CRGB::White);
        break;
      case i2c_base_address + 1:
        fill_solid( &(leds[0]), WS2812_NUM, CRGB::Lime);
        break;
      case i2c_base_address + 2:
        fill_solid( &(leds[0]), WS2812_NUM, CRGB::Blue);
        break;
      case i2c_base_address + 3:
        fill_solid( &(leds[0]), WS2812_NUM, CRGB::Yellow);
        break;
    }
    FastLED.show();
    delay(500);
    
    fill_solid( &(leds[0]), WS2812_NUM, CRGB::Black);
    LEDS.setBrightness(ledBrightness);
    FastLED.show();
    delay(500);
    
  }

}

void loop() {
  delay(100);
  
  magnetStatus();

  
}

void requestEvent() {
  temp = Wire.read();

  switch(temp) {
    case 1: encoderCount.val = 0;
      break;
    case 2:
      setI2cAddress(Wire.read());
      break;
    case 3:
      setLedBrightness(Wire.read());
      LEDS.setBrightness(ledBrightness);
      FastLED.show();
      break;
    case 4:
      Wire.write(magState); 
      break;
    default:
      Wire.write(encoderCount.bval[0]); 
      Wire.write(encoderCount.bval[1]); 
      Wire.write(encoderCount.bval[2]); 
      Wire.write(encoderCount.bval[3]); 
      break;
      
  }

}

//displays magnetic signal strength on LED
void magnetStatus() {
  
  magINCn = digitalRead(MAGINCN_PIN);
  magDECn = digitalRead(MAGINCN_PIN);

  magState = magINCn + magDECn;

  //only update the LED if it needs to change, to avoid flickering
  if(magState != magStatePrev) {
    if(magState == 0) 
      fill_solid( &(leds[0]), WS2812_NUM, CRGB::Green);
    else if(magState == 1) 
      fill_solid( &(leds[0]), WS2812_NUM, CRGB::Yellow);
    else if(magState == 2) 
      fill_solid( &(leds[0]), WS2812_NUM, CRGB::Red);

    FastLED.show();
  }

  magStatePrev = magState;
  
}

void encoderIsr() {
    static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    static uint8_t enc_val = 0;
    
    enc_val = enc_val << 2;
    enc_val = enc_val | ((PIND & 0b1100) >> 2);
 
    encoderCount.val = encoderCount.val + lookup_table[enc_val & 0b1111];
}

void setI2cAddress(byte i2cAddress) {
  EEPROM.put(1, i2cAddress);
}

void setLedBrightness(byte brightness) {
  ledBrightness = brightness;
  EEPROM.put(2, ledBrightness);
}

void reinitialize()
{
  setI2cAddress(i2c_address);
  setLedBrightness(ledBrightness);
}

