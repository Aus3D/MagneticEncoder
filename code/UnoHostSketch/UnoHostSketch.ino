#include <Wire.h>

#define TICKS_PER_MM 2048

typedef union{
    volatile long val = 0;
    byte bval[4];
}i2cLong;

i2cLong encoderCount;

#define ADDR 0x30

#define I2C_REPORT_POSITION     0
#define I2C_REPORT_STATUS       1
#define I2C_REPORT_VERSION      2

#define I2C_REPORT_ENC_AGC      20
#define I2C_REPORT_ENC_STATUS   21
#define I2C_REPORT_ENC_RAW      22

#define I2C_REQ_REPORT          0
#define I2C_RESET_COUNT         1
#define I2C_SET_ADDR            2
#define I2C_SET_REPORT_MODE     3
#define I2C_CLEAR_EEPROM        4

#define I2C_ENC_LED_PAR_MODE    10
#define I2C_ENC_LED_PAR_BRT     11
#define I2C_ENC_LED_PAR_RATE    12
#define I2C_ENC_LED_PAR_RGB     13
#define I2C_ENC_LED_PAR_HSV     14

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(250000);  // start serial for output

  Wire.beginTransmission(ADDR);
  byte error = Wire.endTransmission();

  if(error == 0) {
    Serial.println("Module found at address!");
  } else {
    Serial.println("Error! Module not detected at address!");
  }

  //delay(3000);
  //setLedParameter(10,0,1);
  //setLedParameter(10,1,1);
  //setLedParameter(11,0,5);
  //setLedParameter(11,1,5);

  //delay(500);
  //setLedParameter(10,0,0);
  //setLedParameter(10,1,0);
  //setLedParameter(11,0,50);
  //setLedParameter(11,1,50);

  //changeAddr(31);
  
}

void loop() {
  static float mm=0;
  static float prevMm=0;
  static long lastLoopTime = 0;
  static long avgSpeed = 0;
  //Wire.beginTransmission(ADDR);
  //Wire.write(0);
  //Wire.endTransmission();


  #define READNUM 4
  byte recvd[READNUM];

  
  
  Wire.requestFrom(ADDR,READNUM);
  
  byte i = 0;


  
  while (Wire.available()) { // slave may send less than requested
    byte c = Wire.read(); // receive a byte as character
    //Serial.print(c);
    //Serial.print(" ");
    encoderCount.bval[i] = c;
    //Serial.print(c);         // print the character
    recvd[i] = c;
    i++;
  }

  for(int j = 0; j < READNUM; j++) {
    Serial.print(" | ");
    printBinaryFormatted(recvd[j]);
  }

  

  Serial.print(" | ");
  printDecFormatted(encoderCount.val,6);
  //Serial.print(encoderCount.val);

 // prevMm = mm;
  //mm = (float) (encoderCount.val) /TICKS_PER_MM;

  

  //Serial.print(" | ");
  //Serial.print(mm);

 // static long loopTime = lastLoopTime - millis();
 // lastLoopTime = millis();

  //long instSpeed = abs(prevMm-mm)/(loopTime/1000);
  //avgSpeed = runningAverage(instSpeed);

  //Serial.print(" | ");
 // Serial.print(instSpeed);
  
  //Serial.print(" | ");
  //Serial.println(avgSpeed);

  long anglePre = encoderCount.val * 45;
  float angle = (anglePre / 512.0);// * (45 / 512));
  Serial.print(" | ");
  Serial.print(angle,4);

  

  //change report mode
  Wire.beginTransmission(ADDR);
  Wire.write(I2C_SET_REPORT_MODE);
  Wire.write(I2C_REPORT_ENC_AGC);
  Wire.endTransmission();

  Wire.requestFrom(ADDR,1);

  while (Wire.available()) { // slave may send less than requested
    byte c = Wire.read(); // receive a byte as character
    Serial.print(" | ");
    printDecFormatted(c,3);
    //Serial.print(c);
  }

    //change report mode
  Wire.beginTransmission(ADDR);
  Wire.write(I2C_SET_REPORT_MODE);
  Wire.write(I2C_REPORT_ENC_RAW);
  Wire.endTransmission();

  Wire.requestFrom(ADDR,2);

  byte rawCount[2];
  i = 0;

  while (Wire.available()) { // slave may send less than requested
    rawCount[i] = Wire.read(); // receive a byte as character
    Serial.print(" | ");
    printBinaryFormatted(rawCount[i]);
    //Serial.print(c,BIN);
    i++;
  }

  //change report mode
  Wire.beginTransmission(ADDR);
  Wire.write(I2C_SET_REPORT_MODE);
  Wire.write(0x25);
  Wire.endTransmission();

  Wire.requestFrom(ADDR,4);

  while (Wire.available()) { // slave may send less than requested
    byte c = Wire.read(); // receive a byte as character
    Serial.print(" | ");
    printDecFormatted(c,2);
    //Serial.print(c);
  }

  //change report mode
  Wire.beginTransmission(ADDR);
  Wire.write(I2C_SET_REPORT_MODE);
  Wire.write(I2C_REPORT_POSITION);
  Wire.endTransmission();


  long angleCount = (rawCount[0] << 8) | rawCount[1];
  Serial.print(" | ");
  printDecFormatted(angleCount,4);
  
/*
  for(int i = 0; i < 255; i++) {
    Wire.beginTransmission(ADDR);
    Wire.write(11);   //set mode of
    Wire.write(0);    //led #0
    Wire.write(i); //to mode #
    Wire.endTransmission();
    delay(1);
  }

  for(int i = 255; i > 0; i--) {
    Wire.beginTransmission(ADDR);
    Wire.write(11);   //set mode of
    Wire.write(0);    //led #0
    Wire.write(i); //to mode #
    Wire.endTransmission();
    delay(1);
  }
  */

  Serial.println();
  delay(50);
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

void setLedParameter(byte param, byte led, byte val) {
  Wire.beginTransmission(ADDR);
  Wire.write(param);
  Wire.write(led);
  Wire.write(val);
  Wire.endTransmission();
  delay(100);
}

void setRGB(byte r, byte g, byte b) {
  Wire.beginTransmission(ADDR);
  Wire.write(12);
  Wire.write(r);
  Wire.write(g);
  Wire.write(b);
  Wire.endTransmission();
}

void changeAddr(byte newAddr) {
  Wire.beginTransmission(ADDR);
  Wire.write(I2C_SET_ADDR);
  Wire.write(newAddr);
  Wire.endTransmission();
}

void printBinaryFormatted(int input) {
  for (int k = 0; k < 8; k++)
    {
      if (input < pow(2, k))
        Serial.print(B0);
    }
    if(input != 0)
      Serial.print(input,BIN);
}

void printDecFormatted(int input, int numSpaces) {

  if(input < 0) {
    numSpaces -= 1;
  }
  
  for (int k = 0; k < numSpaces; k++)
  {
    if (abs(input) < pow(10, k))
      Serial.print(" ");
  }
  if(input != 0)
    Serial.print(input);
}

