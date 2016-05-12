#include <Wire.h>

#define TICKS_PER_MM 2048

typedef union{
    volatile long val = 0;
    byte bval[4];
}i2cLong;

i2cLong encoderCount;

#define ADDR 30

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output

  delay(3000);
  setLedParameter(10,0,1);
  setLedParameter(10,1,1);
  setLedParameter(11,0,5);
  setLedParameter(11,1,5);

  delay(500);
  setLedParameter(10,0,0);
  setLedParameter(10,1,0);
  setLedParameter(11,0,50);
  setLedParameter(11,1,50);
  
}

void loop() {
  static float mm=0;
  static float prevMm=0;
  static long lastLoopTime = 0;
  static long avgSpeed = 0;
  //Wire.beginTransmission(ADDR);
  //Wire.write(0);
  //Wire.endTransmission();

  Wire.requestFrom(ADDR,4);
  
  byte i = 0;
  
  while (Wire.available()) { // slave may send less than requested
    byte c = Wire.read(); // receive a byte as character
    Serial.print(c);
    Serial.print(" ");
    encoderCount.bval[i] = c;
    //Serial.print(c);         // print the character
    i++;
  }

  Serial.print(" | ");
  Serial.print(encoderCount.val);

  prevMm = mm;
  mm = (float) (encoderCount.val) /TICKS_PER_MM;

  Serial.print(" | ");
  Serial.print(mm);

  static long loopTime = lastLoopTime - millis();
  lastLoopTime = millis();

  long instSpeed = abs(prevMm-mm)/(loopTime/1000);
  avgSpeed = runningAverage(instSpeed);

  Serial.print(" | ");
  Serial.print(instSpeed);
  
  Serial.print(" | ");
  Serial.println(avgSpeed);

  delay(10);
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

