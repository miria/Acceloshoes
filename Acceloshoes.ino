/*

Acceloshoes
by Miria Grunick

This is the Arduino code for my Acceloshoes project. The shoes 
have an accelerometer and magnetometer module which control the 
brightness and color of a NeoPixel LED. The shoes currently have
three modes:

1. Compass Only Mode: The brightness is fixed and the color is 
determined by the compass heading (imagine overlaying a compass
on a color wheel.

2. Accelerometer Only Mode: Each access is assigned one of the RGB
colors (X = red, Y = blue, Z = green). The brightness is based on 
the amount of change of the accelerometer reading. 

3. Accelerometer and Compass Mode: The color is determined by the 
compass heading. The brightness is determined by the maximum 
amount of change on any axis of the accelerometer. 

This code was written for the Adafruit Trinket.  The button pins
will need to change if you want to use a different version of 
Arduino. This code is based on various bits of sample code from 
the Adafruit sensor libraries.

This is free to use and modify!

*/


#include <Bounce.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_NeoPixel.h>
#include "SPI.h"

const int PIXEL_PIN = 10;
const int BUTTON_PIN = 2;
const int NUM_ACCEL_READINGS_AVG = 10;

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

Bounce pushbutton = Bounce(BUTTON_PIN, 500);

float maxBrightness = 1.0;
float readingsX[NUM_ACCEL_READINGS_AVG];
float readingsY[NUM_ACCEL_READINGS_AVG];
float readingsZ[NUM_ACCEL_READINGS_AVG];

int curIdx = 0;
int buttonState = 0;


void changeState()
{
  buttonState +=  1; 
  if (buttonState == 3) 
  {
    buttonState = 0; 
  } 
        
  if (buttonState == 0)
    flashPixel(127, 0, 0);
  if (buttonState == 1)
    flashPixel(0, 127, 0);
  if (buttonState == 2)
    flashPixel(0, 0, 127);
}

void setup() 
{
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  
  if(!mag.begin())
  {
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!accel.begin())
  {
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  
  // Initialize the readings arrays
  for(int i=0; i<NUM_ACCEL_READINGS_AVG;i++) 
  {
    readingsX[i] = 0.0;
    readingsY[i] = 0.0;
    readingsZ[i] = 0.0;
  }  
  strip.begin();
  strip.show(); 
  
  Serial.println("DONE INITIALIZING!");
}

void loop() 
{

  if (pushbutton.update()) 
  {
    if (pushbutton.fallingEdge()) 
    {
      changeState();
    } 
  }
  Serial.print("State= "); Serial.println(buttonState);
  
  maxBrightness = analogRead(A3)/1023.0;
  Serial.print("Max brightness= ");
  Serial.println(maxBrightness);
  
  if (buttonState == 0) 
  {
    compassOnlyMode();
  } 
  else if (buttonState == 1) 
  {
    accelerometerOnlyMode();  
  } 
  else if (buttonState == 2) 
  {
    compassAccelerometerMode();  
  }
  delay(100);
  
}

// Brightness is constant, color is determined by compass heading.
void compassOnlyMode() 
{
  int colors[3];
  getColors(getCompassHeading(), colors);
  strip.setPixelColor(0, colors[0], colors[1], colors[2]);
  strip.show();
}

// Brightness is determined by accelerometer delta, color is 
// by axis: x = red, y = green, z = blue
void accelerometerOnlyMode() 
{
  float accelDeltas[3];
  getAccelerometerData(accelDeltas);
  
  float brightness[3];
  for (int i=0; i<3; i++) 
  {
    brightness[i] = maxBrightness*getBrightness(accelDeltas[i]);
  }
  
  strip.setPixelColor(0, 127*brightness[0], 127*brightness[1], 127*brightness[2]);
  strip.show();
}

// Brightness is determined by accelerometer delta, color is
// determined my compass heading
void compassAccelerometerMode() 
{
  float heading = getCompassHeading();
  float accelDeltas[3];
  getAccelerometerData(accelDeltas);
  
  float maxDelta = 0;
  for (int i=0; i < 3; i++) 
  {
    if (accelDeltas[i] > maxDelta) 
    {
      maxDelta = accelDeltas[i];
    }
  }
  float brightness = getBrightness(maxDelta);
  
  int colors[3];
  getColors(getCompassHeading(), colors);
  
  strip.setPixelColor(0, colors[0]*brightness, colors[1]*brightness, colors[2]*brightness);
  strip.show();
  
}

float getCompassHeading() 
{
  sensors_event_t event;
  mag.getEvent(&event);
  float Pi = 3.14159;
  // Calculate the angle of the vector y,x  
  float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
  // Normalize to 0-360
  if (heading < 0) 
  {
    heading = 360 + heading;
  }
  
  Serial.print("Compass heading: "); Serial.println(heading); 
  return heading;
}

void flashPixel(int r, int g, int b) 
{
  strip.setPixelColor(0, r, g, b);
  strip.show();
  delay(1000);
  strip.setPixelColor(0, 0, 0, 0);
  strip.show();
}

float getBrightness(float delta) 
{
  float brightness = delta / 10.0;
  brightness = min(brightness, 100.0);
  return brightness; 
}

void getAccelerometerData(float deltas[])
{
  sensors_event_t event; 
  accel.getEvent(&event);
  
  float xAccel = event.acceleration.x;
  float yAccel = event.acceleration.y;
  float zAccel = event.acceleration.z;
 
  float sumX = 0.0;
  float sumY = 0.0;
  float sumZ = 0.0;
  
  for (int i=0; i<NUM_ACCEL_READINGS_AVG; i++) 
  {
     sumX += readingsX[i]; 
     sumY += readingsY[i]; 
     sumZ += readingsZ[i]; 
  }
  deltas[0] = (sumX/NUM_ACCEL_READINGS_AVG) / xAccel;
  deltas[0] = abs(deltas[0]);
  deltas[1] = (sumY/NUM_ACCEL_READINGS_AVG) / yAccel;
  deltas[1] = abs(deltas[1]);
  deltas[2] = (sumZ/NUM_ACCEL_READINGS_AVG) / zAccel;
  deltas[2] = abs(deltas[2]);
  
  readingsX[curIdx] = xAccel;
  readingsY[curIdx] = yAccel;
  readingsZ[curIdx] = zAccel;
  
  curIdx++;
  if (curIdx >= NUM_ACCEL_READINGS_AVG)
    curIdx = 0;
    
  Serial.print("Delta X: "); Serial.print(deltas[0]); Serial.print(" ");
  Serial.print("Y: "); Serial.print(deltas[0]);       Serial.print(" ");
  Serial.print("Z: "); Serial.println(deltas[0]);     Serial.print(" ");
}

void getColors(int WheelPos, int colors[])
{
  switch(WheelPos / 128)
  {
    case 0:
      colors[0] = ((127 - WheelPos % 128)*2*maxBrightness);   //Red down
      colors[1] = (WheelPos % 128)*2*maxBrightness;      // Green up
      colors[2] = 0;                  //blue off
      break; 
    case 1:
      colors[0] = (127 - WheelPos % 128)*2*maxBrightness;  //green down
      colors[1] = (WheelPos % 128)*2*maxBrightness;      //blue up
      colors[2] = 0;                  //red off
      break; 
    case 2:
      colors[0] = (127 - WheelPos % 128)*2*maxBrightness;  //blue down 
      colors[1] = WheelPos % 128*2*maxBrightness;      //red up
      colors[2] = 0;                  //green off
      break; 
  }

}
