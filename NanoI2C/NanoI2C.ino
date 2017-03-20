#include <Wire.h>
#include "RPLidar.h"
RPLidar lidar;          // define lidar as RPLIDAR Object
#define RPLIDAR_MOTOR 3 // motor pin for lidar speed control

void setup() {
  pinMode(RPLIDAR_MOTOR, OUTPUT); // set lidar speed control pin as output
  lidar.begin(Serial);            // begin communication with lidar
  Wire.begin(8);                  // join i2c bus with address 8 as SLAVE
  Wire.onRequest(requestEvent);   // call "requestEvent" function when receives a request
  Wire.onReceive(receiveEvent);   // call "receiveEvent" function when receives a byte
}
int left = 0;                     // variable for detected points on left hand side
int right = 0;                    // variable for detected points on right hand side
unsigned long time0 = millis();    // time variable for reseting variables

byte c1;                           // variable for received integer
float amax = 90;
float amin = 90;
float aMinOld = 90;
float aMaxOld = 90;
float dmax = 0;
float dMaxOld = 0;

void receiveEvent(int bytes) {
  c1 =  Wire.read();    // read the received byte as integer.
}

void requestEvent() {
  // receive byte as a character


  if (c1 == '1') // if master requests a char ("1"), send right data
  {
    Wire.write(right);
  }
  if (c1 == '2') // if master requests a char ("2"), send left data
  {
    Wire.write(left);
  }

}

void loop() {

  if (IS_OK(lidar.waitPoint())) {                         // if lidar is working properly (wait point less than timeout)
    // read angle and distance
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle   = lidar.getCurrentPoint().angle;  // angle value in degree
    bool startBit = lidar.getCurrentPoint().startBit; //whether this point belongs to a new scan
    byte quality = lidar.getCurrentPoint().quality; // quality of the current measurement

    
    if ( distance < 2000 && distance > 100 ) {
      if ( angle > 0 && angle < 10 ) {

        right++;
      }
      if  ( angle > 350 && angle < 360) {
        left ++;
      }
`
    }
  


  // reset obstacle variables every 1 second
  if (millis() - time0 > 1000) { //
    time0 = millis();
    left = 0;
    right = 0;
  }

} else {                                                  // if lidar is not responding
  analogWrite(RPLIDAR_MOTOR, 0);                          //stop the rplidar motor
  rplidar_response_device_info_t info;                    // try to detect RPLIDAR...
  if (IS_OK(lidar.getDeviceInfo(info, 100))) {            // if detected,                         // Dont change this......
    lidar.startScan();                                    // start scan
    analogWrite(RPLIDAR_MOTOR, 255);                      // start motor rotating at max allowed speed (duty cycle =100%)
    delay(100);
  }
}
}


