

#include "Adafruit_BNO055.h"
#include "FlexiTimer2.h"
#include "Adafruit_Sensor.h"
#include <Wire.h>
#include <LiquidCrystal.h>
#include "imumaths.h"
#include <Servo.h>
#include "Adafruit_GPS.h"

Adafruit_GPS GPS(&Serial3);                   // define GPS object
Servo myservo;                                // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // define BNO sensor object
LiquidCrystal lcd( 8, 9, 4, 5, 6, 7); // define lcd pins use these default values for OUR LCD

#define GPSECHO  false
#define lThreshold 5                          // Lidar Threshold
#define dThreshold 10                         // GPS Wall Threshold
#define btnSELECT 4
#define btnUP     1


// Global variables that change across functions
int SLAVE_ADDRESS = 8;                          // I2C slave(nano) address
float Bearing = 0;
int STEERANGLE = 90;                            // servo initial angle (range is 0:180)
float HEADING = 0;     
  int val =0; // heading
int16_t LidarRight;                                 // LIDAR left
int16_t LidarLeft;                                  // LIDAR right
boolean usingInterrupt = false;                 // Using interrupt for reading GPS chars
int carSpeedPin = 2;                            // pin for DC motor (PWM for motor driver)
float errorHeadingRef = 0;                      // Heading error
long int lat = 33.420887 * 100000;              // GPS latitude in degree decimal * 100000 (CURRENT POSITION)
long int lon = -111.934089 * 100000;            // GPS latitude in degree decimal * 100000 (CURRENT POSITION)
long int latDestination = 33.421620 * 100000;   // reference destination (INITIAL DESTINATION)
long int lonDestination = -111.930118 * 100000; // reference destination (INITIAL DESTINATION)


/*
// You need to find an interseption point between  the wall and your intersection line ... 
// you need to add values to x and y right now is zero so you need to do the calculation for the interseption point 


///////////////////////////////////////// Boundary points  //////////////////////////////////////////

long int latPoint1 = 33.421788 * 100000;     // reference destination (Point1)
long int lonPoint1 = -111.934501 * 100000;   // reference destination (Point1)

long int latPoint2 = 33.421806* 100000;     // reference destination (Point2)
long int lonPoint2 =   -111.933594 * 100000;  // reference destination (Point2)

long int latPoint3 = 33.421488 * 100000;     // reference destination (Point3)
long int lonPoint3 = -111.933777 * 100000;   // reference destination (Point3)

long int latPoint4 = 33.421162 * 100000;     // reference destination (Point4)
long int lonPoint4 =  -111.933810 * 100000;   // reference destination (Point4)

long int latPoint5 = 33.421157 * 100000;     // reference destination (Point5)
long int lonPoint5 =  -111.934169 * 100000;   // reference destination (Point5)

long int latPoint6 = 33.421461 * 100000;     // reference destination (Point6)
long int lonPoint6 =  -111.934169 * 100000;   // reference destination (Point6)

/////////////////////////////////////////////////////////////////////////////////////////////////////

*/

int lcd_key     = 0;
int x; 
int adc_key_in  = 0;

void setup() {
  Wire.begin();
  myservo.attach(44);                         // servo is connected to pin 44
  lcd.begin( 16, 2 );                         // LCD type is 16x2 (col & row)
  Serial.begin(9600);                         // serial for monitoring
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }

  byte c_data[22] = {0, 0, 0, 0, 0, 0, 209, 4, 9, 5, 9, 6, 0, 0, 255, 255, 255, 255, 232, 3, 1, 3};               // YOUR Calibration DATA
  bno.setCalibData(c_data);                                                                                       // SET CALIBRATION DATA
  bno.setExtCrystalUse(true);

  // set timer interrupts
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 59016;           // every 0.1 second
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer compare interrupt

  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 336;             // every 1 second
  TCCR4B |= (1 << CS42);    // 256 prescaler
  TIMSK4 |= (1 << TOIE4);   // enable timer compare interrupt
  interrupts();

  // initialize GPS module
  GPS.begin(9600);
   GPS.sendCommand("$PGCMD,33,0*6D"); // Turn Off GPS Antenna Update
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // set RMC & GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);               // notify if antenna detected
  useInterrupt(true);                           // use interrupt for reading gps data
 
}

SIGNAL(TIMER0_COMPA_vect) {                   // Interrupt for reading GPS data. Don't change this...
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
#endif
}

void useInterrupt(boolean v) {                // Interrupt for reading GPS data. Don't change this...
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

ISR(TIMER4_OVF_vect) {      // This function is called every 1 second ....
  sei();                    // set interrupt flag ********VERY IMPORTANT******* you need to set the interrupt flag or programm will stuck here!!!  
  TCNT4  = 336;             //   re-initialize timer value
  GPSRead();                //   read GPS data
}
//double X;
//double Y;
//double FV_x;
//double FV_y;
//double forceX=0;
//double forceY=0;


void GPSRead() {
  boolean dist_state;
    if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
  if (GPS.fix) {
     lat= (GPS.latitudeDegrees,6) * 100000  ;                            // read GPS Latitude in degree decimal  
     lon= (GPS.longitudeDegrees,6) * 100000  ;                            // read GPS Longitude in degree decimal   lat= (GPS.latitudeDegrees,6) * 100000  ;      


  }
}

int read_LCD_buttons(){               // read the buttons
  adc_key_in = analogRead(0); 

if (adc_key_in <790) return btnSELECT; 

  if (adc_key_in < 250 ) return btnUP; 
}







                              // Calculate Distance from each Wall
 /* //wall-1
   dist_state= DistanceState(lat,lon,latPoint1,lonPoint1,latPoint2,lonPoint2);                           // if distance is greater than threshold
     if(dist_state== true){
    FV_x =  -(X-lat);                         
    FV_y =  -(Y-lat);   }                       
    else{
      FV_x=0;
      FV_y=0;
    }
    forceY= forceY+ FV_y;                     // calculate force vector
    forceX= forceX+ FV_x;                     // calculate force vector


    
    //wall-2
   dist_state= DistanceState(lat,lon,latPoint2,lonPoint2,latPoint3,lonPoint3);                           // if distance is greater than threshold
     if(dist_state== true){
    FV_x =  -(X-lat);                         
    FV_y =  -(Y-lat);   }                       
    else{
      FV_x=0;
      FV_y=0;
    }
    forceX= forceX+ FV_x;                     // calculate force vector
    forceY= forceY+ FV_y;                     // calculate force vector

    
    //wall-3
   dist_state= DistanceState(lat,lon,latPoint3,lonPoint3,latPoint4,lonPoint4);                           // if distance is greater than threshold
     if(dist_state== true){
    FV_x =  -(X-lat);                         
    FV_y =  -(Y-lat);   }                       
    else{
      FV_x=0;
      FV_y=0;
    }
    forceX= forceX+ FV_x;                     // calculate force vector
    forceY= forceY+ FV_y;                     // calculate force vector


    
    //wall-4
   dist_state= DistanceState(lat,lon,latPoint4,lonPoint4,latPoint5,lonPoint5);                           // if distance is greater than threshold
     if(dist_state== true){
    FV_x =  -(X-lat);                         
    FV_y =  -(Y-lat);   }                       
    else{
      FV_x=0;
      FV_y=0;
    }
    forceX= forceX+ FV_x;                     // calculate force vector
    forceY= forceY+ FV_y;                     // calculate force vector


    
    //wall-5
   dist_state= DistanceState(lat,lon,latPoint5,lonPoint5,latPoint6,lonPoint6);                           // if distance is greater than threshold
     if(dist_state== true){
    FV_x =  -(X-lat);                         
    FV_y =  -(Y-lat);   }                       
    else{
      FV_x=0;
      FV_y=0;
    }
    forceX= forceX+ FV_x;                     // calculate force vector
    forceY= forceY+ FV_y;                     // calculate force vector
   
    //wall-6
   dist_state= DistanceState(lat,lon,latPoint6,lonPoint6,latPoint1,lonPoint1  );                         // if distance is greater than threshold
     if(dist_state== true){
    FV_x =  -(X-lat);                         
    FV_y =  -(Y-lat);   }                       
    else{
      FV_x=0;
      FV_y=0;
    }
    forceX= forceX+ FV_x;                     // calculate force vector
    forceY= forceY+ FV_y;                     // calculate force vector
    Serial.print(forceX);
    Serial.print("\t");
    Serial.println(forceY);
    
    latDestination = lat+forceY;                          // Set new Destination
    lonDestination = lon+forceX;                         // Set new Destination
  }
  */
//}
/*
double distance;
double k; 
double CalculateDirectionPerpendicularX(double x1, double  y1, double  x2, double  y2, double  x3, double y3) {     // Function to Calculate Horizental vector ---INPUTs:( Current x, Current y, Point1 (x). Point1 (y), Point2 (x), Point2 (y) )


  k = ((y3-y2) * (x1-x2) - (x3-x2) * (y1-y2)) / (sq(y3-y2) + sq(x3-x2));
  distance = x1 - k * (y3-y2);
  return distance;                          // output of function is direction along x-axis
}



double CalculateDirectionPerpendicularY(double x1, double  y1, double  x2, double  y2, double  x3, double y3) {     // Function to Calculate Vertical vector   ---INPUTs:( Current x, Current y, Point1 (x). Point1 (y), Point2 (x), Point2 (y) )

  
  k = ((y3-y2) * (x1-x2) - (x3-x2) * (y1-y2)) / (sq(y3-y2) + sq(x3-x2));
  distance = y1 + k * (x3-x2);
  return distance ;                          // output of function is direction along y-axis
}


double CalculateDistancePerpendicular(double x1, double  y1, double  x2, double  y2, double  x3, double y3) {       // Function to calculate distance from the wall --- INPUTs:( Current x, Current y, Point1 (x). Point1 (y), Point2 (x), Point2 (y) )


double  M = (y3-y2)/(x3-x2);
 distance=abs(y1-(M*x1)-y2+(M*x2))/sqrt(1+(M*M));
  return distance;                         // output of function is distance from the wall

}
/*
boolean DistanceState(double x1, double  y1, double  x2, double  y2, double  x3, double y3){
  double dist;
  boolean state;
  X=CalculateDirectionPerpendicularX(x1, y1,   x2,  y2,   x3,  y3);
  Y=CalculateDirectionPerpendicularY( x1,  y1,  x2,  y2,   x3,  y3);
  if((X>= min(x2,x3) && X<= max(x2,x3)) && (Y >= min(y2,y3) && Y<= max(y2,y3))){
    dist= CalculateDistancePerpendicular( x1,  y1,   x2,   y2,  x3, y3);
    if (dist<=dThreshold) return true;
    else return false; }
    else return false;
    }
*/
void ReadMag() { // Output: HEADING
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);     // read sensor value
  HEADING = euler.x() ;                       // add bias to your measurement since the sensor direction matters. ( we should substract 10 degree from measured data since the actual north and magnetic north has 10 degree difference in phoenix)
  if (HEADING < 0) HEADING = HEADING + 360;   // scale to default range [0 - 360]
}

void CalculateBearing() {               // Calculate Bearing
  float theta = atan2( latDestination - lat, lonDestination - lon); 

  if ( 0 < theta < 90 ) {
  Bearing = (90 - theta); 
  }else if 
    (-90 < theta <0) {
      Bearing = 90 + abs(theta); 
      
      }else if 
      (-180 < theta < -90) {
        Bearing = 90 + abs(theta); 
        }else
        Bearing = 360 - (theta - 90);

 if ((HEADING-Bearing) > 0 && abs(HEADING-Bearing) <180){ 
  val=60; // turn left
   
  }
  if ((HEADING-Bearing) > 4 && abs(HEADING-Bearing) > 180) {
  val=120;    // turn right
 
  }

   if ((HEADING-Bearing) < 0 && abs(HEADING-Bearing) < 180) {
  val=120;    // turn right
 
  }
 if ((HEADING-Bearing) <4  && abs(HEADING-Bearing) > 180) {
  val=60;    // turn right
 
  }

   if (abs(HEADING-Bearing) < 4 ) {
  val=90;    // turn right
 
  }
}


/*void CalculateSteer() { // Input: HEADING // Output: STEERANGLE
 if((HEADING - 180) > 0){ // Correction for the angle number displayed:
    STEERANGLE = 360 - HEADING;
   
  }
  else {
    STEERANGLE = (-1)*HEADING; 
  }
}
*/


void SetCarDirection() {                                                    // Set direction regarding LIDAR
  ReadLidar();                                                              // read lidar
  if ( LidarRight > LidarLeft && LidarRight > lThreshold ){
    val = 60; 
    }
   if ( LidarLeft > LidarRight && LidarLeft> lThreshold ){
    val = 120; 
    }
                                                                            // if no obstacle is detected, drive towards destination
  myservo.write(val);

}

void SetCarSpeed() {                                                        // set car's speed

    analogWrite(carSpeedPin, 25 );                                    // set the pwm duty cycle
}

void ReadLidar() {                                                          // Read Lidar data 
  Wire.beginTransmission(8);                                                // read left side data
  Wire.write('1');                
  Wire.endTransmission();
  Wire.requestFrom(SLAVE_ADDRESS, 1);
  while(Wire.available()==0);
  LidarRight= Wire.read();
//   Serial.print("right angle is :  ");
//  Serial.print(LidarRight);
//   Serial.print("\t"); 
  
  Wire.beginTransmission(8);                                                // read right side data
  Wire.write('2');                
  Wire.endTransmission();
  Wire.requestFrom(SLAVE_ADDRESS, 1);
  while(Wire.available()==0);
  LidarLeft= Wire.read();
//  Serial.print("left angle is :  ");
//  Serial.println(LidarLeft);
     
     

  if (100 - LidarRight < LidarLeft - 80){
//   Serial.println("obstacle is on the left turn the robot right");

  } else {
//   Serial.println("obstacle is on the right, turn the robot left");
}}

ISR(TIMER1_OVF_vect) {        // This function is called every 0.1 seconds
  sei();                      // set interrupt flag ********VERY IMPORTANT******* you need to set the interrupt flag or programm will stuck here!!!
  TCNT1  = 59016;
  ReadMag();                                                                // Read Heading
  CalculateBearing();                                                       // Calculate Bearing 
//  CalculateSteer();                                                         // Calculate Steer angle 
  SetCarDirection();                                                        // Set steer angle
  SetCarSpeed();                                                            // Set Car speed
}


void printHeadingOnLCD() {
  lcd.setCursor( 0, 1 );
//  lcd.print(forceX);       // print heading data on LCD
  lcd.setCursor( 0, 0 );
  //lcd.print(forceY);       // print heading data on LCD
}

void printLocationOnLCD() {
  lcd.setCursor( 7, 0 );
  lcd.print(lat);           // print latitude data on LCD
  lcd.setCursor( 6, 1 );
  lcd.print(lon);           // print longitude data on LCD
}



void printObstacleOnLCD() {
  lcd.setCursor( 7, 0 );
  lcd.print(LidarLeft);     // print Left side Lidar data on LCD
  lcd.setCursor( 6, 1 );
  lcd.print(LidarRight);    // print Right side Lidar data on LCD
}

void loop() {
  lcd.clear();
   ReadLidar(); 
 printHeadingOnLCD();
 printLocationOnLCD();
  //  printObstacleOnLCD();
  delay(100);
}




