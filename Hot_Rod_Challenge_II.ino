//Line Following Robot
//Cassie Tarakajian, Adam Merritt, Kyle Rohrbach, Bernard Cheng
//530.421.01 Mechatronics Spring 2012
//28 Mar 2012

//line following constants
float kp = 48; //temp kp value
float ki = 3;
float kd = 1;

#define RMOTOR_B 195
#define LMOTOR_B 195
#define KP 48
#define KI 3
#define KD 1

#include "QTRSensors.h"
#include <Servo.h> 
#define NUM_SENSORS   5     // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading

QTRSensorsAnalog qtra((unsigned char[]) {1,2,3,4,5},
  (unsigned char) NUM_SENSORS, NUM_SAMPLES_PER_SENSOR);

unsigned int sensorValues[NUM_SENSORS];

unsigned int sensor_test_values[NUM_SENSORS];


//sensors are connected analog pins 1-5
int right_sensor = 1;
int middle_sensor = 2;
int left_sensor = 3;
int lwheel_sensor = 4;
int rwheel_sensor = 5;
int r_val = 0;
int m_val = 0;
int l_val = 0;
int rw_val = 0;
int lw_val = 0;
int state = 0;
int error = 0;
float time = 0.0;
int pos;

//update motor velocities
int u = 0;
float last_time = 0;
float deltat = 0;
int integral = 0;
int derivative = 0;
int last_error = 0;
int RMotor_base = RMOTOR_B;
int LMotor_base = LMOTOR_B;
int RMotor_counts = 0;
int LMotor_counts = 0;

//Motor driver pins: PWM pins 4-7
int RMotor_f = 4;
int RMotor_b = 5;
int LMotor_f = 6;
int LMotor_b = 7;

//digital pins 2-13 are PWM pins
Servo servo_arm;
Servo servo_plate;
int arm_pin = 2;
int plate_pin = 3;

//digital pin 22 for contact switch
int contact_pin = 22;

//picking up block mode
boolean pickup_box = false;
boolean box_intow = false;



//gap checking
boolean turn_white = false;
boolean gap_cleared = false;
int last_u = 0;

void setup()
{
  //Calibrate reflective sensors
   delay(500);
  int i;
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on LED to indicate we are in calibration mode
  for (i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();     // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    delay(10);
  }
  digitalWrite(13, LOW);     // turn off LED to indicate we are through with calibration
  
  Serial.begin(9600); 
  
  //make this delay smaller
  delay(3000);
  
  analogWrite(RMotor_f, RMotor_base);
  analogWrite(LMotor_f, LMotor_base);
  analogWrite(RMotor_b, 0);
  analogWrite(LMotor_b, 0);
  
  last_time = millis();
  
  
}

void loop()
{
  //read sensor values
  // Take into account the fact that the sensor values return to 0
  unsigned int pos = qtra.readLine(sensorValues);
  
  if (sensorValues[0] > 850 || sensorValues[1] > 850 || sensorValues[2] > 850){
    turn_white = false;
  }
  if ((sensorValues[0]) > 850 && (sensorValues[1]) > 850) {
    turn_left();
    if(box_intow = false){
      pos = qtra.readLine(sensorValues);
      find_error();
      //go into picking up box mode
      pickup_box = true;
      RMotor_base = 50;
      LMotor_base = 50;
      kp = 12;
      //drop arm
      drop_arm();
      //Serial.println(error); 
    }
  }
  else if ((sensorValues[1]) > 850 && (sensorValues[2]) > 850) {
    turn_right();
    pos = qtra.readLine(sensorValues); 
    find_error();
  }
  else if ((sensorValues[0] < 850 && sensorValues[1] < 850 && sensorValues[2] < 850)){
    //GAP CLEARING CODE
    if (turn_white == false) {
      turn_white = true;
      last_time = millis();
      time = millis();
    }
    else if (turn_white == true) {
      time = millis();
    }
    if (time - last_time > 2000){  //If 2 seconds pass we're in the gap
      analogWrite(RMotor_f, RMOTOR_B);
      analogWrite(LMotor_f, LMOTOR_B);  //Go straight
      while(gap_cleared == false){
        //check line sensors, wheel sensors, then time
        pos = qtra.readLine(sensorValues); 
        if((sensorValues[0] > 850 || sensorValues[1] > 850 || sensorValues[2] > 850)){
          //Any of the sensors go dark
          gap_cleared = true;        
        }
        else if(sensorValues[3] > 850){
          just_made_gap_left();          
          gap_cleared = true;          
        }
        else if (sensorValues[4] > 850) {
          just_made_gap_right();
          gap_cleared = true;
        }
        else if (time - last_time > 7000){ //Robot has gone too far in 7 seconds
          gone_too_far(); //write this
          gap_cleared = true;        
        }
      }

    time = millis();
    }
   }
  
  find_error();
  
  
  float time = millis();  // [[[[Is this bad to declare 'float' on every loop?]]]]
  deltat = time - last_time;
  integral = integral + error;
  derivative = error - last_error;
  
  u = kp*error;
  u = kp*error + ki*((float) integral) + kd*((float) derivative);
  
  RMotor_counts = RMotor_base + u;
  LMotor_counts = LMotor_base - u;
  
  if (u != 0){
    last_u = u;
  }
  
  if (RMotor_counts < 0) {
    RMotor_counts = 0; 
  }
  if (LMotor_counts < 0) {
    LMotor_counts = 0; 
  }
  if (RMotor_counts > 255) {
    RMotor_counts = 255; 
    //LMotor_counts = LMotor_counts - 20;
  }
  if (LMotor_counts > 255) {
    LMotor_counts = 255; 
    //RMotor_counts = RMotor_counts - 20;
  }
  

  
  analogWrite(RMotor_f, RMotor_counts);
  analogWrite(LMotor_f, LMotor_counts);
  
  last_time = time;
  last_error = error;
  
  if (pickup_box) {
    if (digitalRead(contact_pin == HIGH)) {
      stop_cart();
      lift_arm();
      turn_180();
      //reset line following values to original speed
      RMotor_base = RMOTOR_B;
      LMotor_base = LMOTOR_B;
      kp = KP;  
      pickup_box = false;
      box_intow = true;

    }
  }
  
  //Need some delay maybe?
  delay(30);
}

void turn_left()
{
  RMotor_counts = 150;
  LMotor_counts = 150;  //Do we want to slow down more?
  unsigned int pos = qtra.readLine(sensorValues);
  //Wait until wheel sensor is on line
  analogWrite(RMotor_f, RMotor_counts); 
  analogWrite(LMotor_f, LMotor_counts);  
  while ((sensorValues[4]) < 800) {      //Make sure this is lw sensor
     pos = qtra.readLine(sensorValues);
  }
  analogWrite(RMotor_f, 0);
  analogWrite(LMotor_f, 0);
 
  pos = qtra.readLine(sensorValues); 
  analogWrite(RMotor_f, 70);
  analogWrite(LMotor_b, 70);
  delay(800);                           //Might need trimming
  //wait until middle sensor is over line
  while ((sensorValues[1]) < 800) {
    pos = qtra.readLine(sensorValues);
  }
  //then move robot straight
  analogWrite(LMotor_b, 0);
  analogWrite(RMotor_f, RMotor_counts);
  analogWrite(LMotor_f, LMotor_counts);
  error = 0;
  return;
}
void turn_right()
{
  RMotor_counts = 150;
  LMotor_counts = 150;
  unsigned int pos = qtra.readLine(sensorValues);
  //Wait until wheel sensor is on line
  analogWrite(RMotor_f, RMotor_counts); //These lines are unecessary
  analogWrite(LMotor_f, LMotor_counts);
  while ((sensorValues[3]) < 800) {    
     pos = qtra.readLine(sensorValues);
  }
  
  analogWrite(RMotor_f, 0);
  analogWrite(LMotor_f, 0);
 
  pos = qtra.readLine(sensorValues); 
  analogWrite(RMotor_b, 70);
  analogWrite(LMotor_f, 70);
  delay(800);                            //This could use trimming
  //wait until middle sensor is over line
  while ((sensorValues[1]) < 800) {
    pos = qtra.readLine(sensorValues);
  }
  //then move robot straight
  analogWrite(RMotor_b, 0);
  analogWrite(RMotor_f, RMotor_counts);
  analogWrite(LMotor_f, LMotor_counts);
  error = 0;
  return;
}


void just_made_gap_right(){
  //must turn right a bit
  RMotor_counts = 40;
  LMotor_counts = 75;
  pos = qtra.readLine(sensorValues);  
  analogWrite(RMotor_f, RMotor_counts);
  analogWrite(LMotor_f, LMotor_counts);
  while (sensorValues[1] < 800){
    pos = qtra.readLine(sensorValues);
  }
  return;
}

void just_made_gap_left(){
  //must turn left a bit
  RMotor_counts = 75;
  LMotor_counts = 40;
  pos = qtra.readLine(sensorValues);  
  analogWrite(RMotor_f, RMotor_counts);
  analogWrite(LMotor_f, LMotor_counts);
  while (sensorValues[1] < 800){
    pos = qtra.readLine(sensorValues);
  }
  return;

}

void gone_too_far(){
  //if last_u > 0 right motor is faster, i.e. jtt turning left
  if(last_u > 0){
    //correct to the right
    RMotor_counts = 40;
    LMotor_counts = 75;
    pos = qtra.readLine(sensorValues);  
    analogWrite(RMotor_f, RMotor_counts);
    analogWrite(LMotor_f, LMotor_counts);
    while (sensorValues[1] < 800){
      pos = qtra.readLine(sensorValues);
    }
  }
  else if(last_u < 0) {
    //correct to the left
    RMotor_counts = 75;
    LMotor_counts = 40;
    pos = qtra.readLine(sensorValues);  
    analogWrite(RMotor_f, RMotor_counts);
    analogWrite(LMotor_f, LMotor_counts);
    while (sensorValues[1] < 800){
      pos = qtra.readLine(sensorValues);
    }
  }
  return;
}

//after picking up the block, turn cart 180°
void turn_180()
{
  unsigned int pos = qtra.readLine(sensorValues); 
  analogWrite(RMotor_b, 70);
  analogWrite(LMotor_f, 70);
  while ((sensorValues[1]) < 800) {
    pos = qtra.readLine(sensorValues);
  }
  analogWrite(RMotor_b, 0);
  analogWrite(RMotor_f, RMotor_base);
  analogWrite(LMotor_f, LMotor_base);
}

void stop_cart()
{
  analogWrite(LMotor_b, 0);
  analogWrite(RMotor_f, 0);
  analogWrite(LMotor_f, 0);
  analogWrite(RMotor_b, 0);
  return;
}

void find_error()
{
   if (sensorValues[0] > 850 && sensorValues[1] < 850 && sensorValues[2] < 850){
     error = 2;
  }
  else if (sensorValues[0] < 850 && sensorValues[1] > 850 && sensorValues[2] < 850){
     error = 0;
  }
  else if (sensorValues[0] < 850 && sensorValues[1] < 850 && sensorValues[2] > 850){
     error = -2;
  } 
  return;
}

void drop_arm() {
  for (int i = 0; i <= 90; i++) {
    servo_arm.write(180-i);
    delay(5);   // Can't this just accept a position and it will go to it?  Or do you want to slow down the servo speed?
    //Also, while this is happening, our robot is still trudging forward w/o looking at the line
    //we may want to stop it moving first, then lower the arm in this fashion
  } 
}

void lift_arm() {
  for (int i = 0; i <= 90; i++) {
    servo_arm.write(90+i);
    delay(5);
  }
}

/*
  I started this for velocity feedback, its from lab 4.  For now though we
  are not including it, integral feedback should account for slight motor
  differences.  We don't need to control the motor velocities specifically.
  
int read_encoder() {
  digitalWrite(SS,LOW);
  SPI.transfer(RD_CNTR);
  unsigned int quad_count = SPI.transfer(0x00);
  quad_count  = quad_count << 8;
  quad_count |= SPI.transfer(0x00);
  digitalWrite(SS,HIGH);
  return quad_count;
}

float calculate_velocity() {
   unsigned int count = read_encoder();
   float time = millis();
   delta = count - last_count;
   deltat = time - last_time;
   //velocity is counts per second
   float velocity = ((delta/deltat)/3200.0)*1000.0;
   last_count = count;
   last_time = time;
   return velocity;
*/
  

