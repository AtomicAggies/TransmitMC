#include "TransmitFuncs.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>

//Serial
#define Comm Serial2
#define Xbee Serial1
#define COMM_RX_PIN 15
#define COMM_TX_PIN 33
#define COMM_BAUD 115200
#define XBEE_BAUD 9600
// power
#define SET 27
#define UNSET 12
#define PAYLOAD 21
//Thresholds
#define ACCEL_LAUNCH_THRESHOLD 25 // m/s^2
#define TIME_ACCEL_THRESHOLD 500 //milliseconds 

Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);
sensors_event_t event;

//messages
String comm_msg;  //incoming blackbox msgs
String xbee_msg;  //incoming groundstation msgs

//State
enum ActivationState { INACTIVE, ACTIVATED, A_AWAIT_RESPONSE } activation_state;
enum PayloadState { NOT_INITIATED, INITIATED, P_AWAIT_RESPONSE } payload_state;
enum RocketState{LAUNCH_PAD = 0,LAUNCH = 10,APOGEE = 20,LANDING = 30}rocket_state;
enum AirbrakesState{NOT_DEPLOYED,DEPLOYED} airbrakes_state;
//acceleration detection vars
boolean is_accelerating = false;  //must be initially be false
unsigned long time_accelerating = 0; 
unsigned long accel_start_time = 0; 
float tot_accel = 0;

void setup(){

  Comm.begin(COMM_BAUD, SERIAL_8N1, COMM_RX_PIN, COMM_TX_PIN);
  Serial.begin(COMM_BAUD);
  Xbee.begin(XBEE_BAUD, SERIAL_8N1, 14, 32);
  init_pin_modes();
  // // init accel
  // if(!accel.begin()) {
  //   Serial.println("ADXL343 failed begin");
  // }
  // else{
  //   Serial.println("ADXL343 succesful begin");
  // }
  // accel.setRange(ADXL343_RANGE_16_G);


  digitalWrite(PAYLOAD, LOW); //artifact/code left over from system or possibly previous payload

  deactivate_avionics();

  Serial.println("TransmitCode Setup Complete");
  delay(2000);
  
}
void loop(){
  comm_msg = "";
  if(Comm.available()>0){ //read from blackbox
    comm_msg = Comm.readStringUntil('\n');

  }  
  xbee_msg = "";
  if(Xbee.available()>0){ //read from ground station
    xbee_msg = Xbee.readStringUntil('\n');
    
    if(xbee_msg.equals("AV_ACTIVATE\r")){
      activate_avionics();
      Xbee.println("AV_ACTIVE");
    }
    else if(xbee_msg.equals("DEACTIVATE\r")){
      deactivate_avionics();
      Xbee.println("AV_INACTIVE");
    }
    //elif payload
  }


  //
  // String data_accel = parse_accel_xyz(accel);
  // Xbee.println(data_accel);

  //auto activation in case of failed messages
  float tot_accel = calc_tot_accel(accel);
  if(activation_state == INACTIVE){ 
    if(is_accelerating == false && tot_accel > ACCEL_LAUNCH_THRESHOLD){ //first detected acceleration
      is_accelerating = true;
      accel_start_time = millis();
    }
    else if (is_accelerating == true && tot_accel > ACCEL_LAUNCH_THRESHOLD){   // acceleration detected in previous loop() iteration // I am still accelerating
      unsigned long curr_time = millis();
      time_accelerating = curr_time - accel_start_time;
      if(time_accelerating > TIME_ACCEL_THRESHOLD){ // time accelerating is greater than threshold
          activate_avionics();
          Xbee.println("AV_ACTIVE");
      }
    }
    else{ // I have stopped accelerating //this is to avoid false launches
      is_accelerating = false;
      time_accelerating = 0;
    }
  }

  
}

void init_pin_modes(){
  pinMode(SET, OUTPUT);
  pinMode(UNSET, OUTPUT);
  pinMode(PAYLOAD, OUTPUT);

}

//PRE: ADXL343 object
//POST: returns "ACCEL,x,y,z,ENDACCEL"
String parse_accel_xyz(Adafruit_ADXL343& localAccel){
  // Read accelerometer data
  localAccel.getEvent(&event);
  
  // Format the accelerometer data
  String accelData = "ACCEL,"; 
  accelData += String(event.acceleration.x) + ",";
  accelData += String(event.acceleration.y) + ",";
  accelData += String(event.acceleration.z) + ",";
  accelData += "ENDACCEL";

  return accelData;
}

//Pre: defined UNSET and SET pins
//POST: Latching Relay is closed. Activation State set to Inactive
void deactivate_avionics(){
  digitalWrite(UNSET, HIGH);
  delay(10);
  digitalWrite(UNSET, LOW);
  activation_state = INACTIVE;
}
//Pre: defined UNSET and SET pins, Xbee defined as a Serial
//POST: Latching Relay is closed. Activation State set to active
void activate_avionics(){
      digitalWrite(SET, HIGH);
      delay(10);
      digitalWrite(SET, LOW);
      activation_state = ACTIVATED;
}

float calc_tot_accel(Adafruit_ADXL343& localAccel){
    localAccel.getEvent(&event);
    float tot_accel = sqrt((event.acceleration.x * event.acceleration.x) + (event.acceleration.y * event.acceleration.y) + (event.acceleration.z * event.acceleration.z));
    return tot_accel;
}
