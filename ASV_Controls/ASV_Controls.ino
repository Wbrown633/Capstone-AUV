#include <Servo.h>

// ---------------------------------Declare Variables------------------------------------------
// RC Input Pins
byte thrusterPin = 2;
byte steeringPin = 3;
byte winchPin = 6;
byte doorPin = 7;
byte airPumpPin = 8;

// Relay Output Pins
byte relayChargerMagnetPin = 10;
byte relayDoorMagnetPin = 11; 
byte relayAirPumpPin = 12;
byte relayLightPin = 13;

// PWM Output Pins 
byte doorServoOutputPin = 15;
byte winchServoOutputPin = 16;
byte leftThrusterOutputPin = 17;
byte rightThrusterOutputPin = 18;

// current direction to steer
// 0 - left 
// 1 - straight
// 2 - right
byte direction = 1;


// To store pin values
int steering_value;
int throttle_value;
int winchPin_value;
int airPumpPin_value;
int doorPin_value;
int doorMagnetPin_value;
int thrusterPin_value;
int steeringPin_value;
double currentReading;
int servoVal;
int winchServoVal;
int leftThrusterVal;
int rightThrusterVal;
int throttleVal;
int steerVal;

// Servo Objects for use with servo library
Servo doorServo;
Servo winchServo;
Servo leftThrusterServo;
Servo rightThrusterServo;

// boolean values for 2 position switches
boolean airPumpOn = false;
boolean doorLocked = true;
boolean doorMagnetOn = true;

// byte for storing winch status 
// 0 : down 
// 1 : stationary 
// 2 : up 
byte winchStatus = 1;
byte prevWinchStatus = 1;

// byte for storing door state and the last state it was in 
// 0 : closed
// 1 : open 
byte doorState = 0;
byte lastDoorState = 0;

// PWM related variables 
byte prev[] = {0,0};
int high_time[] = {0,0};
int pulse[] = {0,0};
int thrusterMidPoint= 1500; 
int thrusterOffset = 0; 

// Constants for current sensor 
int mVperAmp = 185; // use 100 for 20A Module and 66 for 30A Module
int RawValue= 0;
int ACSoffset = 2500; 
double Voltage = 0;
double Amps = 0;


void setup() {

// --------------------------------------Initiate pins------------------------------------
// INPUTS
pinMode(thrusterPin, INPUT);
pinMode(steeringPin, INPUT);
pinMode(winchPin, INPUT);
pinMode(doorPin, INPUT);
pinMode(airPumpPin, INPUT);

// OUTPUTS 
pinMode(relayChargerMagnetPin, OUTPUT);
pinMode(relayDoorMagnetPin, OUTPUT);
pinMode(relayAirPumpPin, OUTPUT);
pinMode(relayLightPin, OUTPUT);
pinMode(doorServoOutputPin, OUTPUT);
pinMode(winchServoOutputPin, OUTPUT);
pinMode(leftThrusterOutputPin, OUTPUT);
pinMode(rightThrusterOutputPin, OUTPUT);


// attach servos to it's output pin
doorServo.attach(doorServoOutputPin);
winchServo.attach(winchServoOutputPin);
leftThrusterServo.attach(leftThrusterOutputPin);
rightThrusterServo.attach(rightThrusterOutputPin);
winchServo.writeMicroseconds(1500);

// Set Relay Pins LOW to turn the relays OFF 
digitalWrite(relayChargerMagnetPin, LOW);
digitalWrite(relayDoorMagnetPin, LOW);
digitalWrite(relayAirPumpPin, LOW);
digitalWrite(relayLightPin, LOW);

// Start serial communication
Serial.begin(9600);

}

void loop() {
  
  // Listen for PWM pulses on these pins (pin name, status)
  pulse[0] = pulseIn(thrusterPin, HIGH);
  pulse[1] = pulseIn(steeringPin, HIGH);
  winchPin_value = pulseIn(winchPin, HIGH);
  airPumpPin_value = pulseIn(airPumpPin, HIGH);
  doorPin_value = pulseIn(doorPin, HIGH);
  
  // convert switch values into their boolean representations
  if (airPumpPin_value < 1500){
    airPumpOn = true; 
  }
  else {
    airPumpOn = false;
  }
  
  updateDoorState(doorPin_value);

  // convert 3 position switch value 
  if (winchPin_value > 1600){
    winchStatus = 0; // down
  }
  else if (winchPin_value < 1400) {
    winchStatus = 2; // up
  }
  else {
    winchStatus = 1; // stationary
  }
  
  currentReading = currentScaled(analogRead(A0));
  
  // Do stuff with the info we just got
  changeThruster();
  moveDoor(doorState, lastDoorState);
  moveWinch(winchStatus);
  airPumpState(airPumpOn);
  chargingState(currentReading);

}

// Print the values we just read for testing
void serialPrintDebug(){

  Serial.println("Winch : " + String(winchStatus));
  Serial.println("Door State : " + String(doorState));
  
  // Wait so that the print out is actually readable 
  delay(1000);
  
}

// move the door to the required position
boolean moveDoor(byte doorState, byte lastDoorState){
  // we only need to trigger door movement on a change in switch position
  if (doorState != lastDoorState){
    if(doorState == 1){
      openDoor();
    }
    else{
      closeDoor();
    }
  }
}

void changeThruster(){
  // calculate steering adjustments
  throttleVal = pulse[0];
  steerVal = pulse[1];
  
  leftThrusterVal = throttleVal;
  rightThrusterVal = throttleVal;
  
  thrusterOffset = steerVal - thrusterMidPoint;
  
  direction = findDirection(steerVal);
  
  // TODO: make this also work while going in reverse 
  switch (direction){
    case 0:
    // left
    // thruster Offset is negative
    leftThrusterVal = leftThrusterVal + thrusterOffset;
    leftThrusterVal = normalizeThrusterValue(leftThrusterVal);
    rightThrusterVal = normalizeThrusterValue(rightThrusterVal);
    break;
    
    case 2: 
    // right
    // thruster Offset is negative
    rightThrusterVal = rightThrusterVal - thrusterOffset;
    leftThrusterVal = normalizeThrusterValue(leftThrusterVal);
    rightThrusterVal = normalizeThrusterValue(rightThrusterVal);
    break;
    
    default:
    // straight
    leftThrusterVal = normalizeThrusterValue(leftThrusterVal);
    rightThrusterVal = normalizeThrusterValue(rightThrusterVal);
    break; 
  }
  
  leftThrusterServo.writeMicroseconds(leftThrusterVal);
  rightThrusterServo.writeMicroseconds(rightThrusterVal);
}

// update the status of the winch 
void moveWinch(byte winchStatus){
  
  switch (winchStatus){
    case 0:
    // down
    if (prevWinchStatus != 0) {
      winchServo.writeMicroseconds(1400);
    }
    
    prevWinchStatus = winchStatus;
    break;
    
    case 2: 
    // up 
    if (prevWinchStatus != 2) {
      winchServo.writeMicroseconds(1550);
      changeMagnetState(relayChargerMagnetPin, 0); // turn charger magnet OFF 
      delay(1000);
      changeMagnetState(relayChargerMagnetPin, 1); // put charger magnet back ON
      prevWinchStatus = winchStatus;
    }
    prevWinchStatus = winchStatus;
     
    break;
    
    default:
    // stationary
    if (prevWinchStatus != 1) {
      winchServo.writeMicroseconds(1500);
    }
    prevWinchStatus = winchStatus;
    break; 
  }
}

// Turns Air Pump on or Off 
void airPumpState(boolean airPumpOn){
  
  if (airPumpOn){
    digitalWrite(relayAirPumpPin, HIGH);
  }
  
  else {
    digitalWrite(relayAirPumpPin, LOW);
  }
}


// Set magnet state 
// 0 : Magnet OFF (receiving power)
// 1 : Magnet ON (receiving no power)
void changeMagnetState(byte magnetPin, byte state){
  
  if (state == 0) {
    // Giving power DE-MAGNETIZES THE MAGNET (TURNS OFF)
    digitalWrite(magnetPin, HIGH);
  }
  
  else {
    // no power means the MAGNET IS MAGNETIZED (TURNS ON) 
    digitalWrite(magnetPin, LOW);
  }
}

// change the state of the charger magnet based on the state given 
void changeChargerMagnetState(boolean chargerMagnetOn){
  int state; 
  if (chargerMagnetOn){
    state = 1; 
  }
  else {
    state = 0;
  }
  changeMagnetState(relayChargerMagnetPin, state);
}


// two wrapper functions because interrupt functions can't take arguments
void pwmThrottle(){
  pwm(0);
  throttle_value = pulse[0];
}

void pwmSteering(){
  pwm(1);
  steering_value = pulse[1];
}

// interupt function, called on change to determine the PWM signal width 
void pwm(unsigned int chan) {
    // pulse is high
  if (prev[chan] == 0){
    high_time[chan] = micros();
    prev[chan] = 1;
  }
  // pulse is low 
  else {
    pulse[chan] = micros() - high_time[chan];
    prev[chan] = 0;
  }
}

// update the door state (open/closed) 
void updateDoorState(int doorPin_value) {
  lastDoorState = doorState;
  if (doorPin_value < 1500){
    doorState = 1;
  }
  else {
    doorState = 0;
  }
}

// open the door 
void openDoor(){
    
    changeMagnetState(relayDoorMagnetPin, 0); // turn the magnet off to open the door
    delay(100);
    doorServo.write(90);
    delay(500); // wait before turning the door magnet back on
    changeMagnetState(relayDoorMagnetPin, 1);
    lastDoorState = 1;
}

// close the door 
void closeDoor(){
  // send servo to home position (closed)
  doorServo.write(0);
  changeMagnetState(relayDoorMagnetPin, 1);
  digitalWrite(relayDoorMagnetPin, LOW);
  lastDoorState = 0;
}

// Make sure the pwm value cannot be outside the values the motor can handle 
int normalizeThrusterValue(int value){
  // if the value is really low, the RC transmitter is off. Default to stationary
  if (value < 100){
    value = 1500;
    return value;
  }
  // if our subtraction puts us under max reverse, pass the min value
  if (value < 1100){
    value = 1100;
    return value;
  }
  // if our addition puts us over the max throttle, pass the max value 
  if (value > 1900){
    value = 1900;
    return value;
  }
  // allow the thrusters to more easily be held at 0
  if (value > 1400 && value < 1500) {
    value = 1500;
    return value;
  }
  
  return value;
}

int findDirection(int steeringVal){
  
  if (steeringVal < 1492){
    // left
    return 0;
  }
  
  else if (steeringVal > 1492){
    // right
    return 2;
  }
  
  else {
    // straight 
    return 1;
  }
}

// turn on the charging light if the phone is charging 
void chargingState(int currentReading){
  if (currentReading < -0.5){
    // turn the light on 
    digitalWrite(relayLightPin, HIGH);
  }
  else {
    digitalWrite(relayLightPin, LOW);
  }
}

// process current reading 
double currentScaled(int rawReading){
  Voltage = (rawReading / 1024.0) * 5000; // Gets you mV
  Amps = ((Voltage - ACSoffset) / mVperAmp);
  return Amps;
}


  
  