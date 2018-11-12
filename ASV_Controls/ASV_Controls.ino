#include <Servo.h>

// ---------------------------------Declare Variables------------------------------------------
// RC Input Pins
byte thrusterPin = 3;
byte steeringPin = 5;
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


// To store pin values
int steering_value;
int throttle_value;
int winchPin_value;
int airPumpPin_value;
int doorPin_value;
int doorMagnetPin_value;
int currentReading;
int servoVal;
int winchServoVal;

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

// set up interrupts for reading joystick inputs
attachInterrupt(digitalPinToInterrupt(steeringPin), pwmSteering, CHANGE);
attachInterrupt(digitalPinToInterrupt(thrusterPin), pwmThrottle, CHANGE);

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
  //thrusterPin_value = pulseIn(thrusterPin, HIGH);
  //steeringPin_value = pulseIn(steeringPin, HIGH);
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
  
  currentReading = analogRead(A0);
  // Debug Print Statements 
  //serialPrintDebug();
  
  // Do stuff with the info we just got
  changeThruster();
  moveDoor(doorState, lastDoorState);
  moveWinch(winchStatus);
  airPumpState(airPumpOn);
  //changeChargerMagnetState(chargerMagnetOn);
  //Serial.println("Steering: " + String(pulse[1]));
  //Serial.println("Thruster: " + String(pulse[0]));
  Serial.println("Current Reading: " + String(currentReading));
}

// Print the values we just read for testing
void serialPrintDebug(){
  
  
  Serial.println("Steering : " + String(pulse[1]));
  Serial.println("Thruster : " + String(pulse[0]));
  Serial.println("Winch : " + String(winchStatus));
  Serial.println("Air Pump: " + String(airPumpOn));
  Serial.println("Door Lock : " + String(doorLocked));
  
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
  leftThrusterServo.writeMicroseconds(pwm[0]);
  rightThrusterServo.writeMicroseconds(pwm[0]);
}

// update the status of the winch 
void moveWinch(byte winchStatus){
  
  switch (winchStatus){
    case 0:
    // down
    winchServo.write(0);
    prevWinchStatus = winchStatus;
    break;
    
    case 2: 
    // up 
    if (prevWinchStatus != 2) {
      changeMagnetState(relayChargerMagnetPin, 0); // turn charger magnet OFF 
      delay(1000);
      winchServo.write(180);
      changeMagnetState(relayChargerMagnetPin, 1); // put charger magnet back ON
      prevWinchStatus = winchStatus;
    }
    
    else {
      winchServo.write(180);
      prevWinchStatus = winchStatus;
    }
     
    break;
    
    default:
    // stationary
    winchServo.write(90);
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
    doorServo.write(90);
    changeMagnetState(relayDoorMagnetPin, 0); // turn the magnet off to open the door
    delay(2000); // wait before turning the door magnet back on
    changeMagnetState(relayDoorMagnetPin, 1);
}

// close the door 
void closeDoor(){
  // send servo to home position (closed)
  doorServo.write(0);
}



  
  