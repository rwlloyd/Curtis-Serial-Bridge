
//v1.2 changes fixed estop

// TO DO:
// Estop and enable functions will need work, mainly integration into the binary protocol

#include <Wire.h>                     // include the Wire Library - needed to communicate with the DAC
#include <Adafruit_MCP4725.h>         // inlcude the DAC library - contains the comms protocol needed to communicate with the DAC

// How many motors are we having?
#define numberOfMotors 4
#define speed_multiplier 1.0 // Hard coded speed limit

// Instatiate enough motors
Adafruit_MCP4725 motors[numberOfMotors]; // Create and array of four DAC objects for the motors

// Pin Definitions.
// Array of motors,
// Array of motor directions,
// Estop input pin
int motorSelectPins[] = {12, 9, 6, A0};
int motorDirPins[] = {11, 8, 5, A1};
int motorEnablePins[] = {10, 7, 4, A2};
int errorPin = 2;

// Initialise motor speeds, directions and motor enable variables in arrays
int motorVels[] = {0, 0, 0, 0};         // Array of motor Velocities
bool motorDirs[] = {false, false, false, false};        // Array of motor Directions
bool motorsEnabled = false;         // Make sure non of the motors are enabled during startup

// length of data packet. 2*NumberOfMotors + Error + enable
const int messageLength = 10;
// Array for the received message
int received[messageLength];
// Flag to signal when a message has been received
bool commandReceived = false;

// If we lose connection, we should stop
unsigned long lastMillis;
unsigned long currentMillis;
const unsigned long period = 250;  //the value is a number of milliseconds, ie 2s

bool error = false;
//bool connectionEstablished = false;

void setup() {
  // Setup output pins
  for (int i = 0; i < numberOfMotors; i++) {
    motors[i].begin(0x63);                      // Instatiate the DACS
    pinMode(motorSelectPins[i], OUTPUT);        // Set all of the motor select pins to outputs
    pinMode(motorDirPins[i], OUTPUT);           // Set the motor Direction pins to outputs
    pinMode(motorEnablePins[i], OUTPUT);        // Set the motor enable pins as outputs
    delay(10);
  }
  // Set Estop pin as input. ESTOP is NC
  pinMode(errorPin, INPUT_PULLUP);

  // Setup serial connection, announce device and initiate dacs
  Serial.begin(115200);
  delay(100);
  Serial.write(49);                 // Announce the controller to the PC with '1'
  Serial.write(10);
  Serial.write(13);
  updateDACs();
  Serial.write(50);                 // Announce the dac initialisation with '2'

  // Record the time for connection checking
  lastMillis = millis();
}

void loop() {
  checkConnection();                               // Has the timer expired? Have we lost connection?
  //errorCheck();                                    // Check estop and set error flag
  if (commandReceived == true)                     // This code is executed in non interupt time only when a new command has been recieved
  { // A new command has been recieved when a \n or \r character is recieved.
    processSerialCommand();                        // Process the command
    //commandReceived = false;                       // Clear the command pending flag. // done in process serial command now
  }
  errorCheck();
  enableMotors();                                  // if everything is ok, enable the motors
  delay(1);
}

// function to check the time since the last serial command
void checkConnection() {
  currentMillis = millis();
  if (currentMillis - lastMillis >= period) {
    motorsEnabled = false;
  }
}

//// function to check the ESTOP pin
//void errorCheck() {
//  if (digitalRead(errorPin) == true) {
//    error == true;
//    motorsEnabled == false;
//  }
//  if (digitalRead(errorPin) == false) {
//    error = false;
//    motorsEnabled == true;
//  }
//}

void errorCheck(){
  if (digitalRead(errorPin) == HIGH){
    error == true;
    motorsEnabled = false;
  }
  else{
    error == false;
  }
}

// function to enable motors
void enableMotors() {
  if (error == false && motorsEnabled == true) {
    for (int i = 0; i < numberOfMotors; i++) {
      digitalWrite(motorEnablePins[i], HIGH);
    }
  } else {
    for (int i = 0; i < numberOfMotors; i++) {
      digitalWrite(motorEnablePins[i], LOW);
    }
  }
}

// When new characters are received, the serialEvent interrupt triggers this function
void serialEvent()   {
  // Read the Serial Buffer
  for (int i = 0; i < messageLength; i++) {
    received[i] = Serial.read();
    delay(1);
  }
  // Change the flag because a command has been received
  commandReceived = true;
  // Record the time
  lastMillis = millis();
}

// Function to split up the received serial command and set the appropriate variables
void processSerialCommand() {
  if (error == false) {
    error = bool(received[0]);                                  // Error Flag
  }
  if (error == true) {
    received[0] = byte(error);
  }
  motorsEnabled = bool(received[1]);                          // Motor Enable Flag
  // Motor Directions
  motorDirs[0] = bool(received[2]);
  motorDirs[1] = bool(received[4]);
  motorDirs[2] = bool(received[6]);
  motorDirs[3] = bool(received[8]);
  // Motor Velocities
  motorVels[0] = map(received[3], 0, 100, 200, 4092 * speed_multiplier);
  motorVels[1] = map(received[5], 0, 100, 200, 4092 * speed_multiplier);
  motorVels[2] = map(received[7], 0, 100, 200, 4092 * speed_multiplier);
  motorVels[3] = map(received[9], 0, 100, 200, 4092 * speed_multiplier);
  // Implement the new command
  updateDirections();
  updateDACs();
  //updateUnits();
  // Chirp the message back
  for (int i = 0; i < messageLength; i++) {
    Serial.write(received[i]);
  }
  // Allow a new message
  commandReceived = false;
}

void updateDirections() {
  for (int i = 0; i < numberOfMotors; i++) {
    digitalWrite(motorDirPins[i], (bool)motorDirs[i]);
  }
}

void updateDACs() {
  for (int i = 0; i < numberOfMotors; i++) {
    digitalWrite(motorSelectPins[i], HIGH);
    //delay(10);
    motors[i].setVoltage(motorVels[i], false);
    //delay(1);
    digitalWrite(motorSelectPins[i], LOW);
    //delay(1);
  }
}

void updateUnits() {
  for (int i = 0; i < numberOfMotors; i++) {
    switch (i) {
      case 0:
        // Update Directions
        digitalWrite(motorDirPins[0], (bool)motorDirs[i]);
        // Update velocities
        digitalWrite(motorSelectPins[0], HIGH);
        digitalWrite(motorSelectPins[1], LOW);
        digitalWrite(motorSelectPins[2], LOW);
        digitalWrite(motorSelectPins[3], LOW);
        motors[0].setVoltage(motorVels[0], false);
        digitalWrite(motorSelectPins[0], LOW);
      case 1:
        // Update Directions
        digitalWrite(motorDirPins[1], (bool)motorDirs[i]);
        // Update velocities
        digitalWrite(motorSelectPins[0], LOW);
        digitalWrite(motorSelectPins[1], HIGH);
        digitalWrite(motorSelectPins[2], LOW);
        digitalWrite(motorSelectPins[3], LOW);
        motors[1].setVoltage(motorVels[1], false);
        digitalWrite(motorSelectPins[1], LOW);
      case 2:
        // Update Directions
        digitalWrite(motorDirPins[2], (bool)motorDirs[i]);
        // Update velocities
        digitalWrite(motorSelectPins[0], LOW);
        digitalWrite(motorSelectPins[1], LOW);
        digitalWrite(motorSelectPins[2], HIGH);
        digitalWrite(motorSelectPins[3], LOW);
        motors[2].setVoltage(motorVels[2], false);
        digitalWrite(motorSelectPins[2], LOW);
      case 3:
        // Update Directions
        digitalWrite(motorDirPins[3], (bool)motorDirs[i]);
        // Update velocities
        digitalWrite(motorSelectPins[0], LOW);
        digitalWrite(motorSelectPins[1], LOW);
        digitalWrite(motorSelectPins[2], LOW);
        digitalWrite(motorSelectPins[3], HIGH);
        motors[3].setVoltage(motorVels[3], false);
        digitalWrite(motorSelectPins[3], LOW);
    }
  }
}



/*
   Below is code that can be pasted into processing3 to test the above code.
*/

/*
  // Very Basic Implementation of skid Steer Kinematics to control four curtis units
  // R. Lloyd
  // University of Lincoln. Dec. 2019
  // Implementation maths taken from http://robotsforroboticists.com/drive-kinematics/

  import processing.serial.*;
  Serial myPort;

  float wheel_radius = 0.25; /// lets go metric and in m
  float wheel_base = 1;

  float linear_vel = 0;
  float angular_vel = 0;
  float velocity_left;
  float velocity_right;
  int cmd_vel[] = new int[4];
  int cmd_dir[] = new int[4];
  // Our vehicle direction. True is forward, false is backward
  boolean direction = true;

  // Are we enabling the motors or not
  boolean enable = false;

  // Some arrow objects for visualisation
  float arrowOffset;

  Arrow arrow0;
  Arrow arrow1;
  Arrow arrow2;
  Arrow arrow3;

  int messageLength = 9;
  byte[] inBuffer = new byte[messageLength];
  boolean error = false;
  int[] message = new int[messageLength];
  int[] inString;

  void setup() {
  size(500, 500);
  background(51);
  arrowOffset = width/4;
  arrow0 = new Arrow(arrowOffset, arrowOffset, 1.0, true);
  arrow1 = new Arrow(width - arrowOffset, arrowOffset, 1.0, true);
  arrow2 = new Arrow(arrowOffset, height - arrowOffset, 1.0, true);
  arrow3 = new Arrow(width - arrowOffset, height - arrowOffset, 1.0, true);

  printArray(Serial.list());

  myPort = new Serial(this, Serial.list()[1], 115200);
  myPort.clear();
  }

  void draw() {
  //frameRate(1);
  background(51);
  // Check the serial port
  while (myPort.available() > 0) {
    inBuffer = myPort.readBytes();
    myPort.readBytes(inBuffer);
    if (inBuffer != null) {
      inString = int(inBuffer);
      println((inString)); // Do something more userful in the future
    }
  }
  // Draw a crosshair
  stroke(255);
  strokeWeight(2);
  line(width/2, 0, width/2, height);
  line(0, height/2, width, height/2);

  //  Kinematic model
  //   There are two special cases:
  //   IF velocity_right == velocity_left :
  //   THEN the radius of the arc is infinite so the robot will drive straight.
  //   IF velocity_right == -velocity_left :
  //   THEN the radius of the arc is 0, and the robot rotates in place (ie. point turn)


  if (angular_vel == 0) {
    velocity_left = int((linear_vel - angular_vel *wheel_base / 2.0)/wheel_radius);
    velocity_right = velocity_left;
  } else if (angular_vel == 1 || angular_vel == -1) {
    velocity_left = int((linear_vel - angular_vel *wheel_base / 2.0)/wheel_radius);
    velocity_right = -velocity_left;
  } else {
    // otherwise
    velocity_left = int((linear_vel - angular_vel *wheel_base / 2.0)/wheel_radius);
    velocity_right = int((linear_vel + angular_vel *wheel_base / 2.0)/wheel_radius);
  }

  if (direction == false) {

    velocity_left = int((linear_vel + angular_vel *wheel_base / 2.0)/wheel_radius);
    velocity_left *= -1;

    velocity_right = int((linear_vel - angular_vel *wheel_base / 2.0)/wheel_radius);
    velocity_right *= -1;
  }

  cmd_vel[0] = abs(int(map(velocity_left, 0, 100, 0, 255)));
  cmd_vel[1] = abs(int(map(velocity_right, 0, 100, 0, 255)));
  cmd_vel[2] = abs(int(map(velocity_left, 0, 100, 0, 255)));
  cmd_vel[3] = abs(int(map(velocity_right, 0, 100, 0, 255)));

  if (velocity_right < 0) {
    cmd_dir[1] = 0;
    cmd_dir[3] = 0;
  } else {
    cmd_dir[1] = 1;
    cmd_dir[3] = 1;
  }

  if (velocity_left < 0) {
    cmd_dir[0] = 0;
    cmd_dir[2] = 0;
  } else {
    cmd_dir[0] = 1;
    cmd_dir[2] = 1;
  }

  generateMessageSerial();
  //generateMessageConsole();

  // Visualise the outputs
  // update the arrows
  arrow0.show(cmd_vel[0]/2, boolean(cmd_dir[0]));
  arrow1.show(cmd_vel[1]/2, boolean(cmd_dir[1]));
  arrow2.show(cmd_vel[2]/2, boolean(cmd_dir[2]));
  arrow3.show(cmd_vel[3]/2, boolean(cmd_dir[3]));
  }

  void generateMessageSerial() {
  myPort.write(byte(error));
  myPort.write(byte(enable));
  myPort.write(byte(cmd_dir[0]));
  myPort.write(byte(cmd_vel[0]));
  myPort.write(byte(cmd_dir[1]));
  myPort.write(byte(cmd_vel[1]));
  myPort.write(byte(cmd_dir[2]));
  myPort.write(byte(cmd_vel[2]));
  myPort.write(byte(cmd_dir[3]));
  myPort.write(byte(cmd_vel[3]));
  }

  void generateMessageConsole() {
  println(byte(error), byte(enable), byte(cmd_dir[0]), byte(cmd_vel[0]), byte(cmd_dir[1]), byte(cmd_vel[1]), byte(cmd_dir[2]), byte(cmd_vel[2]), byte(cmd_dir[3]), byte(cmd_vel[3]));
  }

  void mousePressed(){
   // From mouse position, lets get a magnitude value for linear speed between 0 and 100
  // and for turning speed we will go from -1 (left full) to +1 (right full)
  linear_vel = abs(map(mouseY, height, 0, -10, 10));
  angular_vel = map(mouseX, 0, width, -2*PI, 2*PI);

  if (mouseY < height/2) {
    direction = true;
  } else {
    direction = false;
  }
  }

  void keyPressed() {
  // 'e' = toggle enable
  if (key == 'e' || key == 'E') {
    enable =! enable;
  }
  }

  class Arrow
  {
  float size;
  float x;
  float y;
  boolean direction;

  Arrow(float ix, float iy, float isize, boolean idirection)
  {
    x = ix;
    y = iy;
    size = isize;
    direction = idirection;
  }

  void show(float size, boolean direction)
  {
    stroke(0);
    strokeJoin(ROUND);
    strokeWeight(size/5);
    fill(255, 200);
    if (direction)
    {
      beginShape();
      vertex(x, y-(2.5*size));
      vertex(x+(1.5*size), y-size);
      vertex(x+(0.5*size), y-size);
      vertex(x+(0.5*size), y+(2*size));
      vertex(x-(0.5*size), y+(2*size));
      vertex(x-(0.5*size), y+(2*size));
      vertex(x-(0.5*size), y-size);
      vertex(x-(1.5*size), y-size);
      endShape(CLOSE);
    } else {
      beginShape();
      vertex(x, y+(2.5*size));
      vertex(x+(1.5*size), y+size);
      vertex(x+(0.5*size), y+size);
      vertex(x+(0.5*size), y-(2*size));
      vertex(x-(0.5*size), y-(2*size));
      vertex(x-(0.5*size), y-(2*size));
      vertex(x-(0.5*size), y+size);
      vertex(x-(1.5*size), y+size);
      endShape(CLOSE);
    }
  }
  }
*/
