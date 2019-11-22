#include <Wire.h>                     // include the Wire Library - needed to communicate with the DAC
#include <Adafruit_MCP4725.h>         // inlcude the DAC library - contains the comms protocol needed to communicate with the DAC

#define numberOfMotors 4

Adafruit_MCP4725 motors[numberOfMotors]; // Create and array of four DAC objects for the motors

int motorSelectPins[] = {2, 3, 4, 5}; // Array for motor select pin numbers
int motorDirPins[] = {6, 7, 8, 9};
int errorPin = 13;

int motorVels[] = {0, 0, 0, 0};        // Array of motor Velocities
bool motorDirs[] = {0, 0, 0, 0};        // Array of motor Directions

const int messageLength = 9; //((2 * numberOfMotors) + 1);
int received[messageLength];
bool commandReceived = false;

bool error = false;
//bool connectionEstablished = false;

void setup() {
  // Instatiate the DACS
  for (int i = 0; i < numberOfMotors; i++) {
    motors[i].begin(0x61);
    delay(10);
  }
  // Set all of the motor select pins to outputs
  for (int i = 0; i < numberOfMotors; i++) {
    pinMode(motorSelectPins[i], OUTPUT);
  }
  // Set the motor Direction pins to outputs
  for (int i = 0; i < numberOfMotors; i++) {
    pinMode(motorDirPins[i], OUTPUT);
  }
  pinMode(errorPin, OUTPUT);

  Serial.begin(115200);
  delay(100);
  Serial.write(49);                 // Announce the controller to the PC with '1'
  Serial.write(10);
  Serial.write(13);
  updateDACs();
  Serial.write(50);                 // Announce the dac initialisation with '2'

}

void loop()
{
  if (commandReceived == true)                     // This code is executed in non interupt time only when a new command has been recieved
  { // A new command has been recieved when a \n or \r character is recieved.
    processSerialCommand();                        // Process the command
    commandReceived = false;                       // Clear the command pending flag.
  }
  delay(1);
}

void serialEvent()   {                                // As new characters are recieved by the USART hardware an interupt will fire which executes this code
  for (int i = 0; i < messageLength; i++) {
    received[i] = Serial.read();
    delay(1);
  }
  commandReceived = true;
}

void processSerialCommand() {                // Simple command processing from the PC to the Scooter Controller - Each command is echoed back to PC along with calucated percentage and DAC value on one lime.
  error = bool(received[0]);

  motorDirs[0] = bool(received[1]);
  motorDirs[1] = bool(received[3]);
  motorDirs[2] = bool(received[5]);
  motorDirs[3] = bool(received[7]);

  motorVels[0] = map(received[2], 0, 100, 1600, 4092);
  motorVels[1] = map(received[4], 0, 100, 1600, 4092);
  motorVels[2] = map(received[6], 0, 100, 1600, 4092);
  motorVels[3] = map(received[8], 0, 100, 1600, 4092);

  Serial.write(received[0]);
  Serial.write(received[1]);
  Serial.write(received[2]);
  Serial.write(received[3]);
  Serial.write(received[4]);
  Serial.write(received[5]);
  Serial.write(received[6]);
  Serial.write(received[7]);
  Serial.write(received[8]);

  //  for (int i = 0; i < messageLength; i++) {
  //    Serial.write(received[i]);
  //  }
  updateDirections();
  updateDACs();
}

void updateDirections() {
  for (int i = 0; i < numberOfMotors; i++) {
    digitalWrite(motorDirPins[i], (bool)motorDirs[i]);
  }
  //updateDACs();
}

void updateDACs() {
  for (int i = 0; i < numberOfMotors; i++) {
    digitalWrite(motorSelectPins[i], HIGH);
    //delay(1);
    motors[i].setVoltage(motorVels[i], false);
    digitalWrite(motorSelectPins[i], LOW);
    //delay(1);
  }
}

/*
  import processing.serial.*;
  Serial myPort;

  byte[] inBuffer = new byte[3];
  int[] myString;
  boolean error = false;

  int brightness; // 0-255
  boolean direction = false;

  void setup() {
  frameRate(30);
  size(500, 500);
  background(51);
  printArray(Serial.list());

  myPort = new Serial(this, Serial.list()[0], 115200);
  myPort.clear();
  }

  void draw() {

  while (myPort.available() > 0) {
    inBuffer = myPort.readBytes();
    myPort.readBytes(inBuffer);
    if (inBuffer != null) {
      myString = int(inBuffer);
      println(myString);
    }
  }

  brightness = abs(int(map(mouseY, height, 0, -255, 255)));
  if (mouseY >= height/2) {
    direction = false;
  } else {
    direction = true;
  }
  sendCommand();
  }

  void sendCommand(){
    myPort.write(byte(error));
    myPort.write(byte(direction));
    myPort.write(byte(brightness));
    myPort.write(byte(direction));
    myPort.write(byte(brightness));
    myPort.write(byte(direction));
    myPort.write(byte(brightness));
    myPort.write(byte(direction));
    myPort.write(byte(brightness));

  }
*/
