/*Intention Detection
  Written by Garrett Kryt
  July 23rd 2021

  This is the main file for control of the wheelchair. Once the start button is depressed, the Teensy will query the wheel modules at
  20 Hz to gain the pushrim signal. The pushrim signal is then converted to a motor command using the polyLeft and polyRight libraries. The motor 
  command is then multiplied by a gain based on the user intention detection. Once multiplied the motor command is snet to the motor controller.

  The intention detection uses the intention_detection library and SF100000_Trees10_MD5_Garrett.h generated from emlearn.h to detect if the user is
  moving unmoving, turning left or right, or moving straight. A separate library from emlearn.h needs to be generated for any particular user based on
  their unique pushing style.

*/
#include <SD.h>
#include <SPI.h>
#include <Bounce.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//code and declarations for the polynomials that control the bit force sensor to torque conversion
#include "polyLeft.h"     //the include file that contains the lookup table for the trajectories
#include "polyRight.h"
polyLeft PL;
polyRight PR;

//include intention detection library that has the actual random forest classifier
#include "intention_detection.h"
intention_detection ID;
int32_t predictedClass = 0;
char classBuff[4];
float motorCmdFloatLeft = 0.0;
float motorCmdFloatRight = 0.0;
int avgCmd;


const int LEDPin = 33;    //pin for the LED on the control box
elapsedMillis newTimer;   //Time that sets the sampling rate

#define ROBO Serial3                //Serial Port for the roboteq
#define HWSERIAL_L Serial2          //serial port for bluetooth module Serial 2 is left module
#define HWSERIAL_R Serial1          //serial port for bluetooth module Serial 1 is right module


char dataString[256];               //buffer to hold all being sent to card

uint32_t timeCount = 0;                     //time count to increment since code button pressed
uint32_t timeStart = 0;                     //time in milliseconds since button has been pressed
char timeString[10];                         //string to send time to data capture

const int buttonPin = 34;                   //digital output that starts data capture
Bounce pushbutton = Bounce(buttonPin, 10);  // 10 ms debounce

//toggle switch for setting if the controller is adaptive (asphaltMult = 1.5) or normal (asphaltMult = 1.0)
const int togglePin = 36;
int toggleStatus = 0;
Bounce toggleSwitch = Bounce(togglePin, 2);

//toggle switch for setting different gains for the adaptive controller
const int turboPin = 38;
int turboStatus = 0;
Bounce turboSwitch = Bounce(turboPin, 2);



const int chipSelect = BUILTIN_SDCARD;

//the outgoing buffer that is sent to the roboteq
char msgBuff[20]; //buffer for message to send to roboteq

char qResponse[50];     //larger buffer that contains all queries from the roboteq
char receivedChars[10]; //a smaller buffer for each individual query from the robotew
volatile boolean newData = false; //flag to determine when all queries have been received and put into one string
volatile uint16_t z = 0;             //counter for passing through the trajectory
volatile boolean buttonFlag = false;                  //flag for when the button is set

//serial comms to bluetooh definitions
const byte numChars = 128;         //incoming buffer size
char receivedCharsLeft[numChars];     //incoming buffer left module
char receivedCharsRight[numChars];     //incoming buffer right module
char newBuffLeft[numChars];
char newBuffRight[numChars];

char frameBuff[64];               //buffer for frame data
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//buffers for the motor commands
char motorLeft[6];
char motorRight[6];
char sendMotorBuff[10];               //buffer for sending commands to roboteq

int lightCount = 0;                   //variable for counting to blink light at lower rate
static bool LEDSTATE = true;

void setup() {
  /* Initialise the onboard accelerometer sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  pinMode(togglePin, INPUT_PULLUP);
  pinMode(turboPin, INPUT_PULLUP);
  delayMicroseconds(10);
  bno.setExtCrystalUse(true);

  //iniitialize bluetooth
  HWSERIAL_L.begin(230400);  // The Bluetooth Mate defaults to 115200bps
  delay(250);
  HWSERIAL_R.begin(230400);  // The Bluetooth Mate defaults to 115200bps
  delay(250);

  pinMode(LEDPin, OUTPUT);                  //LED pin
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(230400);

  ROBO.begin(115200);  //set serial one to baud to 115200 bps
  while (!ROBO) {
    ; // wait for HW serial port to connect.
  }

  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  Serial.println("Setup done");

  digitalWrite(LEDPin, HIGH); //turn light on so we know we're running
  delay(1000); //quick delay to make sure everything works
}


void loop() {

  if (newTimer >= 50) {
    pushbutton.update();
    if (pushbutton.fallingEdge() && buttonFlag == false) {     //if button has been depressed and collection is not started
      buttonFlag = 1;
      ROBO.println("!MG");                                    //emergency stop release
      delay(4);
      Serial.println("button falg");
    } else if (pushbutton.fallingEdge() == true && buttonFlag == 1) {
      buttonFlag = 0;
      Serial.println("Nope");
    }

    //check toggle switch for adaptive or normal control
    toggleSwitch.update();
    toggleStatus = toggleSwitch.read();

    //check turbo pin if high or medium setting
    turboSwitch.update();
    turboStatus = turboSwitch.read();

    if (buttonFlag == 1) {
      timeCount = millis() - timeStart;            //record time in milliseconds since code start
      sprintf(timeString, "%lu,", timeCount);
      strcpy(dataString, timeString);

      //REC
      recBLELeft();
      processWithStartEndMarkerLeft();
      strcat(dataString, newBuffLeft);
      recBLERight();
      processWithStartEndMarkerRight();
      strcat(dataString, newBuffRight);

      calcMotorLeft();
      calcMotorRight();

      //get orientation data from frame
      sensors_event_t angVelocityData , linearAccelData;
      bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
      bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
      sprintf(frameBuff, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,", linearAccelData.acceleration.x, linearAccelData.acceleration.y, linearAccelData.acceleration.z, angVelocityData.gyro.x, angVelocityData.gyro.y, angVelocityData.gyro.z);
      strcat(dataString, frameBuff);

      //classify
      predictedClass = ID.classify(motorCmdFloatLeft, motorCmdFloatRight, buttonFlag, turboStatus);
      sprintf(classBuff, "%ld,", predictedClass);
      strcat(dataString, classBuff);

      //multiply torque value based on classified terrain
      if (toggleStatus == 1) {                                     //controller is in adaptive mode
        //        Serial.print("Adaptive");
        if (turboStatus == 0) {       //Model One
          //turbo is on
          if (predictedClass == 0) {
            motorCmdLeft = int((float)motorCmdLeft * 1.5);
            motorCmdRight = int((float)motorCmdRight * 1.5);


          } else if (predictedClass == 1) {     //straight assist
            motorCmdLeft = motorCmdLeft;
            motorCmdRight = motorCmdRight;

            //2 is left assist
          } else if (predictedClass == 2) {
            motorCmdLeft = int((float)motorCmdLeft * 1.0);
            motorCmdRight = int((float)motorCmdRight * 1.5);
            
            //3 is right assist
          } else if (predictedClass == 3) {
            motorCmdLeft = int((float)motorCmdLeft * 1.5);
            motorCmdRight = int((float)motorCmdRight * 1.0);
            
          } else {
            motorCmdLeft = 0;
            motorCmdRight = 0;
          }

        } else {         //Model 2
          if (predictedClass == 0) {
            motorCmdLeft = int((float)motorCmdLeft * 0.5);
            motorCmdRight = int((float)motorCmdRight * 0.5);

          } else if (predictedClass == 1) {     //straight assist
            motorCmdLeft = motorCmdLeft;
            motorCmdRight = motorCmdRight;

            //2 is left assist
          } else if (predictedClass == 2) {
            motorCmdLeft = int((float)motorCmdLeft * 1.0);
            motorCmdRight = int((float)motorCmdRight * 1.5);
            
            //3 is right assist
          } else if (predictedClass == 3) {
            motorCmdLeft = int((float)motorCmdLeft * 1.5);
            motorCmdRight = int((float)motorCmdRight * 1.0);
            
          } else {
            motorCmdLeft = 0;
            motorCmdRight = 0;
          }
        }
      } else {
        motorCmdLeft = motorCmdLeft;
        motorCmdRight = motorCmdRight;
      }

      //safeties for motor controller
      if (motorCmdRight >= 1000) {
        motorCmdRight = 1000;
      }
      if (motorCmdRight <= -1000) {
        motorCmdRight = -1000;
      }

      if (motorCmdLeft >= 1000) {
        motorCmdLeft = 1000;
      }
      if (motorCmdLeft <= -1000) {
        motorCmdLeft = -1000;
      }

      snprintf(msgBuff, 64, "!G 1 %i_!G 2 %i", motorCmdLeft, motorCmdRight);      //send motor command to roboteq
      ROBO.println(msgBuff);

      //don;t send any data, but save motor commands
      sprintf(sendMotorBuff, "%i,%i", motorCmdLeft, motorCmdRight);              //append written data commands to data string
      strcat(dataString, sendMotorBuff);

      // open the file.
      File dataFile = SD.open("datalog.txt", FILE_WRITE);

      // if the file is available, write to it:
      if (dataFile) {
        dataFile.println(dataString);
        dataFile.close();
        Serial.println(dataString);
        // print to the serial port too:
      }
      // if the file isn't open, pop up an error:
      else {
        Serial.println("error opening datalog.txt");

      }
      qResponse[0] = 0;                   //zero out data string for safety
      strcpy(dataString, "0");

      //blink LED to indicate data collection has started
      lightCount++;
      if (lightCount > 10 ) {
        if (LEDSTATE == true) {
          digitalWrite(LEDPin, LOW);
          LEDSTATE = false;
        } else {
          digitalWrite(LEDPin, HIGH);
          LEDSTATE = true;
        }
        lightCount = 0;
      }


    } else {
      //if start pushbutton has not been depressed and everything must be zeroed.
      qResponse[0] = 0;
      strcpy(dataString, "0");
      Serial.println("OFF");
      digitalWrite(LEDPin, HIGH); //turn light on so we know we're running
      timeStart = millis();
      ROBO.println("!EX");   //emerg stop to change hysteresis drag
      predictedClass = ID.classify(0, 0, buttonFlag, turboStatus);      //needed to return zero from intention_detection so previous isn't saved
    }
    //needed for timer to function
    newTimer = newTimer - 50;
  }
}

/*
   Function that finds the start and end characters of the received UART message from the bluetooth module of the left wheel
*/
void recBLELeft() {
  int ndx = 0;
  char rc;
  if (HWSERIAL_L.available() > 0) {
    while (HWSERIAL_L.available() > 0) {
      rc = HWSERIAL_L.read();
      //      Serial.print(rc);
      receivedCharsLeft[ndx] = rc;
      ndx++;
    }
  }
}

/*
   Function that finds the start and end characters of the received UART message from the bluetooth module of the right wheel
*/
void recBLERight() {
  int ndx = 0;
  char rc;
  if (HWSERIAL_R.available() > 0) {
    while (HWSERIAL_R.available() > 0) {
      rc = HWSERIAL_R.read();
      //      Serial.print(rc);
      receivedCharsRight[ndx] = rc;
      ndx++;
    }
  }
}


/*
 * processes the UART message from the left bluetooth module. The last part of the message is the pushrim motor command.
 * This code pulls this number out and processes it for use in the intention detection code.
 */
void processWithStartEndMarkerLeft() {
  char startMarker = 'S';
  char endMarker = 'E';
  int buffCount = 0;
  int pndx = 0;
  int startFlag = false;
  int rCnt = 0;
  int mCnt = 0;
  char motorTempLeft[6];      //temp char array for backwards array
  int szTemp = 0;
  int zCnt = 0;

  for (pndx = 0; pndx < numChars; pndx++) {
    if (receivedCharsLeft[pndx] == startMarker) {
      startFlag = true;
    } else if (startFlag == true && receivedCharsLeft[pndx] != endMarker) {  //start message
      newBuffLeft[buffCount] = receivedCharsLeft[pndx];
      buffCount++;
    } else if (receivedCharsLeft[pndx] == endMarker && startFlag == true) {
      newBuffLeft[buffCount] = '\0';
      break;
    }
  }

  //pull out last part of message
  for (rCnt = (buffCount - 2); rCnt > 0; rCnt--) {  //count backwards through message
    if (newBuffLeft[rCnt] == ',') {
      motorTempLeft[mCnt] = '\0';
      mCnt++;
      break;
    } else {
      motorTempLeft[mCnt] = newBuffLeft[rCnt];
      mCnt++;
    }
  }
  //motorLeft has come in backwards. We need to swap it around
  szTemp = mCnt - 1;
  for (zCnt = 0; zCnt < mCnt; zCnt++) {
    if (szTemp == 0) {
      motorLeft[zCnt] = '\0';
    } else {
      motorLeft[zCnt] = motorTempLeft[szTemp - 1];
      szTemp--;
    }
  }

}

/*
 * processes the UART message from the right bluetooth module. The last part of the message is the pushrim motor command.
 * This code pulls this number out and processes it for use in the intention detection code.
 */
void processWithStartEndMarkerRight() {
  char startMarker = 'S';
  char endMarker = 'E';
  int buffCount = 0;
  int pndx = 0;
  int startFlag = false;
  int rCnt = 0;
  int mCnt = 0;
  char motorTempRight[6];      //temp char array for backwards array
  int szTemp = 0;
  int zCnt = 0;

  for (pndx = 0; pndx < numChars; pndx++) {
    if (receivedCharsRight[pndx] == startMarker) {
      startFlag = true;
    } else if (startFlag == true && receivedCharsRight[pndx] != endMarker) {  //start message
      newBuffRight[buffCount] = receivedCharsRight[pndx];
      buffCount++;
    } else if (receivedCharsRight[pndx] == endMarker && startFlag == true) {
      newBuffRight[buffCount] = '\0';
      break;
    }
  }

  //pull out last part of message
  for (rCnt = (buffCount - 2); rCnt > 0; rCnt--) {  //count backwards through message
    if (newBuffRight[rCnt] == ',') {
      motorTempRight[mCnt] = '\0';
      mCnt++;
      break;
    } else {
      motorTempRight[mCnt] = newBuffRight[rCnt];
      mCnt++;
    }

  }


  //motorRight has come in backwards. We need to swap it around
  szTemp = mCnt - 1;
  for (zCnt = 0; zCnt < mCnt; zCnt++) {
    if (szTemp == 0) {
      motorRight[zCnt] = '\0';
    } else {
      motorRight[zCnt] = motorTempRight[szTemp - 1];
      szTemp--;
    }
  }

}

//function to calculate motor input for left wheel
void calcMotorLeft() {
  motorCmdBitLeft = atoi(motorLeft);
  motorCmdFloatLeft = float(motorCmdBitLeft);
  PL.sortBitLeft();
}
//function to calculate motor input for right wheel
void calcMotorRight() {
  motorCmdBitRight = atoi(motorRight);
  motorCmdFloatRight = float(motorCmdBitRight);
  PR.sortBitRight();
}
