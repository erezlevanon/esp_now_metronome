#include <esp_now.h>
#include <WiFi.h>
#include <AccelStepper.h>
#include <Wire.h>


// Define click.
#define CLICK_PIN 5
#define CLICK_DURATION_MS 50

// Stepper definitions
#define DIR_PIN 19   // Direction
#define STEP_PIN 18  // Step


// kamea homing//

//------------------------------------------------------------------------------------
//---AS5600 magnetic position encoder---
//------------------------------------------------------------------------------------
int magnetStatus = 0;  //value of the status register (MD, ML, MH)

int lowbyte;     //raw angle 7:0
word highbyte;   //raw angle 7:0 and 11:8
int rawAngle;    //final raw angle
float degAngle;  //raw angle in degrees (360/4096 * [value between 0-4095])

int quadrantNumber, previousquadrantNumber;  //quadrant IDs
float numberofTurns = 0;                     //number of turns
float correctedAngle = 0;                    //tared angle - based on the startup value
float startAngle = 0;                        //starting angle
float totalAngle = 0;                        //total absolute angular displacement
float previoustotalAngle = 0;                //for the display printing
unsigned long encoderTimer = 0;

//--
//PID parameters - tuned by the user
float proportional = 1.95;  //k_p = 1.35
float integral = 0.005;     //k_i = 0.005
float derivative = 0.1;     //k_d = 0.01
float controlSignal = 0;    //u - Also called as process variable (PV)
int motorDirection = 0;     //direction value 0: CCW, 1: CW. - Stored value
//-----------------------------------
//-----------------------------------
//PID-related
//Target values - Also called as setpoint!
float targetPosition = 123;  //the PID will try to reach this value. In this example it is the total angle
float previousTime = 0;      //for calculating delta t
float previousError = 0;     //for calculating the derivative (edot)
float errorIntegral = 0;     //integral error
float currentTime = 0;       //time in the moment of calculation
float deltaTime = 0;         //time difference
float errorValue = 0;        //error
float edot = 0;              //derivative (de/dt)


const float steps_per_revolution = 1600.0f;
const float gear_reduction = 0.333f;
const float angle_fraction_of_circle = 0.4f;
const int full_movement_steps = steps_per_revolution * (angle_fraction_of_circle / gear_reduction);
const int one_side_movement = full_movement_steps / 2;
// Should match code in the transmitter.
const uint8_t min_recieved_pos = 0;
const uint8_t max_recieved_pos = 255;

//-------------------------------------------------------------------------------------

// DO NOT TOUCH - data structure defined also in transmitter.
typedef struct metronom_struct {
  int position;
  int8_t direction;
  bool trigger_click;
} metronom_struct;

metronom_struct metronom;

// Stepper definition
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

// Variables to manage click, do not touch.
bool click_now = false;
long click_start = 0;

// Callback function that will be executed when data is received.
void OnDataRecv(const esp_now_recv_info *r_info, const unsigned char *incomingData, int len) {
  memcpy(&metronom, incomingData, sizeof(metronom));
  stepper.moveTo(metronom.position);
  click_now = metronom.trigger_click;
  if (click_now) {
    click_start = millis();
    digitalWrite(CLICK_PIN, HIGH);
  }
  // Serial.println(target);
}

void setup() {
  // Setup serial
  Serial.begin(115200);

  // Setup stepper
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  stepper.setMaxSpeed(2000.0);
  stepper.setAcceleration(1500.0);

  // Set home position for stepper motor.
  Wire.begin();            //start i2C  //PB6 = SCL, PB7 = SDA
  Wire.setClock(800000L);  //fast clock
  checkMagnetPresence();   //check the magnet (blocks until magnet is found)
  set_home();

  // Set click
  pinMode(CLICK_PIN, OUTPUT);
  digitalWrite(CLICK_PIN, LOW);


  //Init ESP-NOW
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void set_home() {
  // TODO: Put here the code for moving the arm.
  ReadRawAngle();   //make a reading so the degAngle gets updated
  correctAngle();   //tare the value
  checkQuadrant();  //check quadrant, check rotations, check absolute angular position
  while (abs(targetPosition - totalAngle) > 0.5) {
    calculatePID();
    driveMotor();
    stepper.runSpeed();
    ReadRawAngle();                //ask the value from the sensor
    correctAngle();                //tare the value
    checkQuadrant();               //check quadrant, check rotations, check absolute angular position
    Serial.print("Target:");       //Label
    Serial.print(targetPosition);  //Variable
    Serial.print(" degAngle:");    //Label
    Serial.print(degAngle);        //Variable
    Serial.print(",Actual:");      //Label
    Serial.println(totalAngle);    //Variable
  }
  stepper.setCurrentPosition(0);
}

void loop() {
  if ((millis() - click_start) > CLICK_DURATION_MS) {
    digitalWrite(CLICK_PIN, LOW);
    click_now = false;
  }
  if (stepper.distanceToGo() != 0) {
    stepper.run();
  }
}

//AS5600 --- BEGIN ---
void ReadRawAngle() {
  //7:0 - bits
  Wire.beginTransmission(0x36);  //connect to the sensor
  Wire.write(0x0D);              //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission();        //end transmission
  Wire.requestFrom(0x36, 1);     //request from the sensor

  while (Wire.available() == 0)
    ;                     //wait until it becomes available
  lowbyte = Wire.read();  //Reading the data after the request

  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);  //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);

  while (Wire.available() == 0)
    ;
  highbyte = Wire.read();

  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8;  //shifting to left
  //What is happening here is the following: The variable is being shifted by 8 bits to the left:
  //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
  //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in

  //Finally, we combine (bitwise OR) the two numbers:
  //High: 00001111|00000000
  //Low:  00000000|00001111
  //      -----------------
  //H|L:  00001111|00001111
  rawAngle = highbyte | lowbyte;  //int is 16 bits (as well as the word)

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle = rawAngle * 0.087890625;

  //Serial.print("Deg angle: ");
  //Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle
}

void correctAngle() {
  //recalculate angle
  correctedAngle = degAngle - startAngle;  //this tares the position

  if (correctedAngle < 0)  //if the calculated angle is negative, we need to "normalize" it
  {
    correctedAngle = correctedAngle + 360;  //correction for negative numbers (i.e. -15 becomes +345)
  } else {
    //do nothing
  }
  //Serial.print(" Corrected angle: ");
  //Serial.println(correctedAngle, 2);  //print the corrected/tared angle
}

void checkQuadrant() {
  /*
    //Quadrants:
    4  |  1
    ---|---
    3  |  2
  */

  //Quadrant 1
  if (correctedAngle >= 0 && correctedAngle <= 90) {
    quadrantNumber = 1;
  }

  //Quadrant 2
  if (correctedAngle > 90 && correctedAngle <= 180) {
    quadrantNumber = 2;
  }

  //Quadrant 3
  if (correctedAngle > 180 && correctedAngle <= 270) {
    quadrantNumber = 3;
  }

  //Quadrant 4
  if (correctedAngle > 270 && correctedAngle < 360) {
    quadrantNumber = 4;
  }
  //Serial.print("Quadrant: ");
  //Serial.println(quadrantNumber); //print our position "quadrant-wise"

  if (quadrantNumber != previousquadrantNumber)  //if we changed quadrant
  {
    if (quadrantNumber == 1 && previousquadrantNumber == 4) {
      numberofTurns++;  // 4 --> 1 transition: CW rotation
    }

    if (quadrantNumber == 4 && previousquadrantNumber == 1) {
      numberofTurns--;  // 1 --> 4 transition: CCW rotation
    }
    //this could be done between every quadrants so one can count every 1/4th of transition

    previousquadrantNumber = quadrantNumber;  //update to the current quadrant
  }
  //Serial.print("Turns: ");
  //Serial.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)

  //after we have the corrected angle and the turns, we can calculate the total absolute position
  totalAngle = (numberofTurns * 360) + correctedAngle;  //number of turns (+/-) plus the actual angle within the 0-360 range
  //Serial.print("Total angle: ");
  //Serial.println(totalAngle, 2); //absolute position of the motor expressed in degree angles, 2 digits
}

void checkMagnetPresence() {
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  while ((magnetStatus & 32) != 32)  //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0;  //reset reading

    Wire.beginTransmission(0x36);  //connect to the sensor
    Wire.write(0x0B);              //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission();        //end transmission
    Wire.requestFrom(0x36, 1);     //request from the sensor

    while (Wire.available() == 0)
      ;                          //wait until it becomes available
    magnetStatus = Wire.read();  //Reading the data after the request

    Serial.print("Magnet status: ");
    Serial.println(magnetStatus, BIN);  //print it in binary so you can compare it to the table (fig 21)
  }

  //Status register output: 0 0 MD ML MH 0 0 0
  //MH: Too strong magnet - 100111 - DEC: 39
  //ML: Too weak magnet - 10111 - DEC: 23
  //MD: OK magnet - 110111 - DEC: 55

  Serial.println("Magnet found!");
  delay(1000);
}

//AS5600 --- END ---

void driveMotor() {
  //Driving the motor is done via UART control.
  //It is smoother than polling the pins with the AccelStepper library

  //Determine speed and direction based on the value of the control signal

  //direction determination, can be removed if Accelstepper is used
  /*
  if (controlSignal < 0) //negative value: CCW
  {
    shaft = false; //Set the direction to CCW
  }
  else if (controlSignal > 0) //positive: CW
  {
    shaft = true; //Set the direction to CW    
  }
  */
  //controlsignal = 0 is not handled on purpose


  //Resolution: 360/4096 = 0.0878 deg
  if (fabs(controlSignal) > 0.2)  //we are within +/- 1 degrees of the target, can be fine tuned
  {
    // driver.    shaft(shaft); //set direction
    // driver.VACTUAL(fabs(controlSignal) * 5); //set speed; The multiplier depends on the desired response speed
    //I added a multiplier to make the motor faster
    //However, maybe a correct set of PID parameters would work better

    //-----------------------------------------------------------------------------------
    //Accelstepper way:
    stepper.enableOutputs();                    //Enable power
    stepper.setSpeed(controlSignal * 20 * -1);  //Accelstepper way; The multiplier depends on the desired response speed
    //We let the controlSignal to directly be passed to the setSpeed() function
    //If the signal is too large, it can make the motor to act weird, so a capping might be a smart thing to do.
  } else {
    //driver.VACTUAL(0); //Set the motor speed to zero - STOP
    //-----------------------------------------------------------------------------------
    //Accelstepper implementation
    stepper.setSpeed(0);       //Set speed to 0
    stepper.stop();            //Stop
    stepper.disableOutputs();  //Disable power
  }
}

void calculatePID() {
  //Determining the elapsed time
  currentTime = micros();                                //current time
  deltaTime = (currentTime - previousTime) / 1000000.0;  //time difference in seconds
  previousTime = currentTime;                            //save the current time for the next iteration to get the time difference
  //---
  errorValue = totalAngle - targetPosition;  //Current position - target position (or setpoint)

  edot = (errorValue - previousError) / deltaTime;  //edot = de/dt - derivative term

  errorIntegral = errorIntegral + (errorValue * deltaTime);  //integral term - Newton-Leibniz, notice, this is a running sum!

  controlSignal = (proportional * errorValue) + (derivative * edot) + (integral * errorIntegral);  //final sum, proportional term also calculated here
  //Serial.print("Control Signal: ");
  //Serial.println(controlSignal);
  previousError = errorValue;  //save the error for the next iteration to get the difference (for edot)
}
