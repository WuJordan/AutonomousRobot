#pragma once
// #include <Arduino.h>
#include "Graph.hpp"
#include "ascii2graph.hpp"
#include "shortest_path.hpp"
#include "newGraphToMotion.hpp"


#include "Encoder.hpp"
#include "IMU.hpp"
#include "Motor.hpp"
#include "UltrasonicSensor.hpp"
#include "PIDController.hpp"
#include "MovingAverageFilter.hpp"
#include "MAFUltrasonicSensor.hpp"

#define ICM_20948_USE_DMP
// Set up motors.
mtrn3100::Motor l_motor(2, 4, 3);
mtrn3100::Motor r_motor(7, 5, 6);

mtrn3100::PIDController l_pos_pid(20, 0, 2, 0.05, 80, 45); // kd = 0.5
mtrn3100::PIDController r_pos_pid(20, 0, 2, 0.05, 80, 45); 

// Set up encoders.
void readLeftEncoder();
void readRightEncoder();
mtrn3100::Encoder l_encoder(18, 22, readLeftEncoder, 266);
mtrn3100::Encoder r_encoder(19, 23, readRightEncoder, 266);
void readLeftEncoder() { l_encoder.readEncoder(); }
void readRightEncoder() { r_encoder.readEncoder(); }

// Set up IMU
mtrn3100::IMU imu; // 0x68

// Assuming ultrasonic sensors are used but feel free to change these with LiDARs.
// Distance sensors.
mtrn3100::UltrasonicSensor usf(43, A14);
mtrn3100::UltrasonicSensor usl(41, A13);
mtrn3100::UltrasonicSensor usr(39, A12);
auto withinThreshold = [](float const distance) { return distance < 150; };

// Drive parameters.
constexpr uint8_t pwm = 80;
// constexpr uint16_t forwardDuration = 500;
// constexpr uint16_t turnDuration = 300;
constexpr float wRad = 24;
constexpr float axle = 131; //128
const float rev = 2 * M_PI;
int drivePlanLength;
char drivePlan[80];
int drivePlanIndex = 0;

// IMU global yaw
float yaw = 0.0;
float yawOffset = 0.0;
float trueYaw = 0.0;

// Ultrasonic Distances
float left;
float right;
float front;

constexpr float closeL = 44.5;
constexpr float farR = 63.5;

constexpr float closeR = 48.5;
constexpr float farL = 59.5;

constexpr float closeF = 25;
constexpr float farF = 45;

constexpr float centreF = 34;
constexpr float centreL = 52;
constexpr float centreR = 56;

const byte numChars = 75;
char receivedChars[numChars];
char maze[700] = "";
boolean newData = false;

int line = 0;

int start;
int dest;
int heading = 0;
int startHeading = heading;

enum CardinalDirections {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
};

const String headings = "NESW"; //array of char with the different heading types

#define THRESHOLD 150

// Front sensor
mtrn3100::UltrasonicSensor sensorF(43, A14);
mtrn3100::MAFUltrasonicSensor<20> maf_sensorF(sensorF);
// Left sensor
mtrn3100::UltrasonicSensor sensorL(41, A13);
mtrn3100::MAFUltrasonicSensor<20> maf_sensorL(sensorL);
// Right sensor
mtrn3100::UltrasonicSensor sensorR(39, A12);
mtrn3100::MAFUltrasonicSensor<20> maf_sensorR(sensorR);

bool isWall(double distance) {
        return distance < THRESHOLD;
}

void updateWallData() {
    front = maf_sensorF.value();
    left = maf_sensorL.value();
    right = maf_sensorR.value();
}

void sampleWalls() {
    // Sample all sensors
    for (int i = 1; i <=25; i++) {
        maf_sensorF.sample();
        maf_sensorR.sample();
        maf_sensorL.sample();
    }
    // Check if enough data is collect for output
    if (maf_sensorF.isReady() && maf_sensorL.isReady() && maf_sensorR.isReady()) {
        updateWallData();
    }
}

void checkCentreF() {
    if (front < closeF) {
      Serial3.println(F("Too close to front wall"));
      adjustCentreF();
    } else if (front > farF && isWall(front)) {
      Serial3.println(F("Too far to front wall"));
      adjustCentreF();
    }
}

bool checkCentreLR() {
    if (left < closeL || (right > farR && isWall(right))) {
      Serial3.println(F("Too close to left wall"));
      // adjustCentreLR();
      return false;
    } else if (right < closeR || (left > farL && isWall(left))) {
      Serial3.println(F("Too close to right wall"));
      // adjustCentreLR();
      return false;
    } else {
      return true;
    }
}

void adjustCentreF() {
    Serial3.print(F("Adjusting front distance: "));
    Serial3.println(front);
    float targetL = ((front - centreF)/(2*M_PI*(wRad * 0.997))) * rev;
    float targetR = ((front - centreF)/(2*M_PI*(wRad))) * rev;
    // float targetR = (250/(2*M_PI*(wRad*0.8))) * rev;
    //float target = rev;
    l_pos_pid.zeroEncoderAndSetTarget(l_encoder.position, targetL);
    r_pos_pid.zeroEncoderAndSetTarget(r_encoder.position, targetR);
    float leftMotorsignal = 1;
    float rightMotorsignal = 1;
    while (leftMotorsignal != 0 || rightMotorsignal != 0) {
      sampleIMU();
      leftMotorsignal = l_pos_pid.compute(l_encoder.position);
      rightMotorsignal = r_pos_pid.compute(r_encoder.position);
      r_motor.setPWM(rightMotorsignal);
      l_motor.setPWM(leftMotorsignal);
      
    }
    l_motor.setPWM(0);
    r_motor.setPWM(0);
}

void adjustForwardLR() {
    float dist;
    int direction = 1;
    if (isWall(right) && isWall(left)) {
      dist = (abs(left - centreL) + abs(right - centreR)) / 2.0;
      if (left < centreL) {
        direction = -1;
      } else {
        direction = 1;
      }
    } else if (isWall(right)) {
      dist = centreR - right;
    } else if (isWall(left)) {
      dist = left - centreL;
    }

    float targetAngle = direction * atan(dist/250);
    float num1 = dist*dist;
    unsigned int num2 = 250*250;
    float targetDist = sqrt(num1 + num2);

    if (targetAngle > 4.5) {
      targetAngle = 4.5;
    } else if (targetAngle < -4.5) {
      targetAngle = -4.5;
    }

    // Serial3.print(F("Target Distance is: "));
    // Serial3.println(targetDist);
    Serial3.println(F("Adjusting Left and Right"));
    Serial3.print(F("Distance is: "));
    Serial3.println(dist);
   
    // float targetDist = (dist*dist + 250*250)pow()

//turn the robot to the angle
//left is positive right is negtive
    Serial3.println(F("Setting Angle"));
    float target = ((axle/2*targetAngle)/(2*wRad*M_PI) * rev);
    l_pos_pid.zeroEncoderAndSetTarget(l_encoder.position, -target);
    r_pos_pid.zeroEncoderAndSetTarget(r_encoder.position, target);
    float leftMotorsignal = 1;
    float rightMotorsignal = 1;
    while (leftMotorsignal != 0 && rightMotorsignal != 0) {
      sampleIMU();
      leftMotorsignal = l_pos_pid.compute(l_encoder.position);
      rightMotorsignal = r_pos_pid.compute(r_encoder.position);
      l_motor.setPWM(leftMotorsignal);
      r_motor.setPWM(rightMotorsignal);
    }
    l_motor.setPWM(0);
    r_motor.setPWM(0);

// drive forwards calculated distance
    Serial3.println(F("Driving Adjusted Distance"));

    float targetL = (targetDist/(2*M_PI*(wRad * 0.997))) * rev;
    float targetR = (targetDist/(2*M_PI*(wRad))) * rev;
  
    l_pos_pid.zeroEncoderAndSetTarget(l_encoder.position, targetL);
    r_pos_pid.zeroEncoderAndSetTarget(r_encoder.position, targetR);
    leftMotorsignal = 1;
    rightMotorsignal = 1;
    while (leftMotorsignal != 0 || rightMotorsignal != 0) {
      sampleIMU();
      leftMotorsignal = l_pos_pid.compute(l_encoder.position);
      rightMotorsignal = r_pos_pid.compute(r_encoder.position);
      r_motor.setPWM(rightMotorsignal);
      l_motor.setPWM(leftMotorsignal);
    }
    l_motor.setPWM(0);
    r_motor.setPWM(0);
}

void sampleIMU() {
    // if (imu.dataReady()) {
        imu.read();
        yaw = imu.yaw();
        trueYaw = yaw - yawOffset;
    // }
}

void findOffset() {
    int count = 0;
    while(count < 2000) { //sample the imu till the value is stable
        sampleIMU();
        count++;
    }
    yawOffset = yaw;
}

void checkYaw() {
  //checks the Yaw of the device
  int adjustedHeading = heading - startHeading;
  float correction = 0;
  // Serial3.println(F("Checking Yaw"));
  if (adjustedHeading < 0) {
    adjustedHeading += 4;
  }
  if (adjustedHeading == 0 && (trueYaw > 2.5 || trueYaw < -2.5)) {
    adjustYaw(-trueYaw);
  } else if (adjustedHeading == 1 && (trueYaw > -87.5 || trueYaw < -92.5)) {
    float correction = trueYaw + 90;
    adjustYaw(-correction);
  } else if (adjustedHeading == 2 && (trueYaw > -177.5 || trueYaw < 177.5 || trueYaw > 182.5)) {
    if (trueYaw < 0) {
      float correction = -(180 + trueYaw);
    } else {
      float correction = 180 - trueYaw;
    }
    adjustYaw(correction);
  } else if (adjustedHeading == 3 && (trueYaw > 92.5 || trueYaw < 87.5)) {
    float correction = trueYaw - 90;
    adjustYaw(-correction);
  }

}

void adjustYaw(float correction) {
  Serial3.print(F("Yaw Needs Adjustment: "));
  Serial3.println(correction);
  
  if (abs(correction) > 10) {
    correction = 0;
  }
  
  float target = ((axle/2*(correction*M_PI/180))/(2*wRad*M_PI) * rev);
  l_pos_pid.zeroEncoderAndSetTarget(l_encoder.position, -target);
  r_pos_pid.zeroEncoderAndSetTarget(r_encoder.position, target);
  float leftMotorsignal = 1;
  float rightMotorsignal = 1;
  while (leftMotorsignal != 0 && rightMotorsignal != 0) {
    sampleIMU();
    leftMotorsignal = l_pos_pid.compute(l_encoder.position);
    rightMotorsignal = r_pos_pid.compute(r_encoder.position);
    l_motor.setPWM(leftMotorsignal);
    r_motor.setPWM(rightMotorsignal);
      
    }
    l_motor.setPWM(0);
    r_motor.setPWM(0);
}

// COMPLETE THIS FUNCTION.
// Drive forward then update the robot maze cell pose.

void forward() {
    float targetL = (250/(2*M_PI*(wRad * 0.997))) * rev;
    float targetR = (250/(2*M_PI*(wRad))) * rev;
    l_pos_pid.zeroEncoderAndSetTarget(l_encoder.position, targetL);
    r_pos_pid.zeroEncoderAndSetTarget(r_encoder.position, targetR);

    float leftMotorsignal = 1;
    float rightMotorsignal = 1;
    while (leftMotorsignal != 0 && rightMotorsignal != 0) {
      sampleIMU();
      leftMotorsignal = l_pos_pid.compute(l_encoder.position);
      rightMotorsignal = r_pos_pid.compute(r_encoder.position);
      r_motor.setPWM(rightMotorsignal);
      l_motor.setPWM(leftMotorsignal);
      
    }
    l_motor.setPWM(0);
    r_motor.setPWM(0);
    
}

// Turn left then update the robot maze cell pose.
// COMPLETE THIS FUNCTION.
void turnLeft() {
    float target = ((axle/4*M_PI)/(2*wRad*M_PI) * rev);
    l_pos_pid.zeroEncoderAndSetTarget(l_encoder.position, -target);
    r_pos_pid.zeroEncoderAndSetTarget(r_encoder.position, target);

    // pose change when turning left
    heading--;
    if (heading == -1) {
        heading = 3;
    }

    
    float leftMotorsignal = 1;
    float rightMotorsignal = 1;
    Serial.print(F("Start IMU Sampling Left: "));
    while (leftMotorsignal != 0 && rightMotorsignal != 0) {
      sampleIMU();
      leftMotorsignal = l_pos_pid.compute(l_encoder.position);
      rightMotorsignal = r_pos_pid.compute(r_encoder.position);
      l_motor.setPWM(leftMotorsignal);
      r_motor.setPWM(rightMotorsignal);
      
    }
    l_motor.setPWM(0);
    r_motor.setPWM(0);
}

// COMPLETE THIS FUNCTION.
// Turn right then update the robot maze cell pose.
void turnRight() {
    float target = ((axle/4*M_PI)/(2*wRad*M_PI) * rev);
    // float target = ((axle/2*(M_PI/2+M_PI/6))/(2*wRad*M_PI) * rev);
    l_pos_pid.zeroEncoderAndSetTarget(l_encoder.position, target);
    r_pos_pid.zeroEncoderAndSetTarget(r_encoder.position, -target);

    //pose change when turning right
    heading++;
    if (heading == 4) {
        heading = 0;
    }

    float leftMotorsignal = 1;
    float rightMotorsignal = 1;
    Serial.print(F("Start IMU Sampling Right: "));
    while (leftMotorsignal != 0 && rightMotorsignal != 0) {
      sampleIMU();
      leftMotorsignal = l_pos_pid.compute(l_encoder.position);
      rightMotorsignal = r_pos_pid.compute(r_encoder.position);
      l_motor.setPWM(leftMotorsignal);
      r_motor.setPWM(rightMotorsignal);
      
    }
    l_motor.setPWM(0);
    r_motor.setPWM(0);
}

void recvWithStartEndMarkers() { //receives the strings and seperates the strings by new line using < as start marker and > as end marker
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial3.available() > 0 && newData == false) {
        rc = Serial3.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
    
    if (line < 11) {
        updateMaze();
    } else if (line == 12) {
        updateStartDest();
    }
    
}

void updateMaze() { // adds the received data into the char array maze with \n and \0
    if (newData == true) {
        strcat(maze, receivedChars);
        if (line == 11) {
            strcat(maze, "\0"); // terminate the string
            Serial.println(F("end"));
        } else {
            strcat(maze, "\n");
        }
        newData = false;
        line++;
    }
}

void updateStartDest () {
    if (newData == true) {
        Serial.println(receivedChars);
        char startChar[] = "00";
        char destChar[] = "00";
        startChar[0] = receivedChars[0];
        startChar[1] = receivedChars[1];
        destChar[0] = receivedChars[2];
        destChar[1] = receivedChars[3];
        start = atoi(startChar);
        dest = atoi(destChar);
        if (receivedChars[4] == 'N') {
            heading = 0;
        } 
        else if (receivedChars[4] == 'E') {
            heading = 1;
        }
        else if (receivedChars[4] == 'S') {
            heading = 2;
        }
        else if (receivedChars[4] == 'W') {
            heading = 3;
        }
        line++;
    }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial3.begin(9600);
  DoubleLinked paths;
  while (line < 13) {

    recvWithStartEndMarkers();
    if (line == 11) {
      Serial3.println(F("Received maze. Send Start and Destination"));
      line++;
      Serial.println(line);
    }
  }
  if (line == 13) {
    Serial.print(F("Start: "));
    Serial.println(start);
    Serial.print(F("Destination: "));
    Serial.println(dest);
    Serial.print(F("Heading: "));
    Serial.println(heading);
    line++;
    auto const actual = mtrn3100::ascii2graph(maze); // creates the graph
    Serial.println(F("Got a graph"));
    paths = bfs_multiple(actual, start, dest);
    Serial.print(F("Path size: "));
    Serial.println(paths.size());
    Serial3.print(F("Path size: "));
    Serial3.println(paths.size());
    for (auto path = paths.begin(); path != paths.end(); path++) {
        auto elm = path->value;
        for (auto p = elm.begin(); p != elm.end(); p++) {
          Serial3.print(p->value);
          Serial3.print(F(" "));
        }
        Serial3.println();
      }
  }

    Serial3.println(F("Got a graph"));
    String plan = mtrn3100::makeMotionPlan(paths, start, dest, heading, 80);
    drivePlanLength = plan.length();
    plan.toCharArray(drivePlan, sizeof(drivePlan));
    Serial3.println(F("Best path"));
    Serial3.println(drivePlan);
    Serial.println(F("At the end"));
    Wire.begin();  // This must be called before IMU::begin().
    imu.begin();
    imu.calibrateAcceleration();
    while (imu.dataReady());
    imu.reset();
    
    Serial3.println(F("Booting Up"));
    Serial.println(F("Booting Up"));
    
    findOffset();
    
    Serial3.println(F("Begin Driving"));
}

void loop() {
  // put your main code here, to run repeatedly:
  sampleWalls();

    switch (drivePlan[drivePlanIndex]) {
        case 'F':
            // have two methods of forwards            
            Serial3.println(F("Driving Forwards"));
            if (!checkCentreLR()) {
              Serial3.println(F("Needs forwards Adjusting"));
              adjustForwardLR();
            } else {
              Serial3.println(F("No forwards Adjusting Needed"));
              forward();
            }
            checkYaw();
            break;
        case 'L':
            Serial3.println(F("Turning Left"));
            
            checkCentreF();
            turnLeft();
            checkYaw();
            
            break;
        case 'R':
            Serial3.println(F("Turning Right"));
            checkCentreF();
            turnRight();
            checkYaw();
            break;
    }
    // Increment index
    drivePlanIndex++;
    // delay by something
    // delay(1000);
    int delay = 0;
    while(delay < 50) {
      sampleIMU();
      delay++;
    }
    // sampleWalls
    sampleWalls();

    // delay(100);
    
    // // Capture wall information.
    const bool frontConnected = !isWall(maf_sensorF.value());
    const bool leftConnected = !isWall(maf_sensorL.value());
    const bool rightConnected = !isWall(maf_sensorR.value());

    // delay(1000);

    // // Stop program if drive plan is complete.
    if (drivePlanIndex == drivePlanLength) {
        while (1) {
        }
    }
}