#include <Arduino.h>
#define ICM_20948_USE_DMP

#include "Encoder.hpp"
#include "IMU.hpp"
#include "Graph.hpp"
#include "Motor.hpp"
#include "UltrasonicSensor.hpp"
#include "graph2ascii.hpp"
#include "PIDController.hpp"
#include "MovingAverageFilter.hpp"
#include "MAFUltrasonicSensor.hpp"



// Set up motors.
mtrn3100::Motor l_motor(2, 4, 3);
mtrn3100::Motor r_motor(7, 5, 6);

// mtrn3100::PIDController l_pos_pid(4, 0.0, 0.25, 0.1, 210, 55); // TODO: Not too sure if we have to touch these
// mtrn3100::PIDController r_pos_pid(4, 0.0, 0.25, 0.1, 210, 55); // 70, 0, 50, 0.2 | 50, 0, 50, 0.2

mtrn3100::PIDController l_pos_pid(20, 0, 2, 0.05, 80, 45); // kd = 0.5
mtrn3100::PIDController r_pos_pid(20, 0, 2, 0.05, 80, 45);

// Set up IMU
mtrn3100::IMU imu; // 0x68

// Set up encoders.
void readLeftEncoder();
void readRightEncoder();
mtrn3100::Encoder l_encoder(18, 22, readLeftEncoder);
mtrn3100::Encoder r_encoder(19, 23, readRightEncoder);
void readLeftEncoder() { l_encoder.readEncoder(); }
void readRightEncoder() { r_encoder.readEncoder(); }

// COMPLETE THIS BLOCK.
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
constexpr float axle = 131;
// constexpr float axle = 128;
const float rev = 2 * M_PI;

String incomingByte;

size_t cellsExplored = 0;

enum CardinalDirections {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
};

// // Robot pose using maze cell coordinates and heading.
// int row = 0;
// int col = 0;
// int head = SOUTH;

// variables for the pose of the robot
int row = 0;
int col = 0;
int heading = 2;
int startHeading = 0;
const String headings = "NESW"; //array of char with the different heading types

float yaw = 0.0;
float yawOffset = 0.0;
float trueYaw = 0.0;

// Ultrasonic Distances
float left;
float right;
float front;

// constexpr float closeL = 42;
// constexpr float farR = 66;

constexpr float closeL = 44.5;
constexpr float farR = 63.5;

// constexpr float closeR = 46;
// constexpr float farL = 62;

constexpr float closeR = 48.5;
constexpr float farL = 59.5;

// constexpr float closeF = 25;
// constexpr float farF = 45;

constexpr float closeF = 25;
constexpr float farF = 45;

constexpr float centreF = 34;
constexpr float centreL = 52;
constexpr float centreR = 56;
// distance one each side till it reaches too close to the wall is about 18 for both right and left sensors

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
    for (int i = 1; i <=20; i++) {
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
      // Serial3.println("Too close to front wall");
      adjustCentreF();
    } else if (front > farF && isWall(front)) {
      // Serial3.println("Too far to front wall");
      adjustCentreF();
    }
}

bool checkCentreLR() {
    if (left < closeL || (right > farR && isWall(right))) {
      // Serial3.println("Too close to left wall");
      // adjustCentreLR();
      return false;
    } else if (right < closeR || (left > farL && isWall(left))) {
      // Serial3.println("Too close to right wall");
      // adjustCentreLR();
      return false;
    } else {
      return true;
    }
}

void adjustCentreF() {
    float targetL = ((front - centreF)/(2*M_PI*(wRad*0.997))) * rev;
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

    // Serial3.print("Target Distance is: ");
    // Serial3.println(targetDist);
    // Serial3.print("Distance is: ");
    // Serial3.println(dist);
   
    // float targetDist = (dist*dist + 250*250)pow()

//turn the robot to the angle
//left is positive right is negtive
    // Serial3.println("Setting Angle");
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
    Serial3.println("Driving Adjusted Distance");
    float targetL = (targetDist/(2*M_PI*(wRad*0.997))) * rev;
    float targetR = (targetDist/(2*M_PI*(wRad))) * rev;
    // float targetR = (250/(2*M_PI*(wRad*0.8))) * rev;
    //float target = rev;
    l_pos_pid.zeroEncoderAndSetTarget(l_encoder.position, targetL);
    r_pos_pid.zeroEncoderAndSetTarget(r_encoder.position, targetR);
    leftMotorsignal = 1;
    rightMotorsignal = 1;
    if (heading == 0) {
        row--;
    } else if (heading == 1) {
        col++;
    } else if (heading == 2) {
        row++;
    } else if (heading == 3) {
        col--;
    }
    while (leftMotorsignal != 0 || rightMotorsignal != 0) {
      // Serial3.println("Enters While loop");
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
        Serial.print("h: ");
        // Serial.println(imu.yaw());
        // Serial.print(",");
        Serial.println(yaw);
        yaw = imu.yaw();
        // yaw = yaw - yawOffset;
        trueYaw = yaw - yawOffset;
    // }
}

void findOffset() {
    int count = 0;
    while(count < 2000) { //sample the imu till the value is stable
        sampleIMU();
        count++;
        Serial.println(count);
    }
    yawOffset = yaw;
    // Serial3.print("Yaw Start: ");
    // Serial3.println(yaw);
    // Serial3.print("Yaw Offset: ");
    // Serial3.println(yawOffset);
}

void checkYaw() {
  //checks the Yaw of the device
  int adjustedHeading = heading - startHeading;
  float correction = 0;
  // Serial3.println("Checking Yaw");
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
  // Serial3.print("Yaw Needs Adjustment: ");
  // Serial3.println(correction);
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
    // float target = (250/(2*M_PI*(wRad))) * rev;
    float targetL = (250/(2*M_PI*(wRad*0.997))) * rev;
    float targetR = (250/(2*M_PI*(wRad))) * rev;
    // float targetR = (250/(2*M_PI*(wRad*0.92))) * rev;
    //float target = rev;
    l_pos_pid.zeroEncoderAndSetTarget(l_encoder.position, targetL);
    r_pos_pid.zeroEncoderAndSetTarget(r_encoder.position, targetR);
    // pose change when moving forwards
    if (heading == 0) {
        row--;
    } else if (heading == 1) {
        col++;
    } else if (heading == 2) {
        row++;
    } else if (heading == 3) {
        col--;
    }

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
    l_pos_pid.zeroEncoderAndSetTarget(l_encoder.position, target);
    r_pos_pid.zeroEncoderAndSetTarget(r_encoder.position, -target);

    //pose change when turning right
    heading++;
    if (heading == 4) {
        heading = 0;
    }

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

void delayIMU() {
  int delay = 0;
  while(delay < 50) {
    sampleIMU();
    delay++;
  }
}

// Initialise maze with nodes.
const int numRows = 5;
const int numCols = 9;
    int p11{1}, p12{2}, p13{3}, p14{4}, p15{5}, p16{6}, p17{7}, p18{8}, p19{9};
    int p21{10}, p22{11}, p23{12}, p24{13}, p25{14}, p26{15}, p27{16}, p28{17}, p29{18};
    int p31{19}, p32{20}, p33{21}, p34{22}, p35{23}, p36{24}, p37{25}, p38{26}, p39{27};
    int p41{28}, p42{29}, p43{30}, p44{31}, p45{32}, p46{33}, p47{34}, p48{35}, p49{36};
    int p51{37}, p52{38}, p53{39}, p54{40}, p55{41}, p56{42}, p57{43}, p58{44}, p59{45};
mtrn3100::Graph<int, int> maze(p11, p12, p13, p14, p15, p16, p17, p18, p19, p21, p22, p23, p24, p25, p26, p27,
                               p28, p29, p31, p32, p33, p34, p35, p36, p37, p38, p39, p41, p42, p43, p44, p45,
                               p46, p47, p48, p49, p51, p52, p53, p54, p55, p56, p57, p58, p59);

// Helper function to convert node positions to node indexes.
// Returns -1 if the position is invalid.
auto pos2index = [](int const row, int const col) -> int {
    // Verify row/col is valid.
    if ((row < 0 || row >= numRows) || (col < 0 || col >= numCols)) {
        return -1;
    }
    return row * numCols + col + 1;
};

// Helper function to insert edges into maze.
// Graph is a directed graph so need to do both directions.
void insert_edge(int cell1, int cell2) {
    maze.insert_edge(cell1, cell2, true);
    maze.insert_edge(cell2, cell1, true);
}

void setup() {
    // COMPLETE THIS SETUP.
    // You may need to do some more setup depending on hardware your team has selected.

    Serial.begin(115200);
    Serial3.begin(9600);
    startHeading = heading;

    Wire.begin();  // This must be called before IMU::begin().
    imu.begin();
    imu.calibrateAcceleration();
    while (imu.dataReady())
        ;
    imu.reset();
    findOffset();
}

void loop() {
    // COMPLETE THIS LOOP.
    // This loop has been partially completed for you and uses the basic drive_by_command sketch.
    // Follow the comments which helps to keep the structure of the control loop.

    // sampleWalls();

    // Get next motion.
    // sampleWalls
    sampleWalls();
    
    // // Capture wall information.s
    const bool frontConnected1 = !isWall(maf_sensorF.value());
    const bool leftConnected1 = !isWall(maf_sensorL.value());
    const bool rightConnected1 = !isWall(maf_sensorR.value());

    if (cellsExplored == 0) {
      if (frontConnected1) {
        insert_edge(1, 10);
      }
      
      if (leftConnected1) {
        insert_edge(1, 2);
      }
    }
    
    Serial.print("cells1: ");
    Serial.println(cellsExplored);
    // Drive Algorithm (Hug right wall)

    if (rightConnected1) {
        // Turn right, Move forward
        checkCentreF();
        turnRight();
        checkYaw();
        sampleWalls();
        // delayIMU();

        if (!checkCentreLR()) {
            // Serial3.println("Needs forwards Adjusting");
            adjustForwardLR();
        } else {
            // Serial3.println("No forwards Adjusting Needed");
            forward();
        }
        checkYaw();

        // delayIMU();
    }

    else if (frontConnected1) {
        // Move forward
        if (!checkCentreLR()) {
            // Serial3.println("Needs forwards Adjusting");
            adjustForwardLR();
        } else {
            // Serial3.println("No forwards Adjusting Needed");
            forward();
        }
        checkYaw();
        
        // delayIMU();
    }

    else if (leftConnected1) {
        // Turn left, move forward
        checkCentreF();
        turnLeft();
        checkYaw();

        // delayIMU();
        
        if (!checkCentreLR()) {
            // Serial3.println("Needs forwards Adjusting");
            adjustForwardLR();
        } else {
            // Serial3.println("No forwards Adjusting Needed");
            forward();
        }
        checkYaw();
        
        // delayIMU();
    }

    else {
        // Turn around (left + left), move forward
        
        checkCentreF();
        turnLeft();
        checkYaw();
        sampleWalls();
        // delayIMU();

        checkCentreF();
        turnLeft();
        checkYaw();
        sampleWalls();
        // delayIMU();

        if (!checkCentreLR()) {
            // Serial3.println("Needs forwards Adjusting");
            adjustForwardLR();
        } else {
            // Serial3.println("No forwards Adjusting Needed");
            forward();
        }
        checkYaw();

        // delayIMU();
    
    }

    cellsExplored++;
    Serial.print("cells2: ");
    Serial.println(cellsExplored);
    // delayIMU();

    sampleWalls();
    // // Capture wall information.
    const bool frontConnected = !isWall(maf_sensorF.value());
    const bool leftConnected = !isWall(maf_sensorL.value());
    const bool rightConnected = !isWall(maf_sensorR.value());


    // // Check the heading which affects where front, left, and right cells are.
    // // Cell being -1 means no cell is connected.
    const int currentCell = pos2index(row, col);
    int frontCell = -1;
    int leftCell = -1;
    int rightCell = -1;
    if (heading == 0) { // North
        frontCell = pos2index(row - 1, col);
        leftCell = pos2index(row, col - 1);
        rightCell = pos2index(row, col + 1);
    }
    if (heading == 1) { // East
        frontCell = pos2index(row, col + 1);
        leftCell = pos2index(row - 1, col);
        rightCell = pos2index(row + 1, col);
    }
    if (heading == 2) { // South
        frontCell = pos2index(row + 1, col);
        leftCell = pos2index(row, col + 1);
        rightCell = pos2index(row, col - 1);
    }
    if (heading == 3) { // West
        frontCell = pos2index(row, col - 1);
        leftCell = pos2index(row + 1, col);
        rightCell = pos2index(row - 1, col);
    }

    // // Connect cells if there is no wall by inserting edges.
    if (frontConnected) {
        insert_edge(currentCell, frontCell);
        Serial3.print(currentCell);
        Serial3.print(" <-> ");
        Serial3.println(frontCell);
    }
    if (leftConnected) {
        insert_edge(currentCell, leftCell);
        Serial3.print(currentCell);
        Serial3.print(" <-> ");
        Serial3.println(leftCell);
    }
    if (rightConnected) {
        insert_edge(currentCell, rightCell);
        Serial3.print(currentCell);
        Serial3.print(" <-> ");
        Serial3.println(rightCell);
    }

    // Add delay so we know when robot has stopped.
    // delay(1000);

    // Stop program if drive plan is complete.
    if (cellsExplored == 50) {
        // Prints edges out.
        for (auto edge : maze.edges()) {
            Serial3.print(mtrn3100::get<0>(edge.value));
            Serial3.print(" <-> ");
            Serial3.println(mtrn3100::get<1>(edge.value));
        }
        Serial3.println("Final Maze: ");
        auto mazeOut = mtrn3100::graph2ascii(maze);
        Serial3.println(mazeOut);
        
        while (1) {
        }
    }
    if (Serial3.available() > 0) {
        incomingByte = Serial3.readString();
        if (incomingByte == "G") {
          auto mazeOut = mtrn3100::graph2ascii(maze);
          Serial3.println(mazeOut);
          while (1) {
          }
        } else {
          Serial3.println("INVALID COMMAND");
        }
    }
    // delayIMU();

}
