/*
 * Team Id: 2286
 * Author List: Vidhi Singh Rajput, Shreyansh Gupta, Shreyas Rathore, Udit Nayak
 * Filename: robot_control.c
 * Theme: Balancing Builder Bot
 * Functions: moveServo, Grip_close, Grip_open, arm_down, arm_up, forward, backward, balance, 
 *            mot_rencoder_left, mot_rencoder_right, controlMotor1, controlMotor2, setup, loop
 * Global Variables: mpu, gripservo, armservo, accX_offset, accY_offset, accZ_offset,
 *                   gyroX_offset, gyroY_offset, gyroZ_offset, inputPin1, inputPin2, inputPin3,
 *                   inputPin4, EN1, EN2, wheel_pulse_count_left, wheel_pulse_count_right,
 *                   wheel_distance_left, wheel_distance_right, Balance_flag, Kp_balance,
 *                   Ki_balance, Kd_balance, Kp_position, Ki_position, Kd_position, setPoint,
 *                   currentAngle, error, lastError, integral, derivative, output,
 *                   positionSetPoint, currentPosition, positionError, lastPositionError,
 *                   positionIntegral, positionDerivative, positionOutput, previousBTTime,
 *                   BTInterval, previousMPUTime, MPUInterval, wheel_radius, pulses_per_revolution.
 */

#include <MPU6050_light.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Servo.h>

// Define SoftwareSerial pins
SoftwareSerial bluetooth(A1, A0);  // RX, TX

//Mpu and Servo intialisation
MPU6050 mpu(Wire);
Servo gripservo;
Servo armservo;

// Pre-determined offsets for MPU6050
const int accX_offset = 0.01;
const int accY_offset = -0.01;
const int accZ_offset = -0.11;
const int gyroX_offset = -2.26;
const int gyroY_offset = 0.90;
const int gyroZ_offset = -0.99;

// Motor pins
const int inputPin1 = A2;
const int inputPin2 = A3;
const int inputPin3 = 9;
const int inputPin4 = 4;
const int EN1 = 6;
const int EN2 = 5;
float combinedOutput = 0;

// Encoder Pins
#define encodPinAR 7
#define encodPinBR 2
#define encodPinAL 8
#define encodPinBL 3

// Balance control flag
int Balance_flag = 0;

// PID Parameters
float Kp_balance = 61.39, Ki_balance = 750, Kd_balance = 1.55;
float Kp_position = 200, Ki_position = 0, Kd_position = 0;

// PID variables
float setPoint = 0.0, currentAngle = 0.0;
float error = 0.0, lastError = 0.0;
float integral = 0.0, derivative = 0.0, output = 0.0;

// Position PID variables
float positionSetPoint = 0.0, currentPosition = 0.0;
float positionError = 0.0, lastPositionError = 0.0;
float positionIntegral = 0.0, positionDerivative = 0.0, positionOutput = 0.0;

// Timing variables for Bluetooth processing in ms
unsigned long previousBTTime = 0;
const unsigned long BTInterval = 50;
// Timing variables for MPU processing in ms
unsigned long previousMPUTime = 0;
const unsigned long MPUInterval = 10;

// Constants
const float wheel_radius = 0.043; //43mm radius in meters
const int pulses_per_revolution = 300;

// Position control variables
volatile int wheel_pulse_count_left = 0, wheel_pulse_count_right = 0;
float wheel_distance_left = 0.0, wheel_distance_right = 0.0;

/*
 * Function Name: moveServo
 * Input: Servo object reference, targetAngle (int), stepDelay (int, optional)
 * Output: None
 * Logic: Gradually moves the servo to the target position with smooth motion.
 * Example Call: moveServo(gripservo, 90);
 */
void moveServo(Servo &servo, int targetAngle, int stepDelay = 15) {
  int currentAngle = servo.read();
  if (currentAngle < targetAngle) {
    for (int angle = currentAngle; angle <= targetAngle; angle++) {
      servo.write(angle);
      delay(stepDelay);
    }
  } else {
    for (int angle = currentAngle; angle >= targetAngle; angle--) {
      servo.write(angle);
      delay(stepDelay);
    }
  }
}

/* 
 * Function Name: Grip_close
 * Input: None
 * Output: None
 * Logic: Closes the grip by moving the servo to the specified angle.
 * Example Call: Grip_close();
 */
void Grip_close() {
  armservo.detach();
  gripservo.attach(11);
  gripservo.write(85);
}

/* 
 * Function Name: Grip_open
 * Input: None
 * Output: None
 * Logic: Open the grip by moving the servo to the specified angle.
 * Example Call: Grip_close();
 */
void Grip_open() {
  armservo.detach();
  gripservo.attach(11);
  gripservo.write(130);
}

/* 
 * Function Name: arm_up
 * Input: None
 * Output: None
 * Logic: Move the arm up by moving the servo to the specified angle.
 * Example Call: arm_up();
 */
void arm_up() {
  gripservo.detach();
  armservo.attach(13);
  armservo.write(90);
}

/* 
 * Function Name: arm_down
 * Input: None
 * Output: None
 * Logic: MOve the arm down by moving the servo to the specified angle.
 * Example Call: arm_down();
 */
void arm_down() {
  gripservo.detach();
  armservo.attach(13);
  armservo.write(150);
}

/* 
 * Function Name: forward
 * Input: None
 * Output: None
 * Logic: Change the setpoint of MPU to specified value so that it moves forward.
 * Example Call: forward();
 */
void forward() {
  setPoint = 0.8;
  Kp_position = 0;
}

/* 
 * Function Name: backward
 * Input: None
 * Output: None
 * Logic: Change the setpoint of MPU to specified value so that it moves backward.
 * Example Call: backward();
 */
void backward() {
  setPoint = -1.5;
  Kp_position = 0;
}

/* 
 * Function Name: balance
 * Input: None
 * Output: None
 * Logic: Change the setpoint of MPU to 0 so that it balances on its present position.
 * Example Call: balance();
 */
void balance() {
  setPoint = 0;
  armservo.detach();
  gripservo.detach();
}

/* 
 * Function Name: mot_rencoder_left
 * Input: None
 * Output: None
 * Logic: Increments or decrements wheel_pulse_count_left based on the phase difference between encodPinAL and encodPinBL.
 * Example Call: mot_rencoder_left();
 */
void mot_rencoder_left() {
  if (digitalRead(encodPinBL) > digitalRead(encodPinAL)) {
    wheel_pulse_count_left++;
  } else {
    wheel_pulse_count_left--;
  }
}

/* 
 * Function Name: mot_rencoder_right
 * Input: None
 * Output: None
 * Logic: Increments or decrements wheel_pulse_count_right based on encodPinAR and encodPinBR.
 * Example Call: mot_rencoder_right();
 */
void mot_rencoder_right() {
  if (digitalRead(encodPinBR) > digitalRead(encodPinAR)) {
    wheel_pulse_count_right--;
  } else {
    wheel_pulse_count_right++;
  }
}

/* 
 * Function Name: controlMotor1
 * Input: pidOutput
 * Output: None
 * Logic: Rotate the motor clockwise or anti-clockwise based on pid output.
 * Example Call: controlMotor1(255);
 */
void controlMotor1(float pidOutput) {
  if (pidOutput > 0) {
    digitalWrite(inputPin1, HIGH);
    digitalWrite(inputPin2, LOW);
    analogWrite(EN1, pidOutput);
  } else {
    pidOutput = abs(pidOutput);
    digitalWrite(inputPin1, LOW);
    digitalWrite(inputPin2, HIGH);
    analogWrite(EN1, pidOutput);
  }
}

/* 
 * Function Name: controlMotor2
 * Input: pidOutput
 * Output: None
 * Logic: Rotate the motor clockwise or anti-clockwise based on pid output.
 * Example Call: controlMotor2(255);
 */
void controlMotor2(float pidOutput) {
  if (pidOutput > 0) {
    digitalWrite(inputPin3, LOW);
    digitalWrite(inputPin4, HIGH);
    analogWrite(EN2, pidOutput);
  } else {
    pidOutput = abs(pidOutput);
    digitalWrite(inputPin3, HIGH);
    digitalWrite(inputPin4, LOW);
    analogWrite(EN2, pidOutput);
  }
}

/*
 * Function Name: setup
 * Input: None
 * Output: None
 * Logic: Initializes the sensors, motors, servos and Bluetooth communication.
 * Example Call: setup();
 */
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize MPU6050
  byte status = mpu.begin();
  if (status != 0) {
    while (1)
      ;  //Stops if intialization fails.
  }

  //Set the pretermined offset values for accelerometer and gyroscope (for MPU6050).
  mpu.setAccOffsets(accX_offset, accY_offset, accZ_offset);
  mpu.setGyroOffsets(gyroX_offset, gyroY_offset, gyroZ_offset);

  // Motor pin configurations
  pinMode(inputPin1, OUTPUT);
  pinMode(inputPin2, OUTPUT);
  pinMode(inputPin3, OUTPUT);
  pinMode(inputPin4, OUTPUT);

  bluetooth.begin(9600);  // For HC-05 (default baud rate is 9600).

  //Intialization of encoder
  pinMode(8, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), mot_rencoder_left, RISING);
  pinMode(7, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), mot_rencoder_right, RISING);
}

/*
 * Function Name: loop
 * Input: None
 * Output: None
 * Logic: Runs the main control loop, processes MPU6050 data, computes PID, and executes Bluetooth commands.
 * Example Call: loop();
 */
void loop() {
  unsigned long currentTime = millis();  //millis() returns the time elapsed since the program started.

  if (currentTime - previousMPUTime >= MPUInterval) {
    previousMPUTime = currentTime;  //Updates previousMPUTime to keep track of the last execution.

    // Update MPU6050 readings
    mpu.update();
    currentAngle = mpu.getAngleX();  // Get current angle (pitch)

    wheel_distance_left = (wheel_pulse_count_left / float(pulses_per_revolution)) * 2 * PI * wheel_radius;
    wheel_distance_right = (wheel_pulse_count_right / float(pulses_per_revolution)) * 2 * PI * wheel_radius;
    currentPosition = (wheel_distance_left + wheel_distance_right) / 2.0;  //currentPosition is the average distance traveled by both wheels.

    if (Balance_flag == 0) {
      error = positionSetPoint - currentAngle;                        //Computes tilt error.
      integral += error * 0.01;                                       // 10 ms = 0.01 s
      integral = constrain(integral, -50, 50);                        // Prevent windup
      derivative = (error - lastError) / 0.01;                        //calculates the rate of error change.
      float I = Ki_balance * integral;                                //accumulates error over time (preventing drift).
      output = (Kp_balance * error) + I + (Kd_balance * derivative);  //output is the control signal (motor speed adjustment).
      output = constrain(output, -255, 255);                          //limits output to motor speed range.
      lastError = error;
    } else if (Balance_flag == 1) {
      error = setPoint - currentAngle;
      integral += error * 0.01;                 // 10 ms = 0.01 s
      integral = constrain(integral, -50, 50);  // Prevent windup
      derivative = (error - lastError) / 0.01;
      float I = Ki_balance * integral;
      output = (Kp_balance * error) + I + (Kd_balance * derivative);
      output = constrain(output, -255, 255);
      lastError = error;
    }

    // Position PID Calculations: Uses a second PID controller to regulate position based on currentPosition and controls how far the bot has moved, preventing unintended drifts.
    positionError = positionSetPoint - currentPosition;
    positionIntegral += positionError * 1;
    positionIntegral = constrain(positionIntegral, -50, 50);
    positionDerivative = (positionError - lastPositionError) / 0.01;
    positionOutput = (Kp_position * positionError) + (Ki_position * positionIntegral) + (Kd_position * positionDerivative);
    positionOutput = constrain(positionOutput, -50, 50);
    lastPositionError = positionError;

    //The final motor speed is determined by adding balance (output) and position (positionOutput) and Ensures motor speed stays within safe limits.
    float combinedOutput = output + positionOutput;
    combinedOutput = constrain(combinedOutput, -255, 255);

    //Sends the final computed signal to both motors to maintain balance and position.
    controlMotor1(combinedOutput);
    controlMotor2(combinedOutput);
  }
  char bt_command = 'X';

  if (bluetooth.available()) {
    char bt_command = bluetooth.read();
    //'N' → Lowers the robot's arm.
    //'n' → Raises the arm.
    //'M' → Closes the gripper.
    //'m' → Opens the gripper.
    //'F' → Moves forward.
    //'B' → Moves backward.
    //'L' and 'R' adjust motor outputs to turn left and right.
    if (bt_command == 'N') {
      arm_down();
    } else if (bt_command == 'n') {
      arm_up();
    } else if (bt_command == 'M') {
      Grip_close();
    } else if (bt_command == 'm') {
      Grip_open();
    } else if (bt_command == 'F') {
      forward();
    } else if (bt_command == 'B') {
      backward();
    }
    //Loop is used to ensure continuous movement in left and right direction.
    else if (bt_command == 'L') {
      for (int i = 0; i <= 150; i++) {
        controlMotor1(combinedOutput);
        controlMotor2(combinedOutput - 250);
      }
    } else if (bt_command == 'R') {
      for (int i = 0; i <= 150; i++) {
        controlMotor1(combinedOutput - 250);
        controlMotor2(combinedOutput);
      }
    }
    //'S' → Calls balance() (rebalances the robot).
    //'Y' → Sets setPoint to 1.5 (adjusts balance threshold) to increase speed whenever required.
    //'X' → Sets setPoint to -1.5 (adjusts balance threshold differently) to decrease speed whenever required.
    //'J' → Plays a buzzer sound on pin 12 at 1000 Hz for 1 seconds.

    else if (bt_command == 'S') {
      balance();
    } else if (bt_command == 'Y') {
      setPoint = 1.5;
    } else if (bt_command == 'X') {
      setPoint = -1.5;
    } else if (bt_command == 'J') {
      tone(12, 1000, 1000);
    }
  }
}