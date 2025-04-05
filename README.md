Project Documentation: Balancing Builder Bot
Team ID: 2286
Author List: Vidhi Singh Rajput, Shreyansh Gupta, Shreyas Rathore, Udit Nayak
File: robot_control.c
Theme: Balancing Builder Bot
Competition: e-Yantra Robotics Competition (eYRC) 2024-25
🏁 1. Competition Context: e-Yantra Robotics Competition (eYRC)
This project is developed as part of the prestigious e-Yantra Robotics Competition (eYRC) 2024-25, organized by IIT Bombay, under the theme “Balancing Builder Bot”.

e-Yantra is a national-level robotics competition that encourages innovation in embedded systems and robotics by solving real-world problems using hardware-software co-design. The Balancing Builder Bot theme challenges participants to build a robot that:

Balances autonomously using control algorithms

Navigates to specific checkpoints

Picks and places objects using a robotic arm

Interprets and acts on commands in real time

This documentation and code reflect our implementation for Task 2A, which involves combining balancing control with object manipulation using a pick-and-place arm.

🧠 2. Project Overview
The Balancing Builder Bot is a two-wheeled self-balancing robot that uses a combination of sensors, motors, control systems, and communication protocols to:

Maintain an upright posture like an inverted pendulum

Move forward/backward based on user commands

Pick up and place objects using a servo-based robotic arm

Communicate via Bluetooth with external controllers

🔧 3. Hardware Components
Microcontroller: Arduino Uno/Nano

IMU Sensor: MPU6050 (accelerometer + gyroscope)

DC Motors with Encoders (for feedback)

Servo Motors: 2 (for arm and gripper control)

Bluetooth Module: HC-05

Motor Driver Module: L298N or equivalent

Power Supply: Battery Pack (7.4V or 11.1V Li-ion)

Chassis: Custom two-wheeled bot frame with mount for arm

📦 4. Software and Libraries
Arduino IDE

Libraries Used:

MPU6050_light.h

Wire.h

Servo.h

SoftwareSerial.h

🔁 5. Key Functionalities
🤖 a. Self-Balancing Using PID
Reads tilt angle from MPU6050

Calculates PID output to control motor speed/direction

Maintains upright balance

🧭 b. Position Control
Uses wheel encoder feedback

Enables precise movement forward/backward

💡 c. Bluetooth Command Interface
Receives instructions like forward, backward, balance, arm_up, grip_close etc.

Parses commands to trigger corresponding robot actions

🛠 d. Pick and Place Arm
Gripper and arm are controlled via servos

Smooth transition between positions using moveServo()

⚙️ 6. Control Systems
Angle PID: Keeps the robot upright

Position PID: Adds positional correction while in motion

Encoder ISR: Interrupt-driven wheel rotation tracking

📡 7. Communication
Bluetooth (HC-05) is used for wireless control.

SoftwareSerial is mapped to analog pins A0 (TX) and A1 (RX).

The bot continuously checks for serial commands every 50 ms.

🧪 8. Testing and Evaluation
Robot is tested in CoppeliaSim simulation first (if available).

Calibrated and tuned PID values on physical hardware.

Passed Task 2A evaluation for:

Maintaining balance

Successful pick-and-place

Keyboard-controlled forward/backward navigation

🚀 9. Sample Command Flow
Robot starts and balances.

Bluetooth sends "forward" → Robot moves forward.

"arm_down" + "grip_close" → Picks object.

"arm_up" → Lifts object.

"backward" → Moves to drop zone.

"arm_down" + "grip_open" → Drops object.

"balance" → Returns to balancing state.

⚠️ 10. Limitations & Future Scope
MPU6050 can be noisy; applying complementary/Kalman filter could improve results.

Encoder accuracy depends on mechanical alignment.

State machine implementation can make behavior more modular.

Could be extended with autonomous navigation using vision or line-following sensors.

