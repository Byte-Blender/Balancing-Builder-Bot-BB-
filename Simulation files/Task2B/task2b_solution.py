import time
import numpy as np
import control as ctr



class KalmanFilter:
    def _init_(self, measurement_variance, process_variance):
        # Initial state estimates
        self.angle_estimate = 0.0
        self.angular_velocity_estimate = 0.0

        # Initial uncertainty
        self.uncertainty = np.eye(2)  # Shape (2, 2)

        # Measurement noise covariance
        self.measurement_variance = measurement_variance  # Variance of the measurement noise
        self.process_variance = process_variance  # Variance of the process noise

    def predict(self, dt):
        # State transition matrix (assuming constant angular velocity)
        A = np.array([[1, dt],
                      [0, 1]])  # Shape (2, 2)

        # Predict the state
        predicted_state = A @ np.array([[self.angle_estimate],
                                         [self.angular_velocity_estimate]])

        # Update the angle and angular velocity estimates
        self.angle_estimate = predicted_state[0, 0]
        self.angular_velocity_estimate = predicted_state[1, 0]

        # Update the uncertainty
        self.uncertainty = A @ self.uncertainty @ A.T + np.eye(2) * self.process_variance

    def update(self, measured_angle, measured_angular_velocity):
        # Measurement matrix
        H = np.array([[1, 0],
                      [0, 1]])  # Shape (2, 2)

        # Measurement vector (2x1)
        measurement = np.array([[measured_angle],
                                 [measured_angular_velocity]])  # Shape (2, 1)

        # Predicted measurement
        predicted_measurement = H @ np.array([[self.angle_estimate],
                                               [self.angular_velocity_estimate]])  # Shape (2, 1)

        # Measurement residual
        measurement_residual = measurement - predicted_measurement  # Shape (2, 1)

        # Measurement noise covariance
        R = self.measurement_variance * np.eye(2)  # Shape (2, 2)

        # Innovation covariance
        S = H @ self.uncertainty @ H.T + R  # Shape (2, 2)

        # Kalman gain
        K = self.uncertainty @ H.T @ np.linalg.inv(S)  # Shape (2, 2)

        # Update state with measurement
        state_update = np.array([[self.angle_estimate],
                                  [self.angular_velocity_estimate]]) + K @ measurement_residual  # Shape (2, 1)

        # Update the estimates
        self.angle_estimate = state_update[0, 0]
        self.angular_velocity_estimate = state_update[1, 0]

        # Update uncertainty
        self.uncertainty = (np.eye(2) - K @ H) @ self.uncertainty

    def get_estimates(self):
        return self.angle_estimate, self.angular_velocity_estimate
        
    def _init_(self, process_variance, measurement_variance):
        # Initialize Kalman Filter variables
        self.angle_estimate = 0.0  # Estimated angle
        self.angular_velocity_estimate = 0.0  # Estimated angular velocity
        self.uncertainty = np.array([[1, 0], [0, 1]])  # Initial uncertainty
        self.process_variance = process_variance  # Process noise variance
        self.measurement_variance = measurement_variance  # Measurement noise variance

    def predict(self, dt, angular_velocity_input, measured_angle):
        # State transition matrix (angle update from angular velocity)
        A = np.array([[1, dt], [0, 1]])
        # Control matrix
        B = np.array([[0], [1]])

        # State prediction
        state = np.array([[self.angle_estimate], [self.angular_velocity_estimate]])
        control_input = np.array([[angular_velocity_input]])
        state_prediction = A @ state + B @ control_input
        #print(state_prediction)
        # Update the state with prediction
        self.angle_estimate = state_prediction[0, 0]
        self.angular_velocity_estimate = state_prediction[1, 0]

        # Update the uncertainty with process noise
        Q = self.process_variance * np.array([[dt**2, dt], [dt, 1]])
        self.uncertainty = A @ self.uncertainty @ A.T + Q
        
 
            # Measurement matrix
        H = np.array([[1, 0], [0, 1]])  # Shape (2, 2)
        #print("H shape:", H.shape)  # Should be (2, 2)

    # Convert measured values to floats and reshape as column vectors
        measured_angle = float(np.squeeze(measured_angle))
        measured_angular_velocity = float(np.squeeze(angular_velocity_input))

    # Measurement vector (2x1)
        measurement = np.array([[measured_angle], [measured_angular_velocity]])
  #      print("Measurement vector:", measurement)  # Should be (2, 1)
   #     print("Measurement shape:", measurement.shape)  # Should be (2, 1)

    # Predicted state vector (2x1)
        state_prediction = np.array([[self.angle_estimate], [self.angular_velocity_estimate]])
    #    print("State prediction:", state_prediction)  # Should be (2, 1)
     #   print("State prediction shape:", state_prediction.shape)  # Should be (2, 1)

    # Calculate measurement residual (2x1)
        measurement_residual = measurement - (H @ state_prediction).reshape(2,1)
      #  print("Measurement residual:", measurement_residual)  # Should be (2, 1)
       # print("Measurement residual shape:", measurement_residual.shape)  # Should be (2, 1)

    # Measurement noise covariance (2x2)
        R = self.measurement_variance * np.eye(2)  # Shape (2, 2)
   #     print("Measurement noise covariance R:\n", R)  # Should be (2, 2)
    #    print("Measurement noise covariance R shape:", R.shape)  # Should be (2, 2)

    # Innovation covariance (2x2)
        S = H @ self.uncertainty @ H.T + R  # Should result in (2, 2)
     #   print("Innovation covariance S:\n", S)  # Should be (2, 2)
      #  print("Innovation covariance S shape:", S.shape)  # Should be (2, 2)

    # Kalman gain (2x2)
        K = self.uncertainty @ H.T @ np.linalg.inv(S)  # Should be (2, 2)
       # print("Kalman gain K:\n", K)  # Should be (2, 2)
#        print("Kalman gain K shape:", K.shape)  # Should be (2, 2)

    # Update state with measurement (2x1)
        state_update = state_prediction + K @ measurement_residual  # Should result in (2, 1)
 #       print("State update:", state_update)  # Should be (2, 1)
  #      print("State update shape:", state_update.shape)  # Should be (2, 1)

    # Update the state estimates
        self.angle_estimate = state_update[0, 0]
        self.angular_velocity_estimate = state_update[1, 0]
   #     print("Updated angle estimate:", self.angle_estimate)
    #    print("Updated angular velocity estimate:", self.angular_velocity_estimate)

    # Update uncertainty (2x2)
        self.uncertainty = (np.eye(2) - K @ H) @ self.uncertainty
     #   print("Updated uncertainty:\n", self.uncertainty)  # Should be (2, 2)
      #  print("Updated uncertainty shape:", self.uncertainty.shape)  # Should be (2, 2)

    
    def get_estimates(self):
        return self.angle_estimate, self.angular_velocity_estimate

def sysCall_init():
    sim = require('sim')

    # Object initialization
    self.body = sim.getObject('/body')
    self.left_joint = sim.getObject('/body/left_joint')
    self.right_joint = sim.getObject('/body/right_joint')
    
    # Arm initialization
    self.arm_joint = sim.getObject('/body/SBR_Assembly_7/ForceSensor/arm_base/arm_joint')
    self.gripper_prismatic_joint = sim.getObject('/body/SBR_Assembly_7/ForceSensor/arm_base/arm_joint/grip_base/Prismatic_joint')
    
    # Physical parameters
    m_body = 0.248  # Body mass (kg)
    m_wheel = 0.018  # Wheel mass (kg)
    L = 0.1  # Distance from wheel axis to center of mass (m)
    g = 9.81  # Gravitational acceleration (m/s^2)
    b = 0.1  # Damping coefficient (approximate)
    
    # State-space matrices
    A = np.array([[0, 1, 0, 0],
                  [0, -b/m_wheel, (m_body * g) / m_wheel, 0],
                  [0, 0, 0, 1],
                  [0, -b / (m_wheel * L), (m_body * g * L) / (m_wheel * L), 0]])
    B = np.array([[0], [1 / m_wheel], [0], [1 / (m_wheel * L)]])
    
    # LQR cost matrices (adjust to reduce drift)
    self.Q = np.diag([20, 14.61, 3.14, 1.5])  # State cost
    R = np.array([[3]])  # Input cost
    
    
    
    # Compute LQR gain matrix K
    self.K, _, _ = ctr.lqr(A, B, self.Q, R)
    
    # Kalman Filter initialization
    process_variance = 0.01
    measurement_variance = 0.1
    self.kalman_filter = KalmanFilter(process_variance, measurement_variance)
    
    # Initialize desired pitch and movement flags
    self.desired_pitch = 0
    self.forward, self.backward, self.left, self.right, self.stop = False, False, False, False, False
    self.arm_up, self.arm_down, self.arm_open, self.arm_close = False, False, False, False
    self.wheel_radius = 0.02
    self.a = 0

def motor_control(joint):
    dt = 0.05  # Time step for Kalman filter
    bodyPosition = sim.getObjectPosition(self.body, -1)[0]  # x position
    linearVelocity, _ = sim.getObjectVelocity(joint)
    bodyOrientation_d = sim.getObjectOrientation(self.body, -1)
    #print(bodyOrientation_d[0] * (180 / np.pi))
    
    z_axis = bodyOrientation_d[2]*(180/3.14)
    
    if z_axis > -45 and z_axis <= 45:
        #print(0)
        bodyOrientation = bodyOrientation_d[0]*(180/np.pi)  #0
        #print(bodyOrientation)
        
    elif z_axis > 45 and z_axis <= 135:
        #print(90)
        bodyOrientation = bodyOrientation_d[1]*(180/np.pi)  #90
        
        
    elif z_axis > -135 and z_axis <= -45:
        #print("-90 degree me hai")
        bodyOrientation = -bodyOrientation_d[1]*(180/np.pi)   #-90

    else:
        print(180)
        bodyOrientation = -bodyOrientation_d[0]*(180/np.pi)  #180
        #print(bodyOrientation)
    # Measured angle and angular velocity
    #bodyOrientation = bodyOrientation_d[0] * (180 / np.pi)
    _, measured_angular_velocity = sim.getObjectVelocity(joint)

    # Update Kalman Filter with the measurements
    # Example usage of the Kalman Filter

# Set measurement and process variances
    measurement_variance = 0.1  # Variance of the measurement noise
    process_variance = 0.1  # Variance of the process noise

# Create an instance of the KalmanFilter
    kf = KalmanFilter(measurement_variance, process_variance)

    # Simulate a time step
    dt = 0.005  # Time step in seconds

# Simulate some measurements

    # Simulated measured angle and angular velocity
    # Prediction step
    kf.predict(dt,measured_angular_velocity[0],bodyOrientation)

    # Update step with measurements
    kf.update(bodyOrientation, measured_angular_velocity[0])

    # Get current estimates
    angle_estimate, angular_velocity_estimate = kf.get_estimates()
    #print(f"Angle Estimate: {angle_estimate:.2f}, Angular Velocity Estimate: {angular_velocity_estimate:.2f}")
    self.velocity = linearVelocity[0]
    print(linearVelocity[0])
    # Assemble the state vector and calculate control input
    x = np.array([[bodyPosition], [linearVelocity[0]], [angle_estimate], [angular_velocity_estimate]])
    x_desired = np.array([[0], [0], [self.desired_pitch], [0]])
    x_error = x - x_desired
    U = -self.K.dot(x_error)
    return U[0][0]

def sysCall_actuation():
    turning_factor = 1.1
    self.u1 = motor_control(self.left_joint)
    self.u2 = motor_control(self.right_joint)
    sim.setJointTargetVelocity(self.left_joint, self.u1)
    sim.setJointTargetVelocity(self.right_joint, self.u2)
    
    
    
    #time.sleep(0.01)
    # Balance, Forward, Backward, Left, and Right Movement Logic
    if self.forward:
        self.desired_pitch = -1
        
    elif self.backward:
        self.desired_pitch = 2
        
    elif self.left:
        sim.setJointTargetVelocity(self.left_joint, self.u1 )
        sim.setJointTargetVelocity(self.right_joint, self.u2+10 )
    elif self.right:
        sim.setJointTargetVelocity(self.left_joint, self.u1+10)
        sim.setJointTargetVelocity(self.right_joint, self.u2 )
    elif self.stop:
        self.desired_pitch = 0
        """if self.desired_pitch < 0:
            self.desired_pitch = 3
            
        else:
            self.desired_pitch = -3
        self.stop=False"""
        """self.a += 1
        if self.a >= 20:
            self.stop = False
            self.a = 0"""
            
    

def sysCall_sensing():
    message, data, data2 = sim.getSimulatorMessage()
            
    if data[0] == 2007:
        self.forward = True
        self.backward = False
        self.left = False
        self.right = False
        #print("forward")
    elif data[0] == 2008:
        self.forward = False
        self.backward = True
        self.left = False
        self.right = False
        #print("backward")  # Backward
        
    elif data[0] == 2009:
        self.forward = False
        self.backward = False
        self.left = True
        self.right = False
        #print("left")  # Left
        
    elif data[0] == 2010:
        self.forward = False
        self.backward = False
        self.left = False
        self.right = True
        #print("right")  # Right
        
    elif data[0] == 32:
        self.forward = False
        self.backward = False
        self.left = False
        self.right = False
        self.stop = True
        #print("right")  # Right
        
        
    elif data[0] == ord("z"):
        #gripper_down(self.arm_joint)
        print("down")
        self.arm_up = False
        self.arm_down = True
        
        
        
    elif data[0] == ord("c"):
        #gripper_up(self.arm_joint)
        print("up")
        self.arm_up = True
        self.arm_down = False
        
        
        
    elif data[0] == ord("q"):
        #gripper_close(self.gripper_prismatic_joint)
        print("close")
        
        self.arm_open = False
        self.arm_close = True
        
        
    elif data[0] == ord("e"):
        #gripper_open(self.gripper_prismatic_joint)
        print("open")
        
        self.arm_open = True
        self.arm_close = False
        
    
    elif data[0] == ord("b"):
        #gripper_stop(self.arm_joint)
        self.arm_up = False
        self.arm_down = False
        self.arm_open = False
        self.arm_close = False
        
            
            
    #if self.arm_up == False and self.arm_down == False and self.arm_open == False and self.arm_close == False:
     #   sim.setJointTargetVelocity(self.arm_joint, 0)
      #  sim.setJointTargetVelocity(self.gripper_prismatic_joint, 0)
        
    if self.arm_up == True and self.arm_down == False:
        print("going up")
        sim.setJointTargetVelocity(self.arm_joint, -5)
        arm_joint_position = sim.getJointPosition(self.arm_joint)*(180/3.14)

        if arm_joint_position < 0:
            sim.setJointTargetVelocity(self.arm_joint, 0)
            self.arm_up = False
            self.arm_down = False
        
        
    elif self.arm_up == False and self.arm_down == True:
        print("going down")
        sim.setJointTargetVelocity(self.arm_joint, 5)
        arm_joint_position = sim.getJointPosition(self.arm_joint)*(180/3.14)
        
        if arm_joint_position > 60:
            sim.setJointTargetVelocity(self.arm_joint, 0)
            self.arm_up = False
            self.arm_down = False
        
        
    if self.arm_open == True and self.arm_close == False:
        print("opening")
        sim.setJointTargetVelocity(self.gripper_prismatic_joint, -0.1)
        gripper_prismatic_joint_position = sim.getJointPosition(self.gripper_prismatic_joint)
    
        if gripper_prismatic_joint_position <= -0.030:
            sim.setJointTargetVelocity(self.gripper_prismatic_joint, 0)
            self.arm_open = False
            self.arm_close = False
        
    elif self.arm_open == False and self.arm_close == True:
        print("closing")
        sim.setJointTargetVelocity(self.gripper_prismatic_joint, 0.1)
        gripper_prismatic_joint_position = sim.getJointPosition(self.gripper_prismatic_joint)
    
        if gripper_prismatic_joint_position >= 0.005:
            sim.setJointTargetVelocity(self.gripper_prismatic_joint, 0)
            self.arm_open = False
            self.arm_close = False