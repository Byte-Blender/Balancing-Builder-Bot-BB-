import time


def ramp_velocity(current_velocity, target_velocity, ramp_rate):
    if current_velocity < target_velocity:
        return min(current_velocity + ramp_rate, target_velocity)
    elif current_velocity > target_velocity:
        return max(current_velocity - ramp_rate, target_velocity)
    return current_velocity



def clamp(value, min_value, max_value):
    
    return max(min_value, min(value, max_value))


def sysCall_init():
    sim = require('sim')

    # Object initialization
    self.body = sim.getObject('/body')
    self.left_joint = sim.getObject('/body/left_joint')
    self.left_wheel = sim.getObject('/body/left_joint/left_wheel')
    self.right_joint = sim.getObject('/body/right_joint')
    self.right_wheel = sim.getObject('/body/right_joint/right_wheel')
    
    #arm initialization
    
    self.arm_joint = sim.getObject('/body/SBR_Assembly_7/ForceSensor/arm_base/arm_joint')
    #print(self.arm_joint)
    
    self.gripper_prismatic_joint = sim.getObject('/body/SBR_Assembly_7/ForceSensor/arm_base/arm_joint/grip_base/Prismatic_joint')
    #print(self.gripper_prismatic_joint)
    
    # Control gains (tune these values based on system response)
    # x1 - Tilt angle
    # Control gains (tune these values based on system response)
    # x1 - Tilt angle

    
    self.Kp1 = 0
    self.Kd1 = 0.11
    self.Ki1 = 0.01
    # x2 - Angular velocity
    self.Kp2 = 1
    self.Kd2 = 0.11
    self.Ki2 = 0.0
    # x3 - Position (displacement)
    self.Kp3 = -0.5
    self.Kd3 = 0.01
    self.Ki3 = 0.0
    # x4 - Velocity
    self.Kp4 = 0.1
    self.Kd4 = 0.01
    self.Ki4 = 0.0

    # Previous errors
    self.previous_error_x1 = 10
    self.previous_error_x2 = 0
    self.previous_error_x3 = 0
    self.previous_error_x4 = 0

    # Time step
    self.dt = 0.05

    # Integral terms
    self.integral_x1 = 0
    self.integral_x2 = 0
    self.integral_x3 = 0
    self.integral_x4 = 0
    
    self.forward = False
    self.backward = False
    self.left = False
    self.right = False
    
    #arm variables
    self.arm_up = False
    self.arm_down = False
    
    self.arm_open = False
    self.arm_close = False
    
    

def motor_control(joint):
    #bodyOrientation = sim.getObjectOrientation(self.body, -1)
    
    #self.x1 = bodyOrientation[0] * (180 / 3.14)  # Roll (tilt angle)
    
    bodyOrientation = sim.getObjectOrientation(self.body, -1)
    self.x1 = bodyOrientation[0] * (180 / 3.14) #0 degree par
    z_axis = bodyOrientation[2]*(180/3.14)
    #print(z_axis)
    #print(z_axis)
    if z_axis > -45 and z_axis <= 45:
        self.x1 = bodyOrientation[0] * (180 / 3.14)  #0
        #self.Kp1 = -25
        #self.Kd1 = 0.0101
        #self.Ki1 = 0.1
        
        self.Kp1 = -50
        self.Kd1 = 0.11
        self.Ki1 = 0.01
    # x2 - Angular velocity
        self.Kp2 = 1
        self.Kd2 = 0.11
        self.Ki2 = 0.0
    # x3 - Position (displacement)
        self.Kp3 = -0.5
        self.Kd3 = 0.01
        self.Ki3 = 0.0
    # x4 - Velocity
        self.Kp4 = 0.1
        self.Kd4 = 0.01
        self.Ki4 = 0.0
        
    elif z_axis > 45 and z_axis <= 135:
        self.x1 = bodyOrientation[1] * (180 / 3.14)  #90
        #self.Kp1 = -25
        #self.Kd1 = 0.0101
        #self.Ki1 = 0.0
        
        self.Kp1 = -50
        self.Kd1 = 0.11
        self.Ki1 = 0.01
    # x2 - Angular velocity
        self.Kp2 = 1
        self.Kd2 = 0.11
        self.Ki2 = 0.0
    # x3 - Position (displacement)
        self.Kp3 = -0.5
        self.Kd3 = 0.01
        self.Ki3 = 0.0
    # x4 - Velocity
        self.Kp4 = 0.1
        self.Kd4 = 0.01
        self.Ki4 = 0.0
        
    elif (z_axis > 135 and z_axis < 180) or (z_axis <-135 and z_axis > -180):
        self.x1 = bodyOrientation[0] * (180 / 3.14)  #180
        #self.Kp1 = 25
        #self.Kd1 = 0.0101
        #self.Ki1 = 0.0
        #print( "180 me hainn")
        self.Kp1 = 50
        self.Kd1 = 0.11
        self.Ki1 = 0.01
    # x2 - Angular velocity
        self.Kp2 = 1
        self.Kd2 = 0.11
        self.Ki2 = 0.0
    # x3 - Position (displacement)
        self.Kp3 = 0.5
        self.Kd3 = 0.01
        self.Ki3 = 0.0
    # x4 - Velocity
        self.Kp4 = 0.1
        self.Kd4 = 0.01
        self.Ki4 = 0.0
        
    elif z_axis > -135 and z_axis <= -45:
        #print("-90 degree me hai")
        self.x1 = bodyOrientation[1] * (180 / 3.14)   #-90
        #self.Kp1 = 50
        #self.Kd1 = 0
        #self.Ki1 = 0.0
        
        self.Kp1 = 50
        self.Kd1 = 0.11
        self.Ki1 = 0.01
    # x2 - Angular velocity
        self.Kp2 = 1
        self.Kd2 = 0.11
        self.Ki2 = 0.0
    # x3 - Position (displacement)
        self.Kp3 = 0.5
        self.Kd3 = 0.01
        self.Ki3 = 0.0
    # x4 - Velocity
        self.Kp4 = 0.1
        self.Kd4 = 0.01
        self.Ki4 = 0.0
        
    
    _, angularVelocity = sim.getObjectVelocity(joint)
    self.x2 = angularVelocity[0]  # Angular velocity

    bodyPosition = sim.getObjectPosition(self.body, -1)
    self.x3 = bodyPosition[0]  # Position (displacement)

    linearVelocity, _ = sim.getObjectVelocity(joint)
    self.x4 = linearVelocity[0]  # Velocity
    #print(self.x4)

    # PID calculations for each state
    derivative_x1 = (self.x1 - self.previous_error_x1) / self.dt
    self.integral_x1 += self.x1 * self.dt
    control_signal1 = self.Kp1 * self.x1 + self.Kd1 * derivative_x1 + self.Ki1 * self.integral_x1

    derivative_x2 = (self.x2 - self.previous_error_x2) / self.dt
    self.integral_x2 += self.x2 * self.dt
    control_signal2 = self.Kp2 * self.x2 + self.Kd2 * derivative_x2 + self.Ki2 * self.integral_x2

    derivative_x3 = (self.x3 - self.previous_error_x3) / self.dt
    self.integral_x3 += self.x3 * self.dt
    control_signal3 = self.Kp3 * self.x3 + self.Kd3 * derivative_x3 + self.Ki3 * self.integral_x3

    derivative_x4 = (self.x4 - self.previous_error_x4) / self.dt
    self.integral_x4 += self.x4 * self.dt
    control_signal4 = self.Kp4 * self.x4 + self.Kd4 * derivative_x4 + self.Ki4 * self.integral_x4

    # Summing the control signals (tuning this equation to match desired behavior)
    U = control_signal1 + control_signal2 + control_signal3 + control_signal4
    #print(U)
    
    return U
    

def sysCall_actuation():
    # State variables
    self.u1 = motor_control(self.left_joint)
    self.u2 = motor_control(self.right_joint)
    if (self.forward == False) and (self.backward == False) and (self.left == False) and (self.right == False):
        self.u1 = ramp_velocity(self.u1, motor_control(self.left_joint), 1)  # ramp_rate=1 (adjust as needed)
        self.u2 = ramp_velocity(self.u2, motor_control(self.right_joint), 1)

        #self.u1 = motor_control(self.left_joint)
        #self.u2 = motor_control(self.right_joint)
        #sim.setJointTargetVelocity(self.left_joint, 0)   # u1 = 0 right
        #sim.setJointTargetVelocity(self.right_joint, 0)
        sim.setJointTargetVelocity(self.left_joint, self.u1)   # u1 = 0 right
        sim.setJointTargetVelocity(self.right_joint, self.u2)  #u2=0 left
        #time.sleep(0.001)
    elif (self.forward == True) and (self.backward == False) and (self.left == False) and (self.right == False):
        #self.u1 = motor_control(self.left_joint)
        #self.u2 = motor_control(self.right_joint)
        self.u1 = ramp_velocity(self.u1, motor_control(self.left_joint), 1)  # ramp_rate=1 (adjust as needed)
        self.u2 = ramp_velocity(self.u2, motor_control(self.right_joint), 1)
        #print("moving forward")
        sim.setJointTargetVelocity(self.left_joint, self.u1-100)   # u1 = 0 right
        sim.setJointTargetVelocity(self.right_joint, self.u2-100)  #u2=0 left
        time.sleep(0.01)
    elif (self.forward == False) and (self.backward == True) and (self.left == False) and (self.right == False):
        #self.u1 = motor_control(self.left_joint)
        #self.u2 = motor_control(self.right_joint)
        self.u1 = ramp_velocity(self.u1, motor_control(self.left_joint), 1)  # ramp_rate=1 (adjust as needed)
        self.u2 = ramp_velocity(self.u2, motor_control(self.right_joint), 1)
        #print("moving backward")
        sim.setJointTargetVelocity(self.left_joint, self.u1+100)   # u1 = 0 right
        sim.setJointTargetVelocity(self.right_joint, self.u2+100)  #u2=0 left
        time.sleep(0.01)
    elif (self.forward == False) and (self.backward == False) and (self.left == True) and (self.right == False):
        #self.u1 = motor_control(self.left_joint)
        #self.u2 = motor_control(self.right_joint)
        self.u1 = ramp_velocity(self.u1, motor_control(self.left_joint), 1)  # ramp_rate=1 (adjust as needed)
        self.u2 = ramp_velocity(self.u2, motor_control(self.right_joint), 1)
        #sim.setJointTargetVelocity(self.left_joint, self.u1)   # u1 = 0 right
        #sim.setJointTargetVelocity(self.right_joint, self.u2)
        print("moving left")
        sim.setJointTargetVelocity(self.left_joint, 40)   # u1 = 0 right
        sim.setJointTargetVelocity(self.right_joint, self.u2)  #u2=0 left
        time.sleep(0.02)
        #print(self.u2)
    elif (self.forward == False) and (self.backward == False) and (self.left == False) and (self.right == True):
        #self.u1 = motor_control(self.left_joint)
        #self.u2 = motor_control(self.right_joint)
        self.u1 = ramp_velocity(self.u1, motor_control(self.left_joint), 1)  # ramp_rate=1 (adjust as needed)
        self.u2 = ramp_velocity(self.u2, motor_control(self.right_joint), 1)
        #sim.setJointTargetVelocity(self.left_joint, self.u1)   # u1 = 0 right
        #sim.setJointTargetVelocity(self.right_joint, self.u2)
        #print("moving right")
        sim.setJointTargetVelocity(self.left_joint, self.u1)   # u1 = 0 right
        sim.setJointTargetVelocity(self.right_joint, 40)
        time.sleep(0.02)
    #print(self.u1,self.u2)
    # Applying the control signal to both wheels
    #sim.setJointTargetVelocity(self.left_joint, self.u1)   # u1 = 0 right
    #sim.setJointTargetVelocity(self.right_joint, self.u2)  #u2=0 left

    # Update previous errors
    self.previous_error_x1 = self.x1
    self.previous_error_x2 = self.x2
    self.previous_error_x3 = self.x3
    self.previous_error_x4 = self.x4
    
    

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
        sim.setJointTargetVelocity(self.arm_joint, -2)
        arm_joint_position = sim.getJointPosition(self.arm_joint)*(180/3.14)

        if arm_joint_position < 0:
            sim.setJointTargetVelocity(self.arm_joint, 0)
            self.arm_up = False
            self.arm_down = False
        
        
    elif self.arm_up == False and self.arm_down == True:
        print("going down")
        sim.setJointTargetVelocity(self.arm_joint, 1)
        arm_joint_position = sim.getJointPosition(self.arm_joint)*(180/3.14)
        
        if arm_joint_position > 55:
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
    
        if gripper_prismatic_joint_position >= 0.025:
            sim.setJointTargetVelocity(self.gripper_prismatic_joint, 0)
            self.arm_open = False
            self.arm_close = False
    
    #checking arm joint angle
    
        
    
        

            
        
        #print(self.forward ,self.backward ,self.left ,self.right)