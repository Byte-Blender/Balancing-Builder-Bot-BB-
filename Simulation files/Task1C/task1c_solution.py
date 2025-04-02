[10:26 PM, 10/16/2024] +91 72369 80333: import time


def clamp(value, min_value, max_value):
    """
    Clamps the input value to ensure it stays within the min and max bounds.

    Args:
        value (float): The value to be clamped.
        min_value (float): The minimum allowable value.
        max_value (float): The maximum allowable value.

    Returns:
        float: The clamped value.
    """
    return max(min_value, min(value, max_value))


def sysCall_init():
    sim = require('sim')

    # Object initialization
    self.body = sim.getObject('/body')
    self.left_joint = sim.getObject('/body/left_joint')
    self.left_wheel = sim.getObject('/body/left_joint/left_wheel')
    self.right_joint = sim.getObject('/body/right_joint')
    self.right_wheel = sim.getObject('/body/right_joint/r…
task1c_output.txt
[1:07 PM, 10/20/2024] +91 72369 80333: import time


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
    self.previous_error_x1 = 50
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
        self.u1 = motor_control(self.left_joint)
        self.u2 = motor_control(self.right_joint)
        #sim.setJointTargetVelocity(self.left_joint, 0)   # u1 = 0 right
        #sim.setJointTargetVelocity(self.right_joint, 0)
        sim.setJointTargetVelocity(self.left_joint, self.u1)   # u1 = 0 right
        sim.setJointTargetVelocity(self.right_joint, self.u2)  #u2=0 left
        #time.sleep(0.0015)
    elif (self.forward == True) and (self.backward == False) and (self.left == False) and (self.right == False):
        self.u1 = motor_control(self.left_joint)
        self.u2 = motor_control(self.right_joint)
        print("moving forward")
        sim.setJointTargetVelocity(self.left_joint, self.u1-160)   # u1 = 0 right
        sim.setJointTargetVelocity(self.right_joint, self.u2-160)  #u2=0 left
        #time.sleep(0.001)
    elif (self.forward == False) and (self.backward == True) and (self.left == False) and (self.right == False):
        self.u1 = motor_control(self.left_joint)
        self.u2 = motor_control(self.right_joint)
        print("moving backward")
        sim.setJointTargetVelocity(self.left_joint, self.u1+100)   # u1 = 0 right
        sim.setJointTargetVelocity(self.right_joint, self.u2+100)  #u2=0 left
        #time.sleep(0.0015)
    elif (self.forward == False) and (self.backward == False) and (self.left == True) and (self.right == False):
        self.u1 = motor_control(self.left_joint)
        self.u2 = motor_control(self.right_joint)
        #sim.setJointTargetVelocity(self.left_joint, self.u1)   # u1 = 0 right
        #sim.setJointTargetVelocity(self.right_joint, self.u2)
        print("moving left")
        sim.setJointTargetVelocity(self.left_joint, 40)   # u1 = 0 right
        sim.setJointTargetVelocity(self.right_joint, self.u2)  #u2=0 left
        time.sleep(0.005)
        print(self.u2)
    elif (self.forward == False) and (self.backward == False) and (self.left == False) and (self.right == True):
        self.u1 = motor_control(self.left_joint)
        self.u2 = motor_control(self.right_joint)
        #sim.setJointTargetVelocity(self.left_joint, self.u1)   # u1 = 0 right
        #sim.setJointTargetVelocity(self.right_joint, self.u2)
        print("moving right")
        sim.setJointTargetVelocity(self.left_joint, self.u1)   # u1 = 0 right
        sim.setJointTargetVelocity(self.right_joint, 40)
        time.sleep(0.005)
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

    if message == sim.message_keypress:
        if data[0] == 2007:
            self.forward = True
            self.backward = False
            self.left = False
            self.right = False
            print("forward")
        elif data[0] == 2008:
            self.forward = False
            self.backward = True
            self.left = False
            self.right = False
            print("backward")  # Backward
            
        elif data[0] == 2009:
            self.forward = False
            self.backward = False
            self.left = True
            self.right = False
            print("left")  # Left
            
        elif data[0] == 2010:
            self.forward = False
            self.backward = False
            self.left = False
            self.right = True
            print("right")  # Right
            
        elif data[0] == 32:
            self.forward = False
            self.backward = False
            self.left = False
            self.right = False
            print("right")  # Right
        print(self.forward ,self.backward ,self.left ,self.right)