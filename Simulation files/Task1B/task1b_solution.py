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

    # Previous errors
    self.previous_error_x1 = 0
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

def motor_control(joint):
    bodyOrientation = sim.getObjectOrientation(self.body, -1)
    self.x1 = bodyOrientation[0] * (180 / 3.14)  # Roll (tilt angle)

    _, angularVelocity = sim.getObjectVelocity(joint)
    self.x2 = angularVelocity[0]  # Angular velocity

    bodyPosition = sim.getObjectPosition(self.body, -1)
    self.x3 = bodyPosition[0]  # Position (displacement)

    linearVelocity, _ = sim.getObjectVelocity(joint)
    self.x4 = linearVelocity[0]  # Velocity

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
    print(U)
    return U


def sysCall_actuation():
    # State variables
    u1 = motor_control(self.left_joint)
    u2 = motor_control(self.right_joint)
    # Applying the control signal to both wheels
    sim.setJointTargetVelocity(self.left_joint, u1)
    sim.setJointTargetVelocity(self.right_joint, u2)

    # Update previous errors
    self.previous_error_x1 = self.x1
    self.previous_error_x2 = self.x2
    self.previous_error_x3 = self.x3
    self.previous_error_x4 = self.x4

def sysCall_sensing():
    pass