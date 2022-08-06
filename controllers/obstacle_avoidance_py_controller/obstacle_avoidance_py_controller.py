"""epuck2_python_controller controller."""

# You may need to import some classes of the controller module. Ex:
# from controller import Robot, Motor, DistanceSensor
from controller import Robot, PositionSensor, DistanceSensor, Motor, Accelerometer, Camera

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 64
camera_time_step = 64
MAX_SPEED = 6.28
WHEEL_RADIUS = 0.02
AXLE_LENGTH = 0.052
RANGE = 1024/2
braitenberg_coefficients = [
    [0.942, -0.22],
    [0.63, -0.1],
    [0.5, -0.06],
    [-0.06, 0.06],
    [-0.06, -0.06],
    [-0.06, 0.5],
    [-0.19, 0.63],
    [-0.13, 0.942]
]


def computeOdometry(leftPstSensor, rightPstSensor):
    l = leftPstSensor.getValue()
    r = rightPstSensor.getValue()
    dl = l * WHEEL_RADIUS
    dr = r * WHEEL_RADIUS
    da = (dr -dl) / AXLE_LENGTH
    print('estimated distance covered by left wheel: %f m'%dl)
    print('estimated distance covered by right wheel: %f m'%dr)
    print('estimated change of orientation: %f m'%da)
	
	
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
# motor = robot.getMotor('motorname')
# ds = robot.getDistanceSensor('dsname')
# ds.enable(timestep)
# get and enable the camera and accelerometer
camera = robot.getDevice('camera')
camera.enable(camera_time_step)
accelerometer = robot.getDevice('accelerometer')
accelerometer.enable(timestep)

leftMotor = robot.getDevice('left wheel motor')

rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

leftPstSensor = robot.getDevice('left wheel sensor')
rightPstSensor = robot.getDevice('right wheel sensor')
leftPstSensor.enable(timestep)
rightPstSensor.enable(timestep)

ps = []
psNames = ['ps%d'%i for i in range(8)]
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

def obstacle_avoidance():
    a = accelerometer.getValues()
    print('accelerometer values: %.3f, %.3f, %.3f'%(a[0],a[1],a[2]))
    computeOdometry(leftPstSensor, rightPstSensor)
    speed = [0.0, 0.0]
    for i in range(2):
        speed[i] = 0.0
        for j in range(8):
            speed[i] += braitenberg_coefficients[j][i] * (1.0 - (ps_sensor_value[j]/RANGE))
        # Process sensor data here.
        	
        # Enter here functions to send actuator commands, like:
        # motor.setPosition(10.0)
        leftMotor.setVelocity(speed[0])
        rightMotor.setVelocity(speed[1])	

def forward():
    leftMotor.setVelocity(6.0)
    rightMotor.setVelocity(6.0)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    # val = ds.getValue()
    ps_sensor_value = []
    for i in range(8):
        ps_sensor_value.append(ps[i].getValue())
    dis =100
    front_obstacle = ps_sensor_value[0]>dis or ps_sensor_value[7]>dis
    right_obstacle = ps_sensor_value[0]>dis or ps_sensor_value[1]>dis or ps_sensor_value[2]>dis
    left_obstacle = ps_sensor_value[6]>dis or ps_sensor_value[7]>dis or ps_sensor_value[5]>dis
    back_obstacle = ps_sensor_value[3]>dis or ps_sensor_value[4]>dis
    nearing_obstacle = front_obstacle or right_obstacle or left_obstacle or back_obstacle    
    if nearing_obstacle:
        obstacle_avoidance()
    else:
        forward()     

	
# Enter here exit cleanup code.