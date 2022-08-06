#!/usr/bin/python3
# coding=UTF-8
from controller import Robot,Camera,LED,Supervisor
import time,random,math,numpy
import socket
import _thread as thread
import traceback
import numpy as np

robot = Supervisor()
timestep = int(robot.getBasicTimeStep()) #ms
timeFactor = timestep / 100.0
robot_id = int(robot.getCustomData())
sensor_num = 8
ps_sensor_name = ['ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7']
tof_sensor_name = 'tof'
speed_max = 6.28
radius_of_wheel = 0.02
wheelbase = 0.052
max_speed = 7.5
Vm = 0.06
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
# 关联并使能传感器设备
ps_sensor = []
for i_ in range(sensor_num):
    ps_sensor.append(robot.getDevice(ps_sensor_name[i_]))
    ps_sensor[i_].enable(timestep)  
    
tof_sensor = robot.getDevice(tof_sensor_name)
tof_sensor.enable(timestep)
# 关联camera
camera_time_step = timestep
camera = robot.getDevice('camera')
camera.enable(camera_time_step)  
#关联LED
led_name=['led1','led3','led5','led7']
led_num=4
led=[]
for i_ in range(led_num):
    led.append(LED(led_name[i_]))     
# 关联电机设备
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
# 设置电机运行模式
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

root_node = robot.getRoot()
children_field=root_node.getField("children")
## Create the TRAIL Shape.
max_number_of_coordinates = 2000
children_field.importMFNodeFromString(-1,"""
                                        DEF TRAIL Shape {
                                            appearance Appearance {
                                                material Material {
                                                    diffuseColor 0 1 0
                                                    emissiveColor 0 1 0
                                                }
                                            }
                                            geometry DEF TRAIL_LINE_SET_%d IndexedLineSet {
                                                coord Coordinate {
                                                    point [
                                                        %s
                                                    ]
                                                }
                                                coordIndex [
                                                    %s
                                                ]
                                            }
                                        }

                                    """%(robot_id,"0 0 0\n" * max_number_of_coordinates,"0 0 -1\n" * max_number_of_coordinates))

trail_line_set_node = robot.getFromDef("TRAIL_LINE_SET_%d" % robot_id)
coordinates_node = trail_line_set_node.getField("coord").getSFNode()
point_field = coordinates_node.getField("point")
coord_index_field = trail_line_set_node.getField("coordIndex")
index = 0
first_step = True


##################################
####       Turn LEDs On       ####         
################################## 
def TurnLeds(): 
    global opinion    
    if opinion == 1:   
        led[1].set(0x00ff00) #green for white
        pass
    elif opinion == 2:  
        led[1].set(0x0000ff) #blue for black

##################################
####        Movement          ####         
################################## 

def randomMove():
    print("randomMove")
    random_num = -1+2*random.random()
    left_motor.setVelocity(random_num*speed_max)
    right_motor.setVelocity(-random_num*speed_max)

def turnLeft():
    print("turnLeft")
    left_motor.setVelocity(-speed_max)
    right_motor.setVelocity(speed_max)

def turnRight():
    print("turnRight")
    left_motor.setVelocity(speed_max)
    right_motor.setVelocity(-speed_max)

def forward():
    print("forward")
    left_motor.setVelocity(speed_max)
    right_motor.setVelocity(speed_max)

def backward():
    print("backward")
    left_motor.setVelocity(-speed_max)
    right_motor.setVelocity(-speed_max)

def Kp(r):
    res = -1 * np.sign(r) * min(Vm, abs(0.3*r))
    # res = -0.64*(1-math.exp(-r))/(1+math.exp(-r))
    # print("res: %s" % str(res))
    return res

def Kv(r):
    # print(r)
    res =  -1 * np.sign(r) * min(4, abs(8*r))
    # res = -np.sign(r)*min(4.35,abs(2.9*r))
    return res
    
def leader_following():
    global V_i_k_1, U_i_k_1, w_i_k_1, theta_i_k_1, Vx_i_k_1, Vy_i_k_1,X_i_k_1,Y_i_k_1,V_leader_k_1,U_leader_k_1,theta_leader_k_1,w_leader_k_1
    global X_leader_k_1, Y_leader_k_1, Vx_leader_k_1, Vy_leader_k_1
    dt = timestep / 1000
    m4 = 1
    if robot_id == 1:
        expect_x_diff = 0
        expect_y_diff = 0
    elif robot_id == 2:
        expect_x_diff = 0
        expect_y_diff = 0.2
    elif robot_id == 3:
        expect_x_diff = 0
        expect_y_diff = -0.2
    elif robot_id == 4:
        expect_x_diff = 0
        expect_y_diff = 0.4

    U_leader_k = 0
    w_leader_k = 0 # not greater than 4.79
    
    ## obtain the postion and rotation of the leader from the positioning system
    pos_leader = virtual_leader.getPosition()
    rot_leader = virtual_leader_rotation.getSFRotation()
    
    ## compute the current velocity of the leader
    RX_leader_k = pos_leader[2]
    RY_leader_k = pos_leader[0]
    RO_leader_k = rot_leader[3] * np.sign(rot_leader[1]) % (2*math.pi)
    Rtheta_leader_k = (RO_leader_k - math.pi) % (2*math.pi)
    RVx_leader_k_1 = (RX_leader_k - X_leader_k_1) / dt
    RVy_leader_k_1 = (RY_leader_k - Y_leader_k_1) / dt
    RV_leader_k_1 = RVx_leader_k_1*math.cos(theta_leader_k_1) + RVy_leader_k_1*math.sin(theta_leader_k_1)
    RV_leader_k = RV_leader_k_1 + dt * U_leader_k_1
    
    X_leader_k = RX_leader_k
    Y_leader_k = RY_leader_k
    X_vleader_k = X_leader_k + expect_x_diff
    Y_vleader_k = Y_leader_k + expect_y_diff
    theta_leader_k = Rtheta_leader_k
    V_leader_k = RV_leader_k
    # if X_vleader_k >= 1.5:
        # theta_leader_k = (math.pi + theta_leader_k) % (2*math.pi)
           
    # elif X_vleader_k <= -1.5:
        # theta_leader_k = (math.pi + theta_leader_k) % (2*math.pi)
        
    # elif Y_vleader_k >= 1.5:
        # theta_leader_k = (math.pi + theta_leader_k) % (2*math.pi)
        
    # elif Y_vleader_k <= -1.5:
        # theta_leader_k = (math.pi + theta_leader_k) % (2*math.pi) 
        
    # else:
        # theta_leader_k = Rtheta_leader_k

    ## compute the Velocity and Acceleration of the leader in X and Y directions.
    Vx_leader_k = V_leader_k * math.cos(theta_leader_k)
    Vy_leader_k = V_leader_k * math.sin(theta_leader_k)
    Ux_leader_k = math.cos(theta_leader_k) * U_leader_k - math.sin(theta_leader_k) * V_leader_k * w_leader_k
    Uy_leader_k = math.sin(theta_leader_k) * U_leader_k + math.cos(theta_leader_k) * V_leader_k * w_leader_k
    
    print("real_leader  : xl=%.6f \t yl=%.6f \t rvl=%.6f" % (X_leader_k, Y_leader_k,RV_leader_k))
    
    expect_position = [Y_leader_k + expect_y_diff, X_leader_k + expect_x_diff]
    vel_epuck = epuck.getVelocity()
    rot_epuck = epuck_rotation.getSFRotation()
    pos_epuck = epuck.getPosition()
    RX_i_k = pos_epuck[2]
    RY_i_k = pos_epuck[0]
    RV_i_k = vel_epuck[2]
    RO_i_k = rot_epuck[3] * np.sign(rot_epuck[1]) % (2*math.pi)
    Rtheta_i_k = (RO_i_k + math.pi) % (2*math.pi)
 
    alpha = (theta_leader_k - Rtheta_i_k) % (2*math.pi)## angle_diff bettween Velocity direction and leader's velocity direction
    gama = getAngle([str(expect_position[0]),str(expect_position[1])]) ## the angle of target direction
    beta = (gama - Rtheta_i_k) % (2*math.pi) ## angle_diff bettween target direction and Velocity direction
    
    ## compute the angle of the expect velocity direction
    delta_x = expect_position[1] - RX_i_k
    delta_y = expect_position[0] - RY_i_k
    Vy_target = Vm + abs(V_leader_k * math.sin(theta_leader_k))* np.sign(delta_y * math.sin(theta_leader_k))
    Vx_target = Vm + abs(V_leader_k * math.cos(theta_leader_k))* np.sign(delta_x * math.cos(theta_leader_k))
    theta_target = math.atan(abs(Vy_target / Vx_target))
    print(Vy_target,Vx_target)
    if delta_x < 0:
        theta_target = math.pi - theta_target
    if delta_y < 0:
        theta_target = - theta_target
    theta_target = theta_target % (2 * math.pi)
    if abs(delta_x) < 0.025 or abs(delta_y) < 0.025:
        theta_target = theta_leader_k
    if alpha >= math.pi/2 and alpha < math.pi * 3/2:
        theta_target = theta_leader_k
    
    angle_diff = (theta_target - Rtheta_i_k) % (2*math.pi)


    X_i_k = RX_i_k
    Y_i_k = RY_i_k
    theta_i_k = Rtheta_i_k


    RVx_i_k = RV_i_k * math.cos(theta_i_k)
    RVy_i_k = RV_i_k * math.sin(theta_i_k)
    V_i_k = V_i_k_1 + dt * U_i_k_1
    Vx_i_k = V_i_k * math.cos(theta_i_k)
    Vy_i_k = V_i_k * math.sin(theta_i_k)
    TVx_i_k = Vx_i_k - Vx_leader_k
    TVy_i_k = Vy_i_k - Vy_leader_k

    zzx = X_i_k - X_leader_k - expect_x_diff
    zzy = Y_i_k - Y_leader_k - expect_y_diff

    if abs(zzx) < 10e-4 and abs(zzy) < 10e-4:
        V_i_k = V_leader_k
        
    TUx_i_k = Kv(TVx_i_k - Kp(m4*zzx))
    TUy_i_k = Kv(TVy_i_k - Kp(m4*zzy)) 

    Ux_i_k = TUx_i_k + Ux_leader_k
    Uy_i_k = TUy_i_k + Uy_leader_k
    U_i_k = math.cos(theta_i_k) * Ux_i_k + math.sin(theta_i_k) * Uy_i_k
    if V_i_k == 0:
        w_i_k = 0
    else:
        w_i_k = -1*math.sin(theta_i_k) * Ux_i_k/V_i_k + math.cos(theta_i_k)*Uy_i_k/V_i_k
    if w_i_k > 2.43:
        w_i_k = 2.43
    elif w_i_k < -2.43:
        w_i_k = -2.43

    
    left_velocity = V_i_k - wheelbase*w_i_k/2
    right_velocity = V_i_k + wheelbase*w_i_k/2
    
    left_speed = left_velocity / radius_of_wheel
    right_speed = right_velocity / radius_of_wheel

    print("angle_diff: %.6f, theta_target=%.6f,theta_i_k=%.6f,dx=%.6f,dy=%.6f" % (angle_diff*180/math.pi,theta_target*180/3.14,Rtheta_i_k*180/3.14,delta_x,delta_y))

    
    if angle_diff >= math.pi / 2 and angle_diff <= math.pi:
        turnLeft()
    elif angle_diff > math.pi / 2 and angle_diff <= math.pi*3/2:
        turnRight()
    else:
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
    # print(left_speed)
    V_i_k_1 = V_i_k
    U_i_k_1 = U_i_k
    w_i_k_1 = w_i_k
    theta_i_k_1 = theta_i_k
    Vx_i_k_1 = Vx_i_k
    Vy_i_k_1 = Vy_i_k
    X_i_k_1 = X_i_k
    Y_i_k_1 = Y_i_k
    F_translation = [0.1,0.02,-0.1]
    F_translation[0] = Y_i_k
    F_translation[2] = X_i_k
    virtual_follower_translation.setSFVec3f(F_translation)
    F_rotation = [0,1,0,3.14]
    if theta_i_k == 0:
        F_rotation[3] = math.pi
    else:
        F_rotation[3] = theta_i_k + math.pi
    virtual_follower_rotation.setSFRotation(F_rotation)
    print("virt_follower: xf=%.6f \t yf=%.6f \t vf=%.6f \t xf_diff=%.6f \t yf_diff=%.6f \t vx=%.6f \t vy=%.6f \t vx_diff=%.6f" % (X_i_k, Y_i_k, V_i_k, X_i_k-X_leader_k, Y_i_k-Y_leader_k,Vx_i_k,Vy_i_k,Vx_i_k-Vx_leader_k))
    # print("follower: xf_k_1=%s,yf_k_1=%s,vf_k_1=%s,uf_k_1=%s" % (str(X_i_k_1),str(Y_i_k_1),str(V_i_k_1),str(U_i_k_1)))


    print("real_follower: xf=%.6f \t yf=%.6f \t vf=%.6f \t xf_diff=%.6f \t yf_diff=%.6f \t vx=%.6f \t vy=%.6f \t vx_diff=%.6f" % (RX_i_k,RY_i_k,RV_i_k,RX_i_k-X_leader_k, RY_i_k-Y_leader_k,RVx_i_k,RVy_i_k,RVx_i_k-Vx_leader_k))
                    
    V_leader_k_1 = V_leader_k
    U_leader_k_1 = U_leader_k
    theta_leader_k_1 = theta_leader_k
    w_leader_k_1 = w_leader_k
    X_leader_k_1 = X_leader_k
    Y_leader_k_1 = Y_leader_k
    Vx_leader_k_1 = Vx_leader_k
    Vy_leader_k_1 = Vy_leader_k
    print("alpha=%.6f \t beta=%.6f" % (alpha * 180/math.pi, beta * 180/math.pi))
    print("-----------------------------------------")

def stop():
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
def doAction(func):
    func()

##################################
####    obstacle_avoidance     ###         
##################################  
def getAngle(pos):
    epuck = robot.getFromDef("epuck"+str(robot_id))
    self_pos = epuck.getPosition()
    self_rot = epuck.getField("rotation").getSFRotation()
    ro = self_rot[3] % (2*math.pi)
    
    delta_x = eval(pos[0]) - self_pos[0]
    delta_z = eval(pos[1]) - self_pos[2]
    theta = math.atan(delta_x / delta_z)
    gama = (2-np.sign(delta_x)) * math.pi/2 - theta
    return gama
    
def obstacle_avoidance():
    speed = [0.0, 0.0]
    for i in range(2):
        speed[i] = 0.0
        
        for j in range(8):
            speed[i] += braitenberg_coefficients[j][i] * (1.0 - (ps_sensor_value[j]/RANGE))
        # Process sensor data here.
        	
        # Enter here functions to send actuator commands, like:
        # motor.setPosition(10.0)

        left_motor.setVelocity(speed[0])
        right_motor.setVelocity(speed[1]) 

    print(speed)   
        
def formation():
    global RX_i_k_1,RY_i_k_1,RV_i_k_1,Rtheta_i_k_1
    global stuck_step

    global doAction

    dis = 95
    front_obstacle = ps_sensor_value[0]>dis or ps_sensor_value[7]>dis
    right_obstacle = ps_sensor_value[0]>dis or ps_sensor_value[1]>dis or ps_sensor_value[2]>dis
    left_obstacle = ps_sensor_value[6]>dis or ps_sensor_value[7]>dis or ps_sensor_value[5]>dis
    back_obstacle = ps_sensor_value[3]>dis or ps_sensor_value[4]>dis 
    obstacle_closing = tof_sensor_value < 120
    obstacle_nearing = front_obstacle or right_obstacle or left_obstacle or back_obstacle or obstacle_closing
    free = max(ps_sensor_value) < 80
    vel_epuck = epuck.getVelocity()
    rot_epuck = epuck_rotation.getSFRotation()
    pos_epuck = epuck.getPosition()
    RX_i_k = pos_epuck[2]
    RY_i_k = pos_epuck[0]
    RV_i_k = vel_epuck[2]
    RO_i_k = rot_epuck[3] * np.sign(rot_epuck[1]) % (2*math.pi)
    Rtheta_i_k = (RO_i_k + math.pi) % (2*math.pi)
    
    delta_x = RX_i_k - RX_i_k_1
    delta_y = RY_i_k - RY_i_k_1
    delta_theta = Rtheta_i_k - Rtheta_i_k_1
    print("RV=%.6f \t dx=%.6f \t dy=%.6f \t dtheta=%.6f" % (RV_i_k,delta_x,delta_y,delta_theta))
    if abs(RV_i_k)<10e-6 and abs(delta_x) < 10e-6 and abs(delta_y) < 10e-6 and abs(delta_theta) < 10e-6:
        stuck_step = stuck_step + 1
        
    if stuck_step > 5:
        doAction(randomMove)
        stuck_step = 0
                   
    elif obstacle_closing:
        doAction(leader_following)
        doAction(obstacle_avoidance)
    elif obstacle_nearing:
        doAction(obstacle_avoidance)
    elif free:
        # print("leader_following")
        doAction(leader_following)
    else:
        doAction(obstacle_avoidance)
        
    RX_i_k_1 = RX_i_k
    RY_i_k_1 = RY_i_k
    RV_i_k_1 = RV_i_k
    Rtheta_i_k_1 = Rtheta_i_k
##################################
####        ImageProcess      ####         
##################################     


def detect_color(array):
    if array[0] < 10 and array[1] < 20 and array[2] < 20:
        return "black"
    elif array[0] > 200 and array[1] > 200 and array[2] >200:
        return "white"
    elif abs(array[0]-197) < 10 and abs(array[1]-60) <20 and abs(array[2]-60)<20:
        return "red"
    elif abs(array[0]-0) < 20 and abs(array[1]-207) <20 and abs(array[2]-207)<20:
        return "blue"
    else:
        return "undefined"

def image_process(img_array,color):
    black_num = 0
    row = len(img_array)
    colum = len(img_array[0])
    half = row * colum
    for i in range(row):
        for j in range(colum):
            if detect_color(img_array[i][j]) == color:
                black_num = black_num + 1
        if black_num > half*2/3:
            return 1
    return 0
    

def callback(data):
    global current_color
    try:
        cv_bridge = CvBridge()
        cv_image = cv_bridge.imgmsg_to_cv2(data, "rgb8")
        resp = image_process(cv_image)
        if resp:
          print("black")
          current_color = 1
        else:
          print("white")
          current_color = 0
    except CvBridgeError as e:
        rospy.logerr('[tf-pose-estimation] Converting Image Error. ' + str(e))
        return
        
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("camera", Image, callback)
    
def draw_trail(index,translation):
    point_field.setMFVec3f(index,translation)
    if index > 0:
        coord_index_field.setMFInt32(3*(index-1),index-1)
        coord_index_field.setMFInt32(3*(index-1)+1,index)
    elif index == 0 and first_step == False:
        coord_index_field.setMFInt32(3*(max_number_of_coordinates-1),0)
        coord_index_field.setMFInt32(3*(max_number_of_coordinates-1)+1,max_number_of_coordinates-1)
    coord_index_field.setMFInt32(3*index,index)
    coord_index_field.setMFInt32(3*index+1,index)
    

      
##################################
####        Explore           ####         
##################################                            
               

##################################
####        Diffusing         ####         
##################################   



##################################
####       Main Process       ####         
##################################    
if __name__ == '__main__':      

    ##################################
    ####        MainLoop          ####         
    ##################################      
    virtual_leader = robot.getFromDef("virtual_leader")
    virtual_leader_translation = virtual_leader.getField("translation")
    virtual_leader_rotation = virtual_leader.getField("rotation")
    virtual_follower = robot.getFromDef("virtual_follower")
    virtual_follower_translation = virtual_follower.getField("translation")
    virtual_follower_rotation = virtual_follower.getField("rotation")
    virtual_follower_rotation.setSFRotation([0,1,0,3.14])
    epuck = robot.getFromDef("epuck"+str(robot_id)) 
    epuck_rotation = epuck.getField("rotation")
    
    pos_epuck = epuck.getPosition()
    
    
    V_i_k_1 = 0.01
    U_i_k_1 = 0
    w_i_k_1 = 0
    theta_i_k_1 = 0
    Vx_i_k_1 = 0.01
    Vy_i_k_1 = 0 
    X_i_k_1 = pos_epuck[2]
    Y_i_k_1 = pos_epuck[0]
    V_leader_k_1 = 0.07
    U_leader_k_1 = 0
    theta_leader_k_1 = 0
    w_leader_k_1 = 0
    X_leader_k_1 = 0
    Y_leader_k_1 = 0
    Vx_leader_k_1 = 0
    Vy_leader_k_1 = 0
    RX_i_k_1 = 0
    RY_i_k_1 = 0
    RV_i_k_1 = 0
    Rtheta_i_k_1 = 0
    
    stuck_step = 0
    
    while robot.step(timestep) != -1:
        try:
            ps_sensor_value = []
            for i_ in range(sensor_num):
                ps_sensor_value.append(ps_sensor[i_].getValue())
            tof_sensor_value = tof_sensor.getValue()

            
            formation()
            
            ## Draw trail
            pos_epuck = epuck.getPosition()
            translation = [pos_epuck[0],0.02,pos_epuck[2]]
            # draw_trail(index,translation)
            first_step = False
            index = index+1
            index = index % max_number_of_coordinates
        except Exception as e:
            print(traceback.print_exc())
                      
