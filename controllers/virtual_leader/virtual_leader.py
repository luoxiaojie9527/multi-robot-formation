"""virtual_leader controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
import math, random 
import numpy as np

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
root_node = robot.getRoot()
children_field=root_node.getField("children")
children_field.importMFNodeFromString(-1,
                                  """DEF virtual_leader Transform { 
                                              translation 0 0.02 0 
                                              rotation 0 1 0 3.14
                                              children [ 
                                                  Shape { 
                                                      appearance Appearance {
                                                          material Material {
                                                              diffuseColor 1 0 0
                                                          }
                                                      }
                                                      geometry Plane {
                                                          size 0.05 0.05
                                                      }
                                                      castShadows FALSE
                                                  }
                                              ]
                                          }
                                      DEF virtual_follower Transform { 
                                              translation 0.4 0.02 -0.5
                                              children [ 
                                                  Shape { 
                                                      appearance Appearance {
                                                          material Material {
                                                              diffuseColor 0 0 1
                                                          }
                                                      }
                                                      geometry Plane {
                                                          size 0.05 0.05
                                                      }
                                                      castShadows FALSE
                                                  }
                                              ]
                                          }
                                                      
                                   """)
## Create the TRAIL Shape.
max_number_of_coordinates = 1000
children_field.importMFNodeFromString(-1,"""
                                        DEF TRAIL Shape {
                                            appearance Appearance {
                                                material Material {
                                                    diffuseColor 1 0 0
                                                    emissiveColor 1 0 0
                                                }
                                            }
                                            geometry DEF TRAIL_LINE_SET IndexedLineSet {
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

                                    """%("0 0 0\n" * max_number_of_coordinates,"0 0 -1\n" * max_number_of_coordinates))

virtual_leader = robot.getFromDef("virtual_leader")
virtual_leader_translation = virtual_leader.getField("translation")
virtual_leader_rotation = virtual_leader.getField("rotation")
translation = [0, 0.02, 0]
# Main loop:
# - perform simulation steps until Webots is stopping the controller
V_leader_max = 0.03
V_leader_k_1 = 0.03
U_leader_k_1 = 0
theta_leader_k_1 = 0
w_leader_k_1 = 0
X_leader_k_1 = 0
Y_leader_k_1 = 0
Vx_leader_k_1 = 0
Vy_leader_k_1 = 0

trail_line_set_node = robot.getFromDef("TRAIL_LINE_SET")
coordinates_node = trail_line_set_node.getField("coord").getSFNode()
point_field = coordinates_node.getField("point")
coord_index_field = trail_line_set_node.getField("coordIndex")
index = 0
first_step = True

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

while robot.step(timestep) != -1:

    t = robot.getTime()
    dt = timestep / 1000

    # U_leader_k = 0.01*math.sin(4*t)
    # w_leader_k = 0.1*math.cos(0.2*t)
    U_leader_k = 0
    w_leader_k = 0
    
    
    X_leader_k = X_leader_k_1 + dt * Vx_leader_k_1
    Y_leader_k = Y_leader_k_1 + dt * Vy_leader_k_1
    
    if X_leader_k >= 1.3:
        theta_leader_k = (math.pi + theta_leader_k) % (2*math.pi)
        U_leader_k = -U_leader_k
        # w_leader_k = -w_leader_k
            
    elif X_leader_k <= -1.3:
        theta_leader_k = (math.pi + theta_leader_k) % (2*math.pi)
        U_leader_k = -U_leader_k
        # w_leader_k = -w_leader_k
        
    elif Y_leader_k >= 1.3:
        theta_leader_k = (math.pi + theta_leader_k) % (2*math.pi)
        U_leader_k = -U_leader_k
        # w_leader_k = -w_leader_k
        
    elif Y_leader_k <= -1.3:
        theta_leader_k = (math.pi + theta_leader_k) % (2*math.pi) 
        U_leader_k = -U_leader_k
        # w_leader_k = -w_leader_k
        
    else:
        theta_leader_k = theta_leader_k_1 + dt * w_leader_k_1
        V_leader_k = V_leader_k_1 + dt * U_leader_k_1
        
    V_leader_k = V_leader_max    
    Vx_leader_k = V_leader_k * math.cos(theta_leader_k)
    Vy_leader_k = V_leader_k * math.sin(theta_leader_k)
    Ux_leader_k = math.cos(theta_leader_k) * U_leader_k - math.sin(theta_leader_k) * V_leader_k * w_leader_k
    Uy_leader_k = math.sin(theta_leader_k) * U_leader_k + math.cos(theta_leader_k) * V_leader_k * w_leader_k
    
    V_leader_k_1 = V_leader_k
    U_leader_k_1 = U_leader_k
    theta_leader_k_1 = theta_leader_k
    w_leader_k_1 = w_leader_k
    X_leader_k_1 = X_leader_k
    Y_leader_k_1 = Y_leader_k
    Vx_leader_k_1 = Vx_leader_k
    Vy_leader_k_1 = Vy_leader_k
    
    ## Draw leader
    translation[0] =  Y_leader_k
    translation[2] =  X_leader_k
    virtual_leader_translation.setSFVec3f(translation)
    rotation = [0,1,0,3.14]
    rotation[3] = theta_leader_k + math.pi
    virtual_leader_rotation.setSFRotation(rotation)
    print("virt_leader  : xl=%.6f \t yl=%.6f \t vl=%.6f" % (X_leader_k, Y_leader_k,V_leader_k))
    
    ## Draw trail
    # draw_trail(index,translation)
    first_step = False
    index = index+1
    index = index % max_number_of_coordinates

