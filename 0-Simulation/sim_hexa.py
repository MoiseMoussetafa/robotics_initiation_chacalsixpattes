#!/usr/bin/env python
import math
import sys
import os
import time
import argparse
import pybullet as p
from onshape_to_robot.simulation import Simulation
import kinematics
from constants import *
from math import *

# from squaternion import Quaternion
from scipy.spatial.transform import Rotation


# Parameters for legs
class Parameters:
    def __init__(
        self, z=-0.1,
    ):
        self.z = z
        # Angle between the X axis of the leg and the X axis of the robot for each leg
        self.legAngles = LEG_ANGLES
        self.initLeg = []   # INIT LEG POSITIONS
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])

        # Motor re-united by joint name for the simulation
        self.legs = {}
        self.legs[1] = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf"]
        self.legs[6] = ["j_c1_rm", "j_thigh_rm", "j_tibia_rm"]
        self.legs[5] = ["j_c1_rr", "j_thigh_rr", "j_tibia_rr"]
        self.legs[2] = ["j_c1_lf", "j_thigh_lf", "j_tibia_lf"]
        self.legs[3] = ["j_c1_lm", "j_thigh_lm", "j_tibia_lm"]
        self.legs[4] = ["j_c1_lr", "j_thigh_lr", "j_tibia_lr"]


# Init positions of arms
def initRobot(params):
    targets = {}
    for leg_id in range (1,7):
        alphas = kinematics.computeIKOriented(0,0,0,leg_id,params)
        set_leg_angles(alphas, leg_id, targets, params)
    return alphas
    state = sim.setJoints(targets)
    sim.tick()


def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    # q = Quaternion.from_euler(roll, pitch, yaw, degrees=degrees)
    # return [q[1], q[2], q[3], q[0]]

    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    # Convert to quaternions and print
    rot_quat = rot.as_quat()
    # print(rot_quat)
    return rot_quat


# Updates the values of the dictionnary targets to set 3 angles to given leg
def set_leg_angles(alphas, leg_id, targets, params):
    leg = params.legs[leg_id]
    i = -1
    for name in leg:
        i += 1
        targets[name] = alphas[i]


# Calculation of distance between legs
def calcul_dist(list_of_pos):
    distances_pattes = [0,0,0,0,0,0]
    for i in range (0,6):
        distances_pattes[i] = math.sqrt(math.pow(list_of_pos[i][0] - list_of_pos[(i+2)%6][0], 2) +
                                        math.pow(list_of_pos[i][1] - list_of_pos[(i+2)%6][1], 2) +
                                        math.pow(list_of_pos[i][2] - list_of_pos[(i+2)%6][2], 2))
    return distances_pattes



# Menu help : python3 sim_hexa.py -h
parser = argparse.ArgumentParser(description="== List of available functions of hexapod, based on triangle movement in priority ==")
parser.add_argument("-m", "--mode", type=str, default="direct", help="select a MODE")
parser.add_argument("-ultrawalk", help="MODE ultrawalk : holonomic walk", action="store_true")
parser.add_argument("-walk", help="MODE walk : Walking in a straight line (arbitrary direction)", action="store_true")
parser.add_argument("-inverse-all", help="MODE inverse-all : Moving an arbitrary leg to an arbitrary (x, y, z) position", action="store_true")
parser.add_argument("-inverse", help="MODE inverse : Moving leg 1 to an arbitrary (x, y, z) position, 3 visible cursors", action="store_true")
parser.add_argument("-rotation", help="MODE rotation : Rotating without moving the center of the robot ", action="store_true")
parser.add_argument("-rotate", help="MODE rotate : Rotating without moving the center of the robot (the 6 legs staying on the floor) ", action="store_true")
parser.add_argument("-robot-ik", help="MODE robot-ik : Moving the center of the robot to an arbitrary (x, y, z) position (the 6 legs staying on the floor)", action="store_true")
args = parser.parse_args()


controls = {}
robotPath = "phantomx_description/urdf/phantomx.urdf"
sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
pos, rpy = sim.getRobotPose()
sim.setRobotPose([0, 0, 0.5], to_pybullet_quaternion(0,0,0))
params = Parameters ()

# m_friction
# sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)

""" Some custom variables """
raw = 0
pitch = 0
yaw = 0

posx = 0
posy = 0

leg_center_pos = [0.1248, -0.06164, 0.001116 + 0.5]
leg_angle = -math.pi / 4

old_distances_pattes = [0,0,0,0,0,0]
distances_pattes = [0,0,0,0,0,0]

new_time = 0 
old_time = 0

patinage_delta_t = 0.1
patinage_old_t = 0

seuil_patinage_mm = 0.5

bx = 0.07
bz = 0.25



###################################################################################################
###################################################################################################
""" Controls, depending on the mode """

# Dictionary of personal controls, for debug system
controlp = {}
controlp["airpause"] = p.addUserDebugParameter("OFF < airpause > ON", 0, 1, 0)  
controlp["debuglines"] = p.addUserDebugParameter("OFF < debuglines > ON", 0, 1, 0)  
controlp["seuil_patinage_mm"] = p.addUserDebugParameter("seuil_patinage_mm", 0.01, 10, 5)          


# For next if and elif, last values of controls : min, max, default value

if args.mode == "frozen-direct":
    crosses = []
    for i in range(4):
        crosses.append(p.loadURDF("target2/robot.urdf"))
    for name in sim.getJoints():
        print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
        
elif args.mode == "direct":
    for name in sim.getJoints():
        print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)


elif args.mode == "inverse":
    cross = p.loadURDF("target2/robot.urdf")
    alphas = kinematics.computeDK(0, 0, 0, use_rads=True)
    controls["target_x"] = p.addUserDebugParameter("target_x", -0.4, 0.4, alphas[0])
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.4, 0.4, alphas[1])
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.4, 0.4, alphas[2])

elif args.mode == "robot-ik":
    controls["target_x"] = p.addUserDebugParameter("target_x", -0.1, 0.1)
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.1, 0.1)
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.1, 0.1)    

elif args.mode == "rotation":
    controls["target_x"] = p.addUserDebugParameter("target_x", 0, 0.3, 0.180)    
    controls["target_z"] = p.addUserDebugParameter("target_z", -1, 0.5, -0.15)    
    controls["target_h"] = p.addUserDebugParameter("target_h", -0.2, 0.2, 0.001)
    controls["target_w"] = p.addUserDebugParameter("target_w", -0.3, 0.3, 0.1)    
    controls["period"] = p.addUserDebugParameter("period", 0.1, 10, 1)

elif args.mode == "walk":
    controls["x"] = p.addUserDebugParameter("x", -0.1, 0.1, 0)                                     
    controls["height_hexapode"] = p.addUserDebugParameter("z > height_hexapode", -0.1, 0.1, 0)         
    controls["height_arms"] = p.addUserDebugParameter("h", 0, 0.2, 0.005)                 
    controls["amplitude"] = p.addUserDebugParameter("w > amplitude", 0.1, 0.5, 0.1)                     
    controls["speed"] = p.addUserDebugParameter("speed", 0.1, 10, 1)                                
    controls["direction"] = p.addUserDebugParameter("extra-theta > direction", -math.pi, math.pi, 0)                     

elif args.mode == "ultrawalk":
    controls["x"] = p.addUserDebugParameter("x", -0.1, 0.1, 0)                                      
    controls["height_hexapode"] = p.addUserDebugParameter("z > height_hexapode", -0.1, 0.1, -0.02)      
    controls["height_arms"] = p.addUserDebugParameter("h", 0, 0.2, 0.03)                  
    controls["amplitude"] = p.addUserDebugParameter("w > amplitude", 0.001, 0.5, 0.1)                   
    controls["speed"] = p.addUserDebugParameter("speed", 0.1, 10, 1)                                
    controls["direction"] = p.addUserDebugParameter("extra-theta > direction", -math.pi, math.pi, 0)             
    controls["target_w"] = p.addUserDebugParameter("target_w > rotation", -0.3, 0.3, 0)            
    controls["xr"] = p.addUserDebugParameter("xr", 0.15, 0.2, 0.18) 
              
elif args.mode == "ultrawalkcircle":
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.1, 0.1, 0)          
    controls["target_r"] = p.addUserDebugParameter("target_r", 0.01, 0.05, 0.025)                 
    controls["duration"] = p.addUserDebugParameter("duration", 0.01, 5, 0.595)                     
    controls["extra_theta"] = p.addUserDebugParameter("extra_theta", 0, 9.5, 4.75) 
    controls["target_w"] = p.addUserDebugParameter("target_w", -0.04, 0.04, 0)                     
            
elif args.mode == "rotationcircle":
    controls["target_z"] = p.addUserDebugParameter("target_z", -2, 0, -0.1)
    controls["target_r"] = p.addUserDebugParameter("target_r", 0.001, 0.1, 0.040)
    controls["target_duration"] = p.addUserDebugParameter("target_duration", 0.01, 1, 1)

elif args.mode == "walkcircle":
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.1, 0.1, -0.02)          
    controls["target_r"] = p.addUserDebugParameter("target_r", 0.01, 0.05, 0.025)                 
    controls["duration"] = p.addUserDebugParameter("duration", 0.01, 5, 1)                     
    controls["extra_theta"] = p.addUserDebugParameter("extra_theta", 0, 9.5, 4.75)    
       
elif args.mode == "inverse-all":
    alphas = kinematics.computeDK(0, 0, 0, use_rads=True)
    controls["target_x1"] = p.addUserDebugParameter("target_x1", -0.4, 0.4, alphas[0])
    controls["target_y1"] = p.addUserDebugParameter("target_y1", -0.4, 0.4, alphas[1])
    controls["target_z1"] = p.addUserDebugParameter("target_z1", -0.4, 0.4, alphas[2])
    controls["target_x2"] = p.addUserDebugParameter("target_x2", -0.4, 0.4, alphas[0])
    controls["target_y2"] = p.addUserDebugParameter("target_y2", -0.4, 0.4, alphas[1])
    controls["target_z2"] = p.addUserDebugParameter("target_z2", -0.4, 0.4, alphas[2])
    controls["target_x3"] = p.addUserDebugParameter("target_x3", -0.4, 0.4, alphas[0])
    controls["target_y3"] = p.addUserDebugParameter("target_y3", -0.4, 0.4, alphas[1])
    controls["target_z3"] = p.addUserDebugParameter("target_z3", -0.4, 0.4, alphas[2])
    controls["target_x4"] = p.addUserDebugParameter("target_x4", -0.4, 0.4, alphas[0])
    controls["target_y4"] = p.addUserDebugParameter("target_y4", -0.4, 0.4, alphas[1])
    controls["target_z4"] = p.addUserDebugParameter("target_z4", -0.4, 0.4, alphas[2])
    controls["target_x5"] = p.addUserDebugParameter("target_x5", -0.4, 0.4, alphas[0])
    controls["target_y5"] = p.addUserDebugParameter("target_y5", -0.4, 0.4, alphas[1])
    controls["target_z5"] = p.addUserDebugParameter("target_z5", -0.4, 0.4, alphas[2])
    controls["target_x6"] = p.addUserDebugParameter("target_x6", -0.4, 0.4, alphas[0])
    controls["target_y6"] = p.addUserDebugParameter("target_y6", -0.4, 0.4, alphas[1])
    controls["target_z6"] = p.addUserDebugParameter("target_z6", -0.4, 0.4, alphas[2])

elif args.mode == "rotate":
    controls["target_z"] = p.addUserDebugParameter("target_z", -2, 0, -0.1)
    controls["target_r"] = p.addUserDebugParameter("target_r", 0.001, 0.1, 0.023)
    controls["target_duration"] = p.addUserDebugParameter("target_duration", 0.01, 10, 1)
    controls["max_angles"] = p.addUserDebugParameter("max_angles", 8, 50, 15)

elif args.mode == "rotationcirclenew":
    controls["target_z"] = p.addUserDebugParameter("target_z", -2, 0, -0.1)
    controls["target_r"] = p.addUserDebugParameter("target_r", 0.001, 0.1, 0.023)
    controls["target_duration"] = p.addUserDebugParameter("target_duration", 0.01, 1, 1)

elif args.mode == "topkek":
    controls["x"] = p.addUserDebugParameter("x", -0.1, 0.1, 0)                                      
    controls["height_hexapode"] = p.addUserDebugParameter("z > height_hexapode", -0.1, 0.1, -0.02)      
    controls["height_arms"] = p.addUserDebugParameter("h", 0, 0.2, 0.03)                  
    controls["amplitude"] = p.addUserDebugParameter("w > amplitude", 0.001, 0.5, 0.1)                   
    controls["speed"] = p.addUserDebugParameter("speed", 0.1, 10, 1)                                
    controls["direction"] = p.addUserDebugParameter("extra-theta > direction", -math.pi, math.pi, 0)             
    controls["target_w"] = p.addUserDebugParameter("target_w > rotation", -0.3, 0.3, 0)            
    controls["xr"] = p.addUserDebugParameter("xr", 0.15, 0.2, 0.18) 


###################################################################################################
###################################################################################################
""" Init the robot """
initRobot(params)
time.sleep(0.5)

###################################################################################################
###################################################################################################
""" All modes coded here """

while True:

    targets = {}
    for name in sim.getJoints():
        if "c1" in name or "thigh" in name or "tibia" in name:
            targets[name] = 0
    if args.mode == "frozen-direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        points = kinematics.computeDKDetailed(
            targets["j_c1_rf"],
            targets["j_thigh_rf"],
            targets["j_tibia_rf"],
            use_rads=True,
        )
        i = -1
        T = []
        for pt in points:
            # Drawing each step of the DK calculation
            i += 1
            T.append(kinematics.rotaton_2D(pt[0], pt[1], pt[2], leg_angle))
            T[-1][0] += leg_center_pos[0]
            T[-1][1] += leg_center_pos[1]
            T[-1][2] += leg_center_pos[2]
            # print("Drawing cross {} at {}".format(i, T))
            p.resetBasePositionAndOrientation(
                crosses[i], T[-1], to_pybullet_quaternion(0, 0, leg_angle)
            )

        # Temp
        sim.setRobotPose([0, 0, 0.5], to_pybullet_quaternion(0, 0, 0))
        # sim.setRobotPose(
        #     leg_center_pos, to_pybullet_quaternion(0, 0, 0),
        # )
        state = sim.setJoints(targets)


    elif args.mode == "direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        state = sim.setJoints(targets)


    elif args.mode == "inverse":
        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"])
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        dk0 = kinematics.computeDK(0, 0, 0, use_rads=True)
        targets["j_c1_rf"] = alphas[0]
        targets["j_thigh_rf"] = alphas[1]
        targets["j_tibia_rf"] = alphas[2]
        state = sim.setJoints(targets)
        # Temp
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        T = kinematics.rotaton_2D(x, y, z, leg_angle)
        T[0] += leg_center_pos[0]
        T[1] += leg_center_pos[1]
        T[2] += leg_center_pos[2]
        # print("Drawing cross {} at {}".format(i, T))
        p.resetBasePositionAndOrientation(
            cross, T, to_pybullet_quaternion(0, 0, leg_angle)
        )


    elif args.mode == "robot-ik":
        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"])
        for leg_id in range(1,7):
            # To create movement : A * math.sin(2 * math.pi * 0.5 * time.time())
            # with A as amplitude (x, y, z, like 0.03m, or the parameters above)
            alphas = kinematics.computeIKOriented(x, y, z, leg_id, params)
            set_leg_angles(alphas, leg_id, targets, params)
        state = sim.setJoints(targets)


    elif args.mode == "rotation":
        x = p.readUserDebugParameter(controls["target_x"])
        z = p.readUserDebugParameter(controls["target_z"])
        h = p.readUserDebugParameter(controls["target_h"])
        w = p.readUserDebugParameter(controls["target_w"])
        period = p.readUserDebugParameter(controls["period"])
    
        for leg_id in range (1,7):
            if (leg_id == 1) or (leg_id == 3) or (leg_id == 5) :
                alphas = kinematics.triangle_for_rotation(x, z, h, w, sim.t, period)
                set_leg_angles(alphas, leg_id, targets, params)

            elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6):
                alphas = kinematics.triangle_for_rotation(x, z, h, w, sim.t + 0.5 * period, period)
                set_leg_angles(alphas, leg_id, targets, params)
            
        state = sim.setJoints(targets)
            

    elif args.mode == "walk":
        x = p.readUserDebugParameter(controls["x"])
        z = p.readUserDebugParameter(controls["height_hexapode"])
        h = p.readUserDebugParameter(controls["height_arms"])
        w = p.readUserDebugParameter(controls["amplitude"])
        period = p.readUserDebugParameter(controls["speed"])
        direction = p.readUserDebugParameter(controls["direction"])
        for leg_id in range (1,7):
            if (leg_id == 1) or (leg_id == 3) or (leg_id == 5) :
                alphas = kinematics.triangle_w(x, z, h, w, sim.t, period, leg_id, params, direction)
                set_leg_angles(alphas, leg_id, targets, params)
                
            elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6):
                alphas = kinematics.triangle_w(x, z, h, w, sim.t + 0.5 * period, period, leg_id, params, direction)
                set_leg_angles(alphas, leg_id, targets, params)

            state = sim.setJoints(targets)


    elif args.mode == "ultrawalk":
        # Parameters for triangle in walk
        x = p.readUserDebugParameter(controls["x"])
        z = p.readUserDebugParameter(controls["height_hexapode"])
        h = p.readUserDebugParameter(controls["height_arms"])
        w = p.readUserDebugParameter(controls["amplitude"])
        period = p.readUserDebugParameter(controls["speed"])
        direction = p.readUserDebugParameter(controls["direction"])

        # Parameters for triangle in rotation
        xr = p.readUserDebugParameter(controls["xr"]) #0.180  
        zr = -0.15 
        hr = 0.001 
        wr = p.readUserDebugParameter(controls["target_w"])
        periodr = period 

        for leg_id in range (1,7):
            if (leg_id == 1) or (leg_id == 3) or (leg_id == 5) :
                alphas = kinematics.triangle_w(x, z, h, w, sim.t, period, leg_id, params, direction)
                set_leg_angles(alphas, leg_id, targets, params)
                alphas1 = kinematics.triangle_for_rotation(xr, zr, hr, wr, sim.t, periodr)

                A1 = alphas[0] + alphas1[0]
                A2 = alphas[1] + alphas1[1]
                A3 = alphas[2] + alphas1[2]
                ALPHA = [A1, A2, A3]
                set_leg_angles(ALPHA, leg_id, targets, params)

            elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6):
                alphas = kinematics.triangle_w(x, z, h, w, sim.t + 0.5 * period, period, leg_id, params, direction)
                alphas1 = kinematics.triangle_for_rotation(xr, zr, hr, wr, sim.t + 0.5 * periodr, periodr)

                A1 = alphas[0] + alphas1[0]
                A2 = alphas[1] + alphas1[1]
                A3 = alphas[2] + alphas1[2] 
                ALPHA = [A1, A2, A3]
                set_leg_angles(ALPHA, leg_id, targets, params)

        state = sim.setJoints(targets)


    # Not working properly, some skating
    elif args.mode == "ultrawalkcircle":
        x = 0
        z = p.readUserDebugParameter(controls["target_z"])
        r = p.readUserDebugParameter(controls["target_r"])
        duration = p.readUserDebugParameter(controls["duration"])
        extra_theta = p.readUserDebugParameter(controls["extra_theta"])
            
        xr = 0
        zr = -0.5
        w = p.readUserDebugParameter(controls["target_w"])

        for leg_id in range (1,7):
            if (leg_id == 1) or (leg_id == 3) or (leg_id == 5) :
                alphas = kinematics.demicircle(x, z, r, sim.t, duration, leg_id, params, extra_theta)
                if w == 0 :
                    alphas1 = kinematics.segment_oneway_w(0,0,zr,0,0,zr,sim.t,duration,leg_id,params,extra_theta)
                else :
                    alphas1 = kinematics.demicirclefloor(xr, zr, w, sim.t, duration, leg_id, params)
                A1 = alphas[0] + alphas1[0]
                A2 = alphas[1] + alphas1[1]
                A3 = alphas[2] + alphas1[2]
                ALPHA = [A1, A2, A3]
                set_leg_angles(ALPHA, leg_id, targets, params)

            elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6):
                alphas = kinematics.demicircle(x, z, r, sim.t + 0.5*duration, duration, leg_id, params, extra_theta)
                if w == 0 :
                    alphas1 = kinematics.segment_oneway_w(0,0,zr,0,0,zr,sim.t,duration,leg_id,params, extra_theta)
                else :
                    alphas1 = kinematics.demicirclefloor(xr, zr, w, sim.t + 0.5*duration, duration, leg_id, params)
                A1 = alphas[0] + alphas1[0]
                A2 = alphas[1] + alphas1[1]
                A3 = alphas[2] + alphas1[2] 
                ALPHA = [A1, A2, A3]
                set_leg_angles(ALPHA, leg_id, targets, params)

        state = sim.setJoints(targets)    


    elif args.mode == "rotationcircle" :
        x = 0
        z = p.readUserDebugParameter(controls["target_z"])
        r = p.readUserDebugParameter(controls["target_r"])
        duration = p.readUserDebugParameter(controls["target_duration"])
        for leg_id in range (1,7):
            if (leg_id == 1) or (leg_id == 3) or (leg_id == 5) :
                alphas = kinematics.demicirclefloor(x, z, r, sim.t, duration, leg_id, params)
                set_leg_angles(alphas, leg_id, targets, params)

            elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6) :
                alphas = kinematics.demicirclefloor(x, z, r, sim.t + 0.5*duration, duration, leg_id, params)
                set_leg_angles(alphas, leg_id, targets, params)
            
            state = sim.setJoints(targets)


    # Not coded properly, very not OK, i need some help wtf
    elif args.mode == "rotationcirclenew" :
        x = 0
        z = p.readUserDebugParameter(controls["target_z"])
        r = p.readUserDebugParameter(controls["target_r"])
        duration = p.readUserDebugParameter(controls["target_duration"])
        circle_radius_m = 0.3
        max_angle = math.pi/8

        # besoin de faire une variable qui va de 0 à duration, puis de couper duration en 2 et faire les mouvements de pattes 3 par 3
        t = math.fmod(sim.t,duration)
        t2 = math.fmod(0,duration)

        for leg_id in range (1,7):
            if t2 < t :
                if (leg_id == 1) or (leg_id == 3) or (leg_id == 5) :
                    angle = max_angle * math.cos(sim.t) + LEG_ANGLES_2[leg_id - 1]
                    x = circle_radius_m * math.cos(angle)
                    y = circle_radius_m * math.sin(angle)
                    alphas = kinematics.computeIKRobotCentered(x, y, z, leg_id)
                    set_leg_angles(alphas, leg_id, targets, params)
                elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6) :
                    angle = max_angle * math.cos(sim.t) + LEG_ANGLES_2[leg_id - 1]
                    x = circle_radius_m * math.cos(angle)
                    y = circle_radius_m * math.sin(angle)
                    alphas = kinematics.computeIKRobotCentered(x, y, z, leg_id)
                    set_leg_angles(alphas, leg_id, targets, params)
            else :
                if (leg_id == 1) or (leg_id == 3) or (leg_id == 5) :
                    angle = max_angle * math.cos(sim.t) + LEG_ANGLES_2[leg_id - 1]
                    x = circle_radius_m * math.cos(angle)
                    y = circle_radius_m * math.sin(angle)
                    alphas = kinematics.computeIKRobotCentered(x, y, z, leg_id)
                    set_leg_angles(alphas, leg_id, targets, params)
                elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6) :
                    angle = max_angle * math.cos(sim.t) + LEG_ANGLES_2[leg_id - 1]
                    x = circle_radius_m * math.cos(angle)
                    y = circle_radius_m * math.sin(angle)
                    alphas = kinematics.computeIKRobotCentered(x, y, z, leg_id)
                    set_leg_angles(alphas, leg_id, targets, params)
            
        state = sim.setJoints(targets)


    # Need some improvements for obtain the walk with more stabilization 
    elif args.mode == "walkcircle":
        x = 0
        z = p.readUserDebugParameter(controls["target_z"])
        r = p.readUserDebugParameter(controls["target_r"])
        duration = p.readUserDebugParameter(controls["duration"])
        extra_theta = p.readUserDebugParameter(controls["extra_theta"])
        for leg_id in range (1,7):
            if (leg_id == 1) or (leg_id == 3) or (leg_id == 5) :
                alphas = kinematics.demicircle(x, z, r, sim.t, duration, leg_id, params, extra_theta)
                set_leg_angles(alphas, leg_id, targets, params)
                
            elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6):
                alphas = kinematics.demicircle(x, z, r, sim.t + 0.5*duration, duration, leg_id, params, extra_theta)
                set_leg_angles(alphas, leg_id, targets, params)
            state = sim.setJoints(targets)
        

    elif args.mode == "inverse-all":
        x = p.readUserDebugParameter(controls["target_x1"])
        y = p.readUserDebugParameter(controls["target_y1"])
        z = p.readUserDebugParameter(controls["target_z1"])
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        dk1 = kinematics.computeDK(0, 0, 0, use_rads=True)
        targets["j_c1_rf"] = alphas[0]
        targets["j_thigh_rf"] = alphas[1]
        targets["j_tibia_rf"] = alphas[2]
        state = sim.setJoints(targets)

        x = p.readUserDebugParameter(controls["target_x2"])
        y = p.readUserDebugParameter(controls["target_y2"])
        z = p.readUserDebugParameter(controls["target_z2"])
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        dk2 = kinematics.computeDK(0, 0, 0, use_rads=True)
        targets["j_c1_lf"] = alphas[0]
        targets["j_thigh_lf"] = alphas[1]
        targets["j_tibia_lf"] = alphas[2]
        state = sim.setJoints(targets)

        x = p.readUserDebugParameter(controls["target_x3"])
        y = p.readUserDebugParameter(controls["target_y3"])
        z = p.readUserDebugParameter(controls["target_z3"])
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        dk3 = kinematics.computeDK(0, 0, 0, use_rads=True)
        targets["j_c1_lm"] = alphas[0]
        targets["j_thigh_lm"] = alphas[1]
        targets["j_tibia_lm"] = alphas[2]
        state = sim.setJoints(targets)

        x = p.readUserDebugParameter(controls["target_x4"])
        y = p.readUserDebugParameter(controls["target_y4"])
        z = p.readUserDebugParameter(controls["target_z4"])
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        dk4 = kinematics.computeDK(0, 0, 0, use_rads=True)
        targets["j_c1_lr"] = alphas[0]
        targets["j_thigh_lr"] = alphas[1]
        targets["j_tibia_lr"] = alphas[2]
        state = sim.setJoints(targets)

        x = p.readUserDebugParameter(controls["target_x5"])
        y = p.readUserDebugParameter(controls["target_y5"])
        z = p.readUserDebugParameter(controls["target_z5"])
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        dk5 = kinematics.computeDK(0, 0, 0, use_rads=True)
        targets["j_c1_rr"] = alphas[0]
        targets["j_thigh_rr"] = alphas[1]
        targets["j_tibia_rr"] = alphas[2]
        state = sim.setJoints(targets)

        x = p.readUserDebugParameter(controls["target_x6"])
        y = p.readUserDebugParameter(controls["target_y6"])
        z = p.readUserDebugParameter(controls["target_z6"])
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        dk6 = kinematics.computeDK(0, 0, 0, use_rads=True)
        targets["j_c1_rm"] = alphas[0]
        targets["j_thigh_rm"] = alphas[1]
        targets["j_tibia_rm"] = alphas[2]
        state = sim.setJoints(targets)

        # Surelevation robot pose
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

    elif args.mode == "rotate" :
        x = 0
        z = p.readUserDebugParameter(controls["target_z"])
        duration = p.readUserDebugParameter(controls["target_duration"])
        max_angles = math.pi/p.readUserDebugParameter(controls["max_angles"])
        circle_radius_m = 0.3

        for leg_id in range (1,7):
            angle = max_angles * math.cos(sim.t*duration) + LEG_ANGLES_2[leg_id - 1]
            x = circle_radius_m * math.cos(angle)
            y = circle_radius_m * math.sin(angle)
            alphas = kinematics.computeIKRobotCentered(x, y, z, leg_id)
            set_leg_angles(alphas, leg_id, targets, params)

        state = sim.setJoints(targets)

    # Fun and customizable mode 
    elif args.mode == "topkek":
        x = p.readUserDebugParameter(controls["x"])
        z = p.readUserDebugParameter(controls["height_hexapode"])
        h = p.readUserDebugParameter(controls["height_arms"])
        w = p.readUserDebugParameter(controls["amplitude"])
        period = p.readUserDebugParameter(controls["speed"])
        direction = p.readUserDebugParameter(controls["direction"])

        xr = p.readUserDebugParameter(controls["xr"]) #0.180  
        zr = -0.15 
        hr = 0.001 
        wr = p.readUserDebugParameter(controls["target_w"])
        periodr = period 

        for leg_id in range (1,7):
            if (leg_id == 1) or (leg_id == 5) :
                alphas = kinematics.triangle_w(x, z, h, w, sim.t, period, leg_id, params, direction)
                set_leg_angles(alphas, leg_id, targets, params)
                alphas1 = kinematics.triangle_for_rotation(xr, zr, hr, wr, sim.t, periodr)

                A1 = alphas[0] + alphas1[0]
                A2 = alphas[1] + alphas1[1]
                A3 = alphas[2] + alphas1[2]
                ALPHA = [A1, A2, A3]
                set_leg_angles(ALPHA, leg_id, targets, params)

            elif (leg_id == 2) or (leg_id == 4) :
                alphas = kinematics.triangle_w(x, z, h, w, sim.t + 0.5 * period, period, leg_id, params, direction)
                alphas1 = kinematics.triangle_for_rotation(xr, zr, hr, wr, sim.t + 0.5 * periodr, periodr)

                A1 = alphas[0] + alphas1[0]
                A2 = alphas[1] + alphas1[1]
                A3 = alphas[2] + alphas1[2] 
                ALPHA = [A1, A2, A3]
                set_leg_angles(ALPHA, leg_id, targets, params)
            
            elif (leg_id == 3) or (leg_id == 6) :
                alphas = kinematics.segment(0.4,0,0,0.4,0,1,sim.t, duration=1)
               
                set_leg_angles(alphas, leg_id, targets, params)

        state = sim.setJoints(targets)
    

###################################################################################################
###################################################################################################
    """ Fonctions in all modes """


    """Work tests here to obtain some infos about the position and speed of the hexapode"""
    """Need to be improve and write properly after tests"""
    
    """
    oldraw = raw
    raw = rpy[0]
    vraw = (raw - oldraw) / time.time()
    print ("vraw : {0:.1f}".format(vraw))

    oldpitch = pitch
    pitch = rpy[1]   
    vpitch = (pitch - oldpitch) / time.time()
    vpitch = vpitch * 180/math.pi
    print ("vpitch : {0:.1f}".format(vpitch))
    """
    
    """
    oldyaw = yaw
    yaw = rpy[2]
    vyaw = ((yaw - oldyaw)*10**11) / time.time() # A diviser par le temps la période de boucle
    vyaw = vyaw * 180/math.pi
    print ("vyaw : {0:.1f}".format(vyaw))
    """
    
    """
    # To see the speed
    oldposx = posx
    posx = pos[0]
    oldposy = posy
    posy = pos[1]
    
    vitesse = (sqrt(((posx - oldposx)/time.time())**2 + ((posy - oldposy)/time.time())**2 ))**2
    #vitessex = ( ( (posx - oldposx) + (posy - oldposy) ) * 10**13 )/ time.time()
    print("speed : {0:.1f}".format(vitesse*10**27))
    """

    """End of work code test"""


    """ DEBUG """
    
    pos, rpy = sim.getRobotPose()
    # print("position = {}, angle = {}".format(pos, rpy))
    # pos : x, y, z
    # rpy : roll, pitch, yaw

    # Camera fixed on the robot
    robot_pose = (sim.getRobotPose()) # (tuple(3), tuple(3)) -- (x,y,z), (roll, pitch, yaw)
    yaw = robot_pose[1][2]
    sim.lookAt(robot_pose[0])

    # Option for setting the hexapod in the air
    airpause = p.readUserDebugParameter(controlp["airpause"])
    if airpause == 1 :
        rot_quat = to_pybullet_quaternion(rpy[0], rpy[1], rpy[2])
        sim.setRobotPose([pos[0], pos[1], 0.5], [0,0,rot_quat[2],1]) 


    # Tab with : [ position, speed, forces & torques ] 
    state = sim.setJoints(targets)
    

    # Add debug lines, recovering states of motors
    A = p.readUserDebugParameter(controlp["debuglines"])
    if A == 1 :
        list_of_pos = []
        
        """Mode slowmotion??"""
        # p.resetBasePositionAndOrientation(p.loadURDF("target2/robot.urdf"), [0,0,0], [0,0,0,0])

        for leg_id in range (1, 7):
            position = kinematics.computeDK(
                state[params.legs[leg_id][0]][0], 
                state[params.legs[leg_id][1]][0], 
                state[params.legs[leg_id][2]][0])
            
            position = kinematics.rotaton_2D(position[0], position[1], position[2], -LEG_ANGLES[leg_id - 1] + yaw)

            leg_center_position = kinematics.rotaton_2D(
                LEG_CENTER_POS[leg_id - 1][0],
                LEG_CENTER_POS[leg_id - 1][1],
                LEG_CENTER_POS[leg_id - 1][2],
                yaw)

            position[0] += leg_center_position[0] + robot_pose[0][0]
            position[1] += leg_center_position[1] + robot_pose[0][1]
            position[2] += leg_center_position[2] + robot_pose[0][2]
            list_of_pos.append(position)

            sim.addDebugPosition(position, duration=2)            


        """ Calcul skating """
        # Calcul frequency
        old_time = new_time
        new_time = time.time()
        dt = new_time - old_time
        freq =  1 / (new_time - old_time)

        seuil_patinage_mm = p.readUserDebugParameter(controlp["seuil_patinage_mm"])

        if ((time.time() - patinage_old_t) >= patinage_delta_t) :
            # Calcul distances
            old_distances_pattes = distances_pattes
            distances_pattes = calcul_dist(list_of_pos)

            for i in range (0,6):
                diff_dist = abs((old_distances_pattes[i] - distances_pattes[i])*1000)
                if diff_dist >= seuil_patinage_mm :
                    print("SKATING :\n Distance gap between {0} and {1} : {2:.1f} mm".format(i+1, ((i+2)%6)+1, diff_dist) )
            print("period = {0} s et freq = {1:.1f} Hz".format(dt, freq))
            
            patinage_old_t = time.time()
        

    sim.tick()