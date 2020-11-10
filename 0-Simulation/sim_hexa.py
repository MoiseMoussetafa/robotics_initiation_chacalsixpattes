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

# from squaternion import Quaternion
from scipy.spatial.transform import Rotation

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


# m_friction
parser = argparse.ArgumentParser()
parser.add_argument("--mode", "-m", type=str, default="direct", help="test")
args = parser.parse_args()
controls = {}
robotPath = "phantomx_description/urdf/phantomx.urdf"
sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
# sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)
pos, rpy = sim.getRobotPose()
sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

yaw = 0

leg_center_pos = [0.1248, -0.06164, 0.001116 + 0.5]
leg_angle = -math.pi / 4
params = Parameters ()

bx = 0.07
bz = 0.25


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
    # Two last values : min and max
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
    controls["x"] = p.addUserDebugParameter("x", -0.1, 0.1, 0)                              # x
    controls["height_hexapode"] = p.addUserDebugParameter("height_hexapode", -0.1, 0.1, 0)  # z
    controls["height_arms"] = p.addUserDebugParameter("height_arms", 0, 0.2, 0.005)         # h
    controls["amplitude"] = p.addUserDebugParameter("amplitude", 0.1, 0.5, 0.1)             # w
    controls["speed"] = p.addUserDebugParameter("speed", 0.1, 10, 1)                        # period
    controls["direction"] = p.addUserDebugParameter("direction", -math.pi, math.pi, 0)      # extra-theta

elif args.mode == "ultrawalk":
    controls["x"] = p.addUserDebugParameter("x", -0.1, 0.1, 0)                              # x
    controls["height_hexapode"] = p.addUserDebugParameter("height_hexapode", -0.1, 0.1, 0)  # z
    controls["height_arms"] = p.addUserDebugParameter("height_arms", 0, 0.2, 0.005)         # h
    controls["amplitude"] = p.addUserDebugParameter("amplitude", 0.1, 0.5, 0.1)             # w
    controls["speed"] = p.addUserDebugParameter("speed", 0.1, 10, 1)                        # period
    controls["direction"] = p.addUserDebugParameter("direction", -math.pi, math.pi, 0)      # extra-theta
    controls["target_w"] = p.addUserDebugParameter("target_w", -0.3, 0.3, 0)    


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

initRobot(params)
time.sleep(0)

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
            alphas = kinematics.computeIKOriented(
                    x,
                    y,
                    z,
                    leg_id, params)
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
                alphas = kinematics.triangle_for_rotation(x, z, h, w, 
                    sim.t, period)

                set_leg_angles(alphas, leg_id, targets, params)

            elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6):
                alphas = kinematics.triangle_for_rotation(x, z, h, w, 
                    sim.t + 0.5 * period, period)

                set_leg_angles(alphas, leg_id, targets, params)
            
        state = sim.setJoints(targets)
        #sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
            
    elif args.mode == "walk":
        x = p.readUserDebugParameter(controls["x"])
        z = p.readUserDebugParameter(controls["height_hexapode"])
        h = p.readUserDebugParameter(controls["height_arms"])
        w = p.readUserDebugParameter(controls["amplitude"])
        period = p.readUserDebugParameter(controls["speed"])
        direction = p.readUserDebugParameter(controls["direction"])
        
        for leg_id in range (1,7):
            if (leg_id == 1) or (leg_id == 3) or (leg_id == 5) :
                alphas = kinematics.triangle_w(x, z, h, w, 
                    sim.t, period, leg_id, params, direction)

                set_leg_angles(alphas, leg_id, targets, params)
                
            elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6):
                alphas = kinematics.triangle_w(x, z, h, w, 
                    sim.t + 0.5 * period, period, leg_id, params, direction)

                set_leg_angles(alphas, leg_id, targets, params)

            state = sim.setJoints(targets)
            robot_pose = (sim.getRobotPose()) # (tuple(3), tuple(3)) -- (x,y,z), (roll, pitch, yaw)
            yaw = robot_pose[1][2]
            sim.lookAt(robot_pose[0])

    elif args.mode == "ultrawalk":
        x = p.readUserDebugParameter(controls["x"])
        z = p.readUserDebugParameter(controls["height_hexapode"])
        h = p.readUserDebugParameter(controls["height_arms"])
        w = p.readUserDebugParameter(controls["amplitude"])
        period = p.readUserDebugParameter(controls["speed"])
        direction = p.readUserDebugParameter(controls["direction"])

        xr = 0.180  
        zr = -0.15 
        hr = 0.001 
        wr = p.readUserDebugParameter(controls["target_w"])
        periodr = period 

        for leg_id in range (1,7):
            if (leg_id == 1) or (leg_id == 3) or (leg_id == 5) :
                alphas = kinematics.triangle_w(x, z, h, w, 
                    sim.t, period, leg_id, params, direction)

                set_leg_angles(alphas, leg_id, targets, params)
                alphas1 = kinematics.triangle_for_rotation(xr, zr, hr, wr, 
                    sim.t, periodr)

                A1 = alphas[0] + alphas1[0]
                A2 = alphas[1] + alphas1[1]
                A3 = alphas[2] + alphas1[2]
                ALPHA = [A1, A2, A3]
                set_leg_angles(ALPHA, leg_id, targets, params)

            elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6):
                alphas = kinematics.triangle_w(x, z, h, w, 
                    sim.t + 0.5 * period, period, leg_id, params, direction)

                alphas1 = kinematics.triangle_for_rotation(xr, zr, hr, wr, 
                    sim.t + 0.5 * periodr, periodr)

                A1 = alphas[0] + alphas1[0]
                A2 = alphas[1] + alphas1[1]
                A3 = alphas[2] + alphas1[2] 
                ALPHA = [A1, A2, A3]
                set_leg_angles(ALPHA, leg_id, targets, params)

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
    
    pos, rpy = sim.getRobotPose()
    #print("position = {}, angle = {}".format(pos, rpy))
    oldyaw = yaw
    yaw = rpy[2]
    vitesserotationnelle = (yaw - oldyaw) # A diviser par le temps la période de boucle
    print ("Vitesse rotationnelle : {}".format(vitesserotationnelle))


    robot_pose = (sim.getRobotPose()) # (tuple(3), tuple(3)) -- (x,y,z), (roll, pitch, yaw)
    yaw = robot_pose[1][2]
    sim.lookAt(robot_pose[0])
    sim.tick()
