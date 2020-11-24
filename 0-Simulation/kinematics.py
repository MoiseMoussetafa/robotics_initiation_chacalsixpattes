import math
from constants import *
from scipy.optimize import minimize
import numpy as np

# Given the sizes (a, b, c) of the 3 sides of a triangle, returns the angle between a and b using the alKashi theorem.
def alKashi(a, b, c, sign=-1):
    if a * b == 0:
        print("WARNING a or b is null in AlKashi")
        return 0
    # Note : to get the other altenative, simply change the sign of the return :
    return sign * math.acos(min(1, max(-1, (a ** 2 + b ** 2 - c ** 2) / (2 * a * b))))


# Computes the direct kinematics of a leg in the leg's frame
# Given the angles (theta1, theta2, theta3) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the destination point (x, y, z)
def computeDK(
    theta1,
    theta2,
    theta3,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    use_rads=USE_RADS_INPUT,
    use_mm=USE_MM_OUTPUT,
):
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit
    # print(
    #     "corrected angles={}, {}, {}".format(
    #         theta1 * (1.0 / angle_unit),
    #         theta2 * (1.0 / angle_unit),
    #         theta3 * (1.0 / angle_unit),
    #     )
    # )

    planContribution = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)

    x = math.cos(theta1) * planContribution * dist_unit
    y = math.sin(theta1) * planContribution * dist_unit
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3)) * dist_unit

    return [x, y, z]


def computeDKDetailed(
    theta1,
    theta2,
    theta3,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    use_rads=USE_RADS_INPUT,
    use_mm=USE_MM_OUTPUT,
):
    theta1_verif = theta1
    theta2_verif = theta2
    theta3_verif = theta3
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit

    # print(
    #     "corrected angles={}, {}, {}".format(
    #         theta1 * (1.0 / angle_unit),
    #         theta2 * (1.0 / angle_unit),
    #         theta3 * (1.0 / angle_unit),
    #     )
    # )

    planContribution = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)

    x = math.cos(theta1) * planContribution
    y = math.sin(theta1) * planContribution
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3))

    p0 = [0, 0, 0]
    p1 = [l1 * math.cos(theta1) * dist_unit, l1 * math.sin(theta1) * dist_unit, 0]
    p2 = [
        (l1 + l2 * math.cos(theta2)) * math.cos(theta1) * dist_unit,
        (l1 + l2 * math.cos(theta2)) * math.sin(theta1) * dist_unit,
        -l2 * math.sin(theta2) * dist_unit,
    ]
    p3 = [x * dist_unit, y * dist_unit, z * dist_unit]
    p3_verif = computeDK(
        theta1_verif, theta2_verif, theta3_verif, l1, l2, l3, use_rads, use_mm
    )
    if (p3[0] != p3_verif[0]) or (p3[1] != p3_verif[1]) or (p3[2] != p3_verif[2]):
        print(
            "ERROR: the DK function is broken!!! p3 = {}, p3_verif = {}".format(
                p3, p3_verif
            )
        )

    return [p0, p1, p2, p3]


# Computes the inverse kinematics of a leg in the leg's frame
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the angles to apply to the 3 axes
def computeIK(
    x,
    y,
    z,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    verbose=False,
    use_rads=USE_RADS_OUTPUT,
    sign=-1,
    use_mm=USE_MM_INPUT,
):
    dist_unit = 1
    if use_mm:
        dist_unit = 0.001
    x = x * dist_unit
    y = y * dist_unit
    z = z * dist_unit

    # theta1 is simply the angle of the leg in the X/Y plane. We have the first angle we wanted.
    if y == 0 and x == 0:
        # Taking care of this singularity (leg right on top of the first rotational axis)
        theta1 = 0
    else:
        theta1 = math.atan2(y, x)

    # Distance between the second motor and the projection of the end of the leg on the X/Y plane
    xp = math.sqrt(x * x + y * y) - l1
    # if xp < 0:
    #     print("Destination point too close")
    #     xp = 0

    # Distance between the second motor arm and the end of the leg
    d = math.sqrt(math.pow(xp, 2) + math.pow(z, 2))
    # if d > l2 + l3:
    #     print("Destination point too far away")
    #     d = l2 + l3

    # Knowing l2, l3 and d, theta1 and theta2 can be computed using the Al Kashi law
    # There are 2 solutions for most of the points, forcing a convention here
    theta2 = alKashi(l2, d, l3, sign=sign) - Z_DIRECTION * math.atan2(z, xp)
    theta3 = math.pi + alKashi(l2, l3, d, sign=sign)

    if use_rads:

        result = [
            angleRestrict(THETA1_MOTOR_SIGN * theta1, use_rads=use_rads),
            angleRestrict(
                THETA2_MOTOR_SIGN * (theta2 + theta2Correction), use_rads=use_rads
            ),
            angleRestrict(
                THETA3_MOTOR_SIGN * (theta3 + theta3Correction), use_rads=use_rads
            ),
        ]

    else:
        result = [
            angleRestrict(THETA1_MOTOR_SIGN * math.degrees(theta1), use_rads=use_rads),
            angleRestrict(
                THETA2_MOTOR_SIGN * (math.degrees(theta2) + theta2Correction),
                use_rads=use_rads,
            ),
            angleRestrict(
                THETA3_MOTOR_SIGN * (math.degrees(theta3) + theta3Correction),
                use_rads=use_rads,
            ),
        ]
    if verbose:
        print(
            "Asked IK for x={}, y={}, z={}\n, --> theta1={}, theta2={}, theta3={}".format(
                x, y, z, result[0], result[1], result[2],
            )
        )

    return result


# Computes the inverse kinematics of a leg in a frame colinear to the robot's frame (x points in front of the robot, y points to its left, z towards the sky)
# but whose (0,0) point is leg dependent, ie will match the leg's initial position.
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the angles to apply to the 3 axes
def computeIKOriented(x, y, z, legID, params, extra_theta=0, verbose=False):
    ROTATON = rotaton_2D(
        x,
        y,
        z,
        LEG_ANGLES[legID-1] + extra_theta)

    alphas = computeIK(
        ROTATON[0] + params.initLeg[legID-1][0], 
        ROTATON[1] + params.initLeg[legID-1][1], 
        ROTATON[2] + params.z)
    return alphas


# Computes the inverse kinematics of a leg in a frame colinear to the leg's frame (x points in front of the leg, y points to its left, z towards the sky)
# but whose (0,0) point matches the leg's initial position.
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the angles to apply to the 3 axes
def computeIKNotOriented(x, y, z, legID, params, extra_theta=0, verbose=False):
    return computeIK(
        x + params.initLeg[legID - 1][0],
        y + params.initLeg[legID - 1][1],
        z + params.z
    )


def rotaton_2D(x, y, z, theta):
    # Applying a rotation around the Z axis
    new_x = math.cos(theta) * x - math.sin(theta) * y
    new_y = math.sin(theta) * x + math.cos(theta) * y
    return [new_x, new_y, z]


def angleRestrict(angle, use_rads=False):
    if use_rads:
        return modulopi(angle)
    else:
        return modulo180(angle)


# Takes an angle that's between 0 and 360 and returns an angle that is between -180 and 180
def modulo180(angle):
    if -180 < angle < 180:
        return angle

    angle = angle % 360
    if angle > 180:
        return -360 + angle

    return angle


def modulopi(angle):
    if -math.pi < angle < math.pi:
        return angle

    angle = angle % (math.pi * 2)
    if angle > math.pi:
        return -math.pi * 2 + angle

    return angle


def trianglePoints(x, z, h, w):
    """
    Takes the geometric parameters of the triangle and returns the position of the 3 points of the triagles. Format : [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]]
    """
    P1 = [x, 0, z+h]
    P2 = [x, w/2, z]
    P3 = [x, -w/2, z]
    return [P1, P2, P3]

def trianglePoints2(x, z, h, w):
    """
    Takes the geometric parameters of the triangle and returns the position of the 3 points of the triagles. Format : [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]]
    Specific used for walk
    """
    P1 = [0, x, z+h]
    P2 = [w/2, x, z]
    P3 = [-w/2, x, z]
    return [P1, P2, P3]


def segdist(P1, P2):
    """
    Distance between two points, used for triangle
    """
    seg = math.sqrt(math.pow(P2[0]-P1[0],2) + math.pow(P2[1]-P1[1],2) + math.pow(P2[2]-P1[2],2))
    return seg


def triangle_w(x, z, h, w, t, period, leg_id, params, extra_theta):
    """
    Function triangle specially used for walk
    """
    alphas = [0,0,0]
    points = trianglePoints2(x,z,h,w)
    d1 = segdist(points[0],points[1])
    d2 = segdist(points[1],points[2])
    d3 = segdist(points[2],points[0])
    period1 = (d1/(d1+d2+d3)) * period
    period2 = (d2/(d1+d2+d3)) * period
    period3 = (d3/(d1+d2+d3)) * period
    t = math.fmod(t,period)

    if (t <= period1):
        alphas = segment_oneway_w(points[0][0], points[0][1], points[0][2], points[1][0], points[1][1], points[1][2], t, period1, leg_id, params, extra_theta)

    elif (t <= (period1+period2)):
        alphas = segment_oneway_w(points[1][0], points[1][1], points[1][2], points[2][0], points[2][1], points[2][2], t - period1, period2, leg_id, params, extra_theta)

    else :
        alphas = segment_oneway_w(points[2][0], points[2][1], points[2][2], points[0][0], points[0][1], points[0][2], t - period1 - period2, period3, leg_id, params, extra_theta)
    
    return alphas


def triangle_for_rotation(x, z, h, w, t, period=5):
    """
    Function triangle specially used for rotation
    """
    points = trianglePoints(x,z,h,w)
    d1 = segdist(points[0],points[1])
    d2 = segdist(points[1],points[2])
    d3 = segdist(points[2],points[0])
    period1 = (d1/(d1+d2+d3)) * period
    period2 = (d2/(d1+d2+d3)) * period
    period3 = (d3/(d1+d2+d3)) * period
    t = math.fmod(t,period)

    if (t <= period1):
        alphas = segment_oneway(points[0][0], points[0][1], points[0][2], points[1][0], points[1][1], points[1][2], t, period1)

    elif (t <= (period1+period2)):
        alphas = segment_oneway(points[1][0], points[1][1], points[1][2], points[2][0], points[2][1], points[2][2], t - period1, period2)

    else :
        alphas = segment_oneway(points[2][0], points[2][1], points[2][2], points[0][0], points[0][1], points[0][2], t - period1 - period2, period3)
    
    return alphas


def triangle(x, z, h, w, t, period=5):
    """
    Takes the geometric parameters of the triangle and the current time, gives the joint angles to draw the triangle with the tip of th leg. Format : [theta1, theta2, theta3]
    """
    points = trianglePoints(x,z,h,w)
    d1 = segdist(points[0],points[1])
    d2 = segdist(points[1],points[2])
    d3 = segdist(points[2],points[0])
    period1 = (d1/(d1+d2+d3)) * period
    period2 = (d2/(d1+d2+d3)) * period
    period3 = (d3/(d1+d2+d3)) * period
    t = math.fmod(t,period)

    if (t <= period1):
        alphas = segment_oneway(points[0][0], points[0][1], points[0][2], points[1][0], points[1][1], points[1][2], t, period1)

    elif (t <= (period1+period2)):
        alphas = segment_oneway(points[1][0], points[1][1], points[1][2], points[2][0], points[2][1], points[2][2], t - period1, period2)

    else :
        alphas = segment_oneway(points[2][0], points[2][1], points[2][2], points[0][0], points[0][1], points[0][2], t - period1 - period2, period3)
    
    return alphas


def circlePoints(x, z, r, N=16):
    """
    Takes the geometric parameters of the cercle and returns N points approximating the circle. Format : [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], etc]
    """
    None

def circle(x, z, r, t, duration):
    """
    Takes the geometric parameters of the circle and the current time, gives the joint angles to draw the circle with the tip of th leg. Format : [theta1, theta2, theta3]
    """
    y_circle = r * math.cos(2 * math.pi * (1 / duration) * t)
    z_circle =+ r * math.sin(2 * math.pi * (1 / duration) * t)
    alphas = computeIK(x, y_circle, z_circle + z)
    return alphas

def segment(segment_x1, segment_y1, segment_z1,
            segment_x2, segment_y2, segment_z2, t, duration):
    """
    Segment with cosinus
    """
    nt = ((math.cos(2 * math.pi * (1 / duration) * t) + 1) * duration) / 2

    x = (nt / duration) * (segment_x2 - segment_x1) + segment_x1
    y = (nt / duration) * (segment_y2 - segment_y1) + segment_y1
    z = (nt / duration) * (segment_z2 - segment_z1) + segment_z1

    theta1, theta2, theta3 = computeIK(x, y, z)

    return (theta1, theta2, theta3)


def segment_modulo(segment_x1, segment_y1, segment_z1, segment_x2, segment_y2, segment_z2, t, duration):
    """
    Segment with modulo
    """
    nt = math.fmod(t,duration)
    if nt > (duration/2.0):
        nt = (duration/2) - (nt-duration/2)
    nt = 2*nt
        
    x = (nt/duration) * (segment_x2-segment_x1) + segment_x1
    y = (nt/duration) * (segment_y2-segment_y1) + segment_y1
    z = (nt/duration) * (segment_z2-segment_z1) + segment_z1

    theta1, theta2, theta3 = computeIK(x,y,z)

    return (theta1, theta2, theta3)


def segment_oneway(segment_x1, segment_y1, segment_z1, segment_x2, segment_y2, segment_z2, t, duration):
    """
    Function segment used for triangle in the rotation, a segment in only one direction
    """
    nt = math.fmod(t,duration)
        
    x = (nt/duration) * (segment_x2-segment_x1) + segment_x1
    y = (nt/duration) * (segment_y2-segment_y1) + segment_y1
    z = (nt/duration) * (segment_z2-segment_z1) + segment_z1

    theta1, theta2, theta3 = computeIK(x,y,z)

    return (theta1, theta2, theta3)


def segment_oneway_w(segment_x1, segment_y1, segment_z1, segment_x2, segment_y2, segment_z2, t, duration, leg_id, params, extra_theta):
    """
    Function segment used for triangle in the walk, a segment in only one direction
    """
    nt = math.fmod(t,duration)
        
    x = (nt/duration) * (segment_x2-segment_x1) + segment_x1
    y = (nt/duration) * (segment_y2-segment_y1) + segment_y1
    z = (nt/duration) * (segment_z2-segment_z1) + segment_z1

    theta1, theta2, theta3 = computeIKOriented(x,y,z, leg_id, params, extra_theta)

    return (theta1, theta2, theta3)

def demicircle(x, z, r, t, duration, legID, params, extra_theta):
    """
    Demi-circle, used for arms, arc in the air, segment on the floor
    """
    y_circle = r * math.sin(2 * math.pi * (1 / duration) * t)
    z_circle = r * math.cos(2 * math.pi * (1 / duration) * t)
    p1 = [x, y_circle + r ,z]
    p2 = [x, y_circle - r, z]
    d1 = segdist(p1, p2)
    d2 = math.pi * r
    periode = (d1/(d1+d2)) * duration
    #periode = duration/2

    if t < periode :
        alphas = segment_oneway_w(p1[1], p1[1], p1[2], p2[1], p2[1], p2[2], t, periode, legID, params, extra_theta)
    else :
        alphas = computeIKOriented(x, y_circle, z_circle + z, legID, params, extra_theta , verbose=False)
    return alphas

def demicirclefloor(x, z, r, t, duration, legID, params, extra_theta):
    """
    Demi-circle, used for arms, arc in the air, segment on the floor
    """
    y_circle = r * math.sin(2 * math.pi * (1 / duration) * t)
    x_circle = r * math.cos(2 * math.pi * (1 / duration) * t)
    p1 = [x, y_circle + r ,z]
    p2 = [x, y_circle - r, z]
    d1 = segdist(p1, p2)
    d2 = math.pi * r
    periode = (d1/(d1+d2)) * duration

    if t < periode :
        alphas = segment_oneway_w(p1[1], p1[1], p1[2], p2[1], p2[1], p2[2], t, periode, legID, params, extra_theta)
    else :
        alphas = computeIKOriented(x_circle + x, y_circle, z, legID, params, extra_theta , verbose=False)
    return alphas

def demicirclefloorold(x, z, r, t, duration, legID, params):
    """
    Demi-circle, used for arms, arc and segment on the floor
    """
    y_circle = r * math.sin(2 * math.pi * (1 / duration) * t)
    z_circle =+ r * math.cos(2 * math.pi * (1 / duration) * t)
    p1 = [x, y_circle + r ,z]
    p2 = [x, y_circle - r, z]
    d1 = segdist(p1,p2)
    d2 = math.pi * r
    periode = (d1/(d1+d2)) * duration

    if t < periode :
        alphas = demicircle(x, z, r, t, duration, legID, params, extra_theta = 0)
    else :
        alphas = computeIKNotOriented(x, y_circle, z_circle + z, legID, params, verbose=False)
    return alphas


def main():
    print(
        "0, -90, -90 --> ", computeDK(0, -90, -90, l1=constL1, l2=constL2, l3=constL3)
    )
    print("0, 0, 0 --> ", computeDK(0, 0, 0, l1=constL1, l2=constL2, l3=constL3))
    print("90, 0, 0 --> ", computeDK(90, 0, 0, l1=constL1, l2=constL2, l3=constL3))
    print(
        "180, -30.501, -67.819 --> ",
        computeDK(180, -30.501, -67.819, l1=constL1, l2=constL2, l3=constL3),
    )
    print(
        "0, -30.645, 38.501 --> ",
        computeDK(0, -30.645, 38.501, l1=constL1, l2=constL2, l3=constL3),
    )


if __name__ == "__main__":
    main()
