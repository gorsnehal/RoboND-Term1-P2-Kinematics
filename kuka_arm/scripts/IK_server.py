#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

### Rotation
def rot_x(q):              
    
    R_x = Matrix([[ 	 1,      0, 	  0,  	0],
	          [      0, cos(q), -sin(q),  	0],
	          [ 	 0, sin(q),  cos(q),    0],
	          [      0,      0,       0,   1]])
    
    return R_x

def rot_y(q):              
    
    R_y = Matrix([[ cos(q),     0, sin(q),  0],
	          [      0,     1,      0,  0],
	          [-sin(q),     0, cos(q),  0],
	          [      0,     0,      0,  1]])
    
    return R_y

def rot_z(q):    

    R_z = Matrix([[ cos(q), -sin(q),    0,  0],
	          [ sin(q),  cos(q),    0,  0],
	          [      0,       0,    1,  0],
	          [      0,       0,    0,  1]])
    
    return R_z

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

	hfPi = pi / 2
	
	# Create Modified DH parameters
	s = {alpha0:    0,  a0:     0,  d1:     0.75,
	     alpha1:-hfPi,  a1:  0.35,  d2:        0,   q2: q2-hfPi,
	     alpha2:    0,  a2:  1.25,  d3:        0,
	     alpha3:-hfPi,  a3:-0.054,  d4:      1.5,
	     alpha4: hfPi,  a4:     0,  d5:        0,
	     alpha5:-hfPi,  a5:     0,  d6:        0,
	     alpha6:    0,  a6:     0,  d7:    0.303,   q7: 0}	

	
	### Homogenous Transform
	#Base link to Link1
	T0_1 = Matrix([[             cos(q1),             -sin(q1),            0,              a0],
		       [ sin(q1)*cos(alpha0),  cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
		       [ sin(q1)*sin(alpha0),  cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
		       [                   0,                    0,            0,               1]])
	T0_1 = T0_1.subs(s)

	#Link1 to Link2
	T1_2 = Matrix([[             cos(q2),             -sin(q2),            0,              a1],
		       [ sin(q2)*cos(alpha1),  cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
		       [ sin(q2)*sin(alpha1),  cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
		       [                   0,                    0,            0,               1]])
	T1_2 = T1_2.subs(s)

	#Link2 to Link3
	T2_3 = Matrix([[             cos(q3),             -sin(q3),            0,              a2],
		       [ sin(q3)*cos(alpha2),  cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
		       [ sin(q3)*sin(alpha2),  cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
		       [                   0,                    0,            0,               1]])
	T2_3 = T2_3.subs(s)

	#Link3 to Link4
	T3_4 = Matrix([[             cos(q4),             -sin(q4),            0,              a3],
		       [ sin(q4)*cos(alpha3),  cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
		       [ sin(q4)*sin(alpha3),  cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
		       [                   0,                    0,            0,               1]])
	T3_4 = T3_4.subs(s)

	#Link4 to Link5
	T4_5 = Matrix([[             cos(q5),             -sin(q5),            0,              a4],
		       [ sin(q5)*cos(alpha4),  cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
		       [ sin(q5)*sin(alpha4),  cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
		       [                   0,                    0,            0,               1]])
	T4_5 = T4_5.subs(s)

	#Link5 to Link6
	T5_6 = Matrix([[             cos(q6),             -sin(q6),            0,              a5],
		       [ sin(q6)*cos(alpha5),  cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
		       [ sin(q6)*sin(alpha5),  cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
		       [                   0,                    0,            0,               1]])
	T5_6 = T5_6.subs(s)

	#Link6 to Gripper
	T6_7 = Matrix([[             cos(q7),             -sin(q7),            0,              a6],
		       [ sin(q7)*cos(alpha6),  cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
		       [ sin(q7)*sin(alpha6),  cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
		       [                   0,                    0,            0,               1]])
	T6_7 = T6_7.subs(s)

	### Composition - Forward
	T0_2 = (T0_1 * T1_2)
	T0_3 = (T0_2 * T2_3)
	T0_4 = (T0_3 * T3_4)
	T0_5 = (T0_4 * T4_5)
	T0_6 = (T0_5 * T5_6)
	T0_7 = (T0_6 * T6_7)

	### Correction to match URDF referce frame
	### 1. Rotation w.r.t z by 180 & then w.r.t y by -90
	r1, r2 = symbols('r1:3')
	    
	RCorrection = (rot_z(r1) * rot_y(r2))
	RCorrection = RCorrection.evalf(subs={r1: pi, r2: -hfPi})

	# Forward Kinematics
	T_Final = (T0_7 * RCorrection)
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    eu1,eu2,eu3 = symbols('eu1:4')
	    Rrpy = rot_z(eu1) * rot_y(eu2) * rot_x(eu3) * RCorrection
	    Rrpy = Rrpy.evalf(subs={eu1: yaw, eu2: pitch, eu3: roll})
	    
	    # Calculate joint angles using Geometric IK method
	    wcx = px - (d7).subs(s) * Rrpy[0,2]
	    wcy = py - (d7).subs(s) * Rrpy[1,2]
	    wcz = pz - (d7).subs(s) * Rrpy[2,2]

	    theta1 = atan2(wcy, wcx)

	    #Calculate WC position w.r.t to joint 2
            r_2 = sqrt(wcx * wcx + wcy * wcy) - (a1).subs(s)
	    s_2 = wcz - (d1).subs(s)

            # Calculate sides of triangles
	    sideB = sqrt(r_2 * r_2 + s_2 * s_2)
    	    sideA = sqrt((d4).subs(s) * (d4).subs(s) + (a3).subs(s) * (a3).subs(s))
	    sideC = (a2).subs(s)

	    #Using cosine rule
	    angleA = acos((sideB * sideB + sideC * sideC - sideA * sideA) / (2 * sideB * sideC))
	    theta2 = (pi/2) - ((angleA) + atan2(s_2, r_2))

            # Default link 4 angle
	    angleDef = atan2(-(a3).subs(s), (d4).subs(s))
            # Using Cosine rule
	    angleB = acos((sideA * sideA + sideC * sideC - sideB * sideB) / (2 * sideA * sideC))
	    theta3 = (pi/2) - angleB - angleDef

	    #Calculate R0_3
	    R0_3 = T0_3[0:3,0:3]
	    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

	    R3_6 = R0_3.inv("LU") * Rrpy[0:3,0:3]

	    #Calculate theta 4,5 & 6
	    theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]),R3_6[1,2])
	    theta4 = atan2(R3_6[2,2],-R3_6[0,2])
	    theta6 = atan2(-R3_6[1,1], R3_6[1,0])

	    #Calculated EE position as per Forward Kinematics
	    T_Final = T_Final.evalf(subs={q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6, q7:0})
	    Calculated_ee = [T_Final[0,3],T_Final[1,3],T_Final[2,3]]
          
   	    # Find FK EE error
	    if not(sum(Calculated_ee)==3):
        	ee_x_e = abs(Calculated_ee[0]-px)
        	ee_y_e = abs(Calculated_ee[1]-py)
        	ee_z_e = abs(Calculated_ee[2]-pz)
        	ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        	print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        	print ("End effector error for y position is: %04.8f" % ee_y_e)
        	print ("End effector error for z position is: %04.8f" % ee_z_e)
        	print ("Overall end effector offset is: %04.8f units \n" % ee_offset)
	    
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
