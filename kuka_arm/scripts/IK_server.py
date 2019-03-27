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
from mpmath import radians
from sympy import *



# Defining DH transformation matrix here
def TF_Matrix(alp, a, d, q):
    TF = Matrix([
                [            cos(q),            -sin(q),            0,            a ],
                [   sin(q)*cos(alp),    cos(q)*cos(alp),    -sin(alp),  -sin(alp)*d ],
                [   sin(q)*sin(alp),    cos(q)*sin(alp),     cos(alp),   cos(alp)*d ],
                [                 0,                  0,            0,            1 ]
                    ])
    return TF


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
    	#DH param symbols
   	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    	alp0, alp1, alp2, alp3, alp4, alp5, alp6 = symbols('alp0:7')

    	#joint angles
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

	#
	# Create Modified DH parameters

    	DH_tab = 	{ 
                alp0:    0.,  a0:   0.0,  d1:  0.75, q1:          q1,
                alp1:-pi/2.,  a1:   0.35, d2:     0, q2:-pi/2. +  q2,
                alp2:    0.,  a2:   1.25, d3:     0, q3:          q3,
                alp3:-pi/2.,  a3: -0.054, d4:   1.5, q4:          q4,
                alp4: pi/2.,  a4:      0, d5:     0, q5:          q5,
                alp5:-pi/2.,  a5:      0, d6:     0, q6:          q6,
                alp6:    0.,  a6:      0, d7: 0.303, q7:          q7,

			}
	# Define Modified DH Transformation matrix
	### Defined in the beginning in the function TFMatrix
	# Create individual transformation matrices
    	T0_1 = TF_Matrix(alp0, a0, d1, q1).subs(DH_tab)
    	T1_2 = TF_Matrix(alp1, a1, d2, q2).subs(DH_tab)
    	T2_3 = TF_Matrix(alp2, a2, d3, q3).subs(DH_tab)
    	T3_4 = TF_Matrix(alp3, a3, d4, q4).subs(DH_tab)
    	T4_5 = TF_Matrix(alp4, a4, d5, q5).subs(DH_tab)
    	T5_6 = TF_Matrix(alp5, a5, d6, q6).subs(DH_tab)
	T6_E = TF_Matrix(alp6, a6, d7, q7).subs(DH_tab)

	T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_E


        #Get Rotation Matrices from roll-pitch-yaw
        r, p, y = symbols('r p y')
    
        ROT_x = Matrix([[1,      0,       0],
        	        [0, cos(r), -sin(r)],
        	        [0, sin(r),  cos(r)]])
        
        ROT_y = Matrix([[cos(p), 0, sin(p)],
    	                [     0, 1,      0],
    	                [-sin(p), 0, cos(p)]])
    
        ROT_z = Matrix([[cos(y), -sin(y),  0],
    	                [sin(y),  cos(y),  0],
    	                [     0,       0,  1]])



	ROT_EE = ROT_z * ROT_y * ROT_x
	Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
        ROT_EE = ROT_EE * Rot_Error


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
            ROT_EE = ROT_EE.subs({'r':roll, 'p':pitch, 'y':yaw})

            EE = Matrix([[px],
                         [py],
                         [pz]])

            # Calculate Wrist center: Using following equation
            #                              _   _       _    _             _   _ 
            #   0         0            0  |  0  |     |  Px  |        0  |  0  |
            #    r      =  r      -  d. R |  0  |  =  |  Py  |   -  d. R |  0  |
            #     WC/0      EE/0       6  |_ 1 _|     |_ Pz _|        6  |_ 1 _|
            #

            WC = EE - (0.303) * ROT_EE[:,2]

	    # Compensate for rotation discrepancy between DH parameters and Gazebo
            #SSS triangle for theta2 and theta3
            side_a = 1.501
            side_b = sqrt(pow((sqrt(WC[0]**2 + WC[1]**2 ) -0.35), 2) + pow((WC[2] - 0.75),2))
            side_c = 1.25
            
            angle_a = acos((side_b**2 + side_c**2 - side_a**2) / (2 * side_b * side_c))
            angle_b = acos((side_a**2 + side_c**2 - side_b**2) / (2 * side_a * side_c))
            angle_c = acos((side_a**2 + side_b**2 - side_c**2) / (2 * side_b * side_c))



	    # Calculate joint angles using Geometric IK method
            theta1 = atan2(WC[1],WC[0])
            theta2 = pi/2. - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0]**2 + WC[1]**2) -0.35)
            theta3 = pi/2. - (angle_b + 0.036) # 0.036 because of sag in link4 i.e. -0.054m

	    R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3]* T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs = {q1: theta1, q2: theta2, q3: theta3})


            # As per rotation matrix property, inverse of rotation matrix is equal to its transpose.
            R3_6 = R0_3.T * ROT_EE
            #print(R3_6)

            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            #print joint_trajectory_point.positions
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
