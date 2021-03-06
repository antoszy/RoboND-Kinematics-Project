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


def handle_calculate_IK(req):
    '''A function that calculates angles of kuka arm joints required to reach poses defined as end effector positon and orientation in quaternions '''

    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        y, p, r = symbols('y p r')
        q1, q2, q3, q4, q5, q6 = symbols('q1:7')

    	#
    	#
    	# Create Modified DH parameters
    	#
    	#
    	# Define Modified DH Transformation matrix
    	#
    	#
    	# Create individual transformation matrices

        # yaw pitch roll rotation from euler angles enables to derive orientation matrix
    	Rrpy_gaz = Matrix([
                          [cos(p)*cos(y),     sin(p)*sin(r)*cos(y) - sin(y)*cos(r),    sin(p)*cos(r)*cos(y) + sin(r)*sin(y)],
                          [sin(y)*cos(p),     sin(p)*sin(r)*sin(y) + cos(r)*cos(y),    sin(p)*sin(y)*cos(r) - sin(r)*cos(y)],
                          [      -sin(p),                            sin(r)*cos(p),                           cos(p)*cos(r)]])


        # Rotation matrix for first 3 joints
        R0_3 = Matrix([
                      [sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1)],
                      [sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3),  cos(q1)],
                      [        cos(q2 + q3),        -sin(q2 + q3),        0]])

        # Matrix required to change from ROS to DH notation
        R_corr_inv = Matrix([
                            [0, 0,  1],
                            [0, -1, 0],
                            [1., 0, 0]])


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
            end_pose = Matrix([px,py,pz])

            ### IK code

            ## Calculate writst center position by moving 0.303 units along x axis of gazebo's gripper link
            Rrpy_gaz_eval = Rrpy_gaz.evalf(subs={y: yaw, p: pitch, r:roll})  # get rotation matrix from euler angles
            L_xyz = Rrpy_gaz_eval[0:3,0] # get firs collumn representing x direction of the girpper link in ROS
            Wrc = simplify(end_pose - L_xyz*0.303) #wrist center
            Wrc2 = Matrix([ sqrt(Wrc[0]**2+Wrc[1]**2)-0.35, 0,  Wrc[2]-0.75]) #wrist center with respect to joint 2

            ## Calculate joints 1-3 values
            theta1 = atan2(Wrc[1], Wrc[0])
            # Calculate leghts of sids of triangle representing joint 2 and 4
            lenA = sqrt(1.5**2 + 0.054**2)
            lenB = sqrt(Wrc2[0]**2 + Wrc2[2]**2)
            lenC = 1.25
            # calculate angles from cosine law
            alph = acos( (lenB**2 + lenC**2 - lenA**2) / (2*lenB*lenC))
            beta = acos( (lenA**2 + lenC**2 - lenB**2) / (2*lenA*lenC))
            epsi = atan(Wrc2[2]/Wrc2[0])
            delt = atan(0.054/1.5)
            # derive joint angles
            theta2 = pi/2-epsi-alph
            theta3 = pi/2 - beta - delt

            ## Calculate joionts 4-6
            R0_3_eval = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3}) # evaluate matrix representig transition from base link to link 3 using previously derived joint angles
            R3_6 = R0_3_eval.inverse_ADJ() * Rrpy_gaz_eval * R_corr_inv # compute matrxi representing transition from link 3 to gripper link in modified DH notation
            # comput joint angles
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])

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
