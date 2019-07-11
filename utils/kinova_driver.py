#!/usr/bin/env python
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import math
from kortex_driver.srv import *
from kortex_driver.msg import *
import time


def MoveFingers(x):
    rospy.wait_for_service('SendGripperCommand')

    try:
        function_SendGripperCommand = rospy.ServiceProxy('SendGripperCommand', SendGripperCommand)

        gripperCommand = GripperCommand()
        gripper = Gripper()
        finger = []
        for i in range(0, 2):
            f = Finger()
            f.finger_identifier = i
            f.value = x
            finger.append(f)
        gripper.finger = finger
        gripperCommand.gripper = gripper
        gripperCommand.mode = 3
        gripperCommand.duration = 0

        function_SendGripperCommand(gripperCommand)

    except rospy.ServiceException as e:
        print "Service call failed: %s"%e


def CloseFingers():
    MoveFingers(-1)


def OpenFingers():
    MoveFingers(1)


def PlayCartesian_client(disp_x, disp_y, disp_z, ang_x, ang_y, ang_z):
    
    rospy.wait_for_service('PlayCartesianTrajectory')
    rospy.wait_for_service('RefreshFeedback')
    
    try:
        function_PlayCartesianTrajectory = rospy.ServiceProxy('PlayCartesianTrajectory', PlayCartesianTrajectory)
        function_RefreshFeedback = rospy.ServiceProxy('RefreshFeedback', RefreshFeedback)
        
        current_feedback = function_RefreshFeedback()
        request = PlayCartesianTrajectoryRequest()

        current_x = current_feedback.output.base.tool_pose_x
        current_y = current_feedback.output.base.tool_pose_y
        current_z = current_feedback.output.base.tool_pose_z

        current_theta_x = current_feedback.output.base.tool_pose_theta_x
        current_theta_y = current_feedback.output.base.tool_pose_theta_y
        current_theta_z = current_feedback.output.base.tool_pose_theta_z

        # Creating our next target (a Cartesian pose)
        request.input.target_pose.x = current_x + disp_x
        request.input.target_pose.y = current_y + disp_y
        request.input.target_pose.z = current_z + disp_z

        request.input.target_pose.theta_x = current_theta_x + ang_x
        request.input.target_pose.theta_y = current_theta_y + ang_y
        request.input.target_pose.theta_z = current_theta_z + ang_z

        poseSpeed = CartesianSpeed()
        poseSpeed.translation = 0.1
        poseSpeed.orientation = 15

        request.input.constraint.oneof_type.speed.append(poseSpeed)

        function_PlayCartesianTrajectory(request)
        time.sleep((abs(disp_x) + abs(disp_y) + abs(disp_z)) * 15)
        
    except rospy.ServiceException as e:
        print "Service call failed: %s"%e


def PlayJoint_client(new_pos):
    rospy.wait_for_service('PlayJointTrajectory')
    rospy.wait_for_service('RefreshFeedback')

    try:
        function_PlayJointTrajectory = rospy.ServiceProxy('PlayJointTrajectory', PlayJointTrajectory)
        function_RefreshFeedback = rospy.ServiceProxy('RefreshFeedback', RefreshFeedback)

        current_feedback = function_RefreshFeedback()
        request = PlayJointTrajectoryRequest()
        constraintJointAngles = ConstrainedJointAngles()

        angles = []

        for i, actuator in enumerate(current_feedback.output.actuators):
            jointAngle = JointAngle()
            jointAngle.joint_identifier = i
            jointAngle.value = new_pos[i]
            angles.append(jointAngle)

        constraintJointAngles.joint_angles.joint_angles = angles
        # Creating our next target (a Cartesian pose)
        request.input = constraintJointAngles

        function_PlayJointTrajectory(request)

    except rospy.ServiceException as e:
        print "Service call failed: %s" % e


# Create protection zone

# Get measured cartesian pose
def GetMeasuredCartesianPose_client():
    rospy.wait_for_service('GetMeasuredCartesianPose')

    try:
        function_GetMeasuredCartesian = rospy.ServiceProxy('GetMeasuredCartesianPose', GetMeasuredCartesianPose)
        pose = function_GetMeasuredCartesian()
        return pose.output
    except rospy.ServiceException as e:
        print "Service call failed: %s" % e

def MoveToCartesianPosition(pose):
    rospy.wait_for_service('PlayCartesianTrajectory')
    rospy.wait_for_service('RefreshFeedback')

    try:
        function_PlayCartesianTrajectory = rospy.ServiceProxy('PlayCartesianTrajectory', PlayCartesianTrajectory)

        request = PlayCartesianTrajectoryRequest()
        # Creating our next target (a Cartesian pose)
        request.input.target_pose = pose
        poseSpeed = CartesianSpeed()
        poseSpeed.translation = 0.1
        poseSpeed.orientation = 15

        request.input.constraint.oneof_type.speed.append(poseSpeed)

        function_PlayCartesianTrajectory(request)
        time.sleep(3)

    except rospy.ServiceException as e:
        print "Service call failed: %s" % e