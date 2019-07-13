import rospy
from kortex_driver.srv import *
from kortex_driver.msg import *
import time

# This file contains functions for Kinova robot movement

#################################
#   Gripper Control Section     #
#################################

def move_fingers(x):
    """
    This method allows the gripper of the arm to be closed or open under position mode (refer to the Kinova user guide)
    :param x    -1 for completely closing, 1 for completely opening
    """
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
        print "Service call failed: %s" % e


def close_fingers():
    """
    Makes the fingers close
    """
    move_fingers(-1)


def open_fingers():
    """
    Makes the fingers open
    """
    move_fingers(1)


#####################################
#   Cartesian Movements Section     #
#####################################

def play_cartesian_client(disp_x, disp_y, disp_z, ang_x, ang_y, ang_z):
    """
    This function moves the end-effector to the Cartesian position specified
    by the displacements of the x, y, z dimensions as well as the angles
    :param disp_x: desired change in x, in meters
    :param disp_y: desired change in y, in meters
    :param disp_z: desired change in z, in meters
    :param ang_x: desired change in theta_x, in radians
    :param ang_y: desired change in theta_y, in radians
    :param ang_z: desired change in theta_z, in radians
    """
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
        print "Service call failed: %s" % e


def get_measured_catesian_pose():
    """
    Get current
    :return: the current Cartesian pose relative to the base
    """
    rospy.wait_for_service('GetMeasuredCartesianPose')

    try:
        function_GetMeasuredCartesian = rospy.ServiceProxy('GetMeasuredCartesianPose', GetMeasuredCartesianPose)
        pose = function_GetMeasuredCartesian()
        return pose.output
    except rospy.ServiceException as e:
        print "Service call failed: %s" % e


def move_to_cartesian_position(pose):
    """
    Move the end-effector to the specified Cartesian position
    :param pose: the desired Cartesian position, not in term of displacements
    """
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


#############################
#   Joint Movement Section  #
#############################
def play_joint_client(new_pos):
    """
    Move the robot into the specified position using joint angles
    :param new_pos: the target set of joint angles
    """
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
