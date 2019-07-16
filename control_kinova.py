import rospy, message_filters, os, time
from action_related.action_arr import Action_Arr
import numpy as np
from sensor_msgs.msg import Image
from utils.kinova_driver import *
from utils.kinova_protection_zone import *
from utils.create_zone_shape import *
from utils.kinova_misc import launch_windows
from cv_bridge import CvBridge, CvBridgeError
from kortex_driver.msg import *

arr = Action_Arr()

class Kinova_Controller:

    def __init__(self):
        self.inventory = []
        self.curr_img = []
        self.curr_depth = []
        self.action_colors = [[130, 20, 40], [120, 10, 20], [], [0, 60, 180], [15, 20, 80], [], [180, 30, 20], [],
                              [], [170, 130, 80], []]
        self.curr_action = -1
        self.pickup_flag = False
        self.dropoff_flag = False
        self.reset_pos_0 = False
        self.reset_pos_4 = False

        self.obj_height = 0

        self.first_pick = False

        # Pre-defined joint positions
        self.ini_pos = [277.163269043, 5.39976072311, 359.774963379, 135.213058472, 182.519790649, 319.587249756, 3.0936896801]
        self.drop_pos = [287.831085205, 74.947593689, 6.21474456787, 22.5967712402, 175.749679565, 276.920043945, 18.1025791168]
        self.ini_cart = Pose()

    def spot_max(self):
        max = 0
        for i in range(len(self.curr_depth)):
            for j in range(len(self.curr_depth[0]) / 2 - 20, len(self.curr_depth[0]) / 2 + 20):
                if (self.curr_depth[i][j] > max) and (self.curr_depth[i][j] < 400):
                    max = self.curr_depth[i][j]
        return max

    def spot_min(self):
        min = 1000
        for i in range(len(self.curr_depth)):
            for j in range(len(self.curr_depth[0]) / 2 - 20, len(self.curr_depth[0]) / 2 + 20):
                if (self.curr_depth[i][j] < min) and (self.curr_depth[i][j] > 0):
                    min = self.curr_depth[i][j]
        return min

    def approximate_equal(self, x1, x2):
        return abs(x1 - x2) <= 0.02

    def check_achieved(self, pose_ini):
        pose_curr = get_measured_catesian_pose()
        if (pose_ini is None) or (pose_curr is None):
            return True
        return (self.approximate_equal(pose_curr.x, pose_ini.x)
                and self.approximate_equal(pose_curr.y, pose_ini.y)
                and self.approximate_equal(pose_curr.z, pose_ini.z))

    def initialize(self):
        # Initialize to this position
        play_joint_client(self.ini_pos)
        time.sleep(5)
        open_fingers()
        if not self.check_achieved(self.ini_cart):
            # raw_input("not achieved!")
            play_joint_client(self.ini_pos)
            time.sleep(2)
        # act = int(raw_input("Please specify the action:"))
        # self.curr_action = act
        # # make_decision(act)

    def dropoff_object(self):
        rospy.loginfo("Dropping off")
        play_cartesian_client(0, 0, 0.2, 0, 0, 0)
        # time.sleep(2)
        # pose = get_measured_catesian_pose()
        # pose.x = 0.24
        # pose.y = 0.7
        # pose.z = 0.03
        # move_to_cartesian_position(pose)
        # if not self.check_achieved(pose):
        #     print "NOT ACHIEVED"
        #     move_to_cartesian_position(pose)
        play_joint_client(self.drop_pos)
        time.sleep(4.5)
        open_fingers()
        time.sleep(0.8)
        self.dropoff_flag = False
        # PlayCartesian_client(0, -0.5, 0, 0, 0, 0, 2.5)
        # time.sleep(1)
        self.initialize()

    def pickup_object(self):
        rospy.loginfo("Picking up")
        max_depth = self.spot_max()
        # Max depth - min depth should be height of object
        # Want to be at least half??
        if max_depth > 172 + self.obj_height:
            if max_depth - 172 - self.obj_height > 20:
                play_cartesian_client(0, 0, float(172 + self.obj_height - max_depth) / 1000, 0, 0, 0)
                # time.sleep(3.5)
            else:
                play_cartesian_client(0, 0, -0.01, 0, 0, 0)
                # time.sleep(0.2)
        else:
            if self.curr_action == 5:
                play_cartesian_client(0, 0, -0.02, 0, 0, 0)
                # time.sleep(0.5)
            close_fingers()
            self.pickup_flag = False
            self.dropoff_flag = True
            self.obj_height = 0
            time.sleep(0.8)

    def before_pick(self):
        self.pickup_flag = True
        self.first_pick = True
        if self.curr_action == 3:
            self.obj_height = 120
        elif self.curr_action == 0:
            self.obj_height = 90
        elif self.curr_action == 4:
            self.obj_height = -2
        elif self.curr_action == 6:
            self.obj_height = 2
        self.curr_action = -1

    def perform_action(self):
        # Move the robot arm to the position to the item
        color = self.action_colors[self.curr_action]
        if color == []:
            return
        found = False
        area = 0
        if self.curr_action == 0 and not self.reset_pos_0:
            play_cartesian_client(0.20, 0, 0, 0, 0, 0)
            # time.sleep(1)
            play_cartesian_client(0, -0.1, 0, 0, 0, 0)
            # time.sleep(1)
            self.reset_pos_0 = True
            return
        elif self.curr_action == 4 and not self.reset_pos_4:
            new_pos = get_measured_catesian_pose()
            new_pos.x = new_pos.x + 0.05
            new_pos.y = new_pos.y + 0.50
            move_to_cartesian_position(new_pos)
            time.sleep(2)
            if not self.check_achieved(new_pos):
                move_to_cartesian_position(new_pos)
            self.reset_pos_4 = True
            return
        elif self.curr_action == 3:
            self.before_pick()
            return
        else:
            # Scan the central line for the signature color of the object
            for i in range(len(self.curr_img) / 3, len(self.curr_img)):
                for j in range(len(self.curr_img[0]) / 2 - 100, len(self.curr_img[0]) / 2 + 100):
                    if np.sum(np.abs(np.subtract(self.curr_img[i][j], color))) <= 20:
                        area = area + 1
                        if area > 20:
                            found = True
                            break
            if not found:
                # raw_input("Press Enter to proceed")
                if self.curr_action in [0, 4]:
                    # Move vertically
                    play_cartesian_client(0.05, 0, 0, 0, 0, 0)
                    # time.sleep(0.5)
                else:
                    # Move horizontally
                    play_cartesian_client(0, 0.03, 0, 0, 0, 0)
                    # time.sleep(0.3)
            else:
                if self.curr_action == 6:
                    play_cartesian_client(-0.03, 0, 0, 0, 0, 0)
                    # time.sleep(0.3)
                self.before_pick()
                return


controller = Kinova_Controller()


def callback(img, depth):
    global controller
    bridge = CvBridge()
    try:
        controller.curr_depth = bridge.imgmsg_to_cv2(depth, "16UC1")
        controller.curr_img = bridge.imgmsg_to_cv2(img, "rgb8")
        if controller.curr_action != -1:
            controller.perform_action()
        elif controller.pickup_flag:
            controller.pickup_object()
        elif controller.dropoff_flag:
            controller.dropoff_object()
    except CvBridgeError, e:
        rospy.loginfo("Conversion failed")


def listener():
    rospy.init_node("listener_robot")
    # Two different subscribers for image and openpose json data
    image_sub = message_filters.Subscriber("camera/color/image_rect_color", Image)
    depth_sub = message_filters.Subscriber("camera/depth/image_rect_raw", Image)
    # Messages are synchronized
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], queue_size=2, slop=0.05)
    ts.registerCallback(callback)
    print "Callback started"
    rospy.spin()


def make_decision(action):
    global controller
    if action == 11:
        return
    # MAKE THE ACTION DELETION A FUNCTION!!!!!
    act = arr.act_dict[action]
    arr.del_action(action, controller)

    flag = True
    print controller.inventory
    for i in range(len(act.depend)):
        # print (arr.act_dict[act.depend[i]]).item
        if (arr.act_dict[act.depend[i]]).item not in controller.inventory:
            flag = False
            controller.curr_action = act.depend[i]
    if flag:
        print "Flag!!"
        # Perform the first action in the group
        if arr.head_list[act.group] == [-1] * arr.group_len:
            print "CONGRATULATIONS! ALL COMPLETE!"
        else:
            if arr.head_list[act.group] != -1:
                controller.curr_action = arr.head_list[act.group]
            else:
                for a in arr.head_list:
                    if a != -1:
                        controller.curr_action = a
                        break
    print "The next action is %d" %controller.curr_action
    # act = arr.act_dict[controller.curr_action]
    arr.del_action(controller.curr_action, controller)
    print controller.inventory


def begin():
    launch_windows()

    controller.ini_cart = get_measured_catesian_pose()
    controller.initialize()
    open_fingers()
    listener()
