import rospy, message_filters, os, time
from action_related.action_arr import Action_Arr
import cv2, numpy as np
from sensor_msgs.msg import Image
from utils.kinova_driver import *
from cv_bridge import CvBridge, CvBridgeError
from kortex_driver.msg import *

# arr = Action_Arr()

class Kinova_Controller:

    def __init__(self):
        self.inventory = []
        self.curr_img = []
        self.curr_depth = []
        self.action_colors = [[0, 0, 0], [200, 90, 80], [100, 10, 30], [], [15, 20, 80], [170, 40, 40]]
        self.curr_action = -1
        self.pickup_flag = False
        self.dropoff_flag = False
        self.reset_pos_3 = False
        self.reset_pos_4 = False

        self.first_pick = False
        self.reg_depth = -1

        # Pre-defined joint positions
        self.ini_pos = [323.23815918, 5.01768445969, 21.0555095673, 132.143981934, 177.622116089, 315.249084473, 350.266082764]
        self.drop_pos = [1.06595885754, 58.7315216064, 11.5168209076, 65.5849227905, 169.617950439, 303.822937012, 22.1574840546]
        self.ini_cart = Pose()

    def spot_max(self):
        max = 0
        for i in range(len(self.curr_depth)):
            for j in range(len(self.curr_depth[0]) / 2 - 10, len(self.curr_depth[0]) / 2 + 10):
                if self.curr_depth[i][j] > max:
                    max = self.curr_depth[i][j]
        return max

    def approximate_equal(self, x1, x2):
        return abs(x1 - x2) <= 0.02

    def check_achieved(self, pose_ini):
        pose_curr = get_measured_catesian_pose()
        print pose_curr
        if (pose_ini is None) or (pose_curr is None):
            return True
        print pose_ini
        return (self.approximate_equal(pose_curr.x, pose_ini.x)
                and self.approximate_equal(pose_curr.y, pose_ini.y)
                and self.approximate_equal(pose_curr.z, pose_ini.z))

    def initialize(self):
        # Initialize to this position
        play_joint_client(self.ini_pos)
        time.sleep(3)
        open_fingers()
        if not self.check_achieved(self.ini_cart):
            # raw_input("not achieved!")
            play_joint_client(self.ini_pos)
            time.sleep(3)
        act = int(raw_input("Please specify the next action:"))
        self.curr_action = act

    def dropoff_object(self):
        rospy.loginfo("Dropping off")
        play_cartesian_client(0, 0, 0.1, 0, 0, 0)
        play_joint_client(self.drop_pos)
        time.sleep(3)
        open_fingers()
        time.sleep(2)
        self.dropoff_flag = False
        play_cartesian_client(-0.5, 0, 0, 0, 0, 0)
        # time.sleep(1)
        self.initialize()

    def pickup_object(self):
        rospy.loginfo("Picking up")
        max_depth = self.spot_max()
        if max_depth > 165:
            if max_depth - 165 > 20:
                play_cartesian_client(0, 0, float(165 - max_depth) / 1000, 0, 0, 0)
            else:
                play_cartesian_client(0, 0, -0.01, 0, 0, 0)
        else:
            if self.curr_action == 5:
                play_cartesian_client(0, 0, -0.02, 0, 0, 0)
            close_fingers()
            self.pickup_flag = False
            self.dropoff_flag = True
            time.sleep(2)

    def before_pick(self):
        self.pickup_flag = True
        self.first_pick = True
        self.curr_action = -1
        self.reg_depth = -1

    def perform_action(self):
        # Move the robot arm to the position to the item
        color = self.action_colors[self.curr_action]
        found = False
        area = 0
        if self.curr_action == 3 and not self.reset_pos_3:
            play_cartesian_client(0, -0.30, 0, 0, 0, 0)
            play_cartesian_client(-0.1, 0, 0, 0, 0, 0)
            self.reset_pos_3 = True
            return
        elif self.curr_action == 4 and not self.reset_pos_4:
            new_pos = get_measured_catesian_pose()
            new_pos.x = new_pos.x + 0.56
            new_pos.y = new_pos.y - 0.15
            move_to_cartesian_position(new_pos)
            if not self.check_achieved(new_pos):
                move_to_cartesian_position(new_pos)
            self.reset_pos_4 = True
            return
        else:
            # Scan the central line for the signature color of the object
            for i in range(len(self.curr_img) / 4, len(self.curr_img)):
                for j in range(len(self.curr_img[0]) / 2 - 100, len(self.curr_img[0]) / 2 + 100):
                    if np.sum(np.abs(np.subtract(self.curr_img[i][j], color))) <= 25:
                        area = area + 1
                        if area > 16:
                            found = True
                            break
            if not found:
                # raw_input("Press Enter to proceed")
                if self.curr_action in [3, 4]:
                    play_cartesian_client(0, -0.05, 0, 0, 0, 0)
                else:
                    play_cartesian_client(0.05, 0, 0, 0, 0, 0)
            else:
                raw_input("Press Enter to pick up object")
                if self.curr_action == 5:
                    play_cartesian_client(0, 0.03, 0, 0, 0, 0)
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


def launch_windows():
    os.system("gnome-terminal -e 'bash -c \"roscore; exec bash \"'")
    time.sleep(3)
    os.system("gnome-terminal -e 'bash -c \"rosrun kortex_driver kortex_driver 192.168.1.10 100; exec bash \"'")
    os.system("gnome-terminal -e 'bash -c \"roslaunch kinova_vision kinova_vision.launch; exec bash \"'")
    time.sleep(3)
    os.system("gnome-terminal -e 'bash -c \"rosrun image_view image_view image:=/camera/color/image_raw; exec bash \"'")


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
    act = arr.act_dict[action]
    if act.attribute == "t":
        controller.inventory.append(act.item)
    # Doubly linked list removal
    if act.prev != -1:
        prev = arr.act_dict[act.prev]
        if act.next != -1:
            nex = arr.act_dict[act.next]
            prev.next = act.next
            nex.prev = act.prev
        else:
            # act is tail
            prev.next = -1
    else:
        # act is head
        arr.head_list[act.group] = act.next
        arr.act_dict[act.next].prev = -1

    flag = True
    for i in range(len(act.depend)):
        if (act.depend[i]).item not in controller.inventory:
            flag = False
            controller.curr_action = act.depend[i]
            controller.inventory.append((arr.act_dict[act.depend[i]]).item)
    if flag:
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


launch_windows()
controller.curr_action = 2
controller.ini_cart = get_measured_catesian_pose()
controller.initialize()
open_fingers()
raw_input("Finished a bunch of initialization")
# pos_sub = rospy.Subscriber()
listener()

