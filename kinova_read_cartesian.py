import rospy, pickle, os
from utils.kinova_driver import *
from utils.kinova_misc import launch_windows
from pynput import keyboard


# This is where you can view the cartesian coordinates of the end-effector
# relative to the base of the robot

def on_press(key):
    """
    The handler for a keypress event
    :param key: the key being pressed
    """
    global coordinates
    try:
        if key.char == 's':
            pose = get_measured_catesian_pose()
            coordinates.append(pose)
            print pose
            print 'Recorded current cartesian coordinates'
        elif key.char == 'q':
            print coordinates
            f = open('coordinates.p', 'wb')
            pickle.dump(coordinates, f)
            print 'All coordinates saved to file coordinates.p'
            os._exit(-1)
    except AttributeError:
        print('special key {0} pressed'.format(key))


def on_release(key):
    """
    The handler for a key release event
    :param key: the key being released
    """
    if key == keyboard.Key.esc:
        # Stop listener
        return False


coordinates = []

launch_windows()
raw_input("This program allows you to look at the cartesian coordinates "
          "of the end-effector. Press Enter to start")

rospy.init_node('JustToKeepRunning')

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

# while True:
#     a = 0
rospy.spin()

