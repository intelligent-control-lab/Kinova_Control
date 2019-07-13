import rospy
from kortex_driver.srv import *
from kortex_driver.msg import *

class ZoneShapeWrapper:
    """
    A wrapper around the ZoneShape class
    """
    def __init__(self):
        self.instance = ZoneShape()

    def obtain_info(self):
        self.instance.shape_type = int(raw_input("Please specify ShapeType: "))
        pt = Point()
        print "Specify the origin of the shape (x, y, z). x, y, z are in meters."
        pt.x = float(raw_input("x: "))
        pt.y = float(raw_input("y: "))
        pt.z = float(raw_input("z: "))
        self.instance.origin = pt
        str_rot = raw_input("Specify the rotation matrix, separate each entry with a ,.")