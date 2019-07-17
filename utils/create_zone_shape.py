import rospy, pickle
from kortex_driver.srv import *
from kortex_driver.msg import *


class ZoneShapeWrapper:
    """
    A wrapper around the ZoneShape class
    """
    def __init__(self):
        self.instance = ZoneShape()

    def obtain_info(self):
        """
        Ask user to specify the ZoneShape instance
        """
        # Reading shape type
        print "ShapeType: 1 for CYLINDER, 2 for SPHERE, 3 for RECTANGULAR PRISM"
        self.instance.shape_type = int(raw_input("Please specify ShapeType: "))

        # Reading the dimensions
        dims = []
        if self.instance.shape_type == 1:
            # CYLINDER
            dims.append(float(raw_input("Enter radius of cylinder: ")))
            dims.append(float(raw_input("Enter height of cylinder: ")))
        elif self.instance.shape_type == 2:
            # SPHERE
            dims.append(float(raw_input("Enter radius of sphere: ")))
        elif self.instance.shape_type == 3:
            # RECTANGULAR PRISM
            dims.append(float(raw_input("Enter x dimension of rectangular prism: ")))
            dims.append(float(raw_input("Enter y dimension of rectangular prism: ")))
            dims.append(float(raw_input("Enter z dimension of rectangular prism: ")))
        self.instance.dimensions = dims

        # Reading the origin
        pt = Point()
        print "Specify the origin of the shape (x, y, z). x, y, z are in meters."
        pt.x = float(raw_input("x: "))
        pt.y = float(raw_input("y: "))
        pt.z = float(raw_input("z: "))
        self.instance.origin = pt

        # Reading the rotation matrix for orientation
        # The rotation matrix is a 3 x 3 matrix
        # If there is no rotation, use the identity matrix
        str_rot = raw_input("Specify the 3x3 rotation matrix, separate each entry with a ,.")
        rot_raw = str_rot.strip().split(',')
        while len(rot_raw) != 9:
            print "The dimensions for the rotation matrix is incorrect"
            str_rot = raw_input("Specify the 3x3 rotation matrix, separate each entry with a ,.")
            rot_raw = str_rot.strip().split(',')
        for i in range(len(rot_raw)):
            rot_raw[i] = float(rot_raw[i])
        rot_mat = RotationMatrix()
        row1 = RotationMatrixRow()
        row1.column1 = rot_raw[0]
        row1.column2 = rot_raw[1]
        row1.column3 = rot_raw[2]
        rot_mat.row1 = row1
        row2 = RotationMatrixRow()
        row2.column1 = rot_raw[3]
        row2.column2 = rot_raw[4]
        row2.column3 = rot_raw[5]
        rot_mat.row2 = row2
        row3 = RotationMatrixRow()
        row3.column1 = rot_raw[6]
        row3.column2 = rot_raw[7]
        row3.column3 = rot_raw[8]
        rot_mat.row3 = row3
        self.instance.orientation = rot_mat

        # Reading envelope thickness, it is a float
        self.instance.envelope_thickness = float(raw_input("Enter envelope thickness of the ZoneShape (float): "))

    def create_from_scratch(self):
        """
        Create a new ZoneShape from scratch
        """
        self.obtain_info()
        self.save_zone_shape()

    def save_zone_shape(self):
        """
        Save the created ZoneShape instance
        """
        f = open('zone_shape.p', 'wb')
        pickle.dump(self.instance, f)
        print "ZoneShape saved!"
