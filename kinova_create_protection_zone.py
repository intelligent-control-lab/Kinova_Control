from utils.kinova_protection_zone import *
from utils.create_zone_shape import *
from utils.kinova_misc import *


def del_all_zones():
    """
    Delete all protection zones apart from the base protection zone
    """
    global zone_list
    for zone in zone_list:
        delete_protection_zone(zone.handle)

def create_new_zone():
    """
    Create a new protection zone after creating a zone shape
    :return:
    """
    zsw = ZoneShapeWrapper()
    zsw.create_from_scratch()
    # Create the protection zone
    create_protection_zone(name='human_zone', application_data="", shape=zsw.instance)
    print "========== Creation complete =========="
    zone_list = read_all_zones()
    print zone_list

def update_zone(zone_name):
    global zone_list
    for zone in zone_list:
        if zone.name == zone_name:
            # Do your adjustments here
            # For example, you can adjust the dimensions
            zone.shape.dimensions = [0.2, 0.1]
            update_protection_zone(zone)

launch_windows()
# Use any of the functions above
zone_list = read_all_zones()
print zone_list