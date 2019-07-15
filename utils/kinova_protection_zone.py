import rospy
from kortex_driver.srv import *
from kortex_driver.msg import *


def create_protection_zone(name, application_data='', shape=ZoneShape(), limitations=[],
                           envelope_limitations=[]):
    """
    Create a protection zone from these parameters
    :param name: the name of the protection zone
    :param application_data: application data, reserved for Web API use
    :param shape: the ZoneShape object that specifies the boundaries
    :param limitations: list of Cartesian limitations
    :param envelope_limitations: list of Cartesian limitations
    """
    proc_zone = ProtectionZone()
    proc_zone.is_enabled = True
    proc_zone.name = name
    proc_zone.application_data = application_data
    proc_zone.shape = shape
    proc_zone.limitations = limitations
    proc_zone.envelope_limitations = envelope_limitations

    rospy.wait_for_service('CreateProtectionZone')

    try:
        function_CreateProtectionZone = rospy.ServiceProxy('CreateProtectionZone', CreateProtectionZone)
        proc_zone.handle = function_CreateProtectionZone(proc_zone)
    except rospy.ServiceException as e:
        print "Service call failed: %s" % e


def delete_protection_zone(handle):
    """
    Delete the protection zone specified by the given protection zone handle
    :param handle: the handle of the protection zone we wish to delete
    """
    rospy.wait_for_service('DeleteProtectionZone')

    try:
        function_DeleteProtectionZone = rospy.ServiceProxy('DeleteProtectionZone', DeleteProtectionZone)
        function_DeleteProtectionZone(handle)
    except rospy.ServiceException as e:
        print "Service call failed: %s" % e


def get_protection_zone_state(handle):
    """
    Obtain current protection zone state
    :param handle: the ProtectionZoneHandle of the desired protection zone
    :return: an integer indicating the state
             0 - unspecified
             1 - reached
             2 - entered
             3 - exited
    """
    rospy.wait_for_service('GetProtectionZoneState')

    try:
        function_GetProtectionZoneState = rospy.ServiceProxy('GetProtectionZoneState', GetProtectionZoneState)
        state = function_GetProtectionZoneState(handle)
        return state
    except rospy.ServiceException as e:
        print "Service call failed: %s" % e


def update_protection_zone(new_zone):
    """
    Update the protection zone
    :param new_zone: the protection zone we would like to update into
    """
    rospy.wait_for_service('UpdateProtectionZone')

    try:
        function_UpdateProtectionZone = rospy.ServiceProxy('UpdateProtectionZone', UpdateProtectionZone)
        function_UpdateProtectionZone(new_zone)
    except rospy.ServiceException as e:
        print "Service call failed : %s" % e
