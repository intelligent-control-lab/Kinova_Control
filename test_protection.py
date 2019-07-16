from utils.kinova_protection_zone import *
from utils.create_zone_shape import *
from utils.kinova_misc import *

launch_windows()

zone_list = read_all_zones()
print zone_list
for zone in zone_list:
    # delete_protection_zone(zone.handle)
    if zone.name == 'human_zone':
        zone.shape.dimensions = [0.2, 0.1]
        update_protection_zone(zone)

# zone_list = read_all_zones()
# print zone_list
# zsw = ZoneShapeWrapper()
# zsw.create_from_scratch()
#
# create_protection_zone(name='human_zone', application_data="", shape=zsw.instance)

zone_list = read_all_zones()
print zone_list
