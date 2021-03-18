#!/usr/bin/env python

import rospy
from grid_map import GridMap
from nav_msgs.msg import OccupancyGrid


def publish_map(map):
    pub = rospy.Publisher('/cf1/grid_map', OccupancyGrid, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pub.publish(map)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('grid_map_publisher', anonymous=True)
    # load map
    map_path = rospy.get_param('~map_file_path')
    resolution = rospy.get_param('~map_resolution')

    map = GridMap(map_path, resolution)
    msg = map.get_ros_message()
    
    # publish map
    try:
        publish_map(msg)
    except rospy.ROSInterruptException:
        pass