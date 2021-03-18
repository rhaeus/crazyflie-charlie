import rospy
import numpy as np
import json 
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid


class GridMap:

    def __init__(self, map_path, resolution):
        self.resolution = resolution
        self.unknown_space = -1
        self.free_space = 0
        self.c_space = 128
        self.occupied_space = 254

        self.load_map(map_path)
        
    def load_map(self, map_path):
        with open(map_path, 'rb') as f:
            self.json_world = json.load(f) 
        
        # bounding box of the grid defined by airspace measured in meters
        self.b_min = (self.json_world["airspace"]["min"][0], self.json_world["airspace"]["min"][1])
        self.b_max = (self.json_world["airspace"]["max"][0], self.json_world["airspace"]["max"][1])

        # the x and y resolution measured in meters
        dx = self.resolution
        dy = self.resolution

        # width and heigt of the grid measured in cells
        self.width = int((self.b_max[0] - self.b_min[0]) / float(dx));
        self.height = int((self.b_max[1] - self.b_min[1]) / float(dy));

        print("width:", self.width)
        print("height:", self.height)

        # put origin of the map to (0,0)
        self.origin = Pose()
        self.origin.position.x = self.b_min[0]
        self.origin.position.y = self.b_min[1]


        self.map_data = np.full((self.height, self.width), self.unknown_space, dtype=np.int8)
        self.read_walls()
    
    def coord_to_grid_index(self, coord):
        x_index = int( (coord[0] - self.b_min[0]) / self.resolution)
        y_index = int( (coord[1] - self.b_min[1]) / self.resolution)
        return (x_index, y_index)

    def read_walls(self):
        walls = self.json_world['walls']
        if len(walls) == 0:
            print("no walls in this map")
            return 
        
        for wall in walls:
            # (start_x_coord, start_y_coord) = (wall['plane']['start'][0], wall['plane']['start'][1])
            # (stop_x_coord, stop_y_coord) = (wall['plane']['stop'][0], wall['plane']['stop'][1])
            
            start_coord = (wall['plane']['start'][0], wall['plane']['start'][1])
            stop_coord = (wall['plane']['stop'][0], wall['plane']['stop'][1])

            (start_x_index, start_y_index) = self.coord_to_grid_index(start_coord)
            (stop_x_index, stop_y_index) = self.coord_to_grid_index(stop_coord)

            if start_x_index > stop_x_index:
                h = stop_x_index
                stop_x_index = start_x_index
                start_x_index = h

            if start_y_index > stop_y_index:
                h = stop_y_index
                stop_y_index = start_y_index
                start_y_index = h

            for x in range(start_x_index, stop_x_index + 1):
                for y in range(start_y_index, stop_y_index + 1):
                    self.map_data[y, x] = self.occupied_space
    
    def get_ros_message(self):
        map = OccupancyGrid()

        # Fill in the header
        map.header.stamp = rospy.Time.now()
        map.header.frame_id = 'map'

        # Fill in the info
        map.info.resolution = self.resolution
        map.info.width = self.width
        map.info.height = self.height
        map.info.origin = self.origin

        # Fill in the map data
        map.data = self.map_data.reshape(-1) # (self.__map.size)

        return map
