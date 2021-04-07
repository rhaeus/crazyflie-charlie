import rospy
import numpy as np
import json 
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from math import fabs, hypot


class GridMap:

    def __init__(self, map_path, resolution=0.05, inflation_radius=0.1):
        self.resolution = resolution
        self.unknown_space = -1
        self.free_space = 0
        self.explored_space = 0
        self.c_space = 50
        self.occupied_space = 100
        self.inflation_radius_m = inflation_radius 
        self.map_data = np.empty((0))
        self.b_max = (0,0) 
        self.b_min = (0,0) 

        self.load_map(map_path)
        
    def load_map(self, map_path):
        if map_path == None:
            return

        with open(map_path, 'rb') as f:
            self.json_world = json.load(f) 
        
        # bounding box of the grid defined by airspace measured in meters
        self.b_min = (self.json_world["airspace"]["min"][0], self.json_world["airspace"]["min"][1])
        self.b_max = (self.json_world["airspace"]["max"][0], self.json_world["airspace"]["max"][1])

        # the x and y resolution measured in meters
        dx = self.resolution
        dy = self.resolution

        # width and height of the grid measured in cells
        self.width = int((self.b_max[0] - self.b_min[0]) / float(dx)) + 1
        self.height = int((self.b_max[1] - self.b_min[1]) / float(dy)) + 1

        # print("width:", self.width)
        # print("height:", self.height)

        self.origin = Pose()
        self.origin.position.x = self.b_min[0]
        self.origin.position.y = self.b_min[1]

        self.inflation_radius_cells = int(self.inflation_radius_m / self.resolution)

        self.map_data = np.full((self.height, self.width), self.unknown_space, dtype=np.int8)
        self.read_walls()

        self.inflate_map(self.inflation_radius_cells)

    def inflate_map(self, radius):
        for x in range(self.width):
            for y in range(self.height):
                if self.map_data[y, x] == self.occupied_space:
                    for nx in range(x - radius, x + radius + 1):
                        for ny in range(y - radius, y + radius + 1):
                            if not self.is_index_in_range((nx, ny)):
                                continue
                            # d = hypot(x - nx, y - ny)
                            # if d <= radius:
                            if self.map_data[ny, nx] != self.occupied_space:
                                self.map_data[ny, nx] = self.c_space

    
    def coord_to_grid_index(self, coord):
        if not self.is_coord_in_range(coord):
            print("[GridMap][coord_to_grid_index] coord outside grid boundary!")
            return None

        x_index = int( (coord[0] - self.b_min[0]) / self.resolution)
        y_index = int( (coord[1] - self.b_min[1]) / self.resolution)
        return (x_index, y_index)

    def grid_index_to_coord(self, index):
        if not self.is_index_in_range(index):
            print("[GridMap][grid_index_to_coord] index out of bounds!")
            return None

        # TODO maybe return cell center
        return (index[0] * self.resolution + self.b_min[0], index[1] * self.resolution + self.b_min[1]) 
    
    def get_flattened_index(self, index):
        return index[0] + self.width * index[1]
    
    def is_coord_in_range(self, coord):
        return coord[0] >= self.b_min[0] and coord[0] <= self.b_max[0] and coord[1] >= self.b_min[1] and coord[1] <= self.b_max[1]

    def is_index_in_range(self, index):
        return index[0] >= 0 and index[0] < self.width and index[1] >= 0 and index[1] < self.height

    def get_cell_neighbors(self, cell_index):
        neighbors = []
        for x in range(-1, 2):
            for y in range(-1, 2):
                if x == 0 and y == 0:
                    continue

                neigbor = (cell_index[0] + x, cell_index[1] + y)

                if self.is_index_in_range(neigbor):
                    neighbors.append(neigbor)

        return neighbors

    def get_value(self, cell_index):
        if not self.is_index_in_range(cell_index):
            print("[GridMap][get_value] index out of range")
            return None
        
        return self.map_data[cell_index[1], cell_index[0]]

    def set_value(self, cell_index, value):
        if not self.is_index_in_range(cell_index):
            print("cell index out of range")
        
        self.map_data[cell_index[1], cell_index[0]] = value


    def read_walls(self):
        walls = self.json_world['walls']
        if len(walls) == 0:
            print("no walls in this map")
            return 
        
        for index, wall in enumerate(walls):
            start_coord = (wall['plane']['start'][0], wall['plane']['start'][1])
            stop_coord = (wall['plane']['stop'][0], wall['plane']['stop'][1])

            if not self.is_coord_in_range(start_coord):
                print("[WARNING][GridMap][read_walls] wall", index, " start coordinate outside airspace, wall will be clipped to space!")
                start_coord = self.clip_coord(start_coord)

            if not self.is_coord_in_range(stop_coord):
                print("[WARNING][GridMap][read_walls] wall", index, " stop coordinate outside airspace, wall will be clipped to space!")
                stop_coord = self.clip_coord(stop_coord)

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
                    if self.is_index_in_range((x,y)):
                        self.map_data[y, x] = self.occupied_space
                    else:
                        print("[WARNING][GridMap][read_walls] wall", index, " index", (x,y), " outside airspace", (0, self.width), (0, self.height), " wall will be clipped to space!")
                        (x_clipped, y_clipped) = self.clip_index((x,y))
                        self.map_data[y_clipped, x_clipped] = self.occupied_space

    def clip_index(self, index):
        (x,y) = index
        x_clipped = x
        y_clipped = y
        if x < 0:
            x_clipped = 0
        if x >= self.width:
            x_clipped = self.width-1
        if y < 0:
            y_clipped = 0
        if y >= self.height:
            y_clipped = self.height-1
        return (x_clipped, y_clipped)

    def clip_coord(self, coord):
        (x,y) = coord
        x_clipped = x
        y_clipped = y
        if x < self.b_min[0]:
            x_clipped = self.b_min[0]
        if x > self.b_max[0]:
            x_clipped = self.b_max[0]
        if y < self.b_min[1]:
            y_clipped = self.b_min[1]
        if y > self.b_max[1]:
            y_clipped = self.b_max[1]
        return (x_clipped, y_clipped)

    
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

    # use raytrace from introduction to robotics course 
    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed
