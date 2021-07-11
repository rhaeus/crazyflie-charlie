# crazyflie-charlie
This was a project for course DD2419 Project Course in Robotics and Autonomous Systems at KTH Royal Institute of Technology, Stockholm.
The task was to implement software for a small [drone](https://www.bitcraze.io/products/crazyflie-2-1/) equipped with a camera so that it can autonomously navigate and localize 
itself in a given environment using ARUCO markers and traffic signs. The drone has to systematically explore the space in order to find an "intruder", which is represented by
a defined traffic sign. 
We named our drone "Charlie", hence the project name :) 

Here is a video with a system demo:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=YCbfiAgGdmk
" target="_blank"><img src="http://img.youtube.com/vi/YCbfiAgGdmk/0.jpg" 
alt="System demo" width="480" height="360" border="10" /></a>

The system is divided into 4 modules:

* System Control
* Planning/Exploration
* Perception
* Localization

## System Control
The system is controlled by a state machine. After startup the system waits for localization to be ready.
Once it is localized, the drone takes off and hovers at the starting position. Then it requests a new goal from the
explorer service  and requests a path from its current position to
the goal position by issuing a service call to the path planner service. Once it received the
path, it starts moving and executes the path. Once it has reached its goal its action depends on if it is
at a safe spot (see below) or at a random position:

* random position: spins 360 degrees to explore the position
* safe spot: localizes by using the object in front of it, then spins 360 degrees to explore the position,
then localizes again (since spinning introduces some drifting). 
Localizing requires at least 10 measurements of the object, before the drone moves on. The system only localizes when it is in a safe
spot.

Then it goes back to requesting a new goal from the explorer. If at any time it sees the intruder, it stops
and hovers at the position where it saw the intruder.

## Planning/Exploration
### The safe spots
It is a very hard task to accurately estimate the poses of the traffic signs seen by the camera. To make this task a bit easier and more robust we want to make sure that the traffic sign
is seen in the most optimal way before the system attempts to estimate its pose, meaning the camera should face the traffic sign orthogonally and from a reasonable distance.
To achieve that we introduced the concept of safe spots. Those are defined to be a defined distance (e.g. 0.4m) orthogonally and centered in front of every object, facing the object.

We found Aruco marker pose estimation to be more robust, even when viewing from farther away or at a viewing angle other than orthogonally centered. But we decided to treat Aruco markers and 
traffic signs the same, therefore the safe spot concept also applies for Aruco markers.

### Path planning
For path planning the system uses the A* algorithm in 2D (a 3D version can be found [here](https://github.com/rhaeus/path_planning_3d)). The path planner is a ROS service, that can be called with a start and goal pose, for which it then returns a 
path, if one is available. 
The path returned by the A* algorithm consists of waypoints in the resolution of the grid map, so for example if the grid has a resolution of 5cm, the waypoints are 5cm apart.
Too close together lying waypoints can cause troubles for the drone control.
Therefore the path server performs path sparsening on the path received from A*, so that the resulting path contains as few set points as possible.
Based on the map file the path server creates a grid map representation that can be used by the A* implementation.


### Exploration Strategy
The goal of the system is to explore the entire map in order to find the intruder. In order to achieve a good balance between exploring the map and maintaining localization
(which is done only when in a safe spot),
the system alternates between a random position in the map and one of the safe spots.
The safe spots are calculated in the beginning by reading the object (Aruco marker and traffic signs) positions from the map file. Then with each service call request for a new 
goal pose, the explorer service returns either a safe spot or a random position.
To choose a safe spot, it uses the list of safe spots and simply just uses the next in the list.
To choose a random position it takes 10 random positions in the map and calculates a score based on how much new space we would explore in this spot. 
Then it takes the position with the highest score as new goal.
The drone spins at each goal position to explore the spot. We defined the representation of explored space in the map to be a circle with a given radius around the goal position.
(Note: For simplicity it does not take into account what the drone sees while flying to the goal)

## Perception
The perception system is responsible for observing the environment for detecting Aruco markers and detecting, classifying and positioning traffic signs.
The information gathered in the perception component is made available for other components via ROS topics.

### Detection and classification
Detection and classification of traffic signs is done via a neural network, which is trained on a set of traffic signs. 
The implementation is a separate repository which is used as a submodule. It can be found [here](https://github.com/rhaeus/dd2419_detector_baseline).

### Pose estimation
For the pose estimation of the traffic signs we explored three different approaches

* using the bounding box detected during detection and classification
* extracting SIFT features from the traffic sign 
* extracting contours from the traffic sign and fit a bounding rectangle

The final system uses the contour approach because it is more stable and more accurate than the other two approaches. By extracting the contour and fitting a bounding
box based on that we get a much more accurate and tightly fitting bounding box. The bounding boxes generated by the detector are less precise.
The SIFT feature approach resulted in very jumpy pose estimations due to errors in the feature matching. 
The pose estimation is done by comparing points in the camera image to a reference image and using functions from [OpenCV](https://docs.opencv.org/4.5.2/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d).

## Localization
This component is responsible for localizing the drone in the world by using the information given in the map and information observed by the drone (Aruco marker and traffic sign detections).
More specifically the task is to find the transformation between map and odom, since this transform is not static because of drift of the drone while moving.
To achieve this, we have a node that listens to incoming Aruco marker detections or traffic sign detections and then performs the calculations to retrieve the map to odom transform.
Localization is only performed when the drone is in a safe spot.
To retrieve the transformation between map and odom we 

* look at the position of the detected object relative to the odom frame
* look at the position of the corresponding object read from the map in map frame

Based on that we can obtain a transformation matrix between map and odom, from which we only use x, y and yaw. The reason for this is that the drone handles 
z, roll and pitch very well through its sensors, so we leave that estimation to the drone. 
To be more robust to outliers the localization requires at least 10 measurements before it returns a result.
The localization system does not distinguish between Aruco poses and traffic sign poses, they are treated the same during the calculations.

### Filtering
If no filter is applied the transformation between map and odom is very jumpy, which causes the drone to also jump in its position. To solve this problem
we apply a Kalman filter to the map-odom transformation. As in the map-odom calculation the filter is only applied to x, y and yaw.

### Data association
To make it possible for the drone to move in an environment with unique Aruco markers the localization system applies data association to calculate which object the drone is currently seeing.
For that basically the detected object position is compared to all object positions read from the map file and the closest one is chosen.

