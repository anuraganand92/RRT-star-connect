# RRT-star-connect
#### Created by Anurag Anand

## Introduction
This code implements the Rapidly-exploring Random Trees (RRT*) algorithm to find a path between a start and an end point in an environment. The algorithm creates two trees, one starting from the start point and the other starting from the end point, and tries to connect them until they meet. The algorithm uses a steering function to move towards the random point and checks if the path from the nearest point in the tree to the new point collides with any obstacles. If it does not, the new point is added to the tree and the process is repeated.

## About the Algorithm
RRT* Connect is an extension of the Rapidly-exploring Random Trees (RRT*) algorithm, which is a popular sampling-based motion planning algorithm used in robotics and other related fields. RRT* Connect is designed to solve motion planning problems for systems with multiple robots, where the robots must avoid collisions with each other while moving to their desired positions. This algorithm works by constructing two RRT* trees, one for each robot, and then connecting them in a way that ensures collision-free paths for both robots. The RRT* Connect algorithm has been shown to be effective in solving complex motion planning problems for multiple robots, making it a valuable tool for roboticists and researchers in related fields.

## Imports
The code uses the following imports:
* `numpy`
* `math`: `hypot`, `sqrt`
* `random`: `randrange`
* `opencv`: `cv2`

## Classes
* `Vertex`: A class representing a node in the tree with attributes for position and parent node.
* `RRT_star_connect`: A class that implements the RRT* algorithm for path planning between two points in an environment. 

## Functions
The code contains the following functions:
* `dist(p1,p2)`: Computes the Euclidean distance between two points `p1` and `p2`.
* `__init__(self,env_image)`: Initializes the start and end points, the environment image, and the path list.
* `start_connect(self,env_image=None)`: Implements the RRT* algorithm and returns the path found.
* `steer(self,start_pos,end_pos)`: Computes a new point that is closer to the end point than the start point and within a maximum distance from the start point.
* `collide_line(self,start_pos,end_pos)`: Checks if the line connecting two points collides with any obstacles in the environment.
* `shorten_path(self)`: Shortens the path found by removing intermediate points that can be skipped without colliding with obstacles.
* `draw_path(self,env_image)`: Draws the path found on the environment image.

## Variables
The code also defines some global variables for the GUI, click count, maximum distance between two nodes of a tree, and a radius around the end point to be determined as the finishing area.

## Running the Program
If `gui` is set to `True` or `1`, then the algorithm will display a GUI window that shows the progress of the algorithm in finding the path. <br>
Double click to select the desired starting point, then double click at another position to select the ending point. Make sure to choose the two points on the black parts, as the path generates on the black parts with the light colored parts as obstacles.