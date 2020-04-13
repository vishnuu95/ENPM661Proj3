# Phase 4 - Implementation of A* for robot with non holonomic constraints.


- The project was created towards completion of Project 3 Phase 3: A* for robot with non-holonomic constraints as part of ENPM661- Planning for Autonomous Robots at University of Maryland, College park.
- The configuration space is as shown in the figure below. 
- Sample outputs are also shown below. 
## Dependencies
- Python 3
- Ubuntu 16.04 (might work on future version, not tested)
- OpenCV Version > 3.4.4
- ROS Kinetic
- Gazebo
## Run the code
- Clone this repository and navigate to Phase_4 folder.
- Import the turtlebot_astar folder into the source folder of the existing catkin workspace and navigate to the workspace folder to enter command:
```
catkin_make
```
This will build the files for running the Gazebo Simulation. Navigate to  src/turtlebot_astar/src/ folder.
- To run the A* algorithm for differential drive robot: 
```
python3 Astar_curve.py
```
### Options visible
<p align="center">
  <img width="900" height="300" src="https://github.com/vishnuu95/ENPM661Proj3/blob/master/Phase3_and_4/Images/options.png">
</p>

### Configuration Space
<p align="center">
  <img width="460" height="300" src="https://github.com/vishnuu95/ENPM661Proj3/blob/master/Phase3_and_4/Images/raw_img.png">
</p>

### Sample output for (-4, -3) --> (4, 3)
<p align="center">
  <img width="460" height="300" src="https://github.com/vishnuu95/ENPM661Proj3/blob/master/Phase3_and_4/Images/optimal_path.png">
</p>

### Sample output for (-4, -3) --> (0, 3)
<p align="center">
  <img width="460" height="300" src="https://github.com/vishnuu95/ENPM661Proj3/blob/master/Phase3_and_4/Images/optimal_path2.png">
</p>

### Node exploration in High resolution
<p align="center">
  <img width="460" height="300" src="https://github.com/vishnuu95/ENPM661Proj3/blob/master/Phase3_and_4/Images/node_exploration.png">
</p>

## Run gazebo simulation
- The Astar\_curve.py file generates a nodes\_optimal.txt file, which contains the final path nodes and RPMs
- To run the simulation enter command:
```
roslaunch turtlebot_astar tastar.launch X:=<x-coordinate-in-gazebo-coordinates> Y:=<y-coordinate-in-gazebo-coordinates> T:=<theta-in-radians>
```
- The simulation starts in Gazebo. The X and Y values are in Gazebo environment coordinate,i.e., with the origin(0,0) at the geometric center of the world. Default values are set to X = -4, Y = -3 and T = 0, and the simulations have been run with these coordinates.

