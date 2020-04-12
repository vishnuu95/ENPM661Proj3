# Phase 3 - Implementation of A* for robot with non holonomic constraints.


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
- Clone this repository and navigate to Phase3 folder. 
- To run the A* algorithm for differential drive robot: 
```
python3 Astar_curve.py
```
### Options visible
<p align="center">
  <img width="900" height="300" src="https://github.com/vishnuu95/ENPM661Proj3/blob/master/Phase3/Images/options.png">
</p>

### Configuration Space
<p align="center">
  <img width="460" height="300" src="https://github.com/vishnuu95/ENPM661Proj3/blob/master/Phase3/Images/raw_img.png">
</p>

### Sample output for (-4, -3) --> (4, 3)
<p align="center">
  <img width="460" height="300" src="https://github.com/vishnuu95/ENPM661Proj3/blob/master/Phase3/Images/optimal_path.png">
</p>

### Sample output for (-4, -3) --> (0, 3)
<p align="center">
  <img width="460" height="300" src="https://github.com/vishnuu95/ENPM661Proj3/blob/master/Phase3/Images/optimal_path2.png">
</p>

### Node exploration in High resolution
<p align="center">
  <img width="460" height="300" src="https://github.com/vishnuu95/ENPM661Proj3/blob/master/Phase3/Images/node_exploration.png">
</p>

