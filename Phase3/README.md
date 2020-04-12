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
- The following options should be visible: 


![Options](https://github.com/vishnuu95/ENPM661Proj3/blob/master/Phase3/Images/options.png)User Options
![Configuration space ](https://github.com/vishnuu95/ENPM661Proj3/blob/master/Phase3/Images/raw_img.png )Configuration Space
![Path for (-4, -3) --> (4, 3)](https://github.com/vishnuu95/ENPM661Proj3/blob/master/Phase3/Images/optimal_path.png)Sample Optimal Path : (-4, -3) --> (4, 3)
![Path for (-4, -3) --> (0, 3)](https://github.com/vishnuu95/ENPM661Proj3/blob/master/Phase3/Images/optimal_path2.png)Sample Optimal Path 2 : (-4, -3) --> (0, 3)
