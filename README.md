# f1tenth_lab7

<img src="lab7.gif"/>

This assignment from [F1TENTH](https://f1tenth.org/learn.html) practices on motion planning.

# How to run?
**Before cloning this respository**, make sure you have clone the [simulator](https://github.com/f1tenth/f1tenth_simulator) respository and set up the packages properly. The setup instructions can be found [here](https://f1tenth.readthedocs.io/en/stable/going_forward/simulator/sim_install.html).

Before executing the launch file, you can choose **0 or 1 for RRT or RRT* respectively at line 43 in rrt.py.**

> Run the script:

`roslaunch f1tenth_lab7 f1tenth_lab7-py.launch`

> Once the car is loaded into the Levine map in RViZ, launch rqt to view the occupancy grid /occ_grid in a new terminal.

`rqt`

> Press *n* key to start navigation.

To visualize the sampled points, uncomment *line 298 in script.py*.

# Limitations
- Python codes are generally slower than C++ code
- Remove publisher for sampled points to prevent lag issue 
- Increase minimum goal distance to 0.5 for RRT and 0.6 for RRT* to decrease the time of finding a collision free path that is close to the goal

# Videos

Higher resolution video are attached below:
- [RRT](https://drive.google.com/file/d/1uoO-5A-DiadRHuYyd1e01KXwF101nOKt/view?usp=sharing)
- [RRT*](https://drive.google.com/file/d/1PEt6TQ33-OXFt7AcxmZuD-5yDVazPvFn/view?usp=sharing) 

