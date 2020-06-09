# Nine Men's Morris with robot Franka 

In this repository are all scripts for bachelor's thesis at Faculty of electrical engineering and computing, University of Zagreb.

Abstract is at the end of this short documentation. 

### pygame library ###
GUI was made with the help of [pygame](https://pypi.org/project/pygame/) library. The program won't work without GUI so first step is to install pygame library.
```bash 
pip install pygame 
``` 
### ROS packages ###
Every map above is ROS package. First you need to make your own workspace. Tutorial for that is [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

- artificial_intelligence - all AI scripts
- panda_sim - launch and world files, models used in world and nodes for communication with Franka
- franka_description - 3D models for robot arm and files for Gazebo and RViz (.urdf)
- new\_panda\_moveit_config - MoveIt! planer

### Only game ###
If you wish to play game without Franka following commands are needed:
```bash 
rosrun artificial\_intelligence main\_no\_franka.py 
``` 
You can choose: game depth, mode(Human vs. Human, Human vs. AI, AI vs. AI), heuristic function

### Game with Franka in Gazebo simulation ###

1. ```bash
	roscore
   ```
2. ```bash
   roslaunch panda\_sim game\_world.launch
   ```
3. ```bash
   rosrun artificial_intelligence main.py
   ```


#### Abstract ####
A robotic rival based on artificial intelligence is developed for a popular board game The Mills using the minimax algorithm with alpha-beta pruning. For a more amicable approach to the game, a simple graphical user interface is made. After the move of the game is determined with the minimax algorithm, the movement of the figures with the robot Franka in the simulated world in Gazebo follows. To move the figures using the robot Franka, simple "pick-and-place" actions are implemented, where the path of the hand is automatically calculated with the help of MoveIt!. Communication between control algorithm and robot in Gazebo is carried out by ROS.
