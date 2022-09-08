# Experimental Robotics Laboratory - Assignment 3

## Introduction
This repository contains the third assignment for the University of Genoa's experimental robotics laboratory course 2020. You can use it to experiment with ros. Because it merely provides simulations, no specific hardware is required.

## About The Project
This architecture aims to put the robot on wheels in a simulated environment consisting of six rooms separated by walls. Each room has a colored ball that the robot uses to identify its position in the house. Robot behavior is controlled using a finite state machine with four states (Normal, Sleep, Play, Find) and two substates (TrackNormal and TrackFind). The robot's goal is to map the entire environment, identify all  six rooms, and save their location. The sensors mounted on the robot are a camera and a laser scan.

### Built With

* [ROS noetic](http://wiki.ros.org/noetic/Installation)
* [Python3](https://www.python.org/downloads/)
* [Smach](http://wiki.ros.org/smach)
* [Gazebo](http://gazebosim.org/)
* [Gmapping](http://wiki.ros.org/gmapping)
* [DWA local planner](http://wiki.ros.org/dwa_local_planner)

## Software architecture and state diagram
### Architecture

![Assignment 3 - architecture](https://user-images.githubusercontent.com/25705086/189081874-9fc9cb88-79c1-41b6-8a14-d8c7b8cc7a20.jpg)

#### Components
* Human Interaction
* Behaviour Controller
* Motion Controller
* Ball Tracking

#### Packages
* move_base
* gmapping
* explore_lite

#### Description
The main architecture consists of four components and three external packages for planning and executing robot movements in the environment, creating maps, and exploring unknown parts of the map.

The **Human Interaction** component simulates a human who can execute two actions: send *Play* commands to the robot at random (the frequency can be changed through a ROS parameter set in the *scripts* launch file), and send *go_to* commands with the name of the room where the robot should go, when it is in the Play behaviour and in front of the human waiting for the command.

The **Behaviour Controller** The component contains a finite state machine, which is responsible for changing the behavior of the robot. This exposes the topic's state each time the topic  changes, allowing other components to change their behavior accordingly. The four actions are normal (first action), sleep, play, and search. Normal and Find also have substates, TrackNormal and TrackFind. More details can be found in the State Machines section.

The **Motion Controller** The component handles the robot's motion when the behavior is set to Normal, Sleep, or Play. * / Behavior * Subscribe to the  topic  to get the current state of the state machine.

Under normal conditions, you would choose a random position in your environment, with the maximum dimensions of the simulated map being 8-8 for * y * and 6-6 for * x * at the x and y positions. Get a random number. Then send the target to the action server * move_base * and wait for it to reach the target. If the state changes when the goal has not yet been achieved, the callback of the * send_goal () * function aborts the current goal, allowing the robot to change its behavior immediately accordingly.

In sleep mode, the motion controller sends the home position (obtained from the ros parameter) to the move_base action server and waits. When the goal is achieved, it will be published in the * / home_reached * topic to warn the behavior controller that the robot is at home.

In the play state, you have two options. If the robot is not in front of a human, it will reach the human position (always use the * move_base * action server) and notify the * humanInteractiongenerator * node in * / human_reached. * The topic that it is in front of him and waits for the * go_to * command. If the robot receives a go_to command and its location is already known (stored in the ROS parameter server), the robot will move to that location, wait a random amount of time, then return to the human position, and another. Wait for the * go_to * command. If you don't know the location, it will be published in the * / no_room * topic and the behavior will change to * Find *.
 
The **Ball Tracking** component implements the openCv algorithm to detect the ball (more precisely the color of the ball) and controls the robot movements in the *Track Normal* and *Track Find* behaviours. It subscribes to the robot camera topic (*/robot/camera1/image_raw/compressed*) and, inside the subscriber callback, it uses the OpenCv libraries to detect the balls in the environment. When a ball is detected and the robot is in the *Normal* or *Find* behaviour, it immediately sends a message on the */ball_detected* topic for the Behaviour controller. Since there are six balls of different colours and more than one could be detected at the same time, there is a function called *get_mask_colour()* that selects the bigger (and so closer) ball detected. Then the corresponding mask is created and when the state has transitioned from Find to Track Find or from Normal to Track Normal, the node publishes velocities to the */cmd_vel* topic in order to make the robot reach the closer ball position. At this point, the position of the ball is saved in the parameter server with the name of the color of the ball, and a message alerting the action controller is published to the  topic * / ball_reached *.

If the robot is in the Track Normal state, it returns to the Normal one.

When the robot is in the TrackFind substate and the room corresponding to the detected color is the room specified by the human with the last * go_to * command, when the robot is switched from play and the node  is searched, "* True" is published. increase. * * The theme is / room_found *. In this case, it means that the space you are looking for will eventually be found and the robot will return to play. On the other hand, if  the room is not suitable, * False * will be posted on the  topic * / room_found * and the robot will return to the search state.

This node also implements obstacle avoidance using the LaserScan sensor, allowing the robot to avoid the wall when it reaches the ball.

The **move_base** package provides an implementation of an action that, given a goal in the world, will attempt to reach it with the mobile base of the robot.

The **gmapping** package provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called *slam_gmapping*. Using *slam_gmapping*, a 2-D occupancy grid map from laser and pose data collected by the mobile robot is created.

The **explore_lite** package provides greedy frontier-based exploration. When the node is running, robot will greedily explore its environment until no frontiers could be found. Movement commands will be send to *move_base*.

#### Ros Parameters
* home_x &rarr; home x position on the map
* home_y &rarr; home y position on the map

* sleep_freq &rarr; frequency of the Sleep behaviour
* play_freq &rarr; frequency of Play command sent by the user

Parameters for rooms-colors correspondence:
*  Entrance : Blue
*  Closet : Red
*  Livingroom : Green
*  Kitchen : Yellow
*  Bathroom : Magenta
*  Bedroom : Black

* explore_lite parameters
* gmapping parameters
  
#### Ros Topics 
* /play_command &rarr; topic on which the *human_interaction_gen* node sends the command to make the robot go in the Play behaviour
* /behaviour &rarr; topic on which the current state is published by the behaviour controller
* /ball_detected &rarr; topic on which *ball_tracking* publishes when the ball is detected
* /ball_reached  &rarr; topic on which *ball_tracking* publishes when the robot is in front of the ball
* /room_found &rarr; topic on which *ball_tracking* publishes if the searched room has been found
* /no_room &rarr; topic on which *motion_controller* publishes if the room requested by the GoTo command has been already explored 
* /home_reached &rarr; topic on which *motion_controller* publishes when the home is reached during the Sleep behaviour 
* /human_reached &rarr; topic on which *motion_controller* publishes when the human is reached during the Play behaviour
* /camera1/image_raw/compressed &rarr; topic used to retrieve the images from the robot camera 
* /cmd_vel &rarr; topic used to publish velocities to the robot by the *ball_tracking* component and the *move_base* package
* /odom &rarr; topic used to get the robot odometry
* /scan &rarr; topic for the LaserScan output, used by *ball_tracking* and *move_base* to avoid obstacles
* /map &rarr; topic on which the map (OccupancyGrid) created by gmapping is published
* /goal &rarr; goal for the *move_base* action server
* /result &rarr; result of the *move_base* action server


### State Machine
This is the state machine inside the Behaviour Controller component

![Assignment 3 - state machine](https://user-images.githubusercontent.com/25705086/189082087-25b60605-122d-4c92-91c2-3c73e96f8977.png)

The **Normal** The behavior consists of randomly moving  around the map. When the * ball_tracking * node detects a ball, it moves to the Track Normal substate. Otherwise, it will either transition to the * Play * state when * play_command * is received from the * human_interaction_gen * node, or randomly transition to the * state. * Sleep * state after at least 10 seconds of normal operation.

 In ** Track Normal ** In motion, the robot is controlled by the * ball_following * node, which tracks the detected ball and saves its position. After that, it returns to * normal * operation.

The **Sleep** The action is to move to the starting position and stay there for a while. The transition to the * normal * state occurs after a random time  (20 - 40 seconds) that begins after the robot reaches its home position.

In the **Play** A moving robot walks in front of a human and waits for the * go_to * command, which is a string that represents one of the rooms in the house. If the spatial position is already stored in the ROS parameter, the robot will remain in the * play * state,  reach the desired space, stay there for a while, and then return to the human position. If the spatial position is unknown, the robot will be in the * search * state. After a while in play (randomly 2-6 minutes), the robot will return to normal.

In the **Find** The moving robot uses the explore_lite package to explore the environment. When the  ball is detected, it enters the * TrackFind * substate. After a while (randomly 4-7 minutes), it will return to * play * operation.

In the **Track Find** The moving robot is controlled by the * ball_following * node, allowing it to track the detected ball and save its position. It then checks if the space corresponding to the recognized ball is the space requested by the user with the last * go_to * command. If so, the robot will return to play. If not, the robot returns to the search operation and continues searching for the room.

## Contents of the repository
Here the content of the folders contained in this repository is explained

### Config
Contains the configuration file for Rviz
### Launch
Contains the required startup files. * gmapping.launch * and * move_base.launch * are used to launch the two packages from  other launch files.
 * Simulation.launch * opens Gazebo Simulation (create robots, humans, balls, and worlds) and Rviz and launches the * gmapping * package.
 * scripts.launch * runs the nodes in the * src * and * move_base * folders and loads the parameters to the server.
### Param
Contains the yaml files that define the parameters needed to execute the *move_base* package correctly
### Src
Contains the four python files of the main architecture: *human_interaction.py*, *behaviour_controller.py*, *movement_controller.py* and *ball_following.py*
### Urdf
Contains the descriptions of the robot model and the gazebo file, and the description of the human. 
### Worlds
There is the description of the house that will be loaded on Gazebo.


## Installation and running procedure
First install this three packages, if you don't already have them on your machine: 
```console
sudo apt-get install ros-<ros_distro>-openslam-gmapping
sudo apt-get install ros-<ros_distro>-navigation
sudo apt-get install ros-<ros_distro>-explore-lite
```

Then, after cloning the repository in your Ros workspace, build the package, using the following command:

```console
catkin_make
```
In order to run the system, you have to launch the two following launch files in this order. The first one loads on gazebo the world, the robot and the human and runs the gmapping package, the second one runs the move_base package, loads the necessary ros parameters and launches the rest of the nodes. You can modify the frequency of the Sleep and Play behaviours from the `scripts.launch` launchfile.

```console
roslaunch exp_assignment3 simulation.launch
```
```console
roslaunch exp_assignment3 scripts.launch 
```

## System’s features
The system can achieve the expected behavior in all  tested situations. In two long sessions of continuous  architecture iterations, the robot showed no strange behavior and was able to reach and save everywhere in the house. See the image at the end of this section (all colors are present on the ROS parameter server,  that is, all spatial locations have been examined and saved).

In addition, the robot  always runs the obstacle avoidance algorithm when using the move_base package or when it reaches the ball in the substate of the track, regardless of the state of the robot. In fact, I implemented a simple wall avoidance algorithm inside the node. This allows the robot to track and reach the ball in that area and prevent it from getting caught in the wall when it reaches the wall.

Another system feature is the fact that  the robot detects state in a normal state, thanks to the function and feedback of the ActionServerClient system of * move_base *.

Finally, we used randomness to stress the system. The * play * command sent by the * human_interaction_gen * node is sent at a completely random time (you can change the "frequency" of the startup file) and at the location contained in the * go_to * command. Randomly selected from 6 rooms each time. In addition, the time it takes for the robot to sleep or arrive at a location, and the time elapsed before the robot changes its behavior (play to normal, normal to sleep, or search to play) is also randomized. However, during my testing, the system was able to maintain the expected behavior throughout the  simulation, despite all these random factors.



## System’s limitations
In general, one of the main limitations of the system is that the user's requirements and the exploration of the world under normal conditions are completely random, so it takes a long time to detect and store  the position of every ball  in the house. Is the fact that it can take.

Using the explore_lite package in a search operation, the robot may not have been able to find the  location requested by the user in time. This can happen because the search is based only  on the explore_lite frontier, not  the location of the space already stored. As a result, there may be rooms that have already been explored due to the explore_lite algorithm, but in reality their location is not stored on the parameter server.

If the explore_lite package cannot find a new frontier to explore, that is, if the environment is completely explored from the perspective of this package, the robot will stop and remain standing until the state changes (minutes). takes) .

When the robot switched between search and track search, few strange behaviors were observed. The robot may stop after switching states because the ball is not in the camera's field of view.

In normal motion, some random positions may be outside  the walls of the house, so the robot may get stuck until the motion changes (sleep, normal, or transition to a normal truck). there is.

## Possible technical improvements
* Use a knowledge-based approach instead of explore_lite to explore the environment. For example, use known locations to reduce the time it takes for a robot  to find the requested location and prevent it from returning to the game without finding the right room.

* You can implement a system that prevents you from entering the track's substate when a ball that has already been saved is detected. As it is today, the system will always track the ball when the CV algorithm detects it, unless it was the last ball detected. This can save you a lot of time  finding everywhere.

* *explore_lite* If the  algorithm cannot find the limit, it will stop. The technical improvement is to change the robot's behavior when this happens, rather than waiting for the state machine to return to normal after a few minutes of transition  to reproduce the behavior. The robot does nothing.

* Excludes positions outside the walls of the house from  random points that could be assigned to the robot in normal motion.

* Test the system with a more complex map to see if it works..

## Rqt_graph
### Main Architecture

![rosgraph_assignment3_explore](https://user-images.githubusercontent.com/25705086/189082182-52b7ed3a-aadf-4736-939a-d074ddc08520.png)

## Author
Mohamed Alaaeldin Youssef Mahmoud\
E-mail: mohammed.eldin.88@hotmail.com\
ID: 4844271

