# Introduction
This is Project #3 of the Udacity Robotics Software Nanodegree program. The objective is to exercise the AMCL ROS package. AMCL stand for Adaptive Monte Carlo Localization, a method using sensor data to determine the location, position and orientation, of a robot in a mapped environment. The package will be exercised within a simulation environment created in prior projects.

## Specific Objectives
There are numerous criteria to be met to complete this project. The following are the highlights:

- Create a ROS workspace containing the ROS AMCL package, simluation environment and a robot.
- Create launch files to bring up the simulation.
- Implement methods of driving the robot.
- Configure the RViz application to show status of localization.
- Demonstrate robot's ability to localize.

The simulation is carried out in the simulation environment created earlier and used in the prior project #2 Chase It. The simulation uses the turtlebot robot. Two methods of driving the robot are implemented, a manual method using the teleop keyboard package. By pressing keys the robot is commanded to move linearly or rotate. Speed and rate can be adjusted up or down by key press also.

Another method of robot activation is the move base package. This package receives navigation targets or destinations tthrough mouse clicks on the map in the RViz package. This package avoids obstacles an plan around them.

# Project Results
The following shows the results obtained for this project.

## Install & Run
The application was developed in Linux Ubuntu version 16.04 with ROS distribution Kinetic. The application depends on a few packages that must be present: navigation, map-server, move-base and amcl. These can be installed using,

> sudo apt install ros-kinetic-<name_of_required_package>

The repository should be cloned

> git clone https://github.com/KtGunn/Project3_WhereAmI.git

The user must navigate to the catkin_ws sub-directory. In case the two directories 'devel' and 'setup' are present, they should be removed, followed by the compliation command, catkin_make

> rm -rf devel setup && catkin_make

To bring up the application the following commands are issued, each in its own console,

> roslaunch my_robot world.launch

This brings up the simulation environment and RViz configured to view, observe and monitor the progress of locazliation.

![world_rviz](</workspace/images/launch_world.png>)

Note that the windows have been re-sized and moved. The windows are likely to overlap at start up. Note also that RViz renders a robot in a default map with a laser scan showing. Note the scan matches the walls of the environment and the robot's location within it, a good sign.

The following command issued in a new console brings up the amcl node.

> roslaunch my_robot amcl.launch

Now RViz shows the robot in a localization map with a cluster of particles surrounding the robot. This indicates that the amcl node is up and running, waiting for robot motion and ready to estimate the robot's true location. Note that the robot is accurately located but the amcl algorithm has yet to receive odometry and sensor data to know that.

![world_rviz](</workspace/images/launch_amcl.png>)

Also to note is the localization map in RViz. That map was created using the 'pgm_map_creator' package. This package is not part of the nodes launched. It was used in an off-line mode to create a localization map from the simulation environment as input. The map was edited to include only wall objects, i.e. the fountain, hydrant and circular columns were removed.

Two methods of driving the robot are implemented. The move_base node allows a target location or destination to be indicated on the localization map. That method is already up and running after the second roslaunch command. The other method uses the teletop_twist_keyboard node. This node accepts key hits in the console. Those are tanslated into motion commands such as forward and backward motion, pure rotation, combined linear and rotational motion, etc. That node is launched with the command (in a separate console),

> rosrun teleop_twist_keyboard teleop_twist_keyboard.py 

The image below shows the console where teleop has been started and instrutions how to command the robot into motion.

![world_rviz](</workspace/images/rosrun_teleop.png>)

## Localization

Tests of the AMCL localization method were conducted to discover its characteristics and performance. Of primary importance is ability to locazlize a robot from a highly uncertain location, i.e. with great uncertainty in the robot's position coordinates and heading, i.e. pose.

There are numerous parameters that affect the AMCL packages performance The screen shot below shows the ones that were of interest in the present case. These can be adjusted and their effect observed.

![world_rviz](</workspace/images/amcl_params.png>)

A few tests were done altering these parameters mostly just to see their gross effect as opposed to fine tuning the algorithm. First a base line was established. This used default AMCL parameters. There after parameters were modified and the same simulation run. A fixed location was selected as a point at which to compare the different runs.


### Baseline Simluation

First test is to run AMCL with an unknown position and observe how it converges to the solution, right solution hopefully, or if it converges at all. We arbritrarily initialize the position to the origin and give the tug 1radian in heading error. A large initial covariance sets the position as unknwon. Particles will be spread over the map randomly which is what we wish to see.

The gif below shows how the AMCL method converges, slowly. One parameter that was modified immediately was the maximum number of particles, originally 5,000 set to 2,000. Five thousand particles bogged down the system this project was developed on.

![world_rviz](</workspace/images/Initial_45.gif>)

The convergence characteristics of the AMCL algorithm are on good display. The robot is running between parallel walls. The environment has a few locations where walls are parallel and particles cluster in those locations. This is the multi-modal feature of particle filters in contrast with EKF which is uni-modal (one solution). Eventually clusters where the map indicates presence of walls, but the lidar senses none, drop out and one cluster dominates. Once that point is reached, it is still interesting how slowly the algorithm converges to an exact solution.

To evaluate effects of parameter variations, a fixed point on the robot's path is chosen. When the robot reaches that point, a snap shot of the state of convergence is taken. Comparison of snap shots can reveal the effect the parameter has. Below is the snap shot for the initial test, the baseline.

![world_rviz](</workspace/images/Baseline.png>)

### Max Laser Beams

First up is 'laser_max_beams' intially at 30 but now set to 100. Better convergence especially after the single cluster formation can be expected. A comparison is made between the baseline and each test is done by looking at convergence at a fixed location on the robot's path.

![world_rviz](</workspace/images/30-100beams.png>)


### Laser Z Hit & Rand

![world_rviz](</workspace/images/laser_hit_50-50.png>)

### Minimum Distance

![world_rviz](</workspace/images/min_d_0.75.png>)

### Odom Alphas

![world_rviz](</workspace/images/odom_alphas.png>)
