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
