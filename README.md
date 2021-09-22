# Introduction
This is Project #3 of the Udacity Robotics Software Nanodegree program. The objective is to exercise the AMCL ROS package. AMCL stand for Adaptive Monte Carlo Localization, a method using sensor data to determine the location, position and orientation, of a robot in a mapped environment. The package will be exercised within a simulation environment created in prior projects.

## Specific Objectives
There are numerous criteria to be met to complete this project. The following are the highlights:

- Create a ROS workspace containing the ROS AMCL package, simulation environment and a robot.
- Create launch files to bring up the simulation.
- Implement methods of driving the robot.
- Configure the RViz application to show status of localization.
- Demonstrate robot's ability to localize.

The simulation is carried out in the simulation environment created earlier and used in the prior project #2 Chase It. The simulation uses the turtlebot robot. Two methods of driving the robot are implemented, a manual method using the teleop keyboard package. By pressing keys the robot is commanded to move linearly or rotate. Speed and rate can be adjusted up or down by key press also.

Another method of robot activation is the move_base package. This package receives navigation targets or destinations through mouse clicks on the map in the RViz package. This package avoids obstacles and plans around them. This method is an option.

# Project Results
The following shows the results obtained for this project.

## Install & Run
The application was developed in Linux Ubuntu version 16.04 with ROS distribution Kinetic. The application depends on a few packages that must be present: navigation, map-server, move_base and AMCL. These can be installed using,

> sudo apt install ros-kinetic-<name_of_required_package>

The repository should be cloned

> git clone https://github.com/KtGunn/Project3_WhereAmI.git

The user must navigate to the catkin_ws sub-directory. In case the two directories 'devel' and 'setup' are present, they should be removed, followed by the compilation command, catkin_make

> rm -rf devel setup && catkin_make

To bring up the application the following commands are issued, each in its own console,

> roslaunch my_robot world.launch

This brings up the simulation environment and RViz configured to view, observe and monitor the progress of localization.

![world_rviz](</workspace/images/launch_world.png>)

Note that the windows have been re-sized and moved. The windows are likely to overlap at start up. Note also that RViz renders a robot in a default map with a laser scan showing. Note the scan matches the walls of the environment and the robot's location within it, a good sign.

The following command issued in a new console brings up the AMCL node.

> roslaunch my_robot amcl.launch

Now RViz shows the robot in a localization map with a cluster of particles surrounding the robot. This indicates that the AMCL node is up and running, waiting for robot motion and ready to estimate the robot's true location. Note that the robot is accurately located but the AMCL algorithm has yet to receive odometry and sensor data to know that.

![world_rviz](</workspace/images/launch_amcl.png>)

Also to note is the localization map in RViz. That map was created using the 'pgm_map_creator' package. This package is not part of the nodes launched. It was used in an off-line mode to create a localization map from the simulation environment as input. The map was edited to include only wall objects, i.e. the fountain, hydrant and circular columns were removed.

Two methods of driving the robot are implemented. The move_base node allows a target location or destination to be indicated on the localization map. That method is already up and running after the second roslaunch command. The other method uses the teletop_twist_keyboard node. This node accepts key hits in the console. Those are translated into motion commands such as forward and backward motion, pure rotation, combined linear and rotational motion, etc. That node is launched with the command (in a separate console),

> rosrun teleop_twist_keyboard teleop_twist_keyboard.py 

The image below shows the console where teleop has been started and instructions how to command the robot into motion.

![world_rviz](</workspace/images/rosrun_teleop.png>)

## Localization

Tests of the AMCL localization method were conducted to discover its characteristics and performance. Of primary importance is ability to localize a robot from a highly uncertain location, i.e. with great uncertainty in the robot's position coordinates and heading, i.e. pose.

There are numerous parameters that affect the AMCL packages performance The screen shot below shows the ones that were of interest in the present case. These can be adjusted and their effect observed.

![world_rviz](</workspace/images/amcl_params.png>)

A few tests were done altering these parameters mostly just to see their gross effect as opposed to fine tuning the algorithm. First a base line was established. This used default AMCL parameters. There after parameters were modified and the same simulation run. A fixed location was selected as a point at which to compare the different runs.


### Baseline Simulation

First test is to run AMCL with an unknown position and observe how it converges to the solution, right solution hopefully, or if it converges at all. We arbitrarily initialize the position to the origin and give the tug 1radian in heading error. A large initial covariance sets the position as unknown. Particles will be spread over the map randomly which is what we wish to see.

The gif below shows how the AMCL method converges, slowly. One parameter that was modified immediately was the maximum number of particles, originally 5,000 set to 2,000. Five thousand particles bogged down the system this project was developed on.

![world_rviz](</workspace/images/Initial_45.gif>)

The convergence characteristics of the AMCL algorithm are on good display. The robot is running between parallel walls. The environment has a few locations where walls are parallel and particles cluster in those locations. This is the multi-modal feature of particle filters in contrast with EKF which is unimodal (one solution). Eventually clusters where the map indicates presence of walls, but the lidar senses none, drop out and one cluster dominates. Once that point is reached, it is still interesting how slowly the algorithm converges to an exact solution.

To evaluate effects of parameter variations, a fixed point on the robot's path is chosen. When the robot reaches that point, a snap shot of the state of convergence is taken. Comparison of snap shots can reveal the effect of the parameter. Below is the snap shot for the initial test, the baseline.

![world_rviz](</workspace/images/Baseline.png>)

### Max Laser Beams

First up is 'laser_max_beams' initially at 30 but now set to 100. Better convergence especially after the single cluster formation is expected.

![world_rviz](</workspace/images/30-100beams.png>)

There is not a pronounced effect but it is in the direction expected; faster clustering to a single solution. It also appears that unlikely solutions disappear faster.

### Laser Z Hit & Rand

This test uses the likelihood_field model of a laser. Two parameters, laser_z_hit and laser_z_rand, which add up to one, dial in the mixture between believing laser readings are from the mapped walls versus from random objects. Environments with many random unmapped objects should bump up laser_z_rand and lower laser_z_hit. Default for this test as laser_z_hit of 0.95 and 0.05 for laser_z_rand. Below is a test where both parameters were set even at 0.5.

![world_rviz](</workspace/images/laser_hit_50-50.png>)

This looks worse than baseline and appears to hold onto solutions that are wrong. There likely are important tuning parameters in an actual application.

### Minimum Distance

A balance must be struck between rate of localization updates and conserving computing power for other operations. The update_min_d and update_min_a parameters set the distance and amount of rotation required to trigger a localization. Baseline uses 0.2m and PI/6. We kick the update_min_d up a notch to 0.75 and results are shown in the snap shot below.

![world_rviz](</workspace/images/min_d_0.75.png>)

This looks horrible and confirms that at least for unknown initial position, filter updates must be frequent lset the robot run out of open space before localizing. These look to be important tuning parameters.

### Odom Alphas

Just as testing the filter with widely scatters particles at start, it is useful to know what parameters lump the particles close in to the robot's position when the filter has converged to a solution. We look to the odom_alpha parameters for that purpose. There are five odometry noise parameters, all set initially at 0.2. Below is a test with all five set to an order of magnitude lower than baseline or 0.02.

![world_rviz](</workspace/images/odom_alphas_02.png>)

As expected, the particles appear to tuck in closely to the robot as it localizes accurately.

## Move_base Example

In the preceding sections, the teleop keyboard method was used to move the robot and exercise the AMCL package. The alternative method of activating both the robot and AMCL package, the move_base package, was also installed. Like the AMCL package, the move_base package uses many parameters for tuning the performance of the algorithm. For completeness, a simple test of this package is included in this project.

With the move_base package a destination or goal of the robot is set by clicking the mouse cursor on the map. The move_base package then creates a 'global plan' to reach the destination. As the robot moves toward the destination, it creates a 'local plan' for immediate motion commands. In absence of obstacles, the local plan closely follows the global plan. If obstacles are encountered, the local plan will command the robot around them. Refer to the gif below

![world_rviz](</workspace/images/base_move_example.gif>)

The move_base package publishes on may topics which can be displayed in Rviz.  The vertical red arrow at left is the goal was set by the user. A red line is drawn from the robot’s starting position to the goal and indicates the global plan. As the robot moves toward the goal and obstacles come into view, they are rendered as light blue ovals. These ovals represent how the obstacles are ‘inflated’ and thus keep the robot at a proper stand-off. A very short white curve extending from the robot shows the immediate plan the robot follows at any instant.

We see the robot fight its way to the goal and eventually get there. However the ‘tuning’ appears sub-optimal. In a well tuned application, the robot would make smoother local plans, farther away from the obstacles and not have to struggle away from static obstacles. On the other hand, this is a good example of the move_base packages capabilities. As expected, we also see that the AMCL package is active and continually updating the robot’s location.

