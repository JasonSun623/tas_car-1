tas_car
=======

Basic setup for the TAS stack

For installation howto read INSTALL.md

For launching simulator run
`roslaunch tas_simulator startSimulation.launch`

To start navigation stack in simulation run
`roslaunch tas_simulator startNavigation.launch`

To change to autonomous mode use
`rostopic pub /wii_communication std_msgs/Int16MultiArray `
hit "Tab" a few times and change that 0 to 1

And publish goals:
`rosrun simple_navigation_goals simple_navigation_goals_node`

Additional details should be added to the simulation world in order to provide enough features for fake odometry
