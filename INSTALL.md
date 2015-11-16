## Install SDL library

`sudo apt-get install libsdl2-dev libsdl-image1.2-dev`

## Install ROS Packages

`sudo apt-get install ros-indigo-joy ros-indigo-move-base-msgs ros-indigo-wiimote ros-indigo-controller-manager ros-indigo-hector-gazebo ros-indigo-ros-controllers ros-indigo-ros-control ros-indigo-gazebo-ros-control libiw-dev`

`sudo apt-get install ros-indigo-hector-mapping ros-indigo-map-server ros-indigo-amcl ros-indigo-move-base`

## For WIFI localization
`cd catkin_ws/src`
`git clone https://github.com/RafBerkvens/wifi_scan`
`cat wifi_scan/README.md`


## Download and install orocos-bfl (nicht nötig ... keine Ahnung warum ich das installiert hatte^^)

`cd`

`wget http://people.mech.kuleuven.be/~tdelaet/bfl_tar/orocos-bfl-0.8.0-src.tar.bz2`

`tar xjf orocos-bfl-0.8.0-src.tar.bz2`

`cd orocos-bfl-0.8.0`

`mkdir build`

`cd build`

`cmake ..`

`make`

`sudo make install`
