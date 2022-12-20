export LC_CTYPE=C.UTF-8
export LANG=C.UTF-8

#export PX4_HOME_LAT=24.1940422
#export PX4_HOME_LON=55.1001015

source /opt/ros/melodic/setup.bash

export PX4_PATH=/px4/PX4-Autopilot
source $PX4_PATH/Tools/setup_gazebo.bash $PX4_PATH $PX4_PATH/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_PATH/Tools/sitl_gazebo

roslaunch px4 mavros_posix_sitl.launch vehicle:=iris