command list:

ros2 launch sllidar_ros2 view_sllidar_a1_launch.py

ros2 run my_robot_bringup cmd_vel_listener

ros2 run my_robot_bringup odom_publisher

ros2 launch my_robot_description slam_toolbox_launch.py

ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}, angular: {z: 0.1}}'





setup command:

sudo apt update
sudo apt install \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-rplidar-ros \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro


sudo apt update
sudo apt upgrade

sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

sudo apt update





check command:

echo $AMENT_PREFIX_PATH

unset AMENT_PREFIX_PATH
source ~/ros2_ws/install/setup.bash
echo $AMENT_PREFIX_PATH

ls -l ~/ros2_ws/install/setup.bash

colcon list
(
issue i found -> package is being recognized as a catkin (ROS 1) package, not a native ROS 2
khemin@khemin:~/ros2_ws$ colcon list
my_robot_description	src/my_robot_description	(ros.catkin)
)



numpy downgrade:

sudo apt update
sudo apt install build-essential python3-dev python3-pip python3-setuptools python3-wheel
sudo apt install libatlas-base-dev gfortran

python3 -m pip install --upgrade pip setuptools wheel

python3 -m pip install --upgrade pip

sudo apt-get update
sudo apt-get install --reinstall build-essential python3-dev python3-pip libatlas-base-dev gfortran

pip3 uninstall numpy -y
sudo pip3 uninstall numpy -y

pip3 install numpy==1.21.6 --prefer-binary --user


pip setuptools downgrade:
pip install setuptools==68.2.2




git command:

git pull origin master

git pull --rebase origin master

git push origin master





rosbot3:

sudo apt update
sudo apt install ros-humble-filters
sudo apt install ros-humble-moveit
sudo apt install ros-humble-control-toolbox
sudo apt install ros-humble-rcpputils ros-humble-realtime-tools
sudo apt install ros-humble-rcpputils
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control

sudo apt install --reinstall ros-humble-controller-manager ros-humble-gazebo-ros2-control

mv ~/rosbot_ws/src/open_manipulator_x/open_manipulator_x_joy ~/rosbot_ws/src/_open_manipulator_x_joy_backup

mv src/husarion_controllers/mecanum_drive_controller ~/mecanum_drive_controller_backup

mv ~/rosbot_ws/src/open_manipulator_x/open_manipulator_x_joy ~/rosbot_ws/src/open_manipulator_x/open_manipulator_x_joy.bak

mv ~/rosbot_ws/src/open_manipulator_x/open_manipulator_x_moveit ~/rosbot_ws/src/open_manipulator_x/open_manipulator_x_moveit.bak


colcon build --packages-select rosbot_bringup rosbot_joy rosbot_description husarion_components_description husarion_gz_worlds rosbot_utils rosbot_localization open_manipulator_x_description

colcon build --packages-select husarion_components_description husarion_gz_worlds open_manipulator_x_description open_manipulator_x_moveit rosbot rosbot_bringup rosbot_controller rosbot_description rosbot_gazebo rosbot_joy rosbot_localization rosbot_utils

nano ~/rosbot_ws/src/open_manipulator_x/open_manipulator_x_joy/src/joy2servo.cpp

nano ~/rosbot_ws/src/open_manipulator_x/open_manipulator_x_joy/include/open_manipulator_x_joy/joy2servo.hpp

#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit_msgs/srv/servo_command_type.hpp>

cd ~/rosbot_ws/src/rosbot_ros
colcon build
source ../install/setup.bash
ros2 launch rosbot_gazebo simulation.launch.py robot_model:=rosbot_xl

gedit ~/rosbot_ws/src/rosbot_ros/rosbot_gazebo/launch/spawn_robot.launch.py
gedit ~/rosbot_ws/src/rosbot_ros/rosbot_bringup/launch/bringup.launch.py

from launch_ros.actions import Node, SetParameter, SetRemap
# PushROSNamespace(namespace),

khemin@khemin:~/rosbot_ws/src/rosbot_ros/rosbot_controller/config/rosbot_xl$ gedit mecanum_drive_controller.yaml 
gedit ~/rosbot_ws/src/rosbot_ros/rosbot_controller/config/rosbot_xl/mecanum_drive_controller.yaml

front_left_wheel_command_joint_name: fl_wheel_joint
front_right_wheel_command_joint_name: fr_wheel_joint
rear_left_wheel_command_joint_name: rl_wheel_joint
rear_right_wheel_command_joint_name: rr_wheel_joint

wheels_radius: 0.047

sudo apt update && sudo apt upgrade

kinematics:
      wheel_separation_x: 0.170
      wheel_separation_y: 0.270
      wheels_radius: 0.047   # <--- nested inside 'kinematics'
