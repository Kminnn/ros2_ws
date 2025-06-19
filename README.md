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
