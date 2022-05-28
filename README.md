# Pandaria 2.0
Robothon Grand Challenge 2022 submission of Team Pandaria 2.0

## Equipment used
- Ubuntu 18 PC
- Franka Emika Panda Robot + Controller
- Franka Hand
- 3D printed Gripper (PLA) with thin foam surface
- realsense D435 camera
- 3D printed camera mount (PLA)
- table with white surface
- taskboard

## Software dependency list
- ROS melodic
- roscpp Package
- rospy Package
- Franka ROS
- openCV Python
- Numpy
- SciPy
- Eigen Library
- Panda MoveIt Package
- TF2 Package

## Quick start guide
### Installation
- setup hardware on a table with a white surface
- clone repository on Ubuntu 18 machine
- install software dependencies

### Starting a trial
- initialize Franka hand and open brakes in Franka Control Interface
- source and build workspace
  - inside workspace: `$ source devel/setup.bash`
  - `$ catkin build`
- launch program with: `$ roslaunch pandaria_ros bringup.launch robot_ip:=<ROBOT-IP>`
