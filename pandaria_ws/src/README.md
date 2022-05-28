## ROS packages


### pandaria_ros
#### Instructions
- clone the whole repo in the src folder of your ros-ws
- copy the file `.rosinstall` into the src folder of your ros-ws
- execute `wstool update -t src/` to download dependencies (if you add new dependencies add them to the `.rosinstall` file)
- **enable optimizations** (`catkin config --append-args -DCMAKE_BUILD_TYPE=Release`)
- build `catkin build`

#### Launch

`roslaunch pandaria_ros bringup.launch robot_ip:=...`

### Software Structure

We want to use two main ROS-Nodes:
- one for visually localizing the taskboard
- one for scheduling & executing (planning trajectory, ...) the different tasks (here, refered to as *TaskCoordinator*)

Additionally, we will probably use multiple low-level controllers for the arm and one controller for the hand. Each controller is a ROS-node.

We need multiple low-level arm controllers because we may want to implement custom control laws for certain tasks. These controllers can not run simultaneously and must be switched on and off by the current task (possible via service-calls to the controller manager). Note that the controllers are all loaded at the same time but *merely* are not running at the same time.

#### Task Configuration File (pandaria_ros/config/tasks.yaml)
This file contains task-specific parameters, such as position of a button relative to the taskboard centre. Additionally, the order in which the tasks are listed in this file determines the order in which they are to be executed.

#### TaskCoordinator

The *TaskCoordinator*:
- loads the MoveIt Planner instance
- reads the tasks.yaml (= task configuration) file, and creates the required task executers
- creates ROS-action-clients/ROS-service-clients/ROS-publishers which call the actions for the low-level controllers
- finally, executes each task defined in the order by the task configuration file

Task executers:
- For each task (e.g. button pressing task) there is a certain object responsible for executing this task (e.g. for the button task it is called *TaskButton*)
- These objects are created by the *TaskCoordinator* and get their parameters from the task configuration file
- To execute a task, the *TaskCoordinator* calls the `.execute(...)` method of the respective task executer