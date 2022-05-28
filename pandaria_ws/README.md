## pandaria_ws

First, download required dependencies by running `wstool update -t src/` from this directory
(might need to install `wstool` first)

To compile the project run `catkin_make` from this directory

To keep the workspace clean, ROS wiki recommend the following structure
```
workspace_folder/        -- WORKSPACE (in our case pandaria_ws)
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
```

Before creating a new package or modifying the code, **please** create a new branch and 
try out your stuff, once your branch compiles without errors and ideally it does what
you want create a pull request so that any team member can review your code. Ideally
the other member can clone, compile, and run it in his pc. Once this is done, it can 
be merged into master. This is a tedious process but that way the master branch will 
always compile and everybody trying new stuff will only find bugs in their own new code.

To create a package go **inside** the `pandaria_ws/src/` folder and use
the following command:

`catkin_create_pkg <package_name> [depend1] [depend2] [depend3]`

where `[depend1]` are dependencies such as std_msgs, rospy, roscpp, etc. 


