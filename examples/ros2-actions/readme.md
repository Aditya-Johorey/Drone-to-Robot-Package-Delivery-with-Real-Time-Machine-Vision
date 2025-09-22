# Instructions

Use this example for ROS 2 actions with the instructions on the Azure Wiki page at https://lseq.visualstudio.com/UoY%20Summer%20Robotics%20Project/_wiki/wikis/UoY-Summer-Robotics-Project.wiki/61/Actions-Tutorial

The 'drone_actions' and 'drone_interfaces' folders inside this ros2-actions folder should be placed inside the 'src' folder that's mentioned in the instructions on the Wiki page.

I have copied and pasted the information from the Wiki page below, although it is subject to change.

# Purpose
This tutorial goes over creating a basic structure for a ROS2 action and how to use it. This forms the basis of initiating the drone flight, getting the drone's position and finally getting the package's coordinates.

# Sources

## Learn about actions:
https://docs.ros.org/en/jazzy/Concepts/Basic/About-Actions.html
https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

# Structure

Two ROS2 packages will be created.

Package 1: drone_interfaces - Holds the action interface. Built with CMake.
Package 2: drone_actions - Holds the action client and action server code. Built with Python.

The drone action client is like the manager which tells the action server to perform its task. It will receive feedback (drone's x,y,z position) from the action server, and a final result (package x,y location) at the end.

The drone action server performs its action when triggered by the action client. It will send the drone's x,y,z position at regular intervals as feedback to the client. When the drone finds the correct package, it will return the package's x,y location to the action client.

# Prerequisites
- Ubuntu 24.04
- ROS2 Jazzy

# Disclaimer
The code that's created here should already be in a repo. Repo link - https://lseq.visualstudio.com/UoY%20Summer%20Robotics%20Project/_git/drone

# Create a ROS2 workspace
In a folder on your machine create a folder `ros2_ws`. ROS related things will be located here.
Create a folder inside ros2_ws called `src`. This is where the ROS2 packages will be created.

# Create packages
Go inside the `/ros2_ws/src` folder

## drone_interfaces package
Create the drone_interfaces package:
```
ros2 pkg create drone_interfaces --build-type ament_cmake
```

This should generate a few folders and files.

### Action file
Go into the new `drone_interfaces` folder and create a new folder called `action`.
In the `action` folder, create a file called `DroneMission.action`

A .action file needs to follow this format:

```
Goal variables
---
Result variables
---
Feedback variables
```
For the drone mission, a suitable action file can contain

```
bool start_mission
---
int16 package_x
int16 package_y
---
int16 drone_x
int16 drone_y
int16 drone_z
```

### CMakeLists.txt file
Edit the `CMakeLists.txt` file by adding this content somewhere before the `ament_package()` line

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DroneMission.action"
)
```
### package.xml file
Edit the `package.xml` file by adding this content somewhere before the <export> tags

```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
## drone_actions package
Go inside the `/ros2_ws/src` folder in the terminal.

Create the drone_actions package:
```
ros2 pkg create drone_actions --build-type ament_python --dependencies rclpy drone_interfaces
```

This should generate a few folders and files.

### Server and client files

Go into the `/drone_actions/drone_actions` folder and create:
- The action server file - `drone_mission_server.py`
- The action client file - `drone_mission_client.py`

The code for the files is located in the repo - https://lseq.visualstudio.com/UoY%20Summer%20Robotics%20Project/_git/drone

### Update the setup.py file

The `setup.py` file lets us specify a way to run the files. Include the server and client files in the entry_points

```
entry_points={
    'console_scripts': [
        'drone_mission_server = drone_actions.drone_mission_server:main',
        'drone_mission_client = drone_actions.drone_mission_client:main'
    ]
}
```

# Build the projects

Go to the ROS2 workspace root at `/ros2_ws` and build the projects with
```
colcon build
source install/setup.bash
```
# Run the action

Run the server and the client in different terminals

Ensure that you source the `install/setup.bash` file in each new terminal window
```
source install/setup.bash
```

## Run the drone mission server
```
ros2 run drone_actions drone_mission_server
```

## Run the drone mission client
```
ros2 run drone_actions drone_mission_client
```


# Using the actual drone code

The simulated drone code is created at /ros2_ws/src/drone_actions/src/drone_code.py
This is imported into the file at /ros2_ws/src/drone_actions/drone_actions/drone_mission_server.py

You could do something similar, i.e. put your code in the /ros2_ws/src/drone_actions/src/ folder and import it into drone_mission_server.py
