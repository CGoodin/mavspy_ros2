# mavspy_ros2

The *mavspy_ros2* package is a ROS-2 package for interfacing the [MSU Autonomous Vehicle Simulator (MAVS)](https://www.mavsim.org/) to ROS-2. Instructions for installing MAVS and the mavspy-ros2 package and interfacing it with the [NATURE autonomy stack](https://github.com/CGoodin/nature-stack) are given below.

This instructions and demo are done on Ubuntu 22.04 using ROS2 Humble. These instructions that follow assume that you are already familiar with the Linux operating system, ROS2 workspaces, and Python. Your system must also have Python 3.10 installed. Once these criteria are met, it's easy to get started!

## Installing mavspy

First, navigate to your home directory, download the MAVS Python wheel, and install it using pip. 

```bash
cd
curl -L -O https://github.com/CGoodin/mavspy/releases/download/v1.0.38/mavspy-1.0.38-py3-none-linux_x86_64.whl
pip install mavspy-1.0.35-py3-none-linux_x86_64.whl
```

Note: The wheel file can be deleted after pip installation.

## Creating and Building the Workspace

After *mavspy* is installed, create a ROS2 workspace for building NATURE and mavspy_ros2. We'll also need to install the mavspy_ros2 package for interfacing MAVS to ROS2.

```bash
cd
mdkir mavs_ros2_ws
cd mavs_ros2_ws
mkdir src
```

Navigate to the workspaces src directory and clone the NATURE and mavspy_ros2 repositories:

```bash
cd ~/mavs_ros2_ws/src
git clone https://github.com/CGoodin/nature-stack
git clone https://github.com/CGoodin/mavspy_ros2.git
```

To build the workspace, go back to the top-level workspace directory and use the ROS2 build system.

```bash
cd ~/mavs_ros2_ws
colcon build
source install/setup.bash
```

## Running a Simulation

Now that it's built, we can run our first simulation! First, for convenience, set an environment variable setting the location of the MAVS data folder.

```bash
export MAVS_DATA=$(pip show mavspy | awk '/^Location:/ {print $2}')/mavspy/data
```

Next, launch a simulation using the following command.

```bash
ros2 launch mavspy_ros2 mavs_nature_sim.launch.py scene_file:=${MAVS_DATA}/scenes/cavs_proving_ground.json
vehicle_file:=${MAVS_DATA}/vehicles/rp3d_vehicles/mrzr4_tires_low_gear.json waypoints_file:=~/mavs_ros2_ws/src/nature/config/waypoints_proving_ground_enu.yaml robot_description_file:=~/mavs_ros2_ws/src/nature/config/example_bot.urdf init_x:=-132.0 init_y:=318.0 init_heading:=-0.75
```

This will launch the NATURE stack, a MAVS simulation node, and a visualization window for tracking the progress of the simulation. You can track the progress of the vehicle through the environment from the trajectory map on the left.