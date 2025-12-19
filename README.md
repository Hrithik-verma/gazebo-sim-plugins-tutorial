GAZEBO SIM PLUGINS TUTORIAL
--------------

Gazebo Sim standalone system plugins and Gazebo Sim ROS 2 plugins.
This repository accompanies a YouTube tutorial series on Gazebo Sim plugins.


YouTube Tutorial
----------------
Gazebo Sim Plugin Made Easy !!!

Video link: Coming soon


<br>
<br>

## What’s Inside

```text
gazebo-sim-plugins-tutorial/
├── standalone_gz_sim_plugins/
│   ├── plugin_name_1/              # Standalone Gazebo Sim system plugin
│   ├── plugin_name_2/              # Standalone Gazebo Sim system plugin
│   └── ...                         # More standalone plugins
│
├── ros2_ws/
│   ├── src/
│   │   ├── yt_tutorial_gazebo_ros/      # ROS 2 Gazebo launch & integration
│   │   └── tutorial_gazebo_plugins/     # ROS 2 Gazebo plugins
│   └── ...                               # ROS 2 build, install, log folders
│
└── README.md
```


1) standalone_gz_sim_plugins/ <br>
   Standalone Gazebo Sim system plugins (CMake build)
<br>

2) ros2_ws/<br>
   ROS 2 workspace containing Gazebo Sim ROS 2 plugins + launch files


## Supported Workflows
A) With Docker (recommended if you already have a prepared docker-compose environment)<br>
B) Without Docker (install ROS 2 + Gazebo Sim on your host machine)


## Prerequisites
Common:
- Git
- A Linux environment (Ubuntu recommended)

With Docker:
- Docker
- Docker Compose
- GUI support via X11 (xhost)

Without Docker:
- ROS 2 installed on host
- Gazebo Sim installed on host
- Build tools (CMake / make)
- colcon (for ROS 2 workspace)




<br>
<br>

## Setup

### 1) With Docker
NOTE:
- You need to clone this repo INSIDE the folder where docker-compose.yml exists (from your other docker repo).

Clone:
```bash
cd <inside_docker_repo_folder>
git clone https://github.com/Hrithik-verma/gazebo-sim-plugins-tutorial.git
```

Enter container (assuming container is already built/running):
```bash
xhost +                (required for GUI on host)
docker exec -it <container_name> bash
```

If you need to start the container first, do it from your docker repo as you normally do
(e.g., using docker compose up -d). This repo does not include docker-compose.yml.

<br>

### 2) Without Docker (Host System)
NOTE:
- Make sure you already have ROS 2 + Gazebo Sim installed on your system.
- The main difference vs Docker is simply where your X path points.

Clone:
```bash
git clone https://github.com/Hrithik-verma/gazebo-sim-plugins-tutorial.git
cd gazebo-sim-plugins-tutorial
```

<br>

## Build & Run

### Path Reference (IMPORTANT)

You will see <X> used below:

Without Docker:
  X = ```<path-to>/gazebo-sim-plugins-tutorial```

With Docker:
  (inside the container)
  X = ```/root```

<br>

### A) Standalone Gazebo Sim Plugins
1) Go to a standalone plugin folder:
```bash
cd <X>/standalone_gz_sim_plugins/<plugin_name>
```

2) Build:
```bash
mkdir build && cd build
cmake ..
make
cd ..
```

3) Export plugin path (so Gazebo Sim can find it):
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)/build
```

4) Launch Gazebo Sim with an SDF:
```bash
gazebo sim -v 4 <sdf_file_path>.sdf
```

Notes:
- "-v 4" prints debug logs (useful while developing)
- If Gazebo can’t find the plugin, re-check GZ_SIM_SYSTEM_PLUGIN_PATH and that build succeeded.

<br>
<br>

### B) ROS 2 Workspace (ros2_ws)

1) Go to the ROS 2 workspace:
```bash
cd <X>/ros2_ws
```

2) Build ROS 2 packages:
```bash
colcon build --symlink-install
```

3) Source the workspace:
```bash
source install/setup.bash
```

4) Launch:
```bash
ros2 launch yt_tutorial_gazebo_ros <launch_file>.launch.py
```

Notes:
- If launch fails, confirm package names exist and build completed successfully.
- Always source the workspace in every new terminal where you run ROS 2 commands.

<br>

## Troubleshooting

GUI / Gazebo window not opening (Docker):
- Ensure you ran: xhost +
- Ensure DISPLAY is correctly passed into container (depends on your docker setup)

Plugin not loading:
- Re-run export:
  export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)/build
- Confirm the plugin was built and the build folder contains the compiled library

ROS 2 packages not found:
- Ensure you ran:
  colcon build --symlink-install
  source install/setup.bash



## Author / Maintainer
Hrithik Verma <br>
[GitHub](https://github.com/Hrithik-verma) <br>
[LinkdIn](https://www.linkedin.com/in/hrithik-verma-35a155193) <br>
[Youtube](https://www.youtube.com/@RoboticsWithHrithik) <br>

