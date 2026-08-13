# F1TENTH gym environment ROS2 communication bridge
This is a containerized ROS communication bridge for the F1TENTH gym environment that turns it into a simulation in ROS2.

# Installation

**Supported Systems:**

- Ubuntu native (tested on 22.04, and 24.04) with ROS 2.
- Windows 10/11, macOS, and Ubuntu with an NVIDIA gpu and nvidia-docker2 support.
- Windows 10/11, macOS, and Ubuntu without an NVIDIA gpu (using noVNC).

This installation guide will be split into instruction for installing the ROS 2 package natively, and for systems with or without an NVIDIA gpu in Docker containers.

## Native on Ubuntu 22.04 (Recommended)
Setup a native Ubuntu 22.04 either on your own machine (using dual boot or main OS) or using a virtual machine with your favorite virtualization software ([VMWare](https://www.vmware.com/products/desktop-hypervisor/workstation-and-fusion) recommended).

### Step-by-step install (fresh workspace)
1. **Install ROS 2 Humble.** Follow the instructions [here](https://docs.ros.org/en/humble/Installation.html).
2. **Create the workspace and a venv inside it:**
   ```bash
   mkdir -p $HOME/sim_ws/src
   python3 -m venv --system-site-packages $HOME/sim_ws/.venv
   source $HOME/sim_ws/.venv/bin/activate
   python3 -m pip install -U pip
   ```
3. **Clone `f1tenth_gym_ros` into the workspace (dev-humble):**
   ```bash
   cd $HOME/sim_ws/src
   git clone -b dev-humble https://github.com/f1tenth/f1tenth_gym_ros.git
   ```
4. **Clone `f1tenth_gym` inside `f1tenth_gym_ros` (dev-humble):**
   ```bash
   cd $HOME/sim_ws/src/f1tenth_gym_ros
   git clone -b dev-humble https://github.com/f1tenth/f1tenth_gym.git
   ```
   If the `f1tenth_gym` directory already exists (some clones include it), skip the clone.
5. **Install `f1tenth_gym` (follow its README):**
   - With the venv already active, the minimal pip install is:
     ```bash
     cd $HOME/sim_ws/src/f1tenth_gym_ros/f1tenth_gym
     pip install -e .
     ```
6. **Install ROS dependencies and build the workspace:**
   ```bash
   source /opt/ros/humble/setup.bash
   cd $HOME/sim_ws
   rosdep install -i --from-path src --rosdistro humble -y
   colcon build
   ```

Once you're done and everything is installed, skip to the [Launching the Simulation](#launching-the-simulation) section below.
## Docker ##
(Alternative install, skip if you have already used the native install above)

Visualization uses Foxglove in your host browser, so no GPU passthrough or display forwarding is needed.

**Install [Docker Engine](https://docs.docker.com/engine/install/)** (includes the `docker compose` plugin). If you follow the post-installation steps you won't have to prepend your docker commands with sudo.

1. Clone this repo
2. Build the image and bring up the sim container:
```bash
cd f1tenth_gym_ros
docker compose up -d --build
```
This mounts the repo at `/sim_ws/src/f1tenth_gym_ros` (changes you make on the host also reflect in the container) and publishes the Foxglove bridge websocket on port 8765. `f1tenth_gym` is cloned and installed inside the image at `/sim_ws/f1tenth_gym`.

Build args (passed with `docker compose build --build-arg ...` or `docker build --build-arg ...`):
- `ROS_DISTRO`: ROS 2 distro to build for, `humble` (default) or `jazzy`.
- `GYM_REF`: `f1tenth_gym` branch or tag to install, defaults to `dev-humble`.

`ROS_DISTRO=lyrical` (Ubuntu 26.04) is expected to work as soon as Nav2 is released for Lyrical — as of August 2026 `ros-lyrical-nav2-map-server` is not yet available, which is the only missing dependency (the Python stack installs cleanly on 26.04).

3. In a separate terminal, run the following, and you'll have a bash session in the simulation container — with ROS 2 and the workspace already sourced. `tmux` is available for convenience.
```bash
docker exec -it f1tenth_gym_ros-sim-1 /bin/bash
```
4. Launch the sim inside the container (`open_foxglove:=false` because the container cannot open a browser):
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py open_foxglove:=false
```
5. On the host, open Foxglove and connect to `ws://localhost:8765`: [https://app.foxglove.dev/?ds=foxglove-websocket&ds.url=ws://localhost:8765](https://app.foxglove.dev/?ds=foxglove-websocket&ds.url=ws://localhost:8765)

### RViz fallback (noVNC)

If you prefer RViz over Foxglove, a noVNC service can forward the display to your browser:
```bash
docker compose --profile novnc up -d
```
Then run `rviz2 -d src/f1tenth_gym_ros/config/rviz/gym_bridge.rviz` inside the sim container and navigate to [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html).

# Launching the Simulation

1. `tmux` is included in the contianer, so you can create multiple bash sessions in the same terminal.
2. To launch the simulation, make sure you source the venv, the ROS2 setup script, and the local workspace setup script. Run the following in the bash session from the container:
```bash
source $HOME/sim_ws/.venv/bin/activate
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
By default, Foxglove auto-opens and preselects the websocket connection (`ws://localhost:8765`).

If Foxglove does not auto-open (for example in headless/container setups), open it manually:
- Browser: [https://app.foxglove.dev/?ds=foxglove-websocket&ds.url=ws://localhost:8765](https://app.foxglove.dev/?ds=foxglove-websocket&ds.url=ws://localhost:8765)
- Studio: `foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:8765`

To visualize the simulation, import the layout file `config/foxglove/gym_bridge_foxglove.json`. It ships with robot descriptions for eight cars (the ego plus seven opponents), matching the start poses the shipped `sim.yaml` carries. To visualize even more agents, add a description for each extra opponent in the 3D panel's settings, subscribing to `/opp_robot_description8`, `/opp_robot_description9`, and so on. Foxglove is the recommended setup, but if you prefer RViz (old Gym setup), use `config/rviz/gym_bridge.rviz`.

You can then run another node by creating another bash session in `tmux` or a separate terminal.

# Configuring the simulation
- The configuration file for the simulation is at `f1tenth_gym_ros/config/sim.yaml`.
- Topic names and namespaces can be configured but is recommended to leave uncahnged.
- The map can be changed via the `map_path` parameter. It can be a package-relative path like `maps/levine` or a built-in gym track name like `Spielberg`. The map follows the ROS convention; the image file and the `yaml` file should live together.
- The `num_agent` parameter sets how many cars to simulate. The first agent is the ego car, all further agents are opponents, and a robot model is spawned for every opponent. There is no upper limit; you are bounded by how many start poses you define and by how fast your machine can step the physics. Note that the shipped Foxglove layout only draws the first four cars, so past that you also need to add a robot description panel per extra opponent. You can either set it in `sim.yaml`, or override it at launch time without editing the config:
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py num_agent:=4
```
- The ego and opponent starting poses can also be changed via parameters (`sx`/`sy`/`stheta` for the ego, `sx1`/`sy1`/`stheta1` onwards for the opponents), these are in the global map coordinate frame. Every agent you ask for needs a start pose: if `num_agent` is 3 but `sx2`/`sy2`/`stheta2` are missing, the bridge stops and tells you which parameters to add rather than guessing a spot on the map for you. The poses shipped in `sim.yaml` line up eight cars on the Levine map (two rows of four), so anything up to `num_agent:=8` works out of the box; add `sx8`/`sy8`/`stheta8` and so on to race more, and change them when you change the map.
- A different sim config can be selected at launch time. The value is a file name in `config/`, a package relative path, or an absolute path:
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py config:=my_sim.yaml
```
- Foxglove launch behavior can be configured in `sim.yaml` under `foxglove.ros__parameters`:
  - `open_foxglove`: `True` or `False`
  - `target`: `'browser'` or `'studio'`
- You can also override these at launch time:
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py open_foxglove:=false
ros2 launch f1tenth_gym_ros gym_bridge_launch.py foxglove_target:=studio
```

The entire directory of the repo is mounted to a workspace `/sim_ws/src` as a package. All changes made in the repo on the host system will also reflect in the container. After changing the configuration, run `colcon build` again in the container workspace to make sure the changes are reflected.

# Topics published by the simulation

In **single** agent:

`/scan`: The ego agent's laser scan

`/ego_racecar/odom`: The ego agent's odometry

`/map`: The map of the environment

A `tf` tree is also maintained.

With **multiple** agents (2-4):

In addition to the topics available in the single agent scenario, these topics are also available for each opponent. Opponent namespaces and topics carry the opponent's index as a suffix — the first opponent has no suffix (identical to the previous two-agent setup), the second and third use `2` and `3`:

`/opp_scan`, `/opp_scan2`, `/opp_scan3`: The opponent agents' laser scans

`/ego_racecar/opp_odom`, `/ego_racecar/opp_odom2`, `/ego_racecar/opp_odom3`: The opponent agents' odometry for the ego agent's planner

`/opp_racecar/odom`, `/opp_racecar2/odom`, `/opp_racecar3/odom`: The opponent agents' odometry

`/opp_racecar/opp_odom`, `/opp_racecar2/opp_odom`, `/opp_racecar3/opp_odom`: The ego agent's odometry for each opponent agent's planner

# Topics subscribed by the simulation

In **single** agent:

`/drive`: The ego agent's drive command via `AckermannDriveStamped` messages

`/initalpose`: This is the topic for resetting the ego's pose via RViz's Foxglove's 2D Pose Estimate tool.

With **multiple** agents (2-4):

In addition to all topics in the single agent scenario, these topics are also available:

`/opp_drive`, `/opp_drive2`, `/opp_drive3`: The opponent agents' drive commands via `AckermannDriveStamped` messages. Note that each car only moves when something publishes to its own drive topic.

`/goal_pose`, `/goal_pose2`, `/goal_pose3`: These are the topics for resetting an opponent agent's pose. RViz's or Foxglove's 2D Goal Pose tool publishes to `/goal_pose` (first opponent).

# Keyboard Teleop

The keyboard teleop node from `teleop_twist_keyboard` is also installed as part of the simulation's dependency. To enable keyboard teleop, set `kb_teleop` to `True` in `sim.yaml`. After launching the simulation, in another terminal, with ROS sourced, run:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Then, press `i` to move forward, `u` and `o` to move forward and turn, `,` to move backwards, `m` and `.` to move backwards and turn, and `k` to stop in the terminal window running the teleop node.

# Developing and creating your own agent in ROS 2

There are multiple ways to launch your own agent to control the vehicles.

- The first one is creating a new package for your agent in the `/sim_ws` workspace inside the sim container. After launch the simulation, launch the agent node in another bash session while the sim is running.
- The second one is to create a new ROS 2 container for you agent node. Then create your own package and nodes inside. Launch the sim container and the agent container both. With default networking configurations for `docker`, the behavior is to put The two containers on the same network, and they should be able to discover and talk to each other on different topics. If you're using noVNC, create a new service in `docker-compose.yml` for your agent node. You'll also have to put your container on the same network as the sim and novnc containers.


## FAQ & Debugging
### I have Python < 3.9
The current `f1tenth_gym` requires Python 3.9+. Use Ubuntu 22.04 with ROS 2 Humble or update your Python environment to 3.9+.

### This package is managed externally, PEP 668
You are trying to install the package using the system python. This is outdated and not recommended as per PEP 668. Please ensure you install `f1tenth_gym` inside a virtual environment as instructed with `.venv` above.

### Pyqt6 6.10 cached, fails to install
In rare cases, you might have a newer cached version of pyqt6 which breaks the .toml install. To resolve this, first install pyqt6 first using ```pip3 install pyqt6==6.7.1``` and then install the f1tenth_gym using ```pip3 install -e .```.

### Gym install hangs on PyQt6>6.7.1 installation
This has been documented happening on VMWare Fusion for Mac. This stems from using a server image of Ubuntu 22.04. Specifically, this happens because PyQt6 prompts you to accept its GPL license which you can not see/accept from a standard pip install. Please resort to installing PyQt6 6.7.1 with license as that is the maximum supported version for the Ubuntu 22.04 Arm Server Image. To resolve this, first install pyqt6 first using ```pip3 install pyqt6==6.7.1 --config-settings --config-license= --verbose``` and then install the f1tenth_gym using ```pip3 install -e .```.

### AttributeError: module 'coverage' has no attribute 'types'
This is due to an outdated coverage package. To resolve this, run ```pip3 install --upgrade coverage```. The minimum coverage version required is 7.6.1.

### ImportError: cannot import name 'Transpose' from 'PIL.Image'
This is due to an outdated pillow version. To resolve this, run ```pip3 install --upgrade pillow```. The minimum pillow version required is 9.1.0.

### ValueError: numpy.dtype size changed, may indicate binary incompatibility
This is due to an outdated scipy version. To resolve this, run ```pip3 install --upgrade scipy```. The minimum scipy version required is 1.13.0.

### "opencv>=3. invalid" error on pip install
This indicates that you have outdated pip, wheel or setuptools. To resolve this, you can run ```python3 -m pip install --upgrade pip wheel setuptools```. This will upgrade all tools used by python when pip installing packages.
