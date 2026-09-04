# F1TENTH gym environment ROS 2 communication bridge
This is a ROS 2 bridge for the [F1TENTH gym environment](https://github.com/f1tenth/f1tenth_gym) that turns it into a simulation in ROS 2. The bridge steps the gym's physics, publishes LiDAR scans, odometry, collision and lap-counter topics for every car, and accepts Ackermann drive commands.

# Installation

**Supported systems:**

- Ubuntu 24.04 with ROS 2 Jazzy, natively (recommended).
- Windows 10/11, macOS and Linux through Docker or a VM with Ubuntu 24.04, with Foxglove in the host browser (no GPU or display forwarding needed).

The gym requires Python 3.12 or newer, which is what Ubuntu 24.04 ships with.

## Native on Ubuntu 24.04 (recommended)
Set up a native Ubuntu 24.04 either on your own machine (dual boot or main OS) or in a virtual machine with your favourite virtualization software ([VMWare](https://www.vmware.com/products/desktop-hypervisor/workstation-and-fusion) recommended).

### Step-by-step install (fresh machine)
1. **Install ROS 2 Jazzy.** Follow the instructions [here](https://docs.ros.org/en/jazzy/Installation.html) (the `ros-jazzy-desktop` package).
2. **Install the system packages the gym needs.** Git, the Python venv module, and the runtime libraries for the gym's Qt/OpenGL renderer:
   ```bash
   sudo apt update
   sudo apt install -y git python3-venv python3-pip \
     libxcb-cursor0 libxcb-icccm4 libxcb-keysyms1 libxcb-shape0 libxcb-randr0 \
     libxcb-render-util0 libxcb-xinerama0 libxkbcommon-x11-0 \
     libgl1 libegl1 libopengl0 libgl1-mesa-dri
   ```
3. **Create the workspace and a venv inside it.** The venv sees the system ROS packages (`--system-site-packages`) and receives the gym and its Python dependencies:
   ```bash
   mkdir -p $HOME/sim_ws/src
   python3 -m venv --system-site-packages $HOME/sim_ws/.venv
   source $HOME/sim_ws/.venv/bin/activate
   python3 -m pip install -U pip
   ```
4. **Clone `f1tenth_gym_ros` into the workspace** (branch `dev-jazzy`):
   ```bash
   cd $HOME/sim_ws/src
   git clone -b dev-jazzy https://github.com/f1tenth/f1tenth_gym_ros.git
   ```
5. **Clone `f1tenth_gym` inside `f1tenth_gym_ros`** (branch `dev-jax`):
   ```bash
   cd $HOME/sim_ws/src/f1tenth_gym_ros
   git clone -b dev-jax https://github.com/f1tenth/f1tenth_gym.git
   ```
6. **Install `f1tenth_gym` into the venv.** With the venv still active, an editable install is required:
   ```bash
   pip install -e $HOME/sim_ws/src/f1tenth_gym_ros/f1tenth_gym
   ```
   This installs the CPU build of JAX, which is all the bridge uses. Do **not** use `uv sync` from the gym's own README for the ROS setup: it creates a separate environment that cannot see ROS.
7. **Check the install:**
   ```bash
   python -c "import numpy, rclpy, f1tenth_gym.envs; print('ok, numpy', numpy.__version__)"
   ```
   It should print `ok` with a NumPy 2.x version.
8. **Install ROS dependencies and build the workspace:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   cd $HOME/sim_ws
   rosdep install -i --from-path src --rosdistro jazzy -y
   colcon build --symlink-install
   ```

Once everything is installed, skip to [Launching the Simulation](#launching-the-simulation) below.

### Optional: GPU JAX for your own training code
The gym's batched reinforcement-learning API can use an NVIDIA GPU. That is separate from the bridge, which pins its physics to the CPU. If you want it, install the extra into the same venv (about 3 GB, needs an NVIDIA driver for CUDA 13):
```bash
pip install -e "$HOME/sim_ws/src/f1tenth_gym_ros/f1tenth_gym[cuda]"
```
The bridge keeps running on the CPU and holds no GPU memory. See the gym's [RL guide](https://f1tenth-gym.readthedocs.io/en/latest/rl.html) for the training side.

## Docker 
Only if you do not have access to a native Ubuntu install or an Ubuntu VM.

Visualization uses Foxglove in your host browser, so no GPU passthrough or display forwarding is needed.

**Install [Docker Engine](https://docs.docker.com/engine/install/)** (includes the `docker compose` plugin). If you follow the post-installation steps you won't have to prepend your docker commands with sudo.

1. Clone this repo.
2. Build the image and bring up the sim container:
```bash
cd f1tenth_gym_ros
docker compose up -d --build
```
This mounts the repo at `/sim_ws/src/f1tenth_gym_ros` (changes you make on the host also reflect in the container) and publishes the Foxglove bridge websocket on port 8765. `f1tenth_gym` is cloned and installed inside the image at `/sim_ws/f1tenth_gym`, CPU-only. The image also carries `xvfb`, so the gym's own examples run headless inside it: `xvfb-run -a python3 /sim_ws/f1tenth_gym/examples/waypoint_follow.py`.

Build args (passed with `docker compose build --build-arg ...` or `docker build --build-arg ...`):
- `ROS_DISTRO`: ROS 2 distro to build for, `jazzy` (default).
- `GYM_REF`: `f1tenth_gym` branch or tag to clone, defaults to `dev-jax`.

`ROS_DISTRO=lyrical` (Ubuntu 26.04) is expected to work as soon as Nav2 is released for Lyrical; as of August 2026 `ros-lyrical-nav2-map-server` is not yet available, which is the only missing dependency.

3. In a separate terminal, run the following, and you'll have a bash session in the simulation container with ROS 2 and the workspace already sourced. `tmux` is available for convenience.
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

To launch the simulation, source the venv, the ROS 2 setup script and the workspace setup script, then launch. Natively:
```bash
source $HOME/sim_ws/.venv/bin/activate
source /opt/ros/jazzy/setup.bash
source $HOME/sim_ws/install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
In the Docker container the sourcing is already done for you, and `tmux` is included for your convenience so you can open multiple bash shells in the same terminal.

The first scan appears about four seconds after launch: the gym compiles its contact and LiDAR kernels on the first simulation step.

By default, Foxglove auto-opens and preselects the websocket connection (`ws://localhost:8765`). This can be changed (via argument or through changing the config `.yaml`)

If Foxglove does not auto-open (for example in headless/container setups), open it manually:
- Browser: [https://app.foxglove.dev/?ds=foxglove-websocket&ds.url=ws://localhost:8765](https://app.foxglove.dev/?ds=foxglove-websocket&ds.url=ws://localhost:8765)
- Studio: `foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:8765`

To visualize the simulation, import the layout file `config/foxglove/gym_bridge_foxglove.json`. It ships with robot descriptions for eight cars (the ego plus seven opponents), matching the start poses the shipped `sim.yaml` carries, plus a lap-count gauge and a lap-time readout for the ego car. To visualize even more agents, add a description for each extra opponent in the 3D panel's settings, subscribing to `/opp_robot_description8`, `/opp_robot_description9`, and so on. Foxglove is the recommended setup, but if you prefer RViz (old Gym setup), use `config/rviz/gym_bridge.rviz`.

You can then run another node in another shell (`tmux` or a separate terminal).

# Configuring the simulation
- The configuration file for the simulation is at `f1tenth_gym_ros/config/sim.yaml`.
- Topic names and namespaces can be configured but it is recommended to leave them unchanged.
- The map can be changed via the `map_path` parameter. It can be a package-relative path like `maps/levine` or a built-in gym track name like `Spielberg`. The map follows the ROS convention: the image file and the `yaml` file should live together. Lap counting additionally needs a `<map>_centerline.csv` next to the yaml (optionally a `<map>_raceline.csv` too), in the format the gym's tracks use. `maps/Spielberg` ships both. The levine maps have none, so their lap counters stay at 0.
- The `num_agents` parameter configures how many cars to simulate. The first agent is the ego car, all additional agents are opponents, and a robot model is spawned for every opponent. There is no upper limit; you are bounded by how many start poses you define and by how fast your machine can step the physics. The shipped Foxglove layout draws eight cars; past that, add a robot description panel per extra opponent. You can either set it in `sim.yaml`, or override it at launch time without editing the config:
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py num_agents:=4
```
- Every pose the bridge exchanges refers to `base_link`, which is the rear axle (as in the URDF and on the real car): the `sx`/`sy`/`stheta` parameters, the poses set through Foxglove's pose tools, and the odometry and `tf` it publishes. The LiDAR sits `lidar_base_link_to_lidar_tf` ahead of it (0.275 m by default), matching the laser frame in `tf`. Internally the gym integrates the car's centre of gravity, `lr` ahead of the rear axle; the bridge converts.
- The ego and opponent starting poses can also be changed via parameters (`sx`/`sy`/`stheta` for the ego, `sx1`/`sy1`/`stheta1` onwards for the opponents), these are in the global map coordinate frame. Every agent you spawn needs a start pose: if `num_agents` is 3 but `sx2`/`sy2`/`stheta2` are missing, the bridge will error out informing you which parameters are missing. The poses shipped in `sim.yaml` line up eight cars on the Levine map (two rows of four), so anything up to `num_agents:=8` works out of the box; add `sx8`/`sy8`/`stheta8` and so on to race more, and change them when you change the map. For `maps/Spielberg`, the start of the raceline is `sx: -0.04`, `sy: -0.85`, `stheta: 3.40`.
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

After changing the configuration, run `colcon build` again in the workspace to make sure the changes are reflected (in the container, the repo is mounted at `/sim_ws/src/f1tenth_gym_ros`, so edits on the host show up inside).

# Topics published by the simulation

In **single** agent:

`/scan`: The ego agent's laser scan

`/ego_racecar/odom`: The ego agent's odometry

`/ego_racecar/collision`: `std_msgs/Bool`, true while the sim reports the ego agent in contact with a wall or another car. It is an instantaneous flag, not latched: it flickers while the car grinds along a wall and clears once the car backs off or is reset, so latch it yourself if you need "ever collided".

`/ego_racecar/lap_count`: `std_msgs/Int32`, completed laps. `/ego_racecar/lap_time`: `std_msgs/Float32`, the last completed lap's time in seconds. Laps are only counted on maps that ship a `<map>_centerline.csv` next to the yaml (`maps/Spielberg` does; the levine maps do not, so both stay 0 there). The spawn-to-finish-line stretch is an out lap: the first crossing starts the clock, so lap 1 and every lap time are full circuits. The bridge also logs each completed lap with its time.

`/map`: The map of the environment

A `tf` tree is also maintained.

With **multiple** agents:

In addition to the topics available in the single agent scenario, these topics are also available for each opponent. Opponent namespaces and topics carry the opponent's index as a suffix: the first opponent has no suffix, the second and third use `2` and `3`, and so on:

`/opp_scan`, `/opp_scan2`, `/opp_scan3`: The opponent agents' laser scans

`/ego_racecar/opp_odom`, `/ego_racecar/opp_odom2`, `/ego_racecar/opp_odom3`: The opponent agents' odometry for the ego agent's planner

`/opp_racecar/odom`, `/opp_racecar2/odom`, `/opp_racecar3/odom`: The opponent agents' odometry

`/opp_racecar/opp_odom`, `/opp_racecar2/opp_odom`, `/opp_racecar3/opp_odom`: The ego agent's odometry for each opponent agent's planner

`/opp_racecar/collision`, `/opp_racecar2/collision`, `/opp_racecar3/collision`: Collision state for each opponent agent, same semantics as `/ego_racecar/collision`

`/opp_racecar/lap_count`, `/opp_racecar2/lap_count`, ... and `/opp_racecar/lap_time`, ...: Lap counter for each opponent agent, same semantics as the ego topics

# Topics subscribed by the simulation

In **single** agent:

`/drive`: The ego agent's drive command via `AckermannDriveStamped` messages

`/initialpose`: This is the topic for resetting the ego's pose via RViz's or Foxglove's 2D Pose Estimate tool.

With **multiple** agents:

In addition to all topics in the single agent scenario, these topics are also available:

`/opp_drive`, `/opp_drive2`, `/opp_drive3`: The opponent agents' drive commands via `AckermannDriveStamped` messages. Note that each car only moves when something publishes to its own drive topic.

`/goal_pose`, `/goal_pose2`, `/goal_pose3`: These are the topics for resetting an opponent agent's pose. RViz's or Foxglove's 2D Goal Pose tool publishes to `/goal_pose` (first opponent).

# Keyboard Teleop

The keyboard teleop node from `teleop_twist_keyboard` is installed as part of the simulation's dependencies. Keyboard teleop is enabled by default (`kb_teleop` in `sim.yaml`). After launching the simulation, in another terminal with ROS and the workspace sourced, run:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Then, press `i` to move forward, `u` and `o` to move forward and turn, `,` to move backwards, `m` and `.` to move backwards and turn, and `k` to stop in the terminal window running the teleop node.

# Developing and creating your own agent in ROS 2

There are multiple ways to launch your own agent to control the vehicles.

- Natively: create a new package for your agent in the `sim_ws` workspace, build it, and launch it in another terminal while the sim is running. Your node subscribes to `/scan` and `/ego_racecar/odom` and publishes `AckermannDriveStamped` messages on `/drive`.
- In Docker: either create your package inside the sim container's `/sim_ws` workspace, or create a separate ROS 2 container for your agent node. With default `docker` networking, the two containers are on the same network and discover each other's topics. If you're using noVNC, create a new service in `docker-compose.yml` for your agent node on the same network as the sim and novnc containers.

Python nodes that use the gym's environment directly should run from the workspace venv, which is what the bridge's own entry point does.

## FAQ & Debugging

### The gym refuses to install: requires Python >= 3.12
This version of `f1tenth_gym` needs Python 3.12 or newer. Ubuntu 24.04 with ROS 2 Jazzy ships with it. On Ubuntu 22.04 / ROS 2 Humble we recommend you use the `dev-humble` branches of both `f1tenth_gym_ros` and `f1tenth_gym` instead.

### This package is managed externally, PEP 668
You are trying to install the package using the system python. Install `f1tenth_gym` inside the virtual environment as instructed with `.venv` above.

### Could not load the Qt platform plugin "xcb"
Also printed as "xcb-cursor0 or libxcb-cursor0 is needed to load the Qt xcb platform plugin". Install the system libraries from step 2 of the native install, in particular `libxcb-cursor0`. Only the gym's own render window needs them; the bridge itself does not open a window.

### "An NVIDIA GPU may be present on this machine, but a CUDA-enabled jaxlib is not installed"
Harmless. It means JAX is the CPU build, which is the intended setup for the bridge. The bridge pins JAX to the CPU and silences the notice; you will only see it when running the gym's own examples. Set `JAX_PLATFORMS=cpu` in your shell to silence it there too.

### A module compiled using NumPy 1.x cannot be run in NumPy 2.x
The gym needs NumPy 2, which the venv installs on top of the system NumPy 1.26 from apt. The bridge and standard ROS Python packages are fine with that, but apt-installed compiled extensions such as `python3-cv-bridge` or apt `pandas` are not. Install such packages with `pip` inside the venv instead of using the apt copies.

### The old `num_agent:=N` launch argument
The parameter and launch argument were renamed to `num_agents` (plural, matching the gym). The launch file refuses the old spelling with a message rather than silently running a single car.

### Foxglove bridge: "Couldn't initialize websocket server: Bind Error"
Another bridge is already running on this machine and holds port 8765. Stop it first; only one sim can run at a time.

### Permission denied on `__pycache__` after using Docker
The compose bind mount runs as root inside the container and can leave root-owned `__pycache__` folders in the repo. `sudo chown -R $USER:$USER` the repository directory to fix it.

### I want to run the gym's own test suite
It is not needed to use the simulator. It needs well over 9 GB of free RAM in one process; on a laptop with a desktop session open it will get killed. Run it in CI or on a machine with nothing else open.
