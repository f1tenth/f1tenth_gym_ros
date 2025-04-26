# F1 TENTH SIMULATOR SETUP ROS2 HUMBLE

```markdown
# F1TENTH Gym ROS 2 Simulator (Humble + Virtualenv Setup)

This repository provides a ROS 2 (Humble) wrapper around the F1TENTH Gym simulator.  
It connects an OpenAI Gym environment to ROS 2 nodes for developing and testing F1TENTH autonomous driving controllers.

## Installation Instructions (Ubuntu 22.04)

### 1. Setup Python Virtual Environment (Recommended)

Create a Python 3.10 virtual environment to avoid dependency conflicts:

```bash
sudo apt update
sudo apt install python3.10-venv
python3.10 -m venv ~/f1tenth_env
source ~/f1tenth_env/bin/activate
pip install --upgrade pip setuptools
```

You will work inside this `f1tenth_env` virtual environment when running the simulator.

---

### 2. Install and Patch OpenAI Gym (Version 0.19.0)

Clone and patch OpenAI Gym manually:

```bash
cd ~
git clone https://github.com/openai/gym.git
cd gym
git checkout 0.19.0
```

Edit `setup.py`:
- Remove the line `tests_require=["pytest", "mock"]`.
- Fix any invalid extras (for example, change `opencv-python>=3.` to `opencv-python>=3.0.0`).

Then install:

```bash
pip install -e .
```

---

### 3. Install Additional Python Dependencies

Install system-wide SWIG (required for building Box2D bindings):

```bash
sudo apt install swig
```

Then install Python dependencies:

```bash
pip install numpy==1.23.5 box2d-py transforms3d pyglet==1.4.11 scipy==1.11.4
```

---

### 4. Install F1Tenth Gym

Clone and install the F1Tenth Gym environment:

```bash
cd ~
git clone https://github.com/f1tenth/f1tenth_gym.git
pip install -e ./f1tenth_gym
```

---

### 5. Setup ROS 2 Workspace

Create the workspace:

```bash
mkdir -p ~/f1ws/src
cd ~/f1ws/src
git clone https://github.com/siddarth09/f1tenth_gym_ros_humble.git
```

---

### 6. Update Map Path in sim.yaml

Edit the file `config/sim.yaml` inside your cloned repository:

Change the `map_path` parameter:

```yaml
map_path: '/home/your-username/f1ws/src/f1tenth_gym_ros_humble/maps/levine'
```

Replace `your-username` with your actual Linux username.

---

### 7. Install ROS 2 Dependencies

Install any missing system dependencies using rosdep:

```bash
source /opt/ros/humble/setup.bash
cd ~/f1ws
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```

---

### 8. Build the Workspace

Build the ROS 2 workspace:

```bash
cd ~/f1ws
colcon build
source install/setup.bash
```

---

## Running the Simulator

Each time you want to run the simulator, follow these steps in a new terminal:

```bash
# Activate virtual environment
source ~/f1tenth_env/bin/activate

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Source workspace setup
source ~/f1ws/install/setup.bash

# Launch the simulator
ros2 launch f1tenth_gym_ros gym_bridge_launch.py 
```

You should now see your simulated F1TENTH vehicle with LaserScan and Odometry topics publishing.

---

## Troubleshooting Tips

- Always ensure you have activated your `f1tenth_env` virtual environment when running any simulator or bridge node.
- If you encounter `ModuleNotFoundError: gym`, it means the virtual environment was not activated.
- If you encounter errors related to `np.float`, ensure that numpy is downgraded to version 1.23.5.
- If `box2d-py` fails to build, ensure SWIG is properly installed (`sudo apt install swig`).

---

## Useful Commands Summary

| Task | Command |
|:-----|:--------|
| Activate Virtualenv | `source ~/f1tenth_env/bin/activate` |
| Source ROS 2 Environment | `source /opt/ros/humble/setup.bash` |
| Source Workspace | `source ~/f1ws/install/setup.bash` |
| Build Workspace | `colcon build` |
| Launch Simulator | `ros2 launch f1tenth_gym_ros simulator_launch.py` |

---

## License

This project is licensed under the MIT License.  
Original components © 2020 Hongrui Zheng.  
Modifications and extensions © Siddarth Dayasagar.

```
