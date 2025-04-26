# FOLLOW THIS SETUP 

```markdown
# 🏎️ F1TENTH Gym ROS2 Simulator (Humble + Virtualenv Setup)

This repo provides a ROS2 (Humble) wrapper around the F1TENTH Gym simulator.  
It connects an OpenAI Gym environment to ROS2 nodes for building and testing F1TENTH autonomous controllers.

---

## 📋 Installation Instructions (Ubuntu 22.04)

---
### 2️⃣ Setup Python Virtual Environment (Recommended)

Create a Python 3.10 virtual environment to avoid dependency conflicts:

```bash
sudo apt update
sudo apt install python3.10-venv
python3.10 -m venv ~/f1tenth_env
source ~/f1tenth_env/bin/activate
pip install --upgrade pip setuptools
```

✅ You will work inside this `f1tenth_env` when running the simulator.

---

### 3️⃣ Install and Patch OpenAI Gym (0.19.0)

Clone and patch OpenAI Gym manually:

```bash
cd ~
git clone https://github.com/openai/gym.git
cd gym
git checkout 0.19.0
```

Edit `setup.py`:
- Remove `tests_require=["pytest", "mock"]`
- Fix any invalid extras (e.g., change `opencv-python>=3.` to `opencv-python>=3.0.0`)

Then install:

```bash
pip install -e .
```

---

### 4️⃣ Install Other Python Dependencies

```bash
sudo apt install swig
pip install numpy==1.23.5 box2d-py transforms3d pyglet==1.4.11 scipy==1.11.4
```

✅ `box2d-py` needs SWIG installed system-wide.

---

### 5️⃣ Install F1Tenth Gym

```bash
cd ~
git clone https://github.com/f1tenth/f1tenth_gym.git
pip install -e ./f1tenth_gym
```

✅ This installs the actual F1Tenth Gym simulator into your venv.

---

### 6️⃣ Setup ROS 2 Workspace

Create a workspace:

```bash
mkdir -p ~/f1ws/src
cd ~/f1ws/src
git clone https://github.com/siddarth09/f1tenth_gym_ros_humble.git
```

---

### 7️⃣ Fix Map Path in sim.yaml

Edit `config/sim.yaml` inside your repo:

Change:

```yaml
map_path: '~/f1ws/src/f1tenth_gym_ros_humble/maps/levine'
```

to:

```yaml
map_path: '/home/your-username/f1ws/src/f1tenth_gym_ros_humble/maps/levine'
```

(Replace `your-username` with your Linux username.)

---

### 8️⃣ Install ROS 2 Dependencies

```bash
source /opt/ros/humble/setup.bash
cd ~/f1ws
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```

---

### 9️⃣ Build the Workspace

```bash
cd ~/f1ws
colcon build
source install/setup.bash
```

✅ This builds your gym bridge and ROS2 nodes.

---

## 🚀 Running the Simulator

Every time you want to run the simulator, follow this order:

```bash
# 1. Activate your virtualenv
source ~/f1tenth_env/bin/activate

# 2. Source ROS 2
source /opt/ros/humble/setup.bash

# 3. Source your workspace
source ~/f1ws/install/setup.bash

# 4. Launch the simulator
ros2 launch f1tenth_gym_ros simulator_launch.py
```

✅ You should now see your car in the simulation and LaserScan + Odometry publishing!

---

## 🛠️ Troubleshooting Tips

- Make sure you're always inside the `f1tenth_env` virtual environment when running `gym_bridge`.
- If you get `ModuleNotFoundError: gym`, it means your virtualenv was not activated.
- If you get `np.float` attribute errors, downgrade numpy to 1.23.5.

---

## 📚 Useful Commands

| Task | Command |
|:-----|:--------|
| Activate Virtualenv | `source ~/f1tenth_env/bin/activate` |
| Source ROS2 | `source /opt/ros/humble/setup.bash` |
| Source Workspace | `source ~/f1ws/install/setup.bash` |
| Build Workspace | `colcon build` |
| Launch Simulator | `ros2 launch f1tenth_gym_ros simulator_launch.py` |

---

# 🏁 Ready to build your first F1Tenth controller!

---

# 🛡️ License

MIT License - (c) 2020 Hongrui Zheng (and extensions by Siddarth Dayasagar)

