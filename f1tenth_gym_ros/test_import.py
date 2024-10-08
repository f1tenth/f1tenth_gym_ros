import yaml
from f1tenth_gym.envs.f110_env import F110Env, Track
import gymnasium as gym
import pathlib

# Split the path and the name
path = '/home/ahmad/F1tenth/ros_ws/src/f1tenth_gym_ros/maps/levine'
name = path.split('/')[-1].split('.')[0]
path = path + '.yaml'
print('Loading map: %s from path: %s' % (name, path))

# Load the yaml file
path = pathlib.Path(path)
track = Track.from_track_path(path)

print(track)

vehicle_params = 'f1tenth'
if vehicle_params == 'f1tenth':
    vehicle_params = F110Env.f1tenth_vehicle_params()
elif vehicle_params == 'fullscale':
    vehicle_params = F110Env.fullscale_vehicle_params()
elif vehicle_params == 'f1fifth':
    vehicle_params = F110Env.f1fifth_vehicle_params()
else:
    raise ValueError('vehicle_params should be either f1tenth, fullscale, or f1fifth.')

# env backend
env = gym.make(
                "f1tenth_gym:f1tenth-v0",
                config={
                    "map": track,
                    "num_agents": 1,
                    "timestep": 0.01,
                    "integrator": "rk4",
                    "control_input": ["speed", "steering_angle"],
                    "model": "st",
                    "observation_config": {"type": "kinematic_state"},
                    "params": vehicle_params,
                    "reset_config": {"type": "rl_random_static"},
                    "scale": 1.0,
                },
                render_mode="human",
            )

print("Done!")