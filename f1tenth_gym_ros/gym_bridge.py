# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import math
import pathlib
from functools import partial

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from f1tenth_gym.envs import F110Env
from f1tenth_gym.envs.env_config import (
    EnvConfig,
    ControlConfig,
    SimulationConfig,
    ObservationConfig,
    ResetConfig,
    LoopCounterMode,
)
from f1tenth_gym.envs.action import LongitudinalActionType, SteerActionType
from f1tenth_gym.envs.dynamic_models import (
    DynamicModel,
    get_f1tenth_vehicle_parameters,
    get_fullscale_vehicle_parameters,
    get_f1fifth_vehicle_parameters,
)
from f1tenth_gym.envs.integrators import IntegratorType
from f1tenth_gym.envs.lidar import LiDARConfig
from f1tenth_gym.envs.observation import ObservationType
from f1tenth_gym.envs.reset import ResetStrategy
from f1tenth_gym.envs.track import Track, Raceline


def opp_suffix(opp_index):
    """Suffix appended to opponent namespaces/topics: '' for opp 1, '2'/'3' after."""
    return '' if opp_index == 1 else str(opp_index)


# Visual wheel spin. Radius matches the wheel meshes in urdf/racecar_mesh.xacro
# (0.102 m diameter at f1tenth size); the per-platform factors are the same
# wheelbase ratios that xacro scales the meshes by.
WHEEL_RADIUS = 0.051
VEHICLE_MESH_SCALE = {'f1tenth': 1.0, 'f1fifth': 1.656456, 'fullscale': 7.449941}


def _resolve_yaml_path(base_path: pathlib.Path) -> pathlib.Path:
    if base_path.suffix in (".yaml", ".yml"):
        return base_path
    candidates = (
        base_path.with_suffix(".yaml"),
        base_path.with_suffix(".yml"),
        base_path.parent / f"{base_path.stem}_map.yaml",
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return base_path.with_suffix(".yaml")


def _resolve_map_yaml_path(map_path: str) -> pathlib.Path | None:
    path = pathlib.Path(map_path)
    if not path.is_absolute():
        share_dir = pathlib.Path(get_package_share_directory("f1tenth_gym_ros"))
        path = share_dir / map_path
    yaml_path = _resolve_yaml_path(path)
    return yaml_path if yaml_path.exists() else None


def _load_track_from_yaml(map_yaml_path: pathlib.Path, scale: float) -> tuple[Track, bool]:
    track_spec = Track.load_spec(track=map_yaml_path.stem, filespec=str(map_yaml_path))
    track_spec.resolution = track_spec.resolution * scale
    track_spec.origin = (
        track_spec.origin[0] * scale,
        track_spec.origin[1] * scale,
        track_spec.origin[2],
    )

    image_path = map_yaml_path.parent / track_spec.image
    flip_op = getattr(Image, "Transpose", Image).FLIP_TOP_BOTTOM
    image = Image.open(image_path).transpose(flip_op)
    occupancy_map = np.array(image).astype(np.float32)
    occupancy_map[occupancy_map <= 128] = 0.0
    occupancy_map[occupancy_map > 128] = 255.0

    centerline_path = map_yaml_path.parent / f"{map_yaml_path.stem}_centerline.csv"
    raceline_path = map_yaml_path.parent / f"{map_yaml_path.stem}_raceline.csv"
    centerline = None
    raceline = None
    if centerline_path.exists():
        centerline = Raceline.from_centerline_file(centerline_path, track_scale=scale)
    if raceline_path.exists():
        raceline = Raceline.from_raceline_file(raceline_path, track_scale=scale)

    if raceline is None:
        raceline = centerline
    if centerline is None:
        centerline = raceline

    track = Track(
        spec=track_spec,
        filepath=str(map_yaml_path.absolute()),
        ext=image_path.suffix,
        occupancy_map=occupancy_map,
        centerline=centerline,
        raceline=raceline,
    )
    has_reference_line = centerline is not None or raceline is not None
    return track, has_reference_line


class Opponent:
    """State and topic names for one opponent car."""

    def __init__(self, namespace, scan_topic, drive_topic, odom_topic,
                 ego_odom_topic, odom_in_ego_topic, pose):
        self.namespace = namespace
        self.scan_topic = scan_topic
        self.drive_topic = drive_topic
        self.odom_topic = odom_topic
        # ego's odom republished into this opponent's namespace
        self.ego_odom_topic = ego_odom_topic
        # this opponent's odom republished into the ego namespace
        self.odom_in_ego_topic = odom_in_ego_topic
        self.pose = list(pose)
        self.speed = [0.0, 0.0, 0.0]
        self.requested_speed = 0.0
        self.steer = 0.0
        self.v = 0.0  # signed longitudinal speed, drives the wheel spin
        self.wheel_angle = 0.0
        self.scan = []
        self.scan_pub = None
        self.odom_pub = None
        self.ego_odom_pub = None
        self.odom_in_ego_pub = None
        self.drive_sub = None
        self.reset_sub = None


class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        self.declare_parameter('ego_namespace', 'ego_racecar')
        self.declare_parameter('ego_odom_topic', 'odom')
        self.declare_parameter('ego_opp_odom_topic', 'opp_odom')
        self.declare_parameter('ego_scan_topic', 'scan')
        self.declare_parameter('ego_drive_topic', 'drive')
        self.declare_parameter('opp_namespace', 'opp_racecar')
        self.declare_parameter('opp_odom_topic', 'odom')
        self.declare_parameter('opp_ego_odom_topic', 'opp_odom')
        self.declare_parameter('opp_scan_topic', 'opp_scan')
        self.declare_parameter('opp_drive_topic', 'opp_drive')
        self.declare_parameter('lidar_enabled', True)
        self.declare_parameter('lidar_base_link_to_lidar_tf', [0.275, 0.0, 0.0])
        self.declare_parameter('lidar_noise_std', 0.01)
        self.declare_parameter('scan_num_beams', 1080)
        self.declare_parameter('scan_range_min', 0.0)
        self.declare_parameter('scan_range_max', 30.0)
        self.declare_parameter('scan_angle_min', -135.0)
        self.declare_parameter('scan_angle_max', 135.0)
        self.declare_parameter('map_path', 'levine')
        self.declare_parameter('map_img_ext', '.png')
        self.declare_parameter('num_agent', 1)
        self.declare_parameter('sx', 0.0)
        self.declare_parameter('sy', 0.0)
        self.declare_parameter('stheta', 0.0)
        self.declare_parameter('kb_teleop', True)
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('vehicle_params', 'f1tenth')
        self.declare_parameter('async_mode', True)
        # Flag to know whether to publish the sim time or not
        # Has to be different than use_sim_time so we can still use real time to trigger timer callbacks
        self.declare_parameter('use_sim_time_bridge', False)

        # check num_agents
        num_agents = self.get_parameter('num_agent').value
        if type(num_agents) != int:
            raise ValueError('num_agents should be an int.')
        if num_agents < 1:
            raise ValueError('num_agents should be at least 1.')

        # One start pose per opponent
        # NaN = 'not set by the config'.
        for i in range(1, num_agents):
            self.declare_parameter(f'sx{i}', float('nan'))
            self.declare_parameter(f'sy{i}', float('nan'))
            self.declare_parameter(f'stheta{i}', float('nan'))

        # Checked before the map and the sim env are built so an incomplete
        # config fails immediately instead of after loading everything.
        opp_poses, missing_pose_params = self._opp_start_poses(num_agents)
        if missing_pose_params:
            raise ValueError(
                f'num_agent is {num_agents}, so a start pose is needed for each of '
                f'the {num_agents - 1} opponent(s), but these parameters were not '
                f'set: {", ".join(missing_pose_params)}. Add them to the sim config '
                'this node was launched with (see config/sim.yaml).'
            )

        self.vehicle_params = None
        vehicle_params_key = self.get_parameter('vehicle_params').value
        if vehicle_params_key == 'f1tenth':
            self.vehicle_params = get_f1tenth_vehicle_parameters()
        elif vehicle_params_key == 'fullscale':
            self.vehicle_params = get_fullscale_vehicle_parameters()
        elif vehicle_params_key == 'f1fifth':
            self.vehicle_params = get_f1fifth_vehicle_parameters()
        else:
            raise ValueError('vehicle_params should be either f1tenth, fullscale, or f1fifth.')
        self.wheel_radius = WHEEL_RADIUS * VEHICLE_MESH_SCALE[vehicle_params_key]

        scale = self.get_parameter('scale').value
        map_path = self.get_parameter('map_path').value
        map_yaml_path = _resolve_map_yaml_path(map_path)

        if map_yaml_path is not None:
            self.get_logger().info('Loading map from path: %s' % map_yaml_path)
            try:
                loaded_map = Track.from_track_path(map_yaml_path, track_scale=scale)
                has_reference_line = (
                    loaded_map.centerline is not None or loaded_map.raceline is not None
                )
            except (ValueError, FileNotFoundError) as ex:
                if isinstance(ex, FileNotFoundError) or "centerline" in str(ex) or "raceline" in str(ex):
                    loaded_map, has_reference_line = _load_track_from_yaml(map_yaml_path, scale)
                else:
                    raise
        else:
            self.get_logger().info('Loading map by name: %s' % map_path)
            loaded_map = Track.from_track_name(map_path, track_scale=scale)
            has_reference_line = loaded_map.centerline is not None or loaded_map.raceline is not None

        if not has_reference_line:
            self.get_logger().warning(
                'Map has no centerline/raceline; disabling frenet frame and lap counting.'
            )

        lidar_enabled = self.get_parameter('lidar_enabled').value
        scan_num_beams = self.get_parameter('scan_num_beams').value
        if not isinstance(scan_num_beams, int):
            if isinstance(scan_num_beams, float) and scan_num_beams.is_integer():
                scan_num_beams = int(scan_num_beams)
            else:
                raise ValueError('scan_num_beams must be an integer.')
        lidar_noise_std = self.get_parameter('lidar_noise_std').value
        lidar_base_link_to_lidar_tf = self.get_parameter(
            'lidar_base_link_to_lidar_tf'
        ).value
        if len(lidar_base_link_to_lidar_tf) != 3:
            raise ValueError('lidar_base_link_to_lidar_tf must be [x, y, yaw].')
        lidar_base_link_to_lidar_tf = tuple(lidar_base_link_to_lidar_tf)
        scan_range_min = self.get_parameter('scan_range_min').value
        scan_range_max = self.get_parameter('scan_range_max').value
        scan_angle_min = self.get_parameter('scan_angle_min').value
        scan_angle_max = self.get_parameter('scan_angle_max').value
        lidar_cfg = LiDARConfig(
            enabled=lidar_enabled,
            num_beams=scan_num_beams,
            range_min=scan_range_min,
            range_max=scan_range_max,
            angle_min=np.deg2rad(scan_angle_min),
            angle_max=np.deg2rad(scan_angle_max),
            noise_std=lidar_noise_std,
            base_link_to_lidar_tf=lidar_base_link_to_lidar_tf,
        )
        self.lidar_cfg = lidar_cfg

        loop_counter = (
            LoopCounterMode.FRENET_BASED if has_reference_line else LoopCounterMode.TOGGLE
        )
        compute_frenet = has_reference_line
        self.sim_timestep = 0.01
        simulation_cfg = SimulationConfig(
            timestep=self.sim_timestep,
            integrator_timestep=self.sim_timestep,
            integrator=IntegratorType.RK4,
            dynamics_model=DynamicModel.ST,
            loop_counter=loop_counter,
            compute_frenet_frame=compute_frenet,
        )
        control_cfg = ControlConfig(
            longitudinal_mode=LongitudinalActionType.SPEED,
            steering_mode=SteerActionType.STEERING_ANGLE,
        )
        observation_cfg = ObservationConfig(type=ObservationType.DIRECT)
        reset_cfg = ResetConfig(strategy=ResetStrategy.MAP_RANDOM_STATIC)

        env_config = EnvConfig(
            map_name=loaded_map,
            map_scale=scale,
            params=self.vehicle_params,
            num_agents=num_agents,
            control_config=control_cfg,
            simulation_config=simulation_cfg,
            observation_config=observation_cfg,
            reset_config=reset_cfg,
            lidar_config=lidar_cfg,
            render_enabled=False,
        )
        self.env = F110Env(config=env_config, render_mode=None)

        sx = self.get_parameter('sx').value
        sy = self.get_parameter('sy').value
        stheta = self.get_parameter('stheta').value
        self.ego_pose = [sx, sy, stheta]
        self.ego_speed = [0.0, 0.0, 0.0]
        self.ego_requested_speed = 0.0
        self.ego_steer = 0.0
        self.ego_collision = False
        self.ego_v = 0.0  # signed longitudinal speed, drives the wheel spin
        self.ego_wheel_angle = 0.0
        ego_scan_topic = self.get_parameter('ego_scan_topic').value
        ego_drive_topic = self.get_parameter('ego_drive_topic').value
        self.angle_min = self.lidar_cfg.angle_min
        self.angle_max = self.lidar_cfg.angle_max
        self.angle_inc = self.lidar_cfg.angle_increment
        self.scan_range_min = self.lidar_cfg.range_min
        self.scan_range_max = self.lidar_cfg.range_max
        self.ego_namespace = self.get_parameter('ego_namespace').value
        ego_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_odom_topic').value
        self.scan_tf = self.lidar_cfg.base_link_to_lidar_tf

        # Opponents (agents 1..num_agents-1). Namespaces and topics for opponent i
        # are the opp_* parameters suffixed with the opponent index ('' for the
        # first opponent, so 1 and 2 agent setups behave exactly as before):
        # opp_racecar, opp_racecar2, opp_racecar3 / opp_drive, opp_drive2, ...
        opp_namespace = self.get_parameter('opp_namespace').value
        opp_scan_topic = self.get_parameter('opp_scan_topic').value
        opp_drive_topic = self.get_parameter('opp_drive_topic').value
        opp_odom_topic = self.get_parameter('opp_odom_topic').value
        opp_ego_odom_topic = self.get_parameter('opp_ego_odom_topic').value
        ego_opp_odom_topic = self.get_parameter('ego_opp_odom_topic').value

        self.opps = []
        for i in range(1, num_agents):
            suffix = opp_suffix(i)
            namespace = opp_namespace + suffix
            self.opps.append(Opponent(
                namespace=namespace,
                scan_topic=opp_scan_topic + suffix,
                drive_topic=opp_drive_topic + suffix,
                odom_topic=namespace + '/' + opp_odom_topic,
                ego_odom_topic=namespace + '/' + opp_ego_odom_topic,
                odom_in_ego_topic=self.ego_namespace + '/' + ego_opp_odom_topic + suffix,
                pose=opp_poses[i - 1],
            ))
        self.has_opp = len(self.opps) > 0
        self.get_logger().info(
            'Start poses: ' + '; '.join(
                f'{ns}=({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})'
                for ns, p in [(self.ego_namespace, self.ego_pose)]
                + [(opp.namespace, opp.pose) for opp in self.opps]
            )
        )

        self.env.reset(options={"poses": np.array(self._all_poses())})
        self._update_sim_state()

        if not self.get_parameter('async_mode').value:
            self.get_logger().info('Running in synchronous mode. Simulation will step only on new /drive messages.')
            # topic publishing timer slowly, fallback for if the controller is waiting for a first odom and scan
            self.timer = self.create_timer(1, self.timer_callback)
        else:
            self.get_logger().info('Running in asynchronous mode. Simulation will step using a timer callback.')
            # sim physical step timer
            self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)
            # topic publishing timer
            self.timer = self.create_timer(0.004, self.timer_callback)

        # transform broadcaster
        self.br = TransformBroadcaster(self)

        # publishers
        self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
        self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)
        self.ego_drive_published = False
        for opp in self.opps:
            opp.scan_pub = self.create_publisher(LaserScan, opp.scan_topic, 10)
            opp.odom_pub = self.create_publisher(Odometry, opp.odom_topic, 10)
            opp.ego_odom_pub = self.create_publisher(Odometry, opp.ego_odom_topic, 10)
            opp.odom_in_ego_pub = self.create_publisher(Odometry, opp.odom_in_ego_topic, 10)

        if self.get_parameter('use_sim_time_bridge').value:
            self.get_logger().info('Using simulation time. Will publish /clock topic. Drive and odom will be as fast as possible.')
            self.clock_pub = self.create_publisher(Clock, '/clock', 10)
            if self.get_parameter('async_mode').value:
                # Set drive timer to 0 to trigger the callback asap
                self.drive_timer.timer_period_ns = 0
                self.timer.timer_period_ns = 0

        # subscribers
        self.ego_drive_sub = self.create_subscription(
            AckermannDriveStamped,
            ego_drive_topic,
            self.drive_callback,
            10)
        self.ego_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.ego_reset_callback,
            10)
        for i, opp in enumerate(self.opps):
            opp.drive_sub = self.create_subscription(
                AckermannDriveStamped,
                opp.drive_topic,
                partial(self.opp_drive_callback, opp_index=i),
                10)
            # First opponent keeps the original /goal_pose reset topic,
            # later ones get /goal_pose2, /goal_pose3.
            opp.reset_sub = self.create_subscription(
                PoseStamped,
                '/goal_pose' + opp_suffix(i + 1),
                partial(self.opp_reset_callback, opp_index=i),
                10)

        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.teleop_callback,
                10)

        self.sim_paused = False
        self.pause_subscriber = self.create_subscription(
            Bool,
            '/pause_sim',
            self.pause_callback,
            10)

    def _opp_start_poses(self, num_agents):
        """Read sx/sy/stheta for each opponent, reporting any left unset.

        Returns (poses, missing), where missing lists the parameter names that
        were never given a value. Poses are only usable when missing is empty.
        """
        poses = []
        missing = []
        for i in range(1, num_agents):
            pose = []
            for key in (f'sx{i}', f'sy{i}', f'stheta{i}'):
                value = self.get_parameter(key).value
                if value is None or math.isnan(value):
                    missing.append(key)
                    value = 0.0
                pose.append(float(value))
            poses.append(pose)
        return poses, missing

    def _all_poses(self):
        return [list(self.ego_pose)] + [list(opp.pose) for opp in self.opps]

    def pause_callback(self, msg):
        self.sim_paused = msg.data
        self.get_logger().info(f"Simulation {'paused' if self.sim_paused else 'resumed'}")

    def drive_callback(self, drive_msg):
        if self.sim_paused:
            return  # Skip stepping the sim if paused

        self.ego_requested_speed = drive_msg.drive.speed
        self.ego_steer = np.clip(drive_msg.drive.steering_angle, self.vehicle_params.s_min, self.vehicle_params.s_max)

        if not self.get_parameter('async_mode').value:
            # step the sim immediately and publish odom and scan
            self.drive_timer_callback()
            self.timer_callback()

    def opp_drive_callback(self, drive_msg, opp_index):
        if self.sim_paused:
            return  # Skip stepping the sim if paused

        opp = self.opps[opp_index]
        opp.requested_speed = drive_msg.drive.speed
        opp.steer = np.clip(drive_msg.drive.steering_angle, self.vehicle_params.s_min, self.vehicle_params.s_max)

        if not self.get_parameter('async_mode').value:
            # step the sim immediately and publish odom and scan
            self.drive_timer_callback()
            self.timer_callback()

    def ego_reset_callback(self, pose_msg):
        if self.sim_paused:
            return  # Skip stepping the sim if paused

        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        rtheta = Rotation.from_quat([rqx, rqy, rqz, rqw]).as_euler('xyz')[2]
        self.ego_pose = [rx, ry, rtheta]
        self.env.reset(options={"poses": np.array(self._all_poses())})
        self._update_sim_state()

    def opp_reset_callback(self, pose_msg, opp_index):
        if self.sim_paused:
            return  # Skip stepping the sim if paused

        rx = pose_msg.pose.position.x
        ry = pose_msg.pose.position.y
        rqx = pose_msg.pose.orientation.x
        rqy = pose_msg.pose.orientation.y
        rqz = pose_msg.pose.orientation.z
        rqw = pose_msg.pose.orientation.w
        rtheta = Rotation.from_quat([rqx, rqy, rqz, rqw]).as_euler('xyz')[2]
        self.opps[opp_index].pose = [rx, ry, rtheta]
        self.env.reset(options={"poses": np.array(self._all_poses())})
        self._update_sim_state()

    def teleop_callback(self, twist_msg):
        if self.sim_paused:
            return  # Skip stepping the sim if paused

        self.ego_requested_speed = twist_msg.linear.x

        if twist_msg.angular.z > 0.0:
            self.ego_steer = 0.3
        elif twist_msg.angular.z < 0.0:
            self.ego_steer = -0.3
        else:
            self.ego_steer = 0.0

    def drive_timer_callback(self):
        if self.sim_paused:
            return  # Skip stepping the sim if paused

        actions = [[self.ego_steer, self.ego_requested_speed]]
        actions += [[opp.steer, opp.requested_speed] for opp in self.opps]
        _, _, self.done, _, _ = self.env.step(np.array(actions))
        self._update_sim_state()

        # Advance the visual wheel spin by the no-slip rolling rate v/r. The ST
        # dynamics model carries no wheel-speed states, so this is the closest
        # thing to the actual wheel RPM the sim can provide.
        two_pi = 2.0 * math.pi
        self.ego_wheel_angle = (
            self.ego_wheel_angle + self.ego_v / self.wheel_radius * self.sim_timestep
        ) % two_pi
        for opp in self.opps:
            opp.wheel_angle = (
                opp.wheel_angle + opp.v / self.wheel_radius * self.sim_timestep
            ) % two_pi
        if self.get_parameter('use_sim_time_bridge').value:
            clock_msg = Clock()
            sim_time = self.env.unwrapped.sim_time
            clock_msg.clock.sec = int(sim_time // 1.0)
            clock_msg.clock.nanosec = int((sim_time % 1.0) * 1e9)
            self.clock_pub.publish(clock_msg)

    def timer_callback(self):
        if self.sim_paused:
            return  # Skip stepping the sim if paused

        ts = self.get_clock().now().to_msg()
        if self.get_parameter('use_sim_time_bridge').value:
            # Ensure sim-time stamps the messages
            sim_time = self.env.unwrapped.sim_time
            ts.sec = int(sim_time // 1.0)
            ts.nanosec = int((sim_time % 1.0) * 1e9)

        # pub scans
        self.ego_scan = [float(x) for x in self.ego_scan]
        self.ego_scan_pub.publish(self._make_scan_msg(ts, self.ego_namespace, self.ego_scan))
        for opp in self.opps:
            opp.scan = [float(x) for x in opp.scan]
            opp.scan_pub.publish(self._make_scan_msg(ts, opp.namespace, opp.scan))

        # pub tf
        self._publish_odom(ts)
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)

    def _make_scan_msg(self, ts, namespace, ranges):
        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = namespace + '/laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = self.scan_range_min
        scan.range_max = self.scan_range_max
        scan.ranges = ranges
        return scan

    def _update_sim_state(self):
        sim_state = self.env.unwrapped.sim.state
        scans = sim_state.scans
        poses = sim_state.poses
        std_state = sim_state.standard_state

        self.ego_scan = list(scans[0])
        self.ego_pose[0] = float(poses[0, 0])
        self.ego_pose[1] = float(poses[0, 1])
        self.ego_pose[2] = float(poses[0, 2])
        ego_speed = float(std_state[0, 3])
        ego_beta = float(std_state[0, 6])
        self.ego_v = ego_speed
        self.ego_speed[0] = float(ego_speed * np.cos(ego_beta))
        self.ego_speed[1] = float(ego_speed * np.sin(ego_beta))
        self.ego_speed[2] = float(std_state[0, 5])

        for i, opp in enumerate(self.opps, start=1):
            opp.scan = list(scans[i])
            opp.pose[0] = float(poses[i, 0])
            opp.pose[1] = float(poses[i, 1])
            opp.pose[2] = float(poses[i, 2])
            opp_speed = float(std_state[i, 3])
            opp_beta = float(std_state[i, 6])
            opp.v = opp_speed
            opp.speed[0] = float(opp_speed * np.cos(opp_beta))
            opp.speed[1] = float(opp_speed * np.sin(opp_beta))
            opp.speed[2] = float(std_state[i, 5])

    def _make_odom_msg(self, ts, namespace, pose, speed):
        odom = Odometry()
        odom.header.stamp = ts
        odom.header.frame_id = 'map'
        odom.child_frame_id = namespace + '/base_link'
        odom.pose.pose.position.x = pose[0]
        odom.pose.pose.position.y = pose[1]
        quat = Rotation.from_euler('xyz', [0., 0., pose[2]]).as_quat()
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x = speed[0]
        odom.twist.twist.linear.y = speed[1]
        odom.twist.twist.angular.z = speed[2]
        return odom

    def _publish_odom(self, ts):
        ego_odom = self._make_odom_msg(ts, self.ego_namespace, self.ego_pose, self.ego_speed)
        self.ego_odom_pub.publish(ego_odom)

        for opp in self.opps:
            opp_odom = self._make_odom_msg(ts, opp.namespace, opp.pose, opp.speed)
            opp.odom_pub.publish(opp_odom)
            opp.ego_odom_pub.publish(ego_odom)
            opp.odom_in_ego_pub.publish(opp_odom)

    def _publish_transforms(self, ts):
        for namespace, pose in [(self.ego_namespace, self.ego_pose)] + [
            (opp.namespace, opp.pose) for opp in self.opps
        ]:
            t = Transform()
            t.translation.x = pose[0]
            t.translation.y = pose[1]
            t.translation.z = 0.0
            quat = Rotation.from_euler('xyz', [0.0, 0.0, pose[2]]).as_quat()
            t.rotation.x = quat[0]
            t.rotation.y = quat[1]
            t.rotation.z = quat[2]
            t.rotation.w = quat[3]

            ts_msg = TransformStamped()
            ts_msg.transform = t
            ts_msg.header.stamp = ts
            ts_msg.header.frame_id = 'map'
            ts_msg.child_frame_id = namespace + '/base_link'
            self.br.sendTransform(ts_msg)

    def _publish_wheel_transforms(self, ts):
        for namespace, steer, spin in [
            (self.ego_namespace, self.ego_steer, self.ego_wheel_angle)
        ] + [(opp.namespace, opp.steer, opp.wheel_angle) for opp in self.opps]:
            # Front wheels steer about z first, then roll about their own
            # (steered) axle; rear wheels only roll. 'ZY' = intrinsic Rz @ Ry.
            front_quat = Rotation.from_euler('ZY', [steer, spin]).as_quat()
            rear_quat = Rotation.from_euler('y', spin).as_quat()
            wheel_ts = TransformStamped()
            wheel_ts.header.stamp = ts
            for hinge, wheel, quat in (
                ('front_left_hinge', 'front_left_wheel', front_quat),
                ('front_right_hinge', 'front_right_wheel', front_quat),
                ('back_left_hinge', 'back_left_wheel', rear_quat),
                ('back_right_hinge', 'back_right_wheel', rear_quat),
            ):
                wheel_ts.transform.rotation.x = quat[0]
                wheel_ts.transform.rotation.y = quat[1]
                wheel_ts.transform.rotation.z = quat[2]
                wheel_ts.transform.rotation.w = quat[3]
                wheel_ts.header.frame_id = namespace + '/' + hinge
                wheel_ts.child_frame_id = namespace + '/' + wheel
                self.br.sendTransform(wheel_ts)

    def _publish_laser_transforms(self, ts):
        scan_quat = Rotation.from_euler('xyz', [0.0, 0.0, self.scan_tf[2]]).as_quat()
        for namespace in [self.ego_namespace] + [opp.namespace for opp in self.opps]:
            scan_ts = TransformStamped()
            scan_ts.transform.translation.x = self.scan_tf[0]
            scan_ts.transform.translation.y = self.scan_tf[1]
            scan_ts.transform.translation.z = 0.0
            scan_ts.transform.rotation.x = scan_quat[0]
            scan_ts.transform.rotation.y = scan_quat[1]
            scan_ts.transform.rotation.z = scan_quat[2]
            scan_ts.transform.rotation.w = scan_quat[3]
            scan_ts.header.stamp = ts
            scan_ts.header.frame_id = namespace + '/base_link'
            scan_ts.child_frame_id = namespace + '/laser'
            self.br.sendTransform(scan_ts)


def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)

if __name__ == '__main__':
    main()
