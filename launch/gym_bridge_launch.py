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

import os
import pathlib
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetLaunchConfiguration, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

# Tints the accent mesh so ego and opponents stay as easy to tell apart as the
# blue and orange boxes were. Same colours the primitive models use.
EGO_ACCENT_COLOR = '0.3 0.57 1.0 1.0'
OPP_ACCENT_COLOR = '1.0 0.57 0.1 1.0'


def _resolve_yaml_path(base_path: pathlib.Path) -> pathlib.Path:
    if base_path.suffix in ('.yaml', '.yml'):
        return base_path
    candidates = (
        base_path.with_suffix('.yaml'),
        base_path.with_suffix('.yml'),
        base_path.parent / f"{base_path.stem}_map.yaml",
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return base_path.with_suffix('.yaml')


def _resolve_map_yaml_path(map_path: str, package_share: str) -> pathlib.Path:
    if map_path.startswith(('/', '\\')):
        return _resolve_yaml_path(pathlib.Path(map_path))
    if '/' in map_path or '\\' in map_path:
        return _resolve_yaml_path(pathlib.Path(package_share) / map_path)
    try:
        from f1tenth_gym.envs.track.utils import find_track_dir
        track_dir = find_track_dir(map_path)
        yaml_path = track_dir / f"{track_dir.stem}.yaml"
        if not yaml_path.exists():
            yaml_path = track_dir / f"{track_dir.stem}_map.yaml"
        return yaml_path
    except Exception:
        return _resolve_yaml_path(pathlib.Path(package_share) / 'maps' / map_path)

def _launch_setup(context, *args, **kwargs):
    # The number of cars decides how many robot_state_publishers to spawn, and
    # launch arguments only have values inside an OpaqueFunction, so everything
    # that reads the sim config is built here.
    package_share = get_package_share_directory('f1tenth_gym_ros')
    config = LaunchConfiguration('config').perform(context)
    if not os.path.isabs(config):
        config = os.path.join(package_share, 'config', config)
    with open(config, 'r') as config_file:
        config_dict = yaml.safe_load(config_file)

    # launch accepts undeclared key:=value pairs silently, so the old spelling would
    # quietly run a single car; reject it with a pointer to the new name instead.
    if context.launch_configurations.get('num_agent') is not None:
        raise RuntimeError('num_agent was renamed: use num_agents:=<n> (and num_agents in sim.yaml).')
    num_agents = LaunchConfiguration('num_agents').perform(context)  # command line wins
    num_agents = int(num_agents) if num_agents.strip() else config_dict['bridge']['ros__parameters']['num_agents']
    if num_agents < 1:
        raise RuntimeError(f'num_agents must be at least 1, got {num_agents}.')
    teleop = config_dict['bridge']['ros__parameters']['kb_teleop']
    use_sim_time = config_dict['bridge']['ros__parameters']['use_sim_time']

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config, {
            'num_agents': num_agents,
            'use_sim_time': False,  # Always use real time for the bridge node
            'use_sim_time_bridge': use_sim_time, # Whether to internally use and publish sim time
            }], 
        # Nothing else is useful without the sim, so take the whole launch down
        # with it instead of leaving foxglove_bridge and map_server running.
        # Should make it clearer to the user that the gym died in the logs.
        on_exit=[Shutdown(reason='gym bridge exited')],
    )
    foxglove_host = LaunchConfiguration('foxglove_host')
    foxglove_port = LaunchConfiguration('foxglove_port')
    foxglove_app_url = LaunchConfiguration('foxglove_app_url')
    open_foxglove = LaunchConfiguration('open_foxglove')
    foxglove_target = LaunchConfiguration('foxglove_target')
    foxglove_open_url = LaunchConfiguration('foxglove_open_url')
    foxglove_ws_url_parts = ['ws://', foxglove_host, ':', foxglove_port]
    foxglove_layout = LaunchConfiguration('foxglove_layout')

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{'port': foxglove_port}],
        output='screen',
    )
    foxglove_open = ExecuteProcess(
        cmd=['xdg-open', foxglove_open_url],
        condition=IfCondition(open_foxglove),
        output='screen',
    )
    foxglove_log = LogInfo(msg=['\033[34mFoxglove app: ', foxglove_app_url, '\033[0m'])
    foxglove_ws_log = LogInfo(msg=['\033[34mFoxglove WebSocket: '] + foxglove_ws_url_parts + ['\033[0m'])
    foxglove_target_log = LogInfo(msg=['\033[34mFoxglove target: ', foxglove_target, '\033[0m'])
    foxglove_layout_log = LogInfo(msg=['\033[34mFoxglove layout: ', foxglove_layout, '\033[0m'])

    # Create custom yaml file for map server by copying the original yaml file and scaling the resolution.
    map_path = config_dict['bridge']['ros__parameters']['map_path']
    map_yaml_path = _resolve_map_yaml_path(map_path, package_share)
    with open(map_yaml_path, 'r') as file:
        map_yaml = yaml.safe_load(file)
    scale = config_dict['bridge']['ros__parameters']['scale']
    map_yaml['resolution'] *= scale
    origin = map_yaml['origin']
    scaled_origin = (
        origin[0] * scale,
        origin[1] * scale,
        origin[2],
    )
    map_yaml['origin'] = scaled_origin
    image_field = map_yaml.get('image')
    if not image_field:
        raise ValueError('map yaml is missing image field')
    image_path = pathlib.Path(image_field)
    if not image_path.is_absolute():
        image_path = map_yaml_path.parent / image_path
    map_img_ext = image_path.suffix
    map_yaml['image'] = 'scaled_map' + map_img_ext

    temp_yaml_path = None
    # Create a temporary directory to store the scaled map yaml and image
    # Create a temporary directory to store the scaled map yaml and image in the same location as the script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    temp_dir = os.path.join(script_dir, 'temp')
    os.makedirs(temp_dir, exist_ok=True)

    temp_yaml_path = os.path.join(temp_dir, 'scaled_map.yaml')
    temp_img_path = os.path.join(temp_dir, 'scaled_map' + map_img_ext)

    # Write the scaled map yaml to the temporary file
    with open(temp_yaml_path, 'w') as file:
        yaml.dump(map_yaml, file)

    # Copy the map image to the temporary directory
    map_image_path = image_path
    with open(temp_img_path, 'wb') as file:
        with open(map_image_path, 'rb') as img_file:
            file.write(img_file.read())

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': temp_yaml_path},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': use_sim_time}],
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )


    vehicle_params = config_dict['bridge']['ros__parameters']['vehicle_params']
    urdf_dir = os.path.join(package_share, 'urdf')
    ego_namespace = config_dict['bridge']['ros__parameters'].get('ego_namespace', 'ego_racecar')
    opp_namespace = config_dict['bridge']['ros__parameters'].get('opp_namespace', 'opp_racecar')

    def robot_description(car_name, accent_color):
        """xacro command line for one car."""
        # Quoted because the accent colour is an rgba string with spaces in it
        # and Command shlex-splits what it is given.
        # ParameterValue(value_type=str) stops launch from yaml-parsing the URDF
        # string, which blows up on any "colon + space" in the xacro output.
        return ParameterValue(Command([
            'xacro ', os.path.join(urdf_dir, 'racecar_mesh.xacro'),
            ' car_name:=', car_name,
            ' accent_color:="', accent_color, '"',
            ' vehicle:=', vehicle_params,
        ]), value_type=str)

    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[
            {'robot_description': robot_description(
                ego_namespace, EGO_ACCENT_COLOR)},
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('/robot_description', 'ego_robot_description')]
    )
    # One publisher per opponent, matching the namespaces the bridge publishes
    # TF for: opp_racecar, opp_racecar2, opp_racecar3, ...
    opp_robot_publishers = []
    for i in range(1, num_agents):
        suffix = '' if i == 1 else str(i)  # first opponent keeps the unsuffixed names
        opp_robot_publishers.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='opp_robot_state_publisher' + suffix,
            parameters=[
                {'robot_description': robot_description(
                    opp_namespace + suffix, OPP_ACCENT_COLOR)},
                {'use_sim_time': use_sim_time},
            ],
            remappings=[('/robot_description', 'opp_robot_description' + suffix)]
        ))

    return [
        foxglove_bridge_node,
        foxglove_open,
        bridge_node,
        nav_lifecycle_node,
        map_server_node,
        ego_robot_publisher,
        *opp_robot_publishers,
        foxglove_log,
        foxglove_ws_log,
        foxglove_target_log,
        foxglove_layout_log,
    ]


def generate_launch_description():
    ld = LaunchDescription()
    package_share = get_package_share_directory('f1tenth_gym_ros')
    # Foxglove defaults come from the default config, because launch argument
    # defaults have to exist before config:= can be resolved. Override them on the
    # command line if your config carries different foxglove settings.
    with open(os.path.join(package_share, 'config', 'sim.yaml'), 'r') as config_file:
        foxglove_config = yaml.safe_load(config_file).get('foxglove', {})
    if 'ros__parameters' in foxglove_config:
        foxglove_config = foxglove_config['ros__parameters']
    open_foxglove_default = str(foxglove_config.get('open_foxglove', True)).lower()
    foxglove_target_default = str(foxglove_config.get('target', 'browser')).lower()
    if foxglove_target_default not in ('browser', 'studio'):
        raise ValueError("config/foxglove/target must be either 'browser' or 'studio'.")

    # referenced by the foxglove_open_url substitutions below
    foxglove_host = LaunchConfiguration('foxglove_host')
    foxglove_port = LaunchConfiguration('foxglove_port')
    foxglove_app_url = LaunchConfiguration('foxglove_app_url')

    # finalize
    ld.add_action(
        DeclareLaunchArgument(
            'config',
            default_value='sim.yaml',
            description='Sim config: a file name in config/, or an absolute path.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'num_agents',
            default_value='',
            description='Number of agents (1 ego + opponents). Empty takes it from the sim config.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'foxglove_host',
            default_value='localhost',
            description='Host for foxglove_bridge websocket server.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'foxglove_port',
            default_value='8765',
            description='Port for foxglove_bridge websocket server.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'foxglove_app_url',
            default_value='https://app.foxglove.dev',
            description='Foxglove app URL to open in a browser.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'foxglove_layout',
            default_value=os.path.join(package_share, 'config', 'foxglove', 'gym_bridge_foxglove.json'),
            description='Path to Foxglove layout JSON.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'open_foxglove',
            default_value=open_foxglove_default,
            description='Whether to open Foxglove automatically.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'foxglove_target',
            default_value=foxglove_target_default,
            description="Where to open Foxglove: 'browser' or 'studio'.",
        )
    )
    ld.add_action(
        SetLaunchConfiguration(
            'foxglove_open_url',
            [foxglove_app_url, '/?ds=foxglove-websocket&ds.url=ws://', foxglove_host, ':', foxglove_port],
        )
    )
    ld.add_action(
        SetLaunchConfiguration(
            'foxglove_open_url',
            ['foxglove://open?ds=foxglove-websocket&ds.url=ws://', foxglove_host, ':', foxglove_port],
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('foxglove_target'), 'studio')),
        )
    )
    ld.add_action(OpaqueFunction(function=_launch_setup))

    return ld
