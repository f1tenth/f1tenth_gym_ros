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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
        )
    config_dict = yaml.safe_load(open(config, 'r'))
    has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1
    teleop = config_dict['bridge']['ros__parameters']['kb_teleop']

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge.rviz')]
    )

    # Create custom yaml file for map server by copying the original yaml file and scaling the resolution by the sim.yaml scale
    with open(config_dict['bridge']['ros__parameters']['map_path'] + '.yaml', 'r') as file:
        map_yaml = yaml.safe_load(file)
    map_yaml['resolution'] *= config_dict['bridge']['ros__parameters']['scale']
    origin = map_yaml['origin']
    scaled_origin = (
        origin[0] * config_dict['bridge']['ros__parameters']['scale'],
        origin[1] * config_dict['bridge']['ros__parameters']['scale'],
        origin[2],
    )
    map_yaml['origin'] = scaled_origin
    map_yaml['image'] = 'scaled_map' + config_dict['bridge']['ros__parameters']['map_img_ext']

    temp_yaml_path = None
    # Create a temporary directory to store the scaled map yaml and image
    # Create a temporary directory to store the scaled map yaml and image in the same location as the script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    temp_dir = os.path.join(script_dir, 'temp')
    os.makedirs(temp_dir, exist_ok=True)

    temp_yaml_path = os.path.join(temp_dir, 'scaled_map.yaml')
    temp_img_path = os.path.join(temp_dir, 'scaled_map' + config_dict['bridge']['ros__parameters']['map_img_ext'])

    # Write the scaled map yaml to the temporary file
    with open(temp_yaml_path, 'w') as file:
        yaml.dump(map_yaml, file)

    # Copy the map image to the temporary directory
    map_image_path = os.path.join(config_dict['bridge']['ros__parameters']['map_path'] + config_dict['bridge']['ros__parameters']['map_img_ext'])
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
                    {'use_sim_time': True}]
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )


    ego_xacro = None
    if config_dict['bridge']['ros__parameters']['vehicle_params'] == 'f1tenth':
        ego_xacro = "ego_racecar.xacro"
    elif config_dict['bridge']['ros__parameters']['vehicle_params'] == 'fullscale':
        ego_xacro = "ego_racecar_fullscale.xacro"
    elif config_dict['bridge']['ros__parameters']['vehicle_params'] == 'f1fifth':
        ego_xacro = "ego_racecar_f1fifth.xacro"
    else:
        raise ValueError('vehicle_params should be either f1tenth, fullscale, or f1fifth.')
    
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', ego_xacro)])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )
    opp_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='opp_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'opp_racecar.xacro')])}],
        remappings=[('/robot_description', 'opp_robot_description')]
    )

    # finalize
    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    if has_opp:
        ld.add_action(opp_robot_publisher)

    return ld