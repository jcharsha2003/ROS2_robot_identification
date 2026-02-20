
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_proximity')
    rviz_config_dir = os.path.join(pkg_dir, 'config', 'simulation.rviz')

    # User Defined Robots
    robots = [{'id': 'robot1', 'color': 'red', 'path': 'A,B,C,D,E,J,O,T,S,R,Q,P', 'speed': 0.5, 'radius': 1.5}, {'id': 'robot2', 'color': 'blue', 'path': 'E,D,C,B,A,F,K,P,Q,R,S,T', 'speed': 0.5, 'radius': 1.5}, {'id': 'robot3', 'color': 'green', 'path': 'A,G,M,S,T,O,I,C,B,F,K,Q', 'speed': 0.5, 'radius': 1.5}, {'id': 'robot4', 'color': 'purple', 'path': 'T,S,R,Q,P,K,F,A,B,C,D,E', 'speed': 0.5, 'radius': 1.5}, {'id': 'robot5', 'color': 'orange', 'path': 'P,Q,R,S,T,O,J,E,D,C,B,A', 'speed': 0.5, 'radius': 1.5}, {'id': 'robot6', 'color': 'cyan', 'path': 'K,L,M,NodeN,O,J,I,H,G,F', 'speed': 0.5, 'radius': 1.5}, {'id': 'robot7', 'color': 'magenta', 'path': 'F,G,H,I,J,O,NodeN,M,L,K', 'speed': 0.5, 'radius': 1.5}]

    robot_nodes = []
    robot_ids = []

    for r in robots:
        robot_ids.append(r['id'])
        robot_nodes.append(
            Node(
                package='robot_proximity',
                executable='robot_node',
                name=r['id'],
                parameters=[{
                    'robot_id': r['id'],
                    'color': r['color'],
                    'path': r['path'],
                    'speed': r['speed'],
                    'radius': r['radius']
                }],
                output='screen'
            )
        )

    return LaunchDescription([
        *robot_nodes,

        Node(
            package='robot_proximity',
            executable='proximity_monitor',
            name='proximity_monitor',
            parameters=[{
                'threshold': 3.0,
                'robot_ids': ','.join(robot_ids)
            }],
            output='screen'
        ),
        Node(
            package='robot_proximity',
            executable='graph_visualizer',
            name='graph_visualizer',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )
    ])
