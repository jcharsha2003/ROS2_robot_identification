# Robot Proximity Communication System

This project is a ROS 2 Humble simulation where multiple robots move on a graph, detect nearby robots, and communicate through ROS 2 services. Visualization is shown in RViz2.

## AI Robot Image

```text
          [ AI ROBOT ]
             .----.
+-----------/ .--. \-----------+
|   sensor  | |[]| |  sensor   |
|           | '--' |           |
|        .-./----\.-.          |
|       /   \____/   \         |
|      |  []  ||  []  |        |
|      |______||______|        |
+----------__/  \__------------+
           /__/\__\
```

## Run Commands

Use these commands exactly:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ASSIGN_2_updated
colcon build --packages-select robot_proximity
source install/setup.bash
ros2 run robot_proximity interactive_runner
```

## What This Project Contains

- `graph_manager.py`: Defines graph nodes/edges and path helpers.
- `robot_node.py`: Moves each robot and publishes robot pose/markers.
- `proximity_monitor.py`: Detects nearby robot pairs and triggers communication.
- `graph_visualizer.py`: Publishes graph markers for RViz display.
- `interactive_runner.py`: Interactive launcher for auto/manual robot path setup.

## Main ROS Interfaces

- Topic `/{robot_id}/pose` (`geometry_msgs/Pose`)
- Topic `/{robot_id}/markers` (`visualization_msgs/MarkerArray`)
- Topic `/graph_markers` (`visualization_msgs/MarkerArray`)
- Service `/{robot_id}/communicate` (`std_srvs/Trigger`)
