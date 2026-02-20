# Robot Proximity Communication System

This project is a ROS 2 Humble simulation where multiple robots move on a graph, detect nearby robots, and communicate through ROS 2 services. Visualization is shown in RViz2.

# Robot Interaction Sequence

This demonstration illustrates how a robot uses its circular sensor threshold to detect neighboring robots, initiate communication, and coordinate movement.

### 1. The Lone Robot and its Sensor

First, we have a single robot moving through the environment. It has an active circular sensor field, represented by the glowing blue ring around it, scanning for other robots.

![Lone robot with sensor active](image_0.png)

### 2. The Encounter and Greeting

A second, similar robot enters the central robot's sensor radius. Upon detecting the new robot, the central robot initiates communication by sending a "Hello" message.

![Central robot greeting the newcomer](image_1.png)

### 3. The Response

The second robot receives the greeting and responds with a positive "Let's go" message, indicating its readiness to join the central robot.

![Second robot responding affirmatively](image_2.png)

### 4. Moving Together

Finally, having established communication and agreed to proceed, the two robots move off together, side-by-side, as a pair. The sensor field and messages are no longer visible as they continue their journey.

![Robots moving together side-by-side](image_3.png)

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
