# Fabemia2 - Autonomous Delivery Robot

A ROS 2-based autonomous delivery robot with differential drive locomotion and multi-camera vision system for navigation and house detection tasks.

## Overview

Fabemia2 is a mobile robot platform designed for autonomous navigation and delivery missions. The robot features a differential drive base with four strategically positioned RGB cameras for comprehensive environmental perception and navigation capabilities. [1](#0-0) 

## Hardware Specifications

### Mobile Base
- **Dimensions**: 0.7m × 0.5m × 0.3m (L×W×H)
- **Mass**: 5.0 kg
- **Drive System**: Differential drive with 2 powered rear wheels
- **Wheel Radius**: 0.15 m
- **Wheel Separation**: 0.56 m
- **Support**: 2 frictionless casters (front/rear) [2](#0-1) 

### Vision System
Four RGB cameras configured for different navigation tasks:

| Camera | Position | Orientation | Purpose |
|--------|----------|-------------|---------|
| Front (`camera_link`) | Front elevated | Forward | House detection |
| Bottom-Middle (`camera_linkBM`) | Center bottom | Rear | Line following |
| Bottom-Left (`camera_linkBL`) | Left bottom | Rear | Left intersection detection |
| Bottom-Right (`camera_linkBR`) | Right bottom | Rear | Right intersection detection |

**Camera Specifications:**
- Resolution: 640×480 pixels (R8G8B8)
- Horizontal FOV: ~80° (1.3962634 radians)
- Update Rate: 3 Hz
- Range: 0.1m to 15m [3](#0-2) 

## Software Architecture

### Simulation Integration
The robot integrates with Gazebo simulator through three main plugins:

1. **Differential Drive Plugin** (`gz-sim-diff-drive-system`)
   - Subscribes to `/cmd_vel` for velocity commands
   - Publishes odometry to `/odom`
   - Controls rear wheel joints with 100 Nm max torque [4](#0-3) 

2. **Joint State Publisher** (`gz-sim-joint-state-publisher-system`)
   - Monitors all wheel joints
   - Publishes to `/joint_states` [5](#0-4) 

3. **Camera System** (`gz-sim-camera-system`)
   - Four instances for each camera
   - Publishes raw images and camera info

### Navigation System
The robot includes autonomous navigation capabilities with TSP (Traveling Salesman Problem) optimization for delivery missions. The navigation client handles route planning and execution between target houses. [6](#0-5) 

## File Structure

```
src/
├── my_robot_description/
│   └── urdf/
│       ├── my_robot.urdf.xacro      # Main robot assembly
│       ├── mobile_base.xacro        # Physical structure
│       ├── mobile_base_gazebo.xacro # Simulation plugins
│       ├── camera.xacro             # Front camera
│       ├── cameraBM.xacro           # Bottom-middle camera
│       ├── cameraBL.xacro           # Bottom-left camera
│       └── cameraBR.xacro           # Bottom-right camera
└── my_robot_bringup/
    └── scripts/
        └── deliver_robot_tsp_client.py  # Navigation client
```

## Key Features

- **Autonomous Navigation**: Differential drive locomotion with odometry-based positioning
- **Multi-Camera Vision**: Four-camera system for comprehensive environmental perception
- **TSP Optimization**: Route optimization for efficient delivery missions
- **Gazebo Simulation**: Full physics simulation with realistic sensor models
- **ROS 2 Integration**: Modern ROS 2 architecture with distributed communication

## Usage

The robot is designed for autonomous delivery tasks, navigating between houses using its vision system for detection and line following. The TSP client optimizes delivery routes for maximum efficiency.

---

## Notes

This README is based on the robot's URDF description files and navigation scripts. The robot uses XACRO files for modular URDF composition, with separate files for different subsystems (mobile base, cameras, simulation plugins). The navigation system includes a TSP client for optimized route planning between delivery targets. [7](#0-6) [8](#0-7) [9](#0-8) 

Wiki pages you might want to explore:
- [Robot Hardware Description (liv0ri/fabemia2)](/wiki/liv0ri/fabemia2#6)

### Citations

**File:** src/my_robot_description/urdf/mobile_base.xacro (L5-10)
```text
    <xacro:property name="base_length" value="0.7"/>
    <xacro:property name="base_width"  value="0.5"/>
    <xacro:property name="base_height" value="0.3"/>
    <xacro:property name="wheel_radius" value="0.15"/>
    <xacro:property name="wheel_length" value="0.1"/>

```

**File:** src/my_robot_description/urdf/camera.xacro (L39-62)
```text
        <sensor name="front_camera" type="camera">
            <camera>
                <horizontal_fov>1.3962634</horizontal_fov>
                <image>
                    <width>640</width>
                    <height>480</height>
                    <format>R8G8B8</format>
                </image>
                <clip>
                    <near>0.1</near>
                    <far>15</far>
                </clip>
                <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.007</stddev>
                </noise>
                <optical_frame_id>camera_link_optical</optical_frame_id>
                <camera_info_topic>front_camera/camera_info</camera_info_topic>
            </camera>
            <always_on>1</always_on>
            <update_rate>3</update_rate>
            <visualize>true</visualize>
            <topic>front_camera/image_raw</topic> 
```

**File:** src/my_robot_description/urdf/mobile_base_gazebo.xacro (L25-40)
```text
    <gazebo>
        <plugin
            filename="gz-sim-diff-drive-system"
            name="gz::sim::systems::DiffDrive">
            <left_joint>rear_left_wheel_joint</left_joint>
            <right_joint>rear_right_wheel_joint</right_joint>
            <wheel_separation>0.56</wheel_separation>
            <wheel_radius>0.15</wheel_radius>

            <max_wheel_torque>100</max_wheel_torque>
            <max_wheel_acceleration>2.0</max_wheel_acceleration>

            <frame_id>odom</frame_id>
            <child_frame_id>base_footprint</child_frame_id>
        </plugin>
    </gazebo>
```

**File:** src/my_robot_description/urdf/mobile_base_gazebo.xacro (L43-52)
```text
    <gazebo>
        <plugin
            filename="gz-sim-joint-state-publisher-system"
            name="gz::sim::systems::JointStatePublisher">
            <joint_name>rear_left_wheel_joint</joint_name>
            <joint_name>rear_right_wheel_joint</joint_name>
            <joint_name>rear_caster_joint</joint_name>
            <joint_name>front_caster_joint</joint_name>
        </plugin>
    </gazebo>
```

**File:** src/my_robot_bringup/scripts/deliver_robot_tsp_client.py (L85-121)
```python
    def go_to_house(self, start, target):
        self.get_logger().info(f"Navigating from {start} → {target}")
        msg = String()
        msg.data = json.dumps({'start': start, 'target': target})
        self.nav_publisher.publish(msg)
        # self.get_logger().info(f"Published navigation command: {start} → {target}")

    def send_next_target(self):
        if self.route_index >= len(self.optimized_route):
            self.get_logger().info("Mission complete!")
            self.destroy_node()
            rclpy.shutdown()
            return

        target = self.optimized_route[self.route_index]

        if target == self.current:
            self.route_index += 1
            self.send_next_target()
            return

        self.go_to_house(self.current, target)

    def navigation_done_callback(self, msg):
        data = json.loads(msg.data)
        reached = data['reached']

        if reached != self.optimized_route[self.route_index]:
            self.get_logger().warn("Unexpected house reached, ignoring")
            return

        self.get_logger().info(f"Confirmed arrival at {reached}")

        self.current = reached
        self.route_index += 1

        self.send_next_target()
```

**File:** src/my_robot_description/urdf/cameraBL.xacro (L24-28)
```text
    <joint name="base_camera_jointBL" type="fixed">
        <parent link="base_link" />
        <child link="camera_linkBL" />
        <origin xyz="0 0.2 ${base_height / 2.0 - 0.31}" rpy="0 ${pi/2} 0" />
    </joint>
```

**File:** src/my_robot_description/urdf/cameraBM.xacro (L24-28)
```text
    <joint name="base_camera_jointBM" type="fixed">
        <parent link="base_link" />
        <child link="camera_linkBM" />
        <origin xyz="0 0 ${base_height / 2.0 - 0.31}" rpy="0 ${pi/2} 0" />
    </joint>
```

**File:** src/my_robot_description/urdf/cameraBR.xacro (L24-28)
```text
    <joint name="base_camera_jointBR" type="fixed">
        <parent link="base_link" />
        <child link="camera_linkBR" />
        <origin xyz="0 -0.2 ${base_height / 2.0 - 0.31}" rpy="0 ${pi/2} 0" />
    </joint>
```
