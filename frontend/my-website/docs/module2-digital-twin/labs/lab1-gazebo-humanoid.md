# Lab 1: Launching a Humanoid Robot in Gazebo

## Objective

In this lab, you will learn how to launch a humanoid robot model in the Gazebo simulation environment using ROS 2. You will explore the robot model in the simulation, verify its joint states, and understand the basic structure of a Gazebo launch file.

## Prerequisites

- ROS 2 Iron Irwini installed
- Gazebo (installed as part of ROS 2 desktop package)
- Basic understanding of URDF and ROS 2 launch files
- A humanoid robot URDF model (e.g., a simplified model or a model from a robotics package)

## Setup

### 1. Create a New ROS 2 Package

First, create a new ROS 2 package to hold your launch files and robot model:

```bash
# Navigate to your ROS 2 workspace source directory
cd ~/ros2_ws/src

# Create the package
ros2 pkg create --build-type ament_python gazebo_humanoid_lab --dependencies rclpy std_msgs sensor_msgs geometry_msgs

# Navigate into the package
cd gazebo_humanoid_lab
```

### 2. Create Directory Structure

Create the necessary directories for launch files and robot models:

```bash
mkdir -p launch
mkdir -p models
mkdir -p urdf
```

## Lab Steps

### 1. Create a Simple Humanoid URDF (Optional)

If you don't have a humanoid model, create a simple one. Create a file `urdf/simple_humanoid.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.35"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.15"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Right Arm (similar to left) -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.15"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

### 2. Create a Gazebo Launch File

Create a launch file `launch/launch_humanoid_gazebo.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Paths
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_gazebo_humanoid_lab = FindPackageShare('gazebo_humanoid_lab')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros.get_path(), 'launch', 'gazebo.launch.py'),
        )
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(os.path.join(pkg_gazebo_humanoid_lab.get_path(), 'urdf', 'simple_humanoid.urdf')).read()
        }]
    )

    # Spawn Entity Node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_humanoid',
            '-x', '0', '-y', '0', '-z', '0.5'  # Spawn the robot 0.5m above ground
        ],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare launch options
    ld.add_action(declare_use_sim_time)

    # Add nodes and launch files
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)

    return ld
```

### 3. Build and Run the Simulation

1. **Build the Package**:

```bash
# Navigate to your workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select gazebo_humanoid_lab

# Source the workspace
source install/setup.bash
```

2. **Launch the Simulation**:

```bash
# Launch the simulation
ros2 launch gazebo_humanoid_lab launch_humanoid_gazebo.py
```

This command will:
- Start Gazebo
- Load your humanoid robot model into the simulation
- Set the robot's initial position

### 4. Explore the Simulation

Once the simulation is running:

1. **Visualize in Gazebo**: You should see your humanoid robot model in the Gazebo environment.
2. **Check TF Tree**: Open a new terminal and run:
   ```bash
   ros2 run tf2_tools view_frames
   ```
   This will show the transformation tree of your robot.
3. **Monitor Joint States**: Run:
   ```bash
   ros2 topic echo /joint_states sensor_msgs/msg/JointState
   ```
   This will display the current state of the robot's joints (though initially they won't move unless you command them).
4. **RViz (Optional)**: You can also launch RViz to visualize the robot:
   ```bash
   ros2 run rviz2 rviz2
   ```
   In RViz, add a RobotModel display and set the topic to `/robot_description`.

## Troubleshooting

- **Model Not Loading**: Ensure the URDF file path is correct in the launch file.
- **Gazebo Not Starting**: Make sure Gazebo is installed and sourced correctly.
- **Joint Limits**: Check the joint limits in your URDF are reasonable.
- **Physics Issues**: If the robot falls through the ground or behaves unexpectedly, check inertial properties and collision geometries.

## Conclusion

You have successfully launched a humanoid robot in Gazebo. This foundational setup allows you to test control algorithms, sensor integration, and other robot behaviors in a safe, virtual environment before deploying to physical hardware.