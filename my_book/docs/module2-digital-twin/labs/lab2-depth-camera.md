# Lab 2: Configuring and Verifying Depth Camera Sensor Data in Simulation

## Objective

In this lab, you will learn how to add a depth camera sensor to your humanoid robot model in Gazebo, configure its parameters, and verify the sensor data output. You will use RViz2 to visualize the depth camera feed and understand the data format.

## Prerequisites

- Completion of Lab 1: Launching a humanoid robot in Gazebo
- ROS 2 Iron Irwini installed
- Gazebo and RViz2
- Basic understanding of URDF, Gazebo plugins, and ROS 2 topics

## Setup

### 1. Modify the URDF from Lab 1

We will add a depth camera sensor to the head link of the humanoid robot from Lab 1. Update your `urdf/simple_humanoid.urdf` file by adding the following to the `head` link:

```xml
<!-- Depth Camera Sensor -->
<link name="camera_link">
  <visual>
    <geometry>
      <cylinder length="0.01" radius="0.01"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.01" radius="0.01"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0" rpy="0 0 0"/>
</joint>
```

And add the Gazebo-specific configuration for the depth camera. Add this to the end of your URDF file, just before the closing `</robot>` tag:

```xml
<!-- Gazebo Depth Camera Plugin -->
<gazebo reference="camera_link">
  <sensor type="depth" name="camera_sensor">
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <ros>
        <namespace>/camera</namespace>
        <remapping>~/rgb/image_raw:=image_raw</remapping>
        <remapping>~/depth/image_raw:=depth/image_raw</remapping>
        <remapping>~/depth/camera_info:=depth/camera_info</remapping>
      </ros>
      <update_rate>30</update_rate>
      <!-- Image parameters -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <!-- Noise parameters -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </plugin>
  </sensor>
</gazebo>
```

### 2. Create a New Launch File

Create a new launch file specifically for the depth camera simulation: `launch/launch_humanoid_with_camera.py`.

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
            '-x', '0', '-y', '0', '-z', '0.5'
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

## Lab Steps

### 1. Build the Updated Package

1. **Build the Package**:

```bash
# Navigate to your workspace root
cd ~/ros2_ws

# Build the package (only rebuild if you made changes to C++ code, otherwise just source again)
colcon build --packages-select gazebo_humanoid_lab

# Source the workspace
source install/setup.bash
```

### 2. Launch the Simulation with Depth Camera

```bash
# Launch the simulation with the depth camera
ros2 launch gazebo_humanoid_lab launch_humanoid_with_camera.py
```

### 3. Verify Depth Camera Data

Open a new terminal (while Gazebo is running) and check the available topics:

```bash
# List all ROS 2 topics
ros2 topic list
```

You should see topics related to the camera, such as:
- `/camera/image_raw` (RGB image)
- `/camera/depth/image_raw` (Depth image)
- `/camera/depth/camera_info` (Camera calibration info)

Check the type of the depth image topic:

```bash
# Get the message type for the depth image topic
ros2 topic type /camera/depth/image_raw
# Should output: sensor_msgs/msg/Image
```

Echo a single message from the depth topic to see its structure:

```bash
# Echo a single message from the depth image topic
ros2 topic echo /camera/depth/image_raw sensor_msgs/msg/Image --field data --once
```

### 4. Visualize Depth Data in RViz2

1. **Launch RViz2**:

```bash
# In a new terminal
ros2 run rviz2 rviz2
```

2. **Configure RViz2**:
   - Add a `TF` display to visualize the robot's transforms. Set the `Fixed Frame` to `base_link` or `camera_link`.
   - Add an `Image` display.
     - Set the `Image Topic` to `/camera/depth/image_raw`.
     - You might need to install an image transport plugin if the depth image doesn't display correctly in grayscale. The raw depth values are typically 32-bit float or 16-bit uint, representing distance in meters.

3. **Interpret the Depth Data**:
   - In the depth image, each pixel value represents the distance from the camera to the object at that pixel location.
   - Closer objects appear brighter (higher values), while farther objects appear darker (lower values), up to the sensor's maximum range (10m in our example).

### 5. Write a Simple Depth Data Subscriber (Optional)

Create a simple Python script to subscribe to the depth camera data and print some information. Create `gazebo_humanoid_lab/gazebo_humanoid_lab/depth_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge


class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.bridge = CvBridge()
        self.get_logger().info('Depth Subscriber node initialized.')

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # Print basic information about the depth image
        self.get_logger().info(f'Depth image shape: {cv_image.shape}')
        self.get_logger().info(f'Depth image dtype: {cv_image.dtype}')

        # Example: Print min and max depth values in the image
        if cv_image.size > 0:
            min_depth = np.nanmin(cv_image[np.isfinite(cv_image)])
            max_depth = np.nanmax(cv_image[np.isfinite(cv_image)])
            self.get_logger().info(f'Min depth: {min_depth:.2f}m, Max depth: {max_depth:.2f}m')

        # Example: Print depth at center pixel (be careful with NaN values)
        height, width = cv_image.shape
        center_depth = cv_image[height // 2, width // 2]
        if np.isfinite(center_depth):
            self.get_logger().info(f'Center pixel depth: {center_depth:.2f}m')
        else:
            self.get_logger().info('Center pixel depth: NaN or Inf')


def main(args=None):
    rclpy.init(args=args)
    depth_subscriber = DepthSubscriber()
    rclpy.spin(depth_subscriber)
    depth_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

You'll also need to add `cv_bridge` as a dependency. Update your `package.xml`:

```xml
<depend>cv_bridge</depend>
```

And update your `setup.py` to include the script in the entry points:

```python
entry_points={
    'console_scripts': [
        'depth_subscriber = gazebo_humanoid_lab.depth_subscriber:main',
    ],
},
```

Build again:

```bash
cd ~/ros2_ws
colcon build --packages-select gazebo_humanoid_lab
source install/setup.bash
```

Run the subscriber:

```bash
# In a new terminal, after sourcing the workspace
ros2 run gazebo_humanoid_lab depth_subscriber
```

This will print information about the depth data as it's received.

## Troubleshooting

- **No Depth Data**: Check that the Gazebo plugin is correctly configured in the URDF and that the simulation is running.
- **RViz2 Not Displaying**: Ensure the image transport plugins are available. You might need to install `image_transport_plugins`.
- **Wrong Data Type**: The depth image might be in a format that RViz2 doesn't display well by default. Check the `encoding` field in the `CameraInfo` message.
- **NaN Values**: Depth sensors often return NaN for pixels where no object is detected within the range. Handle these appropriately in your algorithms.

## Conclusion

You have successfully added a depth camera sensor to your simulated humanoid robot and verified its data output. This sensor can now be used for perception tasks like obstacle detection, mapping, and 3D reconstruction in your robotics applications.