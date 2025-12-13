# URDF for Humanoid Robots

Unified Robot Description Format (URDF) is an XML format used in ROS to describe all elements of a robot. [1] For humanoid robots, URDF files are crucial for defining their kinematic and dynamic properties, visual appearance, and collision models. This information is then used by various ROS tools for simulation, visualization, and motion planning.

## 1. Structure of a URDF File

A URDF file primarily consists of two main elements:

*   **`<link>`**: Defines a rigid body segment of the robot (e.g., torso, upper arm, forearm, hand). Each link has associated visual, collision, and inertial properties.
*   **`<joint>`**: Defines the kinematic and dynamic properties of the connection between two links (e.g., shoulder, elbow, wrist joints). Joints can be revolute, prismatic, continuous, fixed, or planar.

### Example: Simple Humanoid Arm Segment

```xml
<robot name="simple_humanoid_arm">

  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <link name="forearm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.7"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="30" velocity="10" lower="-1.57" upper="1.57"/>
  </joint>

</robot>
```

## 2. Importance for Humanoid Robots

For humanoid robots, URDF is critical for:

*   **Accurate Kinematics**: Defines how the joints and links move in relation to each other, essential for inverse and forward kinematics.
*   **Collision Detection**: Specifies the collision geometries, preventing self-collisions and collisions with the environment in simulation.
*   **Visualization**: Allows tools like RViz to display the robot's 3D model accurately.
*   **Dynamic Simulation**: Provides inertial properties (mass, inertia) for realistic physics simulation in environments like Gazebo.
*   **Motion Planning**: Enables motion planners to understand the robot's degrees of freedom and joint limits.

## 3. Xacro: Improving URDF with Macros

While URDF is powerful, it can become verbose for complex robots with many repeating patterns (like fingers on a hand). **Xacro** (XML Macros) is an XML macro language that allows you to write more concise and readable robot descriptions by using macros, properties, and mathematical expressions. [1] Xacro files are typically processed into a standard URDF file before being used by ROS tools.

Using URDF (and Xacro) is a foundational step in bringing humanoid robot designs to life in the ROS 2 ecosystem, enabling both simulation and real-world control.