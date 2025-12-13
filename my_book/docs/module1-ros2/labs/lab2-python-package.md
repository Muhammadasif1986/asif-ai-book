# Lab 2: Building a Simple Python ROS 2 Package

This lab focuses on the complete workflow of creating, configuring, and testing a Python-based ROS 2 package. You will learn to:

1.  Create a ROS 2 package using `ament_python`.
2.  Define package dependencies in `package.xml`.
3.  Configure `setup.py` for Python module and executable installation.
4.  Implement a more complex `rclpy` node.
5.  Build the package using `colcon`.
6.  Run and verify the node's functionality.

## 1. Prerequisites

*   Completed Lab 1: Basic ROS 2 Publisher/Subscriber.
*   A ROS 2 environment with `colcon` installed.

## 2. Creating the Package

Navigate to your ROS 2 workspace `src` directory and create the package:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_complex_python_pkg --dependencies rclpy std_msgs
```

## 3. Defining Dependencies in `package.xml`

Open `~/ros2_ws/src/my_complex_python_pkg/package.xml`. Ensure that `rclpy` and `std_msgs` are listed as dependencies. If you use other ROS 2 interfaces, add them here.

```xml
<package format="3">
  <name>my_complex_python_pkg</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## 4. Implementing a More Complex `rclpy` Node

Let's create a node that publishes a custom message with a counter and subscribes to it, then prints it to the console. For simplicity, we'll use `std_msgs/String` for now, but in a real scenario, you might define custom interfaces (`.msg`, `.srv`, `.action` files).

Create a file `~/ros2_ws/src/my_complex_python_pkg/my_complex_python_pkg/complex_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ComplexPublisherSubscriber(Node):

    def __init__(self):
        super().__init__('complex_pub_sub_node')
        self.publisher_ = self.create_publisher(String, 'complex_topic', 10)
        self.subscription = self.create_subscription(
            String,
            'complex_topic',
            self.listener_callback,
            10)
        self.i = 0
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Complex message count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.i += 1

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    complex_node = ComplexPublisherSubscriber()
    rclpy.spin(complex_node)
    complex_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5. Configuring `setup.py`

Open `~/ros2_ws/src/my_complex_python_pkg/setup.py`. You need to ensure your Python module is found and that your script is registered as an executable entry point. Add the following to the `entry_points` dictionary, and ensure your `packages` and `install_requires` are correct:

```python
from setuptools import find_packages, setup

package_name = 'my_complex_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
    ],
    install_requires=['rclpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A more complex ROS 2 Python package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'complex_talk_listen = my_complex_python_pkg.complex_node:main',
        ],
    },
)
```

**Note**: Remember to update maintainer name and email.

## 6. Building and Running

Build your workspace from the root `~/ros2_ws`:

```bash
cd ~/ros2_ws
colcon build
```

Source the setup files:

```bash
source install/setup.bash # or setup.zsh
```

Now, run your complex node:

```bash
ros2 run my_complex_python_pkg complex_talk_listen
```

You should see messages being published and immediately received and printed by the same node. This demonstrates a more integrated Python ROS 2 package setup and execution. This concludes Lab 2.