# Lab 1: Basic ROS 2 Publisher/Subscriber

This lab will guide you through creating and running a simple ROS 2 publisher and subscriber using Python and the `rclpy` client library. You will learn how to:

1.  Set up a ROS 2 workspace.
2.  Create a new Python ROS 2 package.
3.  Write a publisher node that sends string messages.
4.  Write a subscriber node that receives and prints string messages.
5.  Build and run your ROS 2 nodes.

## 1. Prerequisites

*   A working installation of ROS 2 (e.g., Iron Irwini) on Ubuntu.
*   Familiarity with basic Linux terminal commands.
*   Python 3 installed.

## 2. Setting Up Your ROS 2 Workspace

If you don't have a ROS 2 workspace, create one:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

## 3. Creating a New Python ROS 2 Package

Navigate to your workspace `src` directory and create a new Python package:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_ros2_pkg --dependencies rclpy std_msgs
```

This creates a directory `my_ros2_pkg` with the basic structure for a Python ROS 2 package.

## 4. Writing the Publisher Node

Create a new file `~/ros2_ws/src/my_ros2_pkg/my_ros2_pkg/publisher_member_function.py` and add the following content:

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 5. Writing the Subscriber Node

Create a new file `~/ros2_ws/src/my_ros2_pkg/my_ros2_pkg/subscriber_member_function.py` and add the following content:

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 6. Updating `setup.py`

Open `~/ros2_ws/src/my_ros2_pkg/setup.py` and add the entry points for your nodes within the `entry_points` dictionary. Make sure to keep the existing `data_files` content.

```python
from setuptools import find_packages, setup

package_name = 'my_ros2_pkg'

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
    description='A minimal ROS 2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_ros2_pkg.publisher_member_function:main',
            'listener = my_ros2_pkg.subscriber_member_function:main',
        ],
    },
)
```

**Note**: Replace `maintainer='your_name', maintainer_email='your_email@example.com',` with your actual name and email.

## 7. Building Your Package

Navigate to your workspace root (`~/ros2_ws`) and build:

```bash
cd ~/ros2_ws
colcon build
```

## 8. Running Your Nodes

After building, source your workspace to make the nodes discoverable:

```bash
# In your workspace root (~/ros2_ws)
source install/setup.bash # or setup.zsh for zsh users
```

Now open two separate terminal windows. In the first, run the publisher:

```bash
ros2 run my_ros2_pkg talker
```

In the second terminal, run the subscriber:

```bash
ros2 run my_ros2_pkg listener
```

You should see the publisher sending messages and the subscriber receiving them, demonstrating basic ROS 2 communication. This concludes Lab 1.