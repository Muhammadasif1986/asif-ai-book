# Building Python ROS 2 Packages with `rclpy`

`rclpy` is the Python client library for ROS 2, providing an easy-to-use interface to interact with the ROS 2 ecosystem. This section will guide you through creating a new Python ROS 2 package, writing simple `rclpy` nodes, and understanding the package structure.

## 1. Creating a New ROS 2 Package

ROS 2 uses `colcon` as its build system, and packages are typically created using the `ros2 pkg create` command. [1] This command helps set up the basic directory structure and necessary configuration files.

```bash
# Navigate to your workspace source directory (e.g., ~/ros2_ws/src)
cd ~/ros2_ws/src

# Create a new Python package named 'my_python_pkg'
ros2 pkg create --build-type ament_python my_python_pkg --dependencies rclpy
```

This command creates a directory `my_python_pkg` with a `setup.py` (for Python package setup) and other essential files.

## 2. Package Structure

A typical Python ROS 2 package (`ament_python` build type) will have the following structure:

```
my_python_pkg/
├── my_python_pkg/          # Python module directory
│   └── __init__.py
│   └── my_node.py          # Your Python ROS 2 nodes
├── resource/
│   └── my_python_pkg       # Empty marker file
├── package.xml             # Package metadata (dependencies, maintainer, etc.)
├── setup.py                # Python package installation instructions
├── setup.cfg               # Python configuration file
└── test/
    └── test_copyright.py
    └── test_flake8.py
    └── test_mypy.py
```

## 3. Writing a Simple `rclpy` Node

Let's create a simple publisher and subscriber node in `my_python_pkg/my_python_pkg/my_node.py`.

### Publisher Node Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Node Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4. Building and Running the Package

After creating your nodes, you need to modify `setup.py` and `package.xml` to properly install and make your executables available. These modifications are beyond the scope of this introduction but will be covered in hands-on labs.

To build your workspace:

```bash
# Navigate to your workspace root (e.g., ~/ros2_ws)
cd ~/ros2_ws
colcon build
```

To run your node (after sourcing the `install/setup.bash` or `install/setup.zsh`):

```bash
ros2 run my_python_pkg my_node
```

This provides a basic understanding of building Python ROS 2 packages with `rclpy`. Further details and hands-on exercises will solidify your knowledge.