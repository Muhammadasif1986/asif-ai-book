# Lab 1: Developing a VLA Model for Spoken Commands to ROS 2 Actions

## Objective

In this lab, you will develop a Vision-Language-Action (VLA) model that can interpret spoken commands in the context of visual input and generate appropriate ROS 2 actions. You will integrate OpenAI's Whisper for speech recognition, a vision model for scene understanding, and an LLM for cognitive planning to create a complete VLA pipeline that controls a simulated robot in Gazebo.

## Prerequisites

- ROS 2 Iron Irwini installed
- Gazebo (part of ROS 2 desktop installation)
- Python 3.8+ with pip
- OpenAI API key (for Whisper and GPT models)
- Access to a humanoid robot model in Gazebo
- Basic understanding of ROS 2, computer vision, and NLP concepts
- NVIDIA GPU (recommended for faster processing)

## Setup

### 1. Create a New ROS 2 Package

First, create a new ROS 2 package for the VLA system:

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws/src

# Create the package
ros2 pkg create --build-type ament_python vla_robot_control --dependencies rclpy std_msgs sensor_msgs geometry_msgs cv_bridge message_filters

# Navigate into the package
cd vla_robot_control
```

### 2. Install Required Dependencies

Install the required Python packages:

```bash
pip install openai openai-whisper torch torchvision torchaudio transformers sentence-transformers numpy opencv-python pillow
```

### 3. Create Directory Structure

Create the necessary directories for the project:

```bash
mkdir -p vla_robot_control/nodes
mkdir -p vla_robot_control/utils
mkdir -p config
```

## Lab Steps

### 1. Implement the VLA Node

Create the main VLA node that integrates speech recognition, vision, and action planning. Create `vla_robot_control/nodes/vla_controller.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
import openai
import whisper
import torch
import cv2
import numpy as np
from transformers import pipeline
import json
import re
from message_filters import ApproximateTimeSynchronizer, Subscriber

class VLAController(Node):
    def __init__(self):
        super().__init__('vla_controller')

        # Initialize components
        self.bridge = CvBridge()

        # Initialize Whisper model for speech recognition
        self.whisper_model = whisper.load_model("base")

        # Initialize vision model for scene understanding
        self.vision_pipeline = pipeline("image-to-text", model="nlpconnect/vit-gpt2-image-captioning")

        # Initialize robot capabilities
        self.robot_capabilities = {
            "navigation": ["go to", "move to", "navigate to", "walk to"],
            "manipulation": ["pick up", "grasp", "place", "put down", "move"],
            "interaction": ["greet", "wave", "nod", "point"]
        }

        # ROS 2 publishers and subscribers
        self.audio_sub = self.create_subscription(
            String, 'audio_transcription', self.audio_callback, 10)
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, qos_profile_sensor_data)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Store latest image for context
        self.latest_image = None
        self.image_timestamp = None

        # Robot state
        self.robot_location = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.objects_in_scene = []

        # OpenAI API configuration
        self.openai_client = openai.OpenAI(api_key='YOUR_OPENAI_API_KEY_HERE')

        self.get_logger().info("VLA Controller initialized")

    def image_callback(self, msg):
        """Store the latest image for context"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            self.image_timestamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def audio_callback(self, msg):
        """Process audio transcription and generate robot actions"""
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        if self.latest_image is not None:
            # Generate scene description from image
            scene_description = self.describe_scene(self.latest_image)

            # Plan actions using LLM with visual context
            actions = self.plan_with_context(command, scene_description)

            # Execute the planned actions
            self.execute_actions(actions)
        else:
            self.get_logger().warn("No image available for context")

    def describe_scene(self, image):
        """Generate a textual description of the scene using vision model"""
        try:
            # Resize image if too large
            h, w = image.shape[:2]
            if h > 480 or w > 640:
                aspect = w / h
                new_h = min(h, 480)
                new_w = int(new_h * aspect)
                image = cv2.resize(image, (new_w, new_h))

            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Generate description
            result = self.vision_pipeline(rgb_image)
            return result[0]['generated_text']
        except Exception as e:
            self.get_logger().error(f"Error describing scene: {e}")
            return "Unable to describe scene"

    def plan_with_context(self, command, scene_description):
        """Plan actions using LLM with visual context"""
        try:
            # Create a detailed prompt for the LLM
            prompt = f"""
            You are a Vision-Language-Action (VLA) model controlling a humanoid robot.
            The robot receives a spoken command and must understand the visual scene to take appropriate actions.

            Current scene description: {scene_description}

            Robot capabilities:
            - Navigation: moving to specific locations
            - Manipulation: picking up and placing objects
            - Interaction: performing gestures

            The user says: "{command}"

            Based on the scene and the command, provide a JSON list of actions that the robot should take. Each action should have:
            - "action": the type of action (e.g., "navigate", "manipulate", "interact", "speak")
            - "target": the specific target of the action
            - "parameters": any additional parameters needed
            - "confidence": confidence level (0.0-1.0)

            Example output format:
            [
                {{
                    "action": "navigate",
                    "target": "kitchen counter",
                    "parameters": {{"x": 2.0, "y": 1.5}},
                    "confidence": 0.9
                }},
                {{
                    "action": "manipulate",
                    "target": "red cup",
                    "parameters": {{"action": "pick_up", "height": 1.0}},
                    "confidence": 0.8
                }}
            ]

            Be specific about locations and objects. If the command is ambiguous or cannot be executed based on the scene, explain why.
            """

            # Call OpenAI API
            response = self.openai_client.chat.completions.create(
                model="gpt-4",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1
            )

            # Extract the response
            response_text = response.choices[0].message.content

            # Extract JSON from the response
            json_match = re.search(r'\[.*\]', response_text, re.DOTALL)
            if json_match:
                plan = json.loads(json_match.group())
                return plan
            else:
                self.get_logger().error(f"Could not parse plan from LLM response: {response_text}")
                return []

        except Exception as e:
            self.get_logger().error(f"Error planning with context: {e}")
            return []

    def execute_actions(self, actions):
        """Execute the planned actions"""
        for action in actions:
            action_type = action.get('action')
            target = action.get('target')
            parameters = action.get('parameters', {})
            confidence = action.get('confidence', 0.0)

            # Check confidence threshold
            if confidence < 0.7:
                self.get_logger().warn(f"Skipping action due to low confidence: {confidence}")
                continue

            self.get_logger().info(f"Executing: {action_type} {target} with {parameters}")

            if action_type == "navigate":
                self.execute_navigation(target, parameters)
            elif action_type == "manipulate":
                self.execute_manipulation(target, parameters)
            elif action_type == "interact":
                self.execute_interaction(target, parameters)
            elif action_type == "speak":
                self.execute_speech(target, parameters)
            else:
                self.get_logger().warn(f"Unknown action type: {action_type}")

    def execute_navigation(self, target, parameters):
        """Execute navigation action"""
        # Create and publish velocity command
        twist = Twist()

        # Simple navigation to a target (in a real system, you'd use NavigateToPose action)
        target_x = parameters.get('x', 0.0)
        target_y = parameters.get('y', 0.0)

        # Calculate direction to target
        dx = target_x - self.robot_location['x']
        dy = target_y - self.robot_location['y']

        # Simple proportional controller
        twist.linear.x = min(0.5, max(-0.5, dx * 0.5))
        twist.angular.z = min(0.5, max(-0.5, dy * 0.5))

        self.cmd_vel_pub.publish(twist)

        # Update robot location (in a real system, this would come from odometry)
        self.robot_location['x'] += twist.linear.x * 0.1  # Assuming 10Hz update
        self.robot_location['y'] += twist.linear.y * 0.1

    def execute_manipulation(self, target, parameters):
        """Execute manipulation action"""
        action = parameters.get('action', 'none')
        self.get_logger().info(f"Manipulation: {action} {target}")

    def execute_interaction(self, target, parameters):
        """Execute interaction action"""
        self.get_logger().info(f"Interaction: {target} with {parameters}")

    def execute_speech(self, target, parameters):
        """Execute speech action"""
        self.get_logger().info(f"Speech: {target}")

def main(args=None):
    rclpy.init(args=args)
    vla_controller = VLAController()

    try:
        rclpy.spin(vla_controller)
    except KeyboardInterrupt:
        pass
    finally:
        vla_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Create a Speech Recognition Node

Create a separate node for speech recognition. Create `vla_robot_control/nodes/speech_recognizer.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
import pyaudio
import numpy as np
import whisper
import threading
import queue

class SpeechRecognizer(Node):
    def __init__(self):
        super().__init__('speech_recognizer')

        # Initialize Whisper model
        self.model = whisper.load_model("base")

        # Audio parameters
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.record_seconds = 3  # Record for 3 seconds at a time

        # Audio stream
        self.audio = pyaudio.PyAudio()
        self.stream = None

        # Processing queue
        self.audio_queue = queue.Queue()

        # Publisher for transcriptions
        self.transcription_pub = self.create_publisher(String, 'audio_transcription', 10)

        # Start audio capture
        self.start_audio_capture()

        self.get_logger().info("Speech Recognizer initialized")

    def start_audio_capture(self):
        """Start audio capture in a separate thread"""
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        # Start audio capture thread
        threading.Thread(target=self.capture_audio, daemon=True).start()

        # Start processing loop
        self.process_audio_loop()

    def capture_audio(self):
        """Capture audio in a separate thread"""
        while True:
            data = self.stream.read(self.chunk)
            self.audio_queue.put(data)

    def process_audio_loop(self):
        """Process audio in a timer callback"""
        timer_period = 3.5  # Process every 3.5 seconds to allow for 3-second recordings
        self.timer = self.create_timer(timer_period, self.process_audio)

    def process_audio(self):
        """Process collected audio and generate transcription"""
        # Collect audio frames
        frames = []
        try:
            for _ in range(0, int(self.rate / self.chunk * self.record_seconds)):
                if not self.audio_queue.empty():
                    frames.append(self.audio_queue.get())
        except queue.Empty:
            return

        if len(frames) == 0:
            return

        # Convert to numpy array
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        audio_array = audio_data.astype(np.float32) / 32768.0

        # Transcribe with Whisper
        try:
            result = self.model.transcribe(audio_array)
            transcription = result["text"].strip()

            if transcription:  # Only publish if there's actual text
                self.get_logger().info(f"Transcribed: {transcription}")

                # Publish transcription
                msg = String()
                msg.data = transcription
                self.transcription_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error in transcription: {e}")

def main(args=None):
    rclpy.init(args=args)
    speech_recognizer = SpeechRecognizer()

    try:
        rclpy.spin(speech_recognizer)
    except KeyboardInterrupt:
        pass
    finally:
        speech_recognizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Create a Launch File

Create a launch file to start all components together. Create `launch/vla_system.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # VLA Controller Node
    vla_controller = Node(
        package='vla_robot_control',
        executable='vla_controller',
        name='vla_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Speech Recognizer Node
    speech_recognizer = Node(
        package='vla_robot_control',
        executable='speech_recognizer',
        name='speech_recognizer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare launch options
    ld.add_action(declare_use_sim_time)

    # Add nodes
    ld.add_action(vla_controller)
    ld.add_action(speech_recognizer)

    return ld
```

### 4. Update Package Configuration

Update the package configuration in `setup.py`:

```python
from setuptools import setup
from glob import glob
import os

package_name = 'vla_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='VLA system for controlling robots with spoken commands',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vla_controller = vla_robot_control.nodes.vla_controller:main',
            'speech_recognizer = vla_robot_control.nodes.speech_recognizer:main',
        ],
    },
)
```

### 5. Build and Run the System

1. **Build the Package**:

```bash
# Navigate to your workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select vla_robot_control

# Source the workspace
source install/setup.bash
```

2. **Set Up OpenAI API Key**:

Before running, you need to set your OpenAI API key in the code. In the `vla_controller.py` file, replace `'YOUR_OPENAI_API_KEY_HERE'` with your actual API key, or set it as an environment variable:

```bash
export OPENAI_API_KEY='your-api-key-here'
```

3. **Launch the System**:

```bash
# Terminal 1: Launch the VLA system
ros2 launch vla_robot_control vla_system.launch.py
```

4. **Test with a Simulated Robot**:

In another terminal, start a simulated robot in Gazebo:

```bash
# Terminal 2: Launch a simple robot in Gazebo
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

5. **Monitor the System**:

In another terminal, monitor the system:

```bash
# Terminal 3: Monitor topics
ros2 topic echo /audio_transcription
ros2 topic echo /cmd_vel
```

### 6. Test the VLA System

1. Speak a command to your microphone (the system is listening for audio)
2. The speech recognition node will transcribe your speech
3. The VLA controller will receive the transcription and the current camera image
4. The system will generate a plan based on the command and visual context
5. The planned actions will be executed on the simulated robot

Example commands to try:
- "Move to the red object"
- "Go to the table"
- "Pick up the cup"
- "Navigate to the kitchen"

## Troubleshooting

- **API Key Issues**: Ensure your OpenAI API key is correctly set
- **Audio Input**: Check that your microphone is properly configured
- **Model Loading**: The first run might take longer as models are loaded
- **Performance**: Consider using smaller models for faster inference
- **ROS 2 Connections**: Verify that all nodes are properly connected

## Conclusion

You have successfully implemented a Vision-Language-Action system that can interpret spoken commands in the context of visual input and generate appropriate ROS 2 actions. This system demonstrates the integration of multiple AI technologies to create a more natural and intuitive human-robot interaction interface.

## Further Exploration

- Implement more sophisticated vision processing
- Add multimodal transformers for better VLA integration
- Implement safety checks and validation
- Add more complex action planning capabilities
- Integrate with real robot hardware
- Improve the speech recognition component with wake word detection
- Add emotional recognition and response capabilities