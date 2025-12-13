# Cognitive Planning using LLMs → ROS 2 Action Pipelines

## Introduction to Cognitive Planning in Robotics

Cognitive planning in robotics refers to the high-level decision-making process that enables robots to understand complex tasks, reason about their environment, and generate appropriate action sequences to achieve goals. This involves interpreting natural language commands, understanding context, reasoning about available actions, and creating executable plans. Large Language Models (LLMs) have emerged as powerful tools for this cognitive layer, bridging the gap between human-intention and robot-execution.

## Role of LLMs in Cognitive Robotics

LLMs serve as the cognitive layer in modern robotic systems, providing:

- **Natural Language Understanding**: Interpreting human commands in natural language
- **Contextual Reasoning**: Understanding the current situation and relevant constraints
- **Knowledge Integration**: Leveraging pre-trained world knowledge for planning
- **Action Synthesis**: Generating sequences of actions to achieve goals
- **Adaptability**: Handling novel situations through reasoning rather than pre-programmed responses

## Architecture of LLM-Based Cognitive Planning

### 1. Input Processing Layer

The system begins by processing natural language input:

- **Command Parsing**: Extracting the main intent from the user command
- **Context Extraction**: Identifying relevant context (objects, locations, constraints)
- **Ambiguity Resolution**: Clarifying ambiguous or incomplete commands
- **Validation**: Ensuring the command is feasible and safe

### 2. LLM Planning Layer

The core cognitive planning happens in this layer:

- **Prompt Engineering**: Crafting appropriate prompts for the LLM
- **Chain-of-Thought Reasoning**: Breaking down complex tasks into logical steps
- **Knowledge Retrieval**: Accessing relevant information about the robot's capabilities
- **Plan Generation**: Creating a sequence of high-level actions

### 3. Action Mapping Layer

Translating LLM outputs to ROS 2 actions:

- **Action Translation**: Converting natural language actions to ROS 2 action names
- **Parameter Extraction**: Identifying parameters needed for each action
- **Constraint Application**: Ensuring actions adhere to robot and environment constraints
- **Error Handling**: Managing cases where actions cannot be mapped

### 4. ROS 2 Execution Layer

Executing the planned actions:

- **Action Client Management**: Managing ROS 2 action clients
- **State Monitoring**: Tracking the execution status of actions
- **Feedback Integration**: Incorporating sensor feedback into the plan
- **Replanning**: Adjusting the plan based on execution results

## Implementing LLM-Based Cognitive Planning

### 1. System Architecture

Here's an example architecture for integrating LLMs with ROS 2 action pipelines:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose
from example_interfaces.action import Fibonacci  # Example action

import openai  # Or other LLM provider
import json
import re

class LLMBasedPlanner(Node):
    def __init__(self):
        super().__init__('llm_planner')

        # Publishers and subscribers
        self.command_sub = self.create_subscription(
            String, 'natural_language_command', self.command_callback, 10)
        self.status_pub = self.create_publisher(String, 'planning_status', 10)

        # ROS 2 action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Add other action clients as needed

        # LLM configuration
        self.llm_model = "gpt-4"  # Or other model
        self.api_key = self.get_parameter_or('openai_api_key', 'your-api-key').value

        # Robot capabilities database
        self.robot_capabilities = {
            "navigation": {
                "actions": ["go to", "move to", "navigate to"],
                "parameters": ["location", "pose"]
            },
            "manipulation": {
                "actions": ["pick up", "grasp", "place", "put down"],
                "parameters": ["object", "location"]
            },
            "perception": {
                "actions": ["find", "locate", "detect", "identify"],
                "parameters": ["object", "location"]
            }
        }

        self.get_logger().info("LLM-Based Planner initialized")

    def command_callback(self, msg):
        """Process incoming natural language commands"""
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        try:
            # Plan using LLM
            plan = self.generate_plan(command)

            # Execute the plan
            self.execute_plan(plan)

        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")
            self.publish_status(f"Error: {e}")

    def generate_plan(self, command):
        """Generate a plan using LLM"""
        # Craft a prompt for the LLM
        prompt = f"""
        You are a cognitive planner for a humanoid robot. Your task is to break down a natural language command into a sequence of specific actions that the robot can execute.

        Robot capabilities include:
        - Navigation: moving to specific locations
        - Manipulation: picking up and placing objects
        - Perception: detecting and identifying objects

        The command is: "{command}"

        Please provide a JSON list of actions, where each action has:
        - "action": the type of action (e.g., "navigate", "manipulate", "perceive")
        - "target": the specific target of the action
        - "parameters": any additional parameters needed

        Example output format:
        [
            {{"action": "perceive", "target": "red cup", "parameters": {{"location": "kitchen"}}},
            {{"action": "navigate", "target": "kitchen counter", "parameters": {{}}}},
            {{"action": "manipulate", "target": "red cup", "parameters": {{"action": "pick_up"}}}}
        ]

        Be specific about locations and objects. If the command is ambiguous, ask for clarification.
        """

        # Call the LLM
        response = openai.ChatCompletion.create(
            model=self.llm_model,
            messages=[{"role": "user", "content": prompt}],
            temperature=0.1
        )

        # Parse the response
        plan_text = response.choices[0].message['content']

        # Extract JSON from the response
        json_match = re.search(r'\[.*\]', plan_text, re.DOTALL)
        if json_match:
            plan = json.loads(json_match.group())
            return plan
        else:
            raise Exception(f"Could not parse plan from LLM response: {plan_text}")

    def execute_plan(self, plan):
        """Execute the generated plan"""
        for step in plan:
            action_type = step.get('action')
            target = step.get('target')
            parameters = step.get('parameters', {})

            self.get_logger().info(f"Executing: {action_type} {target} with {parameters}")

            if action_type == "navigate":
                self.execute_navigation(target, parameters)
            elif action_type == "manipulate":
                self.execute_manipulation(target, parameters)
            elif action_type == "perceive":
                self.execute_perception(target, parameters)
            else:
                self.get_logger().warn(f"Unknown action type: {action_type}")

    def execute_navigation(self, target, parameters):
        """Execute navigation action"""
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()

        # Set target pose (this would be derived from target name)
        if target == "kitchen counter":
            goal_msg.pose.pose.position.x = 2.0
            goal_msg.pose.pose.position.y = 1.0
            goal_msg.pose.pose.orientation.z = 0.0
        # Add more location mappings as needed

        goal_msg.pose.header.frame_id = 'map'

        # Send goal
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_done_callback)

    def execute_manipulation(self, target, parameters):
        """Execute manipulation action"""
        # Placeholder for manipulation execution
        self.get_logger().info(f"Manipulation: {parameters.get('action')} {target}")

    def execute_perception(self, target, parameters):
        """Execute perception action"""
        # Placeholder for perception execution
        self.get_logger().info(f"Perception: Find {target} in {parameters.get('location')}")

    def navigation_done_callback(self, future):
        """Handle navigation completion"""
        goal_handle = future.result()
        result = goal_handle.get_result_async()
        result.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info(f"Navigation result: {result}")

    def publish_status(self, status):
        """Publish planning status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    planner = LLMBasedPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Prompt Engineering for Robotics

Effective prompt engineering is crucial for reliable cognitive planning:

- **Role Definition**: Clearly define the LLM's role as a robot planner
- **Context Provision**: Provide information about robot capabilities and environment
- **Format Specification**: Specify the expected output format
- **Example Inclusion**: Include examples of correct planning behavior
- **Constraint Definition**: Specify physical and operational constraints

Example prompt template:

```
You are a cognitive planner for a humanoid robot with the following capabilities:
- Navigation: Can move to locations marked on the map
- Manipulation: Can pick up objects within reach (max 1m height, 2kg weight)
- Perception: Can detect and identify objects in view

The robot is currently in the living room. Available locations: kitchen, bedroom, bathroom.

The user says: "{user_command}"

Plan the following:
1. What the robot should do
2. In what order
3. With what parameters

Output as JSON: [{{"action": "...", "target": "...", "parameters": {{}}}}, ...]
```

### 3. Action Mapping Strategies

Different strategies for mapping LLM outputs to ROS 2 actions:

#### A. Direct Mapping
- Predefined mapping between natural language phrases and ROS actions
- Fast and reliable but limited flexibility
- Good for well-defined, common actions

#### B. Semantic Mapping
- Use embeddings to match LLM outputs to available actions
- More flexible but requires similarity thresholding
- Good for handling variations in language

#### C. Code Generation
- LLM generates ROS 2 code snippets directly
- Maximum flexibility but higher complexity
- Requires careful validation of generated code

## Integration with ROS 2 Ecosystem

### 1. Action Server Integration

LLM-based planners work with standard ROS 2 action servers:

```yaml
# Example action interface
# NavigateToPose.action
geometry_msgs/PoseStamped pose
---
nav2_msgs/ResultCode result_code
builtin_interfaces/Time reached_goal stamp
---
bool nudge
```

### 2. Behavior Trees

Combine LLM planning with behavior trees for robust execution:

- Use LLM for high-level plan generation
- Implement detailed execution logic in behavior trees
- Handle contingencies and failures with BT nodes

### 3. Task Management

Manage complex tasks with multiple subtasks:

- Break down LLM-generated plans into manageable chunks
- Track progress and handle interruptions
- Enable resumption of interrupted tasks

## Safety and Reliability Considerations

### 1. Plan Validation

Validate LLM-generated plans before execution:

- Check for physical feasibility
- Verify safety constraints
- Confirm action availability
- Assess environmental appropriateness

### 2. Human Oversight

Implement human-in-the-loop mechanisms:

- Require approval for complex or risky plans
- Provide clear explanations of planned actions
- Enable human intervention during execution

### 3. Fallback Mechanisms

Handle cases where LLM plans fail:

- Implement default behaviors
- Enable replanning with constraints
- Provide graceful degradation

## Challenges and Limitations

### 1. Hallucination and Errors

LLMs may generate incorrect or impossible plans:

- Implement verification steps
- Use constrained output formats
- Cross-reference with known capabilities

### 2. Latency and Real-time Performance

LLM calls can introduce significant latency:

- Optimize prompt sizes
- Consider edge-based LLMs for simple tasks
- Implement caching for common commands

### 3. Context Understanding

LLMs may lack real-time environmental context:

- Provide current state information in prompts
- Implement context updating mechanisms
- Use multimodal inputs when available

## Best Practices

### 1. Hybrid Approaches

Combine LLMs with traditional planning:

- Use LLMs for high-level reasoning
- Implement detailed motion planning traditionally
- Validate LLM outputs with classical methods

### 2. Incremental Development

Start with simple scenarios and expand:

- Begin with well-structured environments
- Gradually increase complexity
- Maintain fallback capabilities

### 3. Evaluation Metrics

Establish metrics for cognitive planning:

- Plan success rate
- Execution time
- Human intervention frequency
- User satisfaction

## Future Directions

### 1. Multimodal Integration

Combine LLMs with vision and other sensors:

- Visual grounding of language commands
- Real-time scene understanding
- Context-aware planning

### 2. Learning from Interaction

Adapt to user preferences and environment:

- Learn from successful interactions
- Adapt to user's language patterns
- Improve with experience

### 3. Collaborative Planning

Enable multi-robot and human-robot collaboration:

- Coordinate multiple agents
- Handle shared environments
- Enable team-based tasks

LLM-based cognitive planning represents a significant advancement in human-robot interaction, enabling more natural and flexible robot control. When properly integrated with ROS 2 action pipelines, it can significantly enhance the autonomy and usability of robotic systems.