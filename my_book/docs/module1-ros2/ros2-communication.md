# ROS 2 Communication: Nodes, Topics, Services, and Actions

The foundational concepts and information presented in this section are primarily derived from the official Robot Operating System 2 documentation [1] and established robotics literature.

ROS 2 is built around a distributed communication architecture that allows different components of a robot system to work together seamlessly. This section will delve into the core communication mechanisms: nodes, topics, services, and actions.

## 1. Nodes

A **node** is an executable process that performs computation. In a typical robot system, you'll have many nodes working concurrently. Each node should be designed to perform a single, specific task (e.g., a node for reading camera data, a node for controlling motors, a node for path planning). [1]

### Key Characteristics: [1]
*   **Modularity**: Nodes are independent, making development and debugging easier.
*   **Isolation**: If one node crashes, it doesn't necessarily bring down the entire system.
*   **Reusability**: Nodes can be reused in different robotic applications.

### Node Architecture Diagram

```
+-------------------------------------+
|           ROS 2 System              |
|                                     |
|  +------------+    +------------+   |
|  |   Node A   |    |   Node B   |   |
|  |            |    |            |   |
|  | - Publisher|    | - Subscriber|  |
|  | - Service  |    | - Action   |   |
|  +------------+    +------------+   |
|         |                 |          |
|         +--------+--------+          |
|                  |                   |
|    +--------------------------+      |
|    |     Communication Layer  |      |
|    |         (DDS/RMW)        |      |
|    +--------------------------+      |
|                  |                   |
|         +--------+--------+          |
|         |                 |          |
|  +------------+    +------------+   |
|  |   Topic    |    |  Service   |   |
|  |   Channel  |    |    Bus     |   |
|  +------------+    +------------+   |
|                                     |
+-------------------------------------+
```

This diagram shows how nodes interact with each other through the communication layer using different types of channels.

## 2. Topics

**Topics** are a fundamental element for asynchronous, many-to-many communication in ROS 2. They operate on a publish/subscribe model: [1]

*   **Publisher**: A node that sends (publishes) messages to a topic.
*   **Subscriber**: A node that receives (subscribes) messages from a topic.

When a message is published to a topic, all nodes subscribed to that topic receive a copy of the message. Topics are ideal for continuous data streams like sensor readings (e.g., LiDAR, camera images, IMU data) or motor commands.

### Examples:
*   `/camera/image_raw`: Publishes raw image data.
*   `/robot/odom`: Publishes odometry (position and orientation) data.
*   `/cmd_vel`: Subscribes to velocity commands to control the robot's movement.

### Topic Communication Diagram

```
                    +------------------+
                    |    Topic: /cmd_vel    |
                    +------------------+
                             |
              +--------------+--------------+
              |                             |
    +--------------------+    +--------------------+
    |   Publisher Node   |    |   Publisher Node   |
    |                    |    |                    |
    | - Sends velocity   |    | - Sends velocity   |
    |   commands         |    |   commands         |
    +--------------------+    +--------------------+
              |                             |
              |                             |
    +--------------------+    +--------------------+
    |   Subscriber Node  |    |   Subscriber Node  |
    |                    |    |                    |
    | - Receives velocity|    | - Receives velocity|
    |   commands         |    |   commands         |
    +--------------------+    +--------------------+
```

This diagram illustrates the publish/subscribe pattern where multiple publishers can send to a topic and multiple subscribers can receive from the same topic.

## 3. Services

**Services** provide a synchronous request/reply communication mechanism. Unlike topics, services are one-to-one communication, where a client sends a request to a service, and the service processes the request and sends back a single reply. [1] This is useful for tasks that require a specific action to be performed and a direct response, such as:

### Service Communication Diagram

```
+------------------+       REQUEST        +------------------+
|                  | ----------------->  |                  |
|   Client Node    |                     |   Service Node   |
|                  | <-----------------  |                  |
| (Requests action)|    RESPONSE         | (Performs action)|
+------------------+                     +------------------+

Example:
+------------------+       "/get_map"     +------------------+
|   Navigation     | ----------------->  |   Map Server     |
|   Client         |                     |                  |
| (Wants map)      | <-----------------  | (Provides map)   |
|                  |    Map Data         |                  |
+------------------+                     +------------------+
```

This diagram shows the synchronous request/response pattern of services.

*   Triggering a robot to pick up an object.
*   Querying the robot's current state.
*   Changing a configuration parameter.

### Examples:
*   `/set_pose`: Client requests the robot to move to a specific pose; service replies with success/failure.
*   `/get_map`: Client requests a map of the environment; service replies with the map data.

## 4. Actions

**Actions** are designed for long-running, goal-oriented tasks that require periodic feedback and the ability to be preempted (cancelled). They build upon the service concept but add three key features: [1]

*   **Goal**: The desired outcome of the action.
*   **Feedback**: Intermediate updates on the progress of the action.
*   **Result**: The final outcome of the action.

Actions are particularly useful for navigation, manipulation, or complex sequence execution in humanoid robots.

### Examples:
*   `/navigate_to_pose`: Goal is a target pose; feedback includes current position, remaining distance; result is success/failure.
*   `/pick_object`: Goal is to pick a specific object; feedback includes gripper status, object detection progress; result is success/failure.

### Action Communication Diagram

```
+------------------+        GOAL          +------------------+
|                  | ----------------->  |                  |
|   Action Client  | (Request to         |   Action Server  |
|                  |  perform task)      |                  |
| (Requests task)  |                     | (Performs task)  |
+------------------+                     +------------------+
         |                                        |
         |          FEEDBACK (Periodic)           |
         | <--------------------------------------|
         |         (Progress updates)             |
         |                                        |
         |          RESULT (Final)                |
         | <--------------------------------------|
         |        (Success/Failure)               |
         |                                        |
         |          CANCEL (Optional)             |
         | -------------------------------------> |
         |        (Stop current task)             |
+------------------+                     +------------------+
|   User Interface |                     |   Robot System   |
|   (Can cancel)   |                     |   (Reports back) |
+------------------+                     +------------------+

Example - Navigation Action:
+------------------+        GOAL:          +------------------+
|   Navigation     | (x: 5.0, y: 3.0)   |   Navigation     |
|   Client         | ----------------->  |   Server         |
| (Wants to move   |                     | (Moves robot,    |
|  to a location)  |                     |  reports progress)|
|                  |                     |                  |
+------------------+                     +------------------+
         |                                        |
         |     FEEDBACK: (2.5/5.0 to dest)       |
         | <--------------------------------------|
         |                                        |
         |     FEEDBACK: (4.8/5.0 to dest)       |
         | <--------------------------------------|
         |                                        |
         |        RESULT: (Arrived at dest)       |
         | <--------------------------------------|
```

This diagram shows the more complex communication pattern of actions, which includes ongoing feedback and the possibility of cancellation.

## Interconnection

These communication primitives can be interconnected to create complex behaviors. For instance, a navigation action might internally use a service to query map data and topics to publish velocity commands to the motors based on path planning.

Understanding these core communication concepts is vital for developing robust and scalable ROS 2 applications for humanoid robotics.