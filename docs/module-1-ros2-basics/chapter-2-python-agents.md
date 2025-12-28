---
title: Chapter 2 - Python Agents with rclpy
sidebar_position: 2
---

# Chapter 2: Python Agents with rclpy

<div class="learning-objectives">
## Learning Objectives
- Understand how Python agents communicate with ROS 2 using rclpy
- Learn about publishing and subscribing patterns in Python
- Explore the message flow between agents and controllers
</div>

## Python Agents with rclpy

rclpy is the Python client library for ROS 2. It allows Python programs to interact with ROS 2 systems by creating nodes, publishing and subscribing to topics, and providing or using services.

### Basic Node Structure

A simple ROS 2 Python node using rclpy follows this pattern:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Agent → Controller Message Flow

In a typical agent-controller setup:
1. The agent publishes commands or requests to specific topics
2. The controller subscribes to these topics and processes the commands
3. The controller may respond via services or publish status updates to other topics

```
Agent → Controller Message Flow:
┌─────────────┐    publishes    ┌─────────────────┐
│ Python      │ ──────────────▶ │ ROS 2 Topic     │
│ Agent       │                 │ (commands)      │
└─────────────┘                 └─────────────────┘
                                        │
                                        │ subscribes
                                        ▼
                               ┌─────────────────┐
                               │ Robot           │
                               │ Controller      │
                               │ (processes)     │
                               └─────────────────┘
                                        │
                                        │ publishes
                                        ▼
                               ┌─────────────────┐
                               │ ROS 2 Topic     │
                               │ (status)        │
                               └─────────────────┘
                                        │
                                        │ subscribes
                                        ▼
                               ┌─────────────────┐
                               │ Python Agent    │
                               │ (monitors)      │
                               └─────────────────┘
```

The message flow shows how Python agents can send commands to robot controllers through ROS 2 topics and services, creating a distributed system where decision-making and actuation are separated.

## Code Examples

Here are practical examples of implementing publisher, subscriber, and service nodes with rclpy:

### Publisher Node Example
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class PostureCommandPublisher(Node):
    def __init__(self):
        super().__init__('posture_command_publisher')
        self.publisher = self.create_publisher(JointState, 'posture_commands', 10)

    def publish_posture(self, joint_positions):
        msg = JointState()
        msg.name = ['joint1', 'joint2', 'joint3']  # Example joint names
        msg.position = joint_positions
        self.publisher.publish(msg)
        self.get_logger().info(f'Published posture: {joint_positions}')
```

### Subscriber Node Example
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class PostureCommandSubscriber(Node):
    def __init__(self):
        super().__init__('posture_command_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'posture_commands',
            self.posture_callback,
            10)

    def posture_callback(self, msg):
        self.get_logger().info(f'Received posture command: {msg.position}')
        # Process the posture command here
```

## Mini-Task: Write Pseudocode for Publishing a Posture Command

<div class="mini-task">
Write pseudocode that demonstrates how a Python agent might publish a posture command to control a humanoid robot's joint positions.
</div>

## Summary and Key Takeaways
- rclpy enables Python programs to interact with ROS 2
- Publishers send messages to topics, subscribers receive them
- The agent-controller pattern is common in robotics systems