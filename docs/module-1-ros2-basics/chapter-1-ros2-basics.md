---
title: Chapter 1 - ROS 2 Basics
sidebar_position: 1
---

# Chapter 1: ROS 2 Basics

<div class="learning-objectives">
## Learning Objectives
- Understand the concept of ROS 2 as a robot "nervous system"
- Learn about Nodes, Topics, and Services in ROS 2
- Explore simple examples of ROS 2 communication patterns
</div>

## ROS 2 as a Robot "Nervous System"

ROS 2 (Robot Operating System 2) serves as the "nervous system" of a robot, enabling different components to communicate and coordinate with each other. Just as the nervous system allows different parts of a body to work together, ROS 2 allows different software components (nodes) to interact seamlessly.

## Nodes, Topics, and Services

### Nodes
Nodes are the fundamental building blocks of ROS 2. A node is a process that performs computation. In a typical robot system, you might have:
- Sensor nodes that publish data from cameras, lidars, or other sensors
- Control nodes that process data and make decisions
- Actuator nodes that control motors or other hardware

```
Node Communication Example:
┌─────────────┐    publishes    ┌─────────────┐
│  Publisher  │ ──────────────▶ │   Topic     │
│   Node      │                 │   (data)    │
└─────────────┘                 └─────────────┘
                                        │
                                        │ subscribes
                                        ▼
                               ┌─────────────────┐
                               │   Subscriber    │
                               │     Node        │
                               └─────────────────┘
```

### Topics
Topics enable message passing between nodes through a publish-subscribe model:
- Publishers send messages to a topic
- Subscribers receive messages from a topic
- Multiple nodes can publish or subscribe to the same topic

```
Topic Communication Pattern:
┌─────────────┐    publishes    ┌─────────────┐
│  Sensor     │ ──────────────▶ │   sensor    │
│  Publisher  │                 │   data      │
└─────────────┘                 └─────────────┘
                                        │
                                        │ subscribes
                                        ▼
                               ┌─────────────────┐
                               │   Controller    │
                               │   Subscriber    │
                               └─────────────────┘
```

### Services
Services provide request-response communication between nodes:
- A client sends a request to a service
- A server processes the request and returns a response
- This is useful for operations that require a specific response

```
Service Communication Pattern:
┌─────────────┐    request      ┌─────────────┐
│  Client     │ ──────────────▶ │   Service   │
│  Node       │                 │   Server    │
└─────────────┘                 └─────────────┘
                                        │
                                        │ response
                                        ▼
                               ┌─────────────────┐
                               │   Client Node   │
                               │  (receives)     │
                               └─────────────────┘
```

## Simple Examples of ROS 2 Communication Patterns

### Topic Communication Example
A simple publisher-subscriber pattern where a sensor node publishes data and a control node subscribes to it:

```python
# Publisher example
import rclpy
from std_msgs.msg import String

def publisher_example():
    rclpy.init()
    node = rclpy.create_node('sensor_publisher')
    publisher = node.create_publisher(String, 'sensor_data', 10)

    msg = String()
    msg.data = 'Sensor reading: 42.5'
    publisher.publish(msg)
    node.destroy_node()
    rclpy.shutdown()

### Service Communication Example
A request-response pattern where a controller requests a specific action:

```python
# Service client example
import rclpy
from example_interfaces.srv import SetBool

def service_client_example():
    rclpy.init()
    node = rclpy.create_node('service_client')

    client = node.create_client(SetBool, 'toggle_motor')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available')

    request = SetBool.Request()
    request.data = True
    future = client.call_async(request)
    # Handle response when available
    rclpy.spin_until_future_complete(node, future)
```

```python
# Service server example
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class ToggleMotorService(Node):
    def __init__(self):
        super().__init__('toggle_motor_server')
        self.srv = self.create_service(SetBool, 'toggle_motor', self.toggle_motor_callback)

    def toggle_motor_callback(self, request, response):
        self.get_logger().info(f'Request received: {request.data}')
        response.success = True
        response.message = 'Motor toggled successfully'
        return response
```

<div class="mini-task">
Think about three actions a humanoid robot might perform that would require multiple nodes working together. Consider what different types of nodes would be involved in each action.
</div>

## Summary and Key Takeaways
- ROS 2 provides a communication framework for robot components
- Nodes are the basic computational units
- Topics enable asynchronous communication via publish-subscribe
- Services enable synchronous request-response communication