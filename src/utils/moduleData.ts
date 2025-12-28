// Module data with topics mapped to weeks
// This structure maps the existing module content to the required weeks

export const moduleData = [
  {
    moduleId: 1,
    moduleName: "ROS 2 Basics",
    chapters: [
      {
        chapterId: 1,
        title: "Chapter 1 - ROS 2 Basics",
        description: "Understanding ROS 2 as a robot nervous system, Nodes, Topics, and Services",
        weeks: [3, 4, 5],
        content: `# Chapter 1: ROS 2 Basics

## Learning Objectives
- Understand the concept of ROS 2 as a robot "nervous system"
- Learn about Nodes, Topics, and Services in ROS 2
- Explore simple examples of ROS 2 communication patterns

## ROS 2 as a Robot "Nervous System"
ROS 2 (Robot Operating System 2) serves as the "nervous system" of a robot, enabling different components to communicate and coordinate with each other. Just as the nervous system allows different parts of a body to work together, ROS 2 allows different software components (nodes) to interact seamlessly.

## Nodes, Topics, and Services
### Nodes
Nodes are the fundamental building blocks of ROS 2. A node is a process that performs computation. In a typical robot system, you might have:
- Sensor nodes that publish data from cameras, lidars, or other sensors
- Control nodes that process data and make decisions
- Actuator nodes that control motors or other hardware

### Topics
Topics enable message passing between nodes through a publish-subscribe model:
- Publishers send messages to a topic
- Subscribers receive messages from a topic
- Multiple nodes can publish or subscribe to the same topic

### Services
Services provide request-response communication between nodes:
- A client sends a request to a service
- A server processes the request and returns a response
- This is useful for operations that require a specific response

## Summary and Key Takeaways
- ROS 2 provides a communication framework for robot components
- Nodes are the basic computational units
- Topics enable asynchronous communication via publish-subscribe
- Services enable synchronous request-response communication`
      },
      {
        chapterId: 2,
        title: "Chapter 2 - Python Agents with rclpy",
        description: "Understanding how Python agents communicate with ROS 2 using rclpy",
        weeks: [3, 4, 5],
        content: `# Chapter 2: Python Agents with rclpy

## Learning Objectives
- Understand how Python agents communicate with ROS 2 using rclpy
- Learn about publishing and subscribing patterns in Python
- Explore the message flow between agents and controllers

## Python Agents with rclpy
rclpy is the Python client library for ROS 2. It allows Python programs to interact with ROS 2 systems by creating nodes, publishing and subscribing to topics, and providing or using services.

### Basic Node Structure
A simple ROS 2 Python node using rclpy follows this pattern:

\`\`\`python
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
\`\`\`

## Agent → Controller Message Flow
In a typical agent-controller setup:
1. The agent publishes commands or requests to specific topics
2. The controller subscribes to these topics and processes the commands
3. The controller may respond via services or publish status updates to other topics

The message flow shows how Python agents can send commands to robot controllers through ROS 2 topics and services, creating a distributed system where decision-making and actuation are separated.

## Summary and Key Takeaways
- rclpy enables Python programs to interact with ROS 2
- Publishers send messages to topics, subscribers receive them
- The agent-controller pattern is common in robotics systems`
      },
      {
        chapterId: 3,
        title: "Chapter 3 - URDF for Humanoids",
        description: "Understanding URDF (Unified Robot Description Format) for humanoid robots",
        weeks: [3, 4, 5],
        content: `# Chapter 3: URDF for Humanoids

## Learning Objectives
- Understand the basics of URDF (Unified Robot Description Format)
- Learn about links, joints, and sensors in robot descriptions
- Explore how URDF applies specifically to humanoid robots

## URDF Structure and Components
URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including:

### Links
Links represent rigid bodies of the robot. Each link has:
- Physical properties (mass, inertia)
- Visual properties (shape, color, mesh)
- Collision properties (collision geometry)

### Joints
Joints connect links and define how they can move relative to each other:
- Fixed joints (no movement)
- Revolute joints (rotational movement)
- Prismatic joints (linear movement)
- Continuous joints (unlimited rotation)

### Sensors
Sensors define where sensors are located on the robot and their properties.

## Humanoid URDF Structure
A humanoid robot typically includes:
- Torso (main body)
- Head
- Arms (shoulders, elbows, wrists)
- Legs (hips, knees, ankles)
- Feet

## Summary and Key Takeaways
- URDF describes robot structure using links and joints
- Links represent rigid bodies, joints define movement between them
- Humanoid robots have specific structural requirements`
      }
    ]
  },
  {
    moduleId: 2,
    moduleName: "Gazebo & Unity Simulation",
    chapters: [
      {
        chapterId: 1,
        title: "Chapter 1 - Gazebo Physics Simulation",
        description: "Core concepts of physics simulation in Gazebo",
        weeks: [6, 7],
        content: `# Chapter 1: Gazebo Physics Simulation

## Learning Objectives
- Understand the core concepts of physics simulation in Gazebo
- Explain how gravity, collisions, and rigid-body dynamics work in simulated environments
- Describe how physics parameters affect robot behavior and interactions with the environment
- Create simple robot simulation environments in Gazebo

## Introduction to Gazebo Physics
Gazebo is a 3D simulation environment that provides the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. At its core, Gazebo uses physics engines to simulate realistic interactions between objects in the virtual world.

### Key Physics Concepts in Gazebo
#### Gravity
Gravity is a fundamental force in Gazebo that affects all objects with mass. The gravity vector is typically set to (0, 0, -9.81) to simulate Earth's gravitational pull. You can modify this in your world files to simulate different environments like the Moon or Mars.

#### Collisions
Collision detection in Gazebo involves determining when two objects make contact. Gazebo uses collision engines like Bullet, ODE, and DART to handle these interactions. The collision properties are defined in the <collision> tag of your URDF/SDF files.

#### Rigid-Body Dynamics
Rigid-body dynamics govern how objects move and interact when forces are applied. Each object has properties like mass, inertia, and friction coefficients that determine its behavior in the simulation.

## Summary
In this chapter, we've covered the fundamental physics concepts in Gazebo simulation, including gravity, collisions, and rigid-body dynamics. Understanding these concepts is crucial for creating realistic robot simulations that can effectively train and test robotic systems before deployment in the real world.`
      },
      {
        chapterId: 2,
        title: "Chapter 2 - Unity Digital Twins",
        description: "Creating digital twins in Unity for robot simulation",
        weeks: [6, 7],
        content: `# Chapter 2: Unity Digital Twins

## Learning Objectives
- Understand the concept of digital twins in robotics
- Learn how to create realistic 3D environments in Unity
- Explore integration between Unity and ROS 2 systems

## Introduction to Digital Twins
A digital twin is a virtual representation of a physical robot or system that mirrors its real-world counterpart in real-time. In robotics, digital twins serve as powerful tools for:
- Testing algorithms in safe virtual environments
- Training AI models before real-world deployment
- Predictive maintenance and system optimization

## Unity for Robotics Simulation
Unity provides a robust platform for creating digital twins with:
- High-fidelity 3D rendering
- Physics simulation capabilities
- Extensive asset library for robotics components
- Integration with ROS 2 through the Unity Robotics Hub

## Creating Robot Models in Unity
When creating digital twins in Unity:
1. Import 3D models of robot components
2. Configure physics properties for realistic simulation
3. Set up sensors and actuators for data collection
4. Implement control systems that mirror real-world behavior

## Integration with ROS 2
Unity can communicate with ROS 2 systems through:
- ROS TCP Connector for message passing
- Sensor data publishing to ROS topics
- Command subscription from ROS services
- Synchronized time and coordinate systems

## Summary
Digital twins in Unity provide a powerful platform for robot development, allowing for safe testing and validation before deployment to physical systems.`
      },
      {
        chapterId: 3,
        title: "Chapter 3 - Simulated Sensors",
        description: "Implementing and using simulated sensors in robot simulation",
        weeks: [6, 7],
        content: `# Chapter 3: Simulated Sensors

## Learning Objectives
- Understand different types of simulated sensors for robots
- Learn how to implement sensor simulation in virtual environments
- Explore the relationship between simulated and real sensor data

## Types of Simulated Sensors
Robot simulation typically includes various sensor types:

### Vision Sensors
- RGB cameras for visual perception
- Depth cameras for 3D reconstruction
- Stereo cameras for depth estimation
- Thermal cameras for heat signature detection

### Range Sensors
- LiDAR for 3D mapping and navigation
- Ultrasonic sensors for proximity detection
- Infrared sensors for short-range detection

### Inertial Sensors
- IMUs for orientation and acceleration
- Gyroscopes for rotational measurement
- Accelerometers for linear acceleration

## Sensor Accuracy and Noise
Simulated sensors can include realistic noise models and accuracy limitations:
- Gaussian noise for realistic sensor readings
- Occlusion handling for blocked sensors
- Range limitations and field-of-view constraints

## Summary
Simulated sensors provide a safe and cost-effective way to test perception algorithms before deployment on physical robots. Proper modeling of sensor characteristics ensures effective transfer from simulation to reality.`
      }
    ]
  },
  {
    moduleId: 3,
    moduleName: "NVIDIA Isaac AI Brain",
    chapters: [
      {
        chapterId: 1,
        title: "Chapter 1 - Isaac Sim Perception",
        description: "Perception systems using NVIDIA Isaac Sim",
        weeks: [8, 9, 10],
        content: `# Chapter 1: Isaac Sim Perception

## Learning Objectives
- Understand perception systems in NVIDIA Isaac Sim
- Learn about sensor simulation and data processing
- Explore computer vision applications in robotics

## Introduction to Isaac Sim Perception
NVIDIA Isaac Sim provides advanced perception capabilities for robotics simulation, including:
- High-fidelity sensor simulation
- Realistic lighting and environmental effects
- Synthetic data generation for AI training

## Sensor Simulation in Isaac Sim
Isaac Sim offers various perception sensors:
- RGB cameras with realistic distortion models
- Depth sensors with accurate distance measurements
- LiDAR with configurable resolution and range
- IMU and other inertial measurement units

## Synthetic Data Generation
Isaac Sim can generate large datasets for training AI models:
- Photorealistic image datasets
- Ground truth annotations
- Diverse environmental conditions
- Multiple sensor modalities

## Perception Pipeline
The perception pipeline in Isaac Sim typically includes:
1. Sensor data acquisition
2. Preprocessing and calibration
3. Feature extraction and object detection
4. Scene understanding and interpretation

## Summary
Isaac Sim provides comprehensive tools for developing and testing perception systems in robotics, enabling safe and efficient development of AI-powered robots.`
      },
      {
        chapterId: 2,
        title: "Chapter 2 - Isaac ROS VSLAM",
        description: "Visual Simultaneous Localization and Mapping with Isaac ROS",
        weeks: [8, 9, 10],
        content: `# Chapter 2: Isaac ROS VSLAM

## Learning Objectives
- Understand Visual SLAM concepts and implementation
- Learn how to integrate VSLAM systems with Isaac ROS
- Explore mapping and localization in robotics

## Introduction to Visual SLAM
Visual Simultaneous Localization and Mapping (VSLAM) enables robots to:
- Build maps of unknown environments
- Localize themselves within these maps
- Navigate autonomously using visual input

## Isaac ROS VSLAM Components
Isaac ROS provides optimized VSLAM capabilities:
- GPU-accelerated processing
- Integration with NVIDIA hardware
- ROS 2 message compatibility
- Real-time performance optimization

## VSLAM Pipeline
The VSLAM pipeline includes:
1. Visual feature extraction
2. Feature matching and tracking
3. Pose estimation and optimization
4. Map building and maintenance

## Applications in Robotics
VSLAM is crucial for:
- Autonomous navigation
- Exploration tasks
- Environment mapping
- Localization in GPS-denied environments

## Summary
Isaac ROS VSLAM provides powerful tools for developing autonomous robots capable of understanding and navigating their environments using visual input.`
      },
      {
        chapterId: 3,
        title: "Chapter 3 - Nav2 Path Planning",
        description: "Navigation 2 path planning and motion control",
        weeks: [8, 9, 10],
        content: `# Chapter 3: Nav2 Path Planning

## Learning Objectives
- Understand Navigation 2 (Nav2) framework
- Learn path planning algorithms and implementation
- Explore motion control for robot navigation

## Introduction to Nav2
Navigation 2 (Nav2) is the ROS 2 navigation framework that provides:
- Global and local path planning
- Obstacle avoidance and collision prevention
- Motion control and trajectory execution
- Behavior trees for complex navigation tasks

## Path Planning Algorithms
Nav2 includes various path planning algorithms:
- A* for optimal path finding
- Dijkstra for weighted graph traversal
- RRT for high-dimensional spaces
- Custom plugins for specialized applications

## Navigation System Components
The Nav2 system consists of:
1. Global planner for route calculation
2. Local planner for obstacle avoidance
3. Controller for trajectory execution
4. Recovery behaviors for edge cases

## Integration with Isaac
Nav2 integrates seamlessly with Isaac systems:
- GPU-accelerated planning
- Sensor fusion capabilities
- Real-time performance optimization
- Simulation-to-reality transfer

## Summary
Nav2 provides a comprehensive navigation framework that enables robots to autonomously navigate complex environments while avoiding obstacles and following optimal paths.`
      }
    ]
  },
  {
    moduleId: 4,
    moduleName: "VLA & LLM Robotics",
    chapters: [
      {
        chapterId: 1,
        title: "Chapter 1 - Voice to Action",
        description: "Converting voice commands to robotic actions",
        weeks: [11, 12, 13],
        content: `# Chapter 1: Voice to Action

## Learning Objectives
- Understand voice command processing in robotics
- Learn about speech recognition and natural language understanding
- Explore voice-to-action conversion systems

## Introduction to Voice to Action
Voice to action systems enable robots to:
- Interpret human voice commands
- Convert speech to executable actions
- Provide natural human-robot interaction

## Speech Recognition Pipeline
The voice to action pipeline includes:
1. Audio input and preprocessing
2. Speech-to-text conversion
3. Natural language understanding
4. Command mapping and execution

## Natural Language Processing
NLP in voice to action systems:
- Parses command structure
- Identifies key entities and actions
- Maps to robot capabilities
- Handles ambiguous or complex commands

## Integration with Robot Systems
Voice commands integrate with:
- Navigation systems
- Manipulation capabilities
- Communication modules
- Safety and validation checks

## Summary
Voice to action systems provide intuitive interfaces for human-robot interaction, enabling natural communication between humans and robotic systems.`
      },
      {
        chapterId: 2,
        title: "Chapter 2 - Cognitive Planning",
        description: "AI-driven cognitive planning for robotic tasks",
        weeks: [11, 12, 13],
        content: `# Chapter 2: Cognitive Planning

## Learning Objectives
- Understand cognitive planning in AI robotics
- Learn about hierarchical task planning
- Explore decision-making algorithms for robots

## Introduction to Cognitive Planning
Cognitive planning enables robots to:
- Reason about complex tasks
- Plan multi-step actions
- Adapt to changing environments
- Make decisions under uncertainty

## Hierarchical Task Planning
Cognitive planning uses hierarchical structures:
- High-level goal specification
- Task decomposition into subtasks
- Action-level execution planning
- Feedback and adaptation mechanisms

## Planning Algorithms
Cognitive planning employs various algorithms:
- Classical planning (STRIPS, PDDL)
- Probabilistic planning (MCP, POMCP)
- Learning-based planning (RL, IL)
- Multi-agent coordination planning

## Integration with LLMs
Large language models enhance cognitive planning:
- Natural language task specification
- Commonsense reasoning
- Analogical transfer between tasks
- Explanation and interpretability

## Summary
Cognitive planning systems enable robots to perform complex tasks by reasoning about goals, actions, and environmental constraints in sophisticated ways.`
      },
      {
        chapterId: 3,
        title: "Chapter 3 - Autonomous Humanoid",
        description: "Creating fully autonomous humanoid robots",
        weeks: [11, 12, 13],
        content: `# Chapter 3: Autonomous Humanoid

## Learning Objectives
- Understand the components of autonomous humanoid robots
- Learn about integration of perception, planning, and control
- Explore complete autonomy systems

## Autonomous Humanoid Architecture
A complete autonomous humanoid system includes:
- Perception systems for environment understanding
- Planning systems for decision making
- Control systems for movement execution
- Learning systems for adaptation and improvement

## Integration Challenges
Autonomous humanoid integration faces:
- Real-time performance requirements
- Multi-modal sensor fusion
- Robust control in dynamic environments
- Safe human-robot interaction

## Control Hierarchies
Humanoid control operates at multiple levels:
- High-level task planning
- Mid-level trajectory generation
- Low-level motor control
- Balance and stability maintenance

## Learning and Adaptation
Autonomous systems include:
- Reinforcement learning for skill acquisition
- Imitation learning from demonstrations
- Transfer learning between tasks
- Continuous adaptation to new environments

## Summary
Autonomous humanoid robots represent the convergence of multiple AI and robotics technologies, creating systems capable of independent operation in human environments.`
      }
    ]
  }
];

// Function to get topics for a specific week range
export function getTopicsForWeek(week: number) {
  const topics: any[] = [];

  moduleData.forEach(module => {
    module.chapters.forEach(chapter => {
      if (chapter.weeks.includes(week)) {
        topics.push({
          ...chapter,
          moduleName: module.moduleName,
          moduleId: module.moduleId
        });
      }
    });
  });

  return topics;
}

// Function to get topics for a week range (e.g., weeks 1-2)
export function getTopicsForWeekRange(startWeek: number, endWeek: number) {
  const topics: any[] = [];

  for (let week = startWeek; week <= endWeek; week++) {
    const weekTopics = getTopicsForWeek(week);
    weekTopics.forEach(topic => {
      if (!topics.some(t => t.chapterId === topic.chapterId)) {
        topics.push(topic);
      }
    });
  }

  return topics;
}