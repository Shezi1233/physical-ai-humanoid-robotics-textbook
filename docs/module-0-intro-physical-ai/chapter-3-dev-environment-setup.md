---
title: Chapter 3 - Course Roadmap & Dev Environment
sidebar_position: 3
---

# Chapter 3: Course Roadmap & Development Environment

<div class="learning-objectives">
## Learning Objectives
- Understand the 13-week course structure and what each module covers
- Set up a complete development environment for robotics AI
- Install and verify ROS 2, Gazebo, and Python tools
- Run your first simple robot simulation to confirm everything works
</div>

## Course Roadmap: 13 Weeks to Physical AI

This course is structured as a progressive journey through the entire Physical AI stack. Each module builds on the previous one:

```
13-Week Course Roadmap:

Week 1-2:   Introduction to Physical AI (YOU ARE HERE)
             ├── What is Physical AI?
             ├── The Humanoid Robotics Landscape
             └── Dev Environment Setup
                    │
Week 3-5:   Module 1 - ROS 2 Basics ──────────────────┐
             ├── ROS 2 as a robot nervous system        │
             ├── Python agents with rclpy               │
             └── URDF for humanoid robots               │
                    │                                    │
Week 6-7:   Module 2 - Simulation ◄────────────────────┘
             ├── Gazebo physics simulation
             ├── Unity digital twins
             └── Simulated sensors (LiDAR, Depth, IMU)
                    │
Week 8-10:  Module 3 - NVIDIA Isaac AI Brain
             ├── Isaac Sim perception
             ├── Isaac ROS VSLAM
             └── Nav2 path planning for humanoids
                    │
Week 11-12: Module 4 - VLA & LLM Robotics
             ├── Voice to action pipelines
             ├── Cognitive planning with LLMs
             └── Capstone: The autonomous humanoid
                    │
Week 13:    Final Integration & Demo
             └── Complete autonomous humanoid project
```

### Module Progression

| Module | Weeks | Focus | Key Skills |
|---|---|---|---|
| **Module 0** | 1-2 | Introduction & Setup | Concepts, environment, tools |
| **Module 1** | 3-5 | ROS 2 Basics | Nodes, topics, services, URDF |
| **Module 2** | 6-7 | Simulation | Gazebo, Unity, sensor simulation |
| **Module 3** | 8-10 | AI & Navigation | Isaac Sim, VSLAM, path planning |
| **Module 4** | 11-13 | Autonomous AI | VLA, LLMs, full autonomy |

## Development Environment Overview

To work through this course, you'll need the following tools installed:

```
Development Environment Stack:

┌─────────────────────────────────────────────────┐
│  Your Computer                                   │
│                                                   │
│  ┌──────────────┐  ┌──────────────────────────┐ │
│  │   Ubuntu      │  │  OR  Windows + WSL2      │ │
│  │   22.04 LTS   │  │      (Ubuntu 22.04)      │ │
│  └──────┬───────┘  └──────────┬───────────────┘ │
│         │                      │                  │
│         └──────────┬───────────┘                  │
│                    │                              │
│         ┌──────────▼──────────┐                  │
│         │    ROS 2 Humble     │                  │
│         │  (LTS until 2027)   │                  │
│         └──────────┬──────────┘                  │
│                    │                              │
│    ┌───────────────┼───────────────┐             │
│    │               │               │             │
│    ▼               ▼               ▼             │
│ ┌───────┐   ┌──────────┐   ┌──────────┐        │
│ │Python │   │ Gazebo   │   │  VS Code │        │
│ │ 3.10+ │   │  Sim     │   │  + ROS   │        │
│ │       │   │          │   │  plugins │        │
│ └───────┘   └──────────┘   └──────────┘        │
└─────────────────────────────────────────────────┘
```

### Required Software

| Software | Version | Purpose |
|---|---|---|
| **Ubuntu** | 22.04 LTS | Operating system (native or WSL2) |
| **ROS 2** | Humble Hawksbill | Robot middleware framework |
| **Python** | 3.10+ | Programming language |
| **Gazebo** | Fortress or Garden | Robot simulation |
| **VS Code** | Latest | Code editor with ROS extensions |
| **Git** | Latest | Version control |
| **colcon** | Latest | ROS 2 build tool |

## Setting Up Your Environment

### Option A: Ubuntu Native Installation

If you're running Ubuntu 22.04 natively, this is the most straightforward path.

#### Step 1: System Update

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl gnupg2 lsb-release build-essential
```

#### Step 2: Install ROS 2 Humble

```bash
# Add the ROS 2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
```

#### Step 3: Environment Setup

```bash
# Add ROS 2 to your shell environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install colcon build tool
sudo apt install -y python3-colcon-common-extensions

# Install additional ROS 2 packages
sudo apt install -y ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro
```

### Option B: Windows with WSL2

If you're on Windows, WSL2 (Windows Subsystem for Linux) provides a full Linux environment.

#### Step 1: Install WSL2

```powershell
# Run in PowerShell as Administrator
wsl --install -d Ubuntu-22.04
```

#### Step 2: Set Up GUI Support

Windows 11 has built-in WSLg support for GUI applications. For Windows 10, install an X server like VcXsrv.

#### Step 3: Follow Ubuntu Installation

Once inside your WSL2 Ubuntu terminal, follow the same steps as Option A above.

### Option C: Docker-Based Setup (Advanced)

For a fully reproducible environment:

```bash
# Pull the ROS 2 Humble desktop image
docker pull osrf/ros:humble-desktop

# Run with GUI support
docker run -it \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --name ros2_dev \
  osrf/ros:humble-desktop \
  bash
```

## Verifying Your Installation

### Test 1: ROS 2 Core

Open two terminals and run:

**Terminal 1** — Start a talker node:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2** — Start a listener node:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```

You should see the listener receiving messages from the talker:
```
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
[INFO] [listener]: I heard: [Hello World: 3]
```

### Test 2: Gazebo Simulation

```bash
source /opt/ros/humble/setup.bash
ros2 launch gazebo_ros gazebo.launch.py
```

A Gazebo window should open with an empty simulation world. If you see this, your simulation environment is ready.

### Test 3: Python ROS 2

Create a simple test script:

```python
# test_ros2.py
import rclpy
from rclpy.node import Node

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('Hello from Physical AI course!')
        self.get_logger().info('ROS 2 is working correctly!')

def main():
    rclpy.init()
    node = TestNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run it:
```bash
python3 test_ros2.py
```

Expected output:
```
[INFO] [test_node]: Hello from Physical AI course!
[INFO] [test_node]: ROS 2 is working correctly!
```

## Creating Your ROS 2 Workspace

Set up a workspace that you'll use throughout the course:

```bash
# Create workspace directory
mkdir -p ~/physical_ai_ws/src
cd ~/physical_ai_ws

# Build the empty workspace (should succeed with no errors)
colcon build
source install/setup.bash

# Verify workspace
echo $AMENT_PREFIX_PATH
# Should include: /home/<user>/physical_ai_ws/install
```

```
Workspace Structure:

~/physical_ai_ws/
├── src/                 ◄── Your packages go here
│   └── (empty for now)
├── build/               ◄── Build artifacts (auto-generated)
├── install/             ◄── Installed packages (auto-generated)
└── log/                 ◄── Build logs (auto-generated)
```

## Recommended VS Code Extensions

Install these extensions for the best development experience:

| Extension | Publisher | Purpose |
|---|---|---|
| **ROS** | Microsoft | ROS 2 integration, launch files |
| **Python** | Microsoft | Python IntelliSense and debugging |
| **C/C++** | Microsoft | C++ support for ROS packages |
| **URDF** | smilerobotics | URDF file syntax highlighting |
| **XML** | Red Hat | XML editing for launch/config files |
| **Remote - WSL** | Microsoft | WSL2 integration (Windows users) |

## What's Next

With your environment set up, you're ready to begin the technical modules:

- **Module 1 (Weeks 3-5)**: You'll learn ROS 2 in depth — nodes, topics, services, and how to describe a humanoid robot with URDF
- **Module 2 (Weeks 6-7)**: You'll simulate your robot in Gazebo and Unity, adding virtual sensors
- **Module 3 (Weeks 8-10)**: You'll add AI perception and navigation using NVIDIA Isaac
- **Module 4 (Weeks 11-13)**: You'll build a complete autonomous humanoid with voice control and LLM reasoning

## Summary and Key Takeaways

- The course spans **13 weeks** across **5 modules**, progressively building the Physical AI stack
- Your development environment needs **Ubuntu 22.04** (native or WSL2), **ROS 2 Humble**, **Gazebo**, and **Python 3.10+**
- Three installation options: **native Ubuntu**, **Windows + WSL2**, or **Docker**
- Always verify your installation with the **talker/listener test**, **Gazebo launch**, and **Python ROS 2 test**
- Your **ROS 2 workspace** at `~/physical_ai_ws/` will be your home for all course projects

<div class="mini-task">
## Mini Task: Environment Verification

Complete all three verification tests above and take a screenshot of each:
1. The talker/listener output showing messages being exchanged
2. The Gazebo window with an empty world
3. The Python test node output

If any test fails, troubleshoot using the [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html) and ask for help in the course forum.
</div>
