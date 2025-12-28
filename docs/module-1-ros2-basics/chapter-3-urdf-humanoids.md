---
title: Chapter 3 - URDF for Humanoids
sidebar_position: 3
---

# Chapter 3: URDF for Humanoids

<div class="learning-objectives">
## Learning Objectives
- Understand the basics of URDF (Unified Robot Description Format)
- Learn about links, joints, and sensors in robot descriptions
- Explore how URDF applies specifically to humanoid robots
</div>

## URDF Structure and Components

URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including:

```
URDF Structure Example:
┌─────────────────┐
│   robot         │
│   (name="...")  │
└─────────────────┘
         │
         ▼
┌─────────────────┐    ┌─────────────────┐
│   link          │────│   joint         │
│   (base_link)   │    │   (fixed)       │
└─────────────────┘    └─────────────────┘
         │                       │
         ▼                       ▼
┌─────────────────┐    ┌─────────────────┐
│   visual        │    │   parent/child  │
│   collision     │    │   origin        │
│   inertial      │    │   axis          │
└─────────────────┘    └─────────────────┘
```

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

## URDF Examples

Here are examples of basic URDF structures:

### Basic URDF Example
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting links -->
  <joint name="base_to_child" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0.3 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

### Humanoid URDF Example
A simplified humanoid robot structure includes the essential components:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35"/>
  </joint>

  <!-- Limbs would follow similar pattern -->
</robot>
```

## Mini-Task: Identify 5 Required Parts for a Humanoid URDF

<div class="mini-task">
Identify the 5 essential structural parts that any humanoid robot URDF must include to be functional.
</div>

## Summary and Key Takeaways
- URDF describes robot structure using links and joints
- Links represent rigid bodies, joints define movement between them
- Humanoid robots have specific structural requirements