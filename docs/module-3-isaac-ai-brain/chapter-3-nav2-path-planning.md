---
sidebar_position: 3
title: 'Chapter 3: Nav2 Path Planning for Humanoid Movement'
---

# Chapter 3: Nav2 Path Planning for Humanoid Movement

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand Nav2 path planning basics and how they apply to bipedal robots
- Identify path planning challenges specific to humanoid robots
- Create basic Nav2 configurations for bipedal robots
- Explain the concepts of costmaps and navigation strategies

## Introduction to Nav2 for Humanoid Robots

Navigation2 (Nav2) is the navigation stack for ROS 2 that provides path planning, obstacle avoidance, and navigation capabilities for mobile robots. For humanoid robots, Nav2 must account for the unique challenges of bipedal locomotion, including balance, step planning, and complex kinematics.

### Key Components of Nav2

#### Global Planner
The global planner creates a high-level path from start to goal:
- A* or Dijkstra algorithms for optimal pathfinding
- Consideration of static obstacles
- Integration with costmaps for environment awareness

#### Local Planner
The local planner handles real-time navigation and obstacle avoidance:
- Dynamic obstacle avoidance
- Trajectory generation for robot motion
- Integration with robot controllers

#### Costmaps
Costmaps represent the environment with:
- Static obstacles from maps
- Dynamic obstacles from sensors
- Inflation zones for safety margins
- Specialized layers for humanoid-specific constraints

## Path Planning Challenges for Humanoid Robots

### Balance and Stability
Humanoid robots must maintain balance during navigation, which adds complexity to path planning:
- Center of mass considerations
- Step sequence planning
- Support polygon constraints
- Dynamic stability during movement

### Step Planning
Unlike wheeled robots, humanoid robots must plan each step:
- Foot placement positions
- Step height variations
- Terrain traversability
- Balance recovery strategies

### Kinematic Constraints
Humanoid robots have complex kinematic constraints:
- Joint angle limitations
- Reachability constraints
- Self-collision avoidance
- Dynamic motion constraints

## Nav2 Configuration for Bipedal Robots

Here's an example configuration for Nav2 with humanoid-specific parameters:

```yaml
# Nav2 Configuration for Humanoid Robot
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_to_pose_bt_xml: /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node

# Humanoid-specific costmap configuration
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      # Humanoid-specific inflation
      inflation_layer:
        enabled: true
        cost_scaling_factor: 3.0  # Higher for humanoid safety
        inflation_radius: 0.8     # Larger for humanoid size
        inflate_unknown: false
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0  # Humanoid height consideration
          clearing: true
          marking: true

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.4  # Humanoid base radius
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
      static_layer:
        enabled: true
        map_topic: /map
      inflation_layer:
        enabled: true
        cost_scaling_factor: 2.0
        inflation_radius: 1.0
```

## Path Planning for Humanoid Navigation

### Costmap Considerations
Humanoid robots require specialized costmap configurations:
- **Larger safety margins**: Humanoid robots need more space for balance recovery
- **Height considerations**: Costmaps must account for robot height and potential overhead obstacles
- **Step planning**: Costmaps should consider terrain traversability for stepping

### Navigation Strategies
Humanoid robots may use different navigation strategies:
- **Slow, careful navigation**: For challenging terrain
- **Dynamic walking**: For faster navigation on clear paths
- **Step-by-step planning**: For precise foot placement

## Mini-Task: Humanoid Navigation Obstacles

Identify 3 obstacles a humanoid must navigate around:

1. **Low doorways**: The humanoid must consider its height and potentially crouch or find alternative paths.

2. **Narrow passages**: The humanoid needs sufficient width to maintain balance while walking through tight spaces.

3. **Uneven terrain**: Steps, curbs, or slopes require careful foot placement and balance considerations.

## Summary

Nav2 provides the navigation framework for ROS 2 robots, but humanoid robots present unique challenges due to their bipedal nature, balance requirements, and complex kinematics. Specialized configurations for costmaps, planning algorithms, and safety margins are essential for effective humanoid navigation. Understanding these challenges is crucial for developing robots that can navigate safely and efficiently in human environments.