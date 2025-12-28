---
sidebar_position: 2
title: 'Chapter 2: Cognitive Planning with LLMs'
---

# Chapter 2: Cognitive Planning with LLMs

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand how LLMs translate high-level goals into specific ROS 2 action sequences
- Explain the connection between LLM reasoning and ROS nodes/services
- Implement LLM-based cognitive planning for robot tasks
- Create goal decomposition and action sequence generation systems

## Introduction to LLM-Based Cognitive Planning

Large Language Models (LLMs) have revolutionized robotics by enabling high-level goal specification and automated task decomposition. Cognitive planning with LLMs bridges the gap between natural language goals and executable robot actions, allowing non-expert users to specify complex tasks in simple terms.

### Key Concepts in Cognitive Planning

#### Goal Decomposition
LLMs can break down complex goals into:
- Sequential subtasks
- Parallelizable operations
- Prerequisite dependencies
- Error handling strategies

#### Action Sequence Generation
LLMs generate executable sequences by:
- Mapping natural language to ROS actions
- Identifying required services and topics
- Sequencing operations with proper timing
- Handling conditional execution

## LLM Service Integration

### Example LLM Cognitive Planning Implementation

```python
import openai
import json
from typing import Dict, List, Any
from dataclasses import dataclass

@dataclass
class TaskPlan:
    goal: str
    subtasks: List[Dict[str, Any]]
    dependencies: List[Dict[str, str]]
    estimated_duration: float

class LLMCognitivePlanner:
    def __init__(self, api_key: str, model: str = "gpt-4"):
        """
        Initialize LLM-based cognitive planner
        """
        openai.api_key = api_key
        self.model = model

    def decompose_goal(self, goal: str, robot_capabilities: List[str]) -> TaskPlan:
        """
        Decompose a high-level goal into executable subtasks using LLM
        """
        prompt = f"""
        You are a cognitive planning expert for robotics. Given a robot with these capabilities:
        {', '.join(robot_capabilities)}

        Decompose this goal into a sequence of executable subtasks:
        "{goal}"

        Return the result in JSON format with the following structure:
        {{
            "subtasks": [
                {{
                    "id": "unique_id",
                    "description": "what to do",
                    "action": "ros_action_name",
                    "parameters": {{"param_name": "value"}},
                    "dependencies": ["previous_task_id"],
                    "estimated_duration": 10.0
                }}
            ],
            "dependencies": [
                {{"from": "task_id", "to": "task_id", "type": "sequential|parallel"}}
            ]
        }}
        """

        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[{"role": "user", "content": prompt}],
            temperature=0.1  # Low temperature for consistent planning
        )

        result = json.loads(response.choices[0].message.content)

        return TaskPlan(
            goal=goal,
            subtasks=result["subtasks"],
            dependencies=result["dependencies"],
            estimated_duration=sum(task.get("estimated_duration", 1.0) for task in result["subtasks"])
        )

    def generate_ros_sequence(self, task_plan: TaskPlan) -> List[Dict[str, Any]]:
        """
        Convert task plan to ROS 2 action sequence
        """
        ros_sequence = []
        for subtask in task_plan.subtasks:
            ros_action = {
                "action_server": subtask["action"],
                "goal": subtask["parameters"],
                "timeout": subtask.get("estimated_duration", 30.0) + 10.0  # Add buffer
                # Add error handling and feedback mechanisms
            }
            ros_sequence.append(ros_action)

        return ros_sequence

# Example usage
planner = LLMCognitivePlanner(api_key="your-api-key")
robot_caps = [
    "move_base", "pick_object", "place_object", "detect_object",
    "navigate_to_pose", "grasp", "release", "speak"
]

plan = planner.decompose_goal("Go to the kitchen, pick up the red cup, and bring it to me", robot_caps)
ros_sequence = planner.generate_ros_sequence(plan)

print(f"Generated {len(ros_sequence)} ROS actions for the goal")
```

## ROS 2 Action Sequence Mapping

### Mapping LLM Output to ROS Actions

```python
class ROSActionMapper:
    def __init__(self):
        # Define mappings from LLM concepts to ROS actions
        self.action_mappings = {
            "navigate": "move_base_msgs/MoveBaseAction",
            "move_to": "nav2_msgs/NavigateToPose",
            "pick": "manipulation_msgs/PickupAction",
            "grasp": "control_msgs/GripperCommand",
            "place": "manipulation_msgs/PlaceAction",
            "detect": "vision_msgs/DetectObjects",
            "speak": "sound_play/SoundRequest",
            "look_at": "control_msgs/PointHeadAction"
        }

        # Define parameter mappings
        self.param_mappings = {
            "location": "target_pose",
            "object": "object_name",
            "position": "pose",
            "distance": "linear.x"
        }

    def map_to_ros_action(self, llm_action: Dict[str, Any]) -> Dict[str, Any]:
        """
        Map LLM-generated action to ROS action format
        """
        action_name = llm_action["action"]
        parameters = llm_action["parameters"]

        # Map action name to ROS action
        ros_action = self.action_mappings.get(action_name, action_name)

        # Map parameters to ROS format
        ros_params = {}
        for param_name, param_value in parameters.items():
            ros_param_name = self.param_mappings.get(param_name, param_name)
            ros_params[ros_param_name] = param_value

        return {
            "action_server": ros_action,
            "goal": ros_params,
            "action_type": "actionlib"  # or "rclpy_action"
        }

# Example usage
mapper = ROSActionMapper()
llm_action = {
    "action": "navigate",
    "parameters": {
        "location": "kitchen",
        "orientation": "facing_counter"
    }
}

ros_action = mapper.map_to_ros_action(llm_action)
print(f"ROS Action: {ros_action}")
```

## Interactive Examples of LLM Reasoning Process

### Example 1: Kitchen Task
**Goal**: "Bring me a cup of coffee from the kitchen"

**LLM Reasoning Process**:
1. **Goal Analysis**: Identify objects (coffee cup), locations (kitchen), actions (fetch)
2. **Capability Check**: Verify robot can navigate, detect objects, manipulate items
3. **Subtask Generation**:
   - Navigate to kitchen
   - Detect coffee cup
   - Pick up coffee cup
   - Navigate to user
   - Deliver coffee cup
4. **Constraint Application**: Ensure safe navigation, proper grasping, collision avoidance

### Example 2: Multi-Step Assembly
**Goal**: "Assemble the wooden chair using the parts on the table"

**LLM Reasoning Process**:
1. **Goal Analysis**: Identify assembly task, parts location, final configuration
2. **Capability Check**: Verify robot can detect parts, manipulate objects, follow assembly sequence
3. **Subtask Generation**:
   - Navigate to table
   - Detect and identify parts
   - Pick up first part
   - Position for assembly
   - Pick up second part
   - Assemble parts
   - Repeat for remaining parts
4. **Constraint Application**: Maintain part orientation, apply proper force, verify assembly

## Mini-Task: Goal Decomposition

Break down "Pick up the cup" into 5 robot sub-tasks:

1. **Navigate to cup location**: Move robot base to position where cup is accessible
2. **Detect cup**: Use vision system to locate and identify the cup in the environment
3. **Approach cup**: Move manipulator to position above the cup
4. **Grasp cup**: Close gripper around cup handle or body with appropriate force
5. **Lift cup**: Raise manipulator to clear the surface and maintain stable grasp

## Summary

LLM-based cognitive planning enables robots to understand high-level goals and automatically decompose them into executable action sequences. This approach bridges the gap between natural language and ROS operations, making robots more accessible to non-expert users. The key is properly mapping LLM reasoning to ROS actions while maintaining safety and efficiency constraints.