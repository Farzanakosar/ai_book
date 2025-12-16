---
sidebar_position: 3
---

# URDF for Humanoid Robots

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the structure and components of URDF (Unified Robot Description Format)
- Create basic robot models with links and joints
- Define robot kinematics and visual properties
- Work with materials and textures in URDF
- Create simple humanoid robot models

## Prerequisites

Before starting this chapter, you should have:
- Basic understanding of ROS 2 concepts (from previous chapters)
- Understanding of 3D coordinate systems and transformations
- Basic XML knowledge (URDF is XML-based)

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links, joints, inertial properties, and visual appearance.

### Key Components of URDF:
- **Links**: Rigid parts of the robot (e.g., base, arm, wheel)
- **Joints**: Connections between links that allow relative motion
- **Visual**: How the link appears in simulation/visualization
- **Collision**: Collision properties for physics simulation
- **Inertial**: Mass, center of mass, and inertia properties

## Basic URDF Structure

A basic URDF file starts with an XML declaration and a robot tag:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links and joints go here -->
</robot>
```

### Simple Link Definition

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.5 0.5 0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.5 0.5 0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
  </inertial>
</link>
```

## URDF Links

Links represent rigid bodies in the robot. Each link has:
- A unique name
- Visual properties (how it looks)
- Collision properties (for physics simulation)
- Inertial properties (for dynamics)

### Link Geometry Types

URDF supports several geometry types:
- **Box**: `size="width length height"`
- **Cylinder**: `radius="value" length="value"`
- **Sphere**: `radius="value"`
- **Mesh**: `filename="path/to/mesh" scale="x y z"`

## URDF Joints

Joints connect links and define how they can move relative to each other. Joint types include:
- **Fixed**: No movement between links
- **Revolute**: Rotational movement around an axis (with limits)
- **Continuous**: Rotational movement without limits
- **Prismatic**: Linear movement along an axis (with limits)
- **Floating**: 6 DOF movement (rarely used)

### Joint Definition Example

```xml
<joint name="base_to_wheel" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0.2 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

## Simple Humanoid URDF Model

Here's a basic humanoid model with a torso, head, and limbs:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting torso and head -->
  <joint name="neck_joint" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35"/>
  </joint>
</robot>
```

## URDF with Joints Example

A more complex example with movable joints:

```xml
<?xml version="1.0"?>
<robot name="humanoid_with_joints">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.8 0.8 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Joint connecting torso and right arm -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## URDF with Materials

Materials define the visual appearance of links:

```xml
<material name="red">
  <color rgba="1 0 0 1"/>
</material>

<material name="blue">
  <color rgba="0 0 1 1"/>
</material>

<material name="green">
  <color rgba="0 1 0 1"/>
</material>

<material name="yellow">
  <color rgba="1 1 0 1"/>
</material>
```

## URDF Best Practices

1. **Tree Structure**: URDF must form a tree (no loops)
2. **Unique Names**: All links and joints must have unique names
3. **Base Link**: One link should not have a parent (the base/root link)
4. **Units**: Use meters for distances, kilograms for mass
5. **Inertial Properties**: Include realistic inertial properties for simulation
6. **Joint Limits**: Define appropriate limits for revolute joints

## Visualizing URDF Models

To visualize URDF models in ROS 2:

1. **Using RViz2**:
   ```bash
   ros2 run rviz2 rviz2
   ```
   Add a RobotModel display and set the robot description parameter.

2. **Using xacro** (if using xacro preprocessing):
   ```bash
   ros2 run xacro xacro --inorder your_robot.urdf.xacro
   ```

## Running the Examples

### Setup Instructions

1. Make sure your ROS 2 environment is sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Save URDF files with `.urdf` extension.

3. To check if your URDF is valid:
   ```bash
   check_urdf your_robot.urdf
   ```

4. To visualize in RViz2:
   ```bash
   # Set the robot description parameter
   ros2 param set /robot_state_publisher robot_description "$(cat your_robot.urdf)"

   # Then run RViz2
   ros2 run rviz2 rviz2
   ```

## Summary

In this chapter, you learned about URDF (Unified Robot Description Format):

- **Links**: Rigid components that make up the robot structure
- **Joints**: Connections between links that define relative motion
- **Visual Properties**: How the robot appears in simulation and visualization
- **Materials**: Color and appearance definitions
- **Kinematics**: How robot parts are connected and move relative to each other

URDF is fundamental for representing robots in ROS 2, whether for simulation, visualization, or motion planning. The examples provided demonstrate how to create simple to moderately complex robot models for humanoid robots.