# Exercise 6 Solution: Create a URDF for a simple robot

## Problem Statement
Create a URDF for a simple robot with at least 3 links and 2 joints.

## Expected Outcome
A valid URDF file defining a simple robot with at least 3 links and 2 joints.

## Solution

Here's a simple robot URDF that meets the requirements:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.15" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.15" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Upper body -->
  <link name="upper_body">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.3"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting base and upper body -->
  <joint name="base_to_upper_body" type="fixed">
    <parent link="base_link"/>
    <child link="upper_body"/>
    <origin xyz="0 0 0.15"/>
  </joint>

  <!-- Joint connecting upper body and head -->
  <joint name="neck_joint" type="revolute">
    <parent link="upper_body"/>
    <child link="head"/>
    <origin xyz="0 0 0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.78" upper="0.78" effort="10" velocity="1"/>
  </joint>
</robot>
```

## Key Points
- **3 Links**: base_link, upper_body, and head
- **2 Joints**: base_to_upper_body (fixed) and neck_joint (revolute)
- The base is a cylinder representing the robot's base
- The upper body is a box connecting the base to the head
- The head is a sphere that can rotate on the neck joint
- Materials are defined for visual appearance
- Inertial properties are included for physics simulation
- The neck joint allows rotational movement of the head

## Running the Solution
1. Save the URDF to a file (e.g., `simple_robot.urdf`)
2. To validate the URDF: `check_urdf simple_robot.urdf`
3. To visualize in RViz2: Set the robot_description parameter and add a RobotModel display

## Expected Result
A simple robot with a base, body, and head where the head can rotate relative to the body.