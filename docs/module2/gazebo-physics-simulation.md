---
sidebar_position: 1
---

# Gazebo Physics Simulation

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the fundamental concepts of Gazebo physics simulation
- Configure gravity parameters and environmental properties
- Set up collision detection and response
- Create basic physics-enabled worlds and models
- Run and test physics simulations in Gazebo

## Prerequisites

Before starting this chapter, you should have:
- Basic understanding of robotics concepts
- Fundamental knowledge of 3D coordinate systems
- Gazebo Classic installed on your system (version 11.x or newer)

## Introduction to Gazebo Physics

Gazebo is a 3D dynamic simulator with accurate and efficient physics simulation. It provides the foundation for simulating robots in complex indoor and outdoor environments. The physics engine in Gazebo handles the simulation of rigid body dynamics, contact forces, and collisions.

### Key Physics Concepts in Gazebo:
- **Physics Engine**: Handles the simulation of rigid body dynamics (ODE, Bullet, DART)
- **Gravity**: Configurable gravitational acceleration affecting all objects
- **Collisions**: Detection and response to physical contact between objects
- **Materials**: Physical properties like friction, restitution, and density

## Setting Up Physics in Gazebo

### Physics Engine Configuration

Gazebo supports multiple physics engines, with ODE (Open Dynamics Engine) being the most stable and widely used. The physics engine is configured in the world file:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
</physics>
```

### Gravity Parameters

Gravity is a crucial aspect of physics simulation. You can configure gravity parameters in your world file:

```xml
<world name="gravity_test">
  <gravity>0 0 -9.8</gravity>
  <!-- Earth's gravity: 9.8 m/s² downward -->

  <!-- Example: Moon's gravity (about 1/6 of Earth's) -->
  <!-- <gravity>0 0 -1.63</gravity> -->

  <!-- Example: Zero gravity (space environment) -->
  <!-- <gravity>0 0 0</gravity> -->
</world>
```

### Collision Detection

Collision detection is handled through the `<collision>` element in SDF models. This defines the shape and properties used for detecting contacts:

```xml
<collision name="collision">
  <geometry>
    <box>
      <size>1.0 1.0 1.0</size>
    </box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.1</restitution_coefficient>
      <threshold>100000</threshold>
    </bounce>
    <contact>
      <ode>
        <soft_cfm>0</soft_cfm>
        <soft_erp>0.2</soft_erp>
        <kp>1000000000000.0</kp>
        <kd>1.0</kd>
        <max_vel>100.0</max_vel>
        <min_depth>0.001</min_depth>
      </ode>
    </contact>
  </surface>
</collision>
```

## Creating Basic Physics Examples

### Basic Physics World Example

Here's a complete example of a simple physics world with a ground plane and a falling box:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="basic_physics">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Falling box -->
    <model name="falling_box">
      <pose>0 0 2 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.041667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.041667</iyy>
            <iyz>0</iyz>
            <izz>0.041667</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Simple Box Model

Here's an example of a simple box model that can be used in various worlds:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="simple_box">
    <link name="box_link">
      <pose>0 0 0.25 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.083333</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.083333</iyy>
          <iyz>0</iyz>
          <izz>0.083333</izz>
        </inertia>
      </inertial>
      <collision name="box_collision">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
      </collision>
      <visual name="box_visual">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

## Running Physics Simulations

### Launching a Physics World

To launch the basic physics world we created:

1. Save the world file as `basic_physics.world`
2. Launch Gazebo with the world file:
   ```bash
   gazebo basic_physics.world
   ```

### Testing Different Gravity Parameters

Create a world file to test different gravity settings:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="gravity_comparison">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Earth-like gravity zone -->
    <gravity>0 0 -9.8</gravity>
    <model name="earth_box" placement_frame="earth_box::link">
      <pose>0 0 5 0 0 0</pose>
      <include>
        <uri>model://box</uri>
        <pose>0 0 0 0 0 0</pose>
      </include>
    </model>

    <!-- Moon-like gravity zone (simulated with separate model) -->
    <model name="moon_box" placement_frame="moon_box::link">
      <pose>2 0 5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.041667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.041667</iyy>
            <iyz>0</iyz>
            <izz>0.041667</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Collision Detection Examples

### Collision Test World

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="collision_test">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ramps for collision testing -->
    <model name="ramp1">
      <pose>-2 0 0 0 0 0.3</pose>
      <link name="ramp_link">
        <collision name="ramp_collision">
          <geometry>
            <mesh>
              <uri>file://media/meshes/ramp.dae</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="ramp_visual">
          <geometry>
            <mesh>
              <uri>file://media/meshes/ramp.dae</uri>
            </mesh>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.0</iyy>
            <iyz>0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Rolling spheres with different materials -->
    <model name="sphere1">
      <pose>-2.5 0 1.5 0 0 0</pose>
      <link name="sphere_link">
        <collision name="sphere_collision">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.1</mu>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="sphere_visual">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.0002</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0002</iyy>
            <iyz>0</iyz>
            <izz>0.0002</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Environment Setup Examples

### Basic Environment World

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="environment_setup">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Walls -->
    <model name="wall1">
      <pose>5 0 1.5 0 0 0</pose>
      <link name="wall_link">
        <collision name="wall_collision">
          <geometry>
            <box>
              <size>0.2 10 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="wall_visual">
          <geometry>
            <box>
              <size>0.2 10 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100.0</mass>
          <inertia>
            <ixx>100.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>100.0</iyy>
            <iyz>0</iyz>
            <izz>100.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Objects to test physics -->
    <model name="test_box">
      <pose>0 0 2 0 0 0</pose>
      <link name="box_link">
        <collision name="box_collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="box_visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.041667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.041667</iyy>
            <iyz>0</iyz>
            <izz>0.041667</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Exercises

### Exercise 1: Create a Simple Physics World with a Falling Box

Create a world file that contains:
1. A ground plane
2. A box positioned above the ground
3. Proper physics configuration
4. The box should fall and land on the ground due to gravity

### Exercise 2: Configure Gravity Parameters for Different Planetary Environments

Create a world that demonstrates how different gravity values affect object motion:
1. Use Earth's gravity (9.8 m/s²) in one area
2. Use Moon's gravity (1.6 m/s²) in another area
3. Compare the falling speeds of identical objects

## Summary

In this chapter, you learned about the fundamental concepts of Gazebo physics simulation:
- How to configure physics engines and parameters
- Setting up gravity with customizable values
- Implementing collision detection and response
- Creating basic physics-enabled worlds and models

These concepts form the foundation for more complex simulations involving robots, sensors, and dynamic environments. The examples provided demonstrate how to create simple but effective physics simulations that can be extended for more complex scenarios.