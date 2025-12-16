---
sidebar_position: 3
---

# Sensor Simulation

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the principles of sensor simulation in Gazebo and Unity
- Configure LiDAR sensors for point cloud generation
- Set up depth camera simulation for 3D vision
- Implement IMU simulation for inertial measurements
- Integrate sensor data visualization across both platforms
- Create realistic sensor data outputs for robotics applications

## Prerequisites

Before starting this chapter, you should have:
- Completed the Gazebo Physics Simulation chapter
- Completed the Unity Rendering & HRI chapter
- Basic understanding of sensor types and their applications in robotics
- Knowledge of coordinate systems and transforms

## Introduction to Sensor Simulation

Sensor simulation is a critical component of robotics development, enabling:
- Perception algorithm testing without physical hardware
- Safe evaluation of robot behavior in complex environments
- Cost-effective development and debugging
- Repeatable experimental conditions

### Key Sensor Types for Robotics:
- **LiDAR**: Light Detection and Ranging for 3D mapping and navigation
- **Depth Cameras**: RGB-D sensors for 3D scene understanding
- **IMU**: Inertial Measurement Units for orientation and motion tracking
- **Cameras**: Visual sensors for image-based perception
- **GPS**: Global positioning for outdoor navigation
- **Encoders**: Wheel encoders for odometry estimation

## LiDAR Sensor Simulation

### LiDAR Principles

LiDAR sensors emit laser beams and measure the time it takes for reflections to return, creating accurate 3D point clouds of the environment. In simulation, this is achieved by casting rays and calculating distances to surfaces.

### Gazebo LiDAR Configuration

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="lidar_sensor">
    <link name="lidar_link">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>

      <sensor name="lidar_3d" type="ray">
        <pose>0 0 0 0 0 0</pose>
        <ray>
          <scan>
            <horizontal>
              <samples>640</samples>
              <resolution>1</resolution>
              <min_angle>-1.570796</min_angle>  <!-- -90 degrees -->
              <max_angle>1.570796</max_angle>   <!-- 90 degrees -->
            </horizontal>
            <vertical>
              <samples>16</samples>
              <resolution>1</resolution>
              <min_angle>-0.261799</min_angle>  <!-- -15 degrees -->
              <max_angle>0.261799</max_angle>   <!-- 15 degrees -->
            </vertical>
          </scan>
          <range>
            <min>0.1</min>
            <max>30.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <plugin name="lidar_3d_controller" filename="libRayPlugin.so">
          <always_on>true</always_on>
          <update_rate>10</update_rate>
          <topic_name>/lidar_3d_scan</topic_name>
        </plugin>
      </sensor>

      <visual name="lidar_visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
        </material>
      </visual>

      <collision name="lidar_collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

### 2D LiDAR Configuration

For simpler 2D navigation:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="lidar_2d">
    <link name="lidar_2d_link">
      <pose>0 0 0.2 0 0 0</pose>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.00005</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.00005</iyy>
          <iyz>0</iyz>
          <izz>0.00005</izz>
        </inertia>
      </inertial>

      <sensor name="lidar_2d" type="ray">
        <pose>0 0 0 0 0 0</pose>
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>  <!-- -π -->
              <max_angle>3.14159</max_angle>   <!-- π -->
            </horizontal>
          </scan>
          <range>
            <min>0.1</min>
            <max>20.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <plugin name="lidar_2d_controller" filename="libRayPlugin.so">
          <always_on>true</always_on>
          <update_rate>10</update_rate>
          <topic_name>/scan</topic_name>
        </plugin>
      </sensor>

      <visual name="lidar_2d_visual">
        <geometry>
          <box>
            <size>0.04 0.04 0.04</size>
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

## Depth Camera Simulation

### Depth Camera Principles

Depth cameras provide both color (RGB) and depth information for each pixel, enabling 3D reconstruction and spatial understanding. The simulation typically combines a regular camera with a depth sensor.

### Gazebo Depth Camera Configuration

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="depth_camera">
    <link name="camera_link">
      <pose>0 0 0.15 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>

      <sensor name="depth_camera" type="depth">
        <pose>0 0 0 0 0 0</pose>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10.0</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
        <plugin name="camera_controller" filename="libDepthCameraPlugin.so">
          <always_on>true</always_on>
          <update_rate>30</update_rate>
          <topic_name>/camera/rgb/image_raw</topic_name>
          <depth_topic_name>/camera/depth/image_raw</depth_topic_name>
          <point_cloud_topic_name>/camera/depth/points</point_cloud_topic_name>
          <camera_info_topic_name>/camera/rgb/camera_info</camera_info_topic_name>
        </plugin>
      </sensor>

      <visual name="camera_visual">
        <geometry>
          <box>
            <size>0.05 0.05 0.03</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.8 1</ambient>
          <diffuse>0.2 0.2 0.8 1</diffuse>
        </material>
      </visual>

      <collision name="camera_collision">
        <geometry>
          <box>
            <size>0.05 0.05 0.03</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

## IMU Sensor Simulation

### IMU Principles

IMUs measure linear acceleration and angular velocity, providing crucial data for robot localization, navigation, and control. In simulation, IMU data includes realistic noise models.

### Gazebo IMU Configuration

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="imu_sensor">
    <link name="imu_link">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.000001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.000001</iyy>
          <iyz>0</iyz>
          <izz>0.000001</izz>
        </inertia>
      </inertial>

      <sensor name="imu_sensor" type="imu">
        <pose>0 0 0 0 0 0</pose>
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.0017</stddev>  <!-- ~0.1 deg/s stddev -->
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.0017</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.0017</stddev>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.017</stddev>  <!-- ~0.017 m/s² stddev -->
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
              <stddev>0.017</stddev>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.017</stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
        <plugin name="imu_controller" filename="libImuPlugin.so">
          <always_on>true</always_on>
          <update_rate>100</update_rate>
          <topic_name>/imu/data</topic_name>
        </plugin>
      </sensor>

      <visual name="imu_visual">
        <geometry>
          <box>
            <size>0.01 0.01 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.2 1</ambient>
          <diffuse>0.8 0.8 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

## Unity Sensor Visualization

### Point Cloud Visualization in Unity

Creating visualizations for sensor data in Unity:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class PointCloudVisualizer : MonoBehaviour
{
    [Header("Visualization Settings")]
    public GameObject pointPrefab;
    public float pointSize = 0.01f;
    public Color pointColor = Color.red;
    public int maxPoints = 10000;

    [Header("Data Input")]
    public bool useLivePointCloud = false;
    public float updateInterval = 0.1f;

    private List<GameObject> pointObjects;
    private List<Vector3> pointCloudData;
    private float lastUpdateTime = 0f;

    void Start()
    {
        pointObjects = new List<GameObject>();
        pointCloudData = new List<Vector3>();

        if (pointPrefab == null)
        {
            CreateDefaultPointPrefab();
        }
    }

    void CreateDefaultPointPrefab()
    {
        // Create a simple sphere as point prefab
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.name = "PointCloudPoint";
        sphere.GetComponent<Renderer>().material = new Material(Shader.Find("Universal Render Pipeline/Lit"));
        sphere.GetComponent<Renderer>().material.color = pointColor;
        sphere.transform.localScale = Vector3.one * pointSize;
        sphere.SetActive(false);
        pointPrefab = sphere;
    }

    void Update()
    {
        if (useLivePointCloud)
        {
            if (Time.time - lastUpdateTime > updateInterval)
            {
                UpdatePointCloudVisualization();
                lastUpdateTime = Time.time;
            }
        }
    }

    public void UpdatePointCloud(List<Vector3> newPoints)
    {
        pointCloudData.Clear();
        pointCloudData.AddRange(newPoints);

        // Limit the number of points to avoid performance issues
        if (pointCloudData.Count > maxPoints)
        {
            pointCloudData.RemoveRange(maxPoints, pointCloudData.Count - maxPoints);
        }

        UpdatePointCloudVisualization();
    }

    void UpdatePointCloudVisualization()
    {
        // Destroy old points
        foreach (GameObject pointObj in pointObjects)
        {
            if (pointObj != null)
            {
                DestroyImmediate(pointObj);
            }
        }
        pointObjects.Clear();

        // Create new points
        foreach (Vector3 point in pointCloudData)
        {
            GameObject pointObj = Instantiate(pointPrefab, point, Quaternion.identity, transform);
            pointObj.SetActive(true);
            pointObj.GetComponent<Renderer>().material.color = pointColor;
            pointObj.transform.localScale = Vector3.one * pointSize;
            pointObjects.Add(pointObj);
        }
    }

    // Helper method to generate synthetic point cloud for demonstration
    public void GenerateSamplePointCloud()
    {
        List<Vector3> points = new List<Vector3>();

        // Generate a simple spherical point cloud
        for (int i = 0; i < 1000; i++)
        {
            float theta = Random.Range(0f, 2f * Mathf.PI);
            float phi = Random.Range(0f, Mathf.PI);
            float radius = Random.Range(1f, 5f);

            float x = radius * Mathf.Sin(phi) * Mathf.Cos(theta);
            float y = radius * Mathf.Sin(phi) * Mathf.Sin(theta);
            float z = radius * Mathf.Cos(phi);

            points.Add(new Vector3(x, y, z));
        }

        UpdatePointCloud(points);
    }
}
```

### Sensor Data Processing in Unity

Processing and displaying sensor data:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class SensorDataProcessor : MonoBehaviour
{
    [Header("Sensor Data Sources")]
    public PointCloudVisualizer pointCloudVisualizer;
    public Camera depthCamera;
    public TextMesh statusText;

    [Header("Processing Settings")]
    public bool enableRealTimeProcessing = true;
    public float processingRate = 10f; // Hz

    private float lastProcessingTime = 0f;
    private Queue<SensorReading> sensorReadings;
    private bool isProcessing = false;

    [System.Serializable]
    public class SensorReading
    {
        public Vector3 position;
        public Vector3 rotation;
        public Vector3 linearAcceleration;
        public Vector3 angularVelocity;
        public List<Vector3> pointCloud;
        public float timestamp;

        public SensorReading()
        {
            pointCloud = new List<Vector3>();
            timestamp = Time.time;
        }
    }

    void Start()
    {
        sensorReadings = new Queue<SensorReading>();
        SetupSensors();
    }

    void SetupSensors()
    {
        if (depthCamera == null)
        {
            depthCamera = GetComponent<Camera>();
        }

        if (pointCloudVisualizer == null)
        {
            pointCloudVisualizer = FindObjectOfType<PointCloudVisualizer>();
        }

        if (statusText == null)
        {
            // Try to find a status text in child objects
            statusText = GetComponentInChildren<TextMesh>();
        }
    }

    void Update()
    {
        if (enableRealTimeProcessing)
        {
            if (Time.time - lastProcessingTime > 1f / processingRate)
            {
                ProcessSensorData();
                lastProcessingTime = Time.time;
            }
        }

        UpdateStatusDisplay();
    }

    public void ProcessSensorData()
    {
        isProcessing = true;

        // Create a new sensor reading
        SensorReading reading = new SensorReading();
        reading.position = transform.position;
        reading.rotation = transform.eulerAngles;
        reading.linearAcceleration = CalculateLinearAcceleration();
        reading.angularVelocity = CalculateAngularVelocity();

        // Generate or receive point cloud data
        reading.pointCloud = GeneratePointCloudFromDepthImage();

        // Store the reading
        sensorReadings.Enqueue(reading);

        // Limit queue size to prevent memory issues
        if (sensorReadings.Count > 100)
        {
            sensorReadings.Dequeue();
        }

        // Update visualization
        if (pointCloudVisualizer != null)
        {
            pointCloudVisualizer.UpdatePointCloud(reading.pointCloud);
        }

        isProcessing = false;
    }

    Vector3 CalculateLinearAcceleration()
    {
        // Simplified calculation - in real implementation, this would come from IMU simulation
        return new Vector3(
            Mathf.Sin(Time.time) * 0.1f,
            Mathf.Cos(Time.time) * 0.1f,
            -9.81f + Mathf.Sin(Time.time * 2) * 0.05f
        );
    }

    Vector3 CalculateAngularVelocity()
    {
        // Simplified calculation - in real implementation, this would come from IMU simulation
        return new Vector3(
            Mathf.Sin(Time.time * 0.5f) * 0.05f,
            Mathf.Cos(Time.time * 0.3f) * 0.05f,
            Mathf.Sin(Time.time * 0.7f) * 0.05f
        );
    }

    List<Vector3> GeneratePointCloudFromDepthImage()
    {
        List<Vector3> points = new List<Vector3>();

        // This is a simplified version - in a real implementation,
        // this would process actual depth camera data
        for (int i = 0; i < 100; i++)
        {
            // Generate random points in front of the camera
            Vector3 point = transform.position + transform.forward * Random.Range(1f, 10f) +
                           transform.right * Random.Range(-2f, 2f) +
                           transform.up * Random.Range(-1f, 2f);

            points.Add(point);
        }

        return points;
    }

    void UpdateStatusDisplay()
    {
        if (statusText != null)
        {
            statusText.text = $"Sensor Processing: {(isProcessing ? "ACTIVE" : "IDLE")}\n" +
                             $"Queue Size: {sensorReadings.Count}\n" +
                             $"Last Update: {lastProcessingTime:F2}s";
        }
    }

    public SensorReading GetLatestReading()
    {
        if (sensorReadings.Count > 0)
        {
            return sensorReadings.Peek(); // Return the most recent reading
        }
        return null;
    }

    public List<SensorReading> GetHistoricalReadings(int count)
    {
        List<SensorReading> readings = new List<SensorReading>();
        int itemsToTake = Mathf.Min(count, sensorReadings.Count);

        // Take the most recent readings
        var tempQueue = new Queue<SensorReading>(sensorReadings);
        for (int i = 0; i < sensorReadings.Count - itemsToTake; i++)
        {
            tempQueue.Dequeue();
        }

        readings.AddRange(tempQueue);
        return readings;
    }
}
```

## Sensor Integration Examples

### Multi-Sensor Fusion World

Combining multiple sensors in a single simulation:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="multi_sensor_world">
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

    <!-- Robot with multiple sensors -->
    <model name="sensor_robot">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="chassis">
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
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.3 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.3 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>

      <!-- LiDAR sensor -->
      <model name="lidar_2d">
        <pose>0.1 0 0.1 0 0 0</pose>
        <link name="lidar_link">
          <sensor name="lidar_2d" type="ray">
            <pose>0 0 0 0 0 0</pose>
            <ray>
              <scan>
                <horizontal>
                  <samples>360</samples>
                  <resolution>1</resolution>
                  <min_angle>-3.14159</min_angle>
                  <max_angle>3.14159</max_angle>
                </horizontal>
              </scan>
              <range>
                <min>0.1</min>
                <max>20.0</max>
                <resolution>0.01</resolution>
              </range>
            </ray>
            <plugin name="lidar_2d_controller" filename="libRayPlugin.so">
              <always_on>true</always_on>
              <update_rate>10</update_rate>
              <topic_name>/scan</topic_name>
            </plugin>
          </sensor>
        </link>
      </model>

      <!-- Depth camera -->
      <model name="depth_camera">
        <pose>0.15 0 0.15 0 0 0</pose>
        <link name="camera_link">
          <sensor name="depth_camera" type="depth">
            <camera>
              <horizontal_fov>1.047</horizontal_fov>
              <image>
                <width>640</width>
                <height>480</height>
                <format>R8G8B8</format>
              </image>
              <clip>
                <near>0.1</near>
                <far>10.0</far>
              </clip>
            </camera>
            <plugin name="camera_controller" filename="libDepthCameraPlugin.so">
              <always_on>true</always_on>
              <update_rate>30</update_rate>
              <topic_name>/camera/rgb/image_raw</topic_name>
              <depth_topic_name>/camera/depth/image_raw</depth_topic_name>
            </plugin>
          </sensor>
        </link>
      </model>

      <!-- IMU sensor -->
      <model name="imu_sensor">
        <pose>0 0 0.05 0 0 0</pose>
        <link name="imu_link">
          <sensor name="imu_sensor" type="imu">
            <imu>
              <angular_velocity>
                <x><noise type="gaussian"><stddev>0.0017</stddev></noise></x>
                <y><noise type="gaussian"><stddev>0.0017</stddev></noise></y>
                <z><noise type="gaussian"><stddev>0.0017</stddev></noise></z>
              </angular_velocity>
              <linear_acceleration>
                <x><noise type="gaussian"><stddev>0.017</stddev></noise></x>
                <y><noise type="gaussian"><stddev>0.017</stddev></noise></y>
                <z><noise type="gaussian"><stddev>0.017</stddev></noise></z>
              </linear_acceleration>
            </imu>
            <plugin name="imu_controller" filename="libImuPlugin.so">
              <always_on>true</always_on>
              <update_rate>100</update_rate>
              <topic_name>/imu/data</topic_name>
            </plugin>
          </sensor>
        </link>
      </model>
    </model>

    <!-- Obstacles for sensor testing -->
    <model name="obstacle1">
      <pose>-2 1 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
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
      </link>
    </model>

    <model name="obstacle2">
      <pose>1 -1.5 0.3 0 0 0.5</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.6</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.6</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.0625</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0625</iyy>
            <iyz>0</iyz>
            <izz>0.045</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Exercises

### Exercise 1: Configure a LiDAR Sensor and Visualize Its Point Cloud Data

Configure a LiDAR sensor in Gazebo and implement visualization of its point cloud data in Unity:
1. Create a LiDAR sensor configuration with specific parameters (range, resolution, field of view)
2. Set up the sensor in a test world with obstacles
3. Implement a Unity script to visualize the resulting point cloud
4. Adjust sensor parameters and observe the changes in the point cloud

### Exercise 2: Simulate Depth Camera Data for a Simple Scene

Create a depth camera simulation and process its data:
1. Configure a depth camera in Gazebo with realistic parameters
2. Set up a scene with various objects at different depths
3. Implement a Unity visualization for the depth data
4. Process the depth information to reconstruct 3D positions of objects

## Summary

In this chapter, you learned about sensor simulation for robotics applications:
- How to configure LiDAR sensors for 3D mapping and navigation
- Techniques for depth camera simulation and 3D vision
- Implementation of IMU simulation with realistic noise models
- Integration of sensor data visualization across Gazebo and Unity
- Creating multi-sensor fusion scenarios for comprehensive perception

These concepts enable the simulation of realistic sensor data for robotics development, allowing for thorough testing and validation of perception algorithms before deployment on physical robots.