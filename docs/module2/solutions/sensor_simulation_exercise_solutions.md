# Sensor Simulation Exercise Solutions

## Exercise 1 Solution: LiDAR Configuration

### Problem
Configure a LiDAR sensor with 360° horizontal field of view, 16 vertical channels, and 20m maximum range.

### Solution
```xml
<sensor name="lidar_360" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>  <!-- -π radians (-180°) -->
        <max_angle>3.14159</max_angle>   <!-- π radians (180°) -->
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>  <!-- -15° in radians -->
        <max_angle>0.261799</max_angle>   <!-- 15° in radians -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>20.0</max>  <!-- 20m maximum range -->
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libRayPlugin.so">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <topic_name>/laser_scan</topic_name>
  </plugin>
</sensor>
```

### Key Points:
- Horizontal samples: 360 for full 360° coverage
- Min/max angles: ±π radians for full circle
- Vertical channels: 16 for elevation coverage
- Range: 0.1m minimum to 20m maximum
- Update rate: 10Hz for real-time performance

## Exercise 2 Solution: Depth Camera Setup

### Problem
Create a depth camera with 640×480 resolution, 60° horizontal FOV, and realistic noise parameters.

### Solution
```xml
<sensor name="depth_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60° in radians -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>  <!-- 0.1m minimum range -->
      <far>10.0</far>   <!-- 10m maximum range -->
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>  <!-- Realistic depth noise -->
    </noise>
  </camera>
  <plugin name="camera_controller" filename="libDepthCameraPlugin.so">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <topic_name>/camera/depth/image_raw</topic_name>
  </plugin>
</sensor>
```

### Key Points:
- Image dimensions: 640×480 pixels
- Horizontal FOV: 60° (1.047 radians)
- Near/far clipping: 0.1m to 10m range
- Noise: Gaussian with 1cm standard deviation
- Update rate: 30Hz for video-like performance

## Exercise 3 Solution: IMU with Noise Model

### Problem
Configure an IMU sensor with realistic noise parameters for angular velocity and linear acceleration.

### Solution
```xml
<sensor name="imu_sensor" type="imu">
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>  <!-- ~0.1°/s standard deviation -->
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
          <stddev>0.017</stddev>  <!-- ~0.017 m/s² standard deviation -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
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
```

### Key Points:
- Angular velocity noise: ~0.1°/s standard deviation (realistic for tactical-grade IMUs)
- Linear acceleration noise: ~0.017 m/s² standard deviation
- Update rate: 100Hz for high-frequency measurements
- Gaussian noise model for realistic sensor behavior

## Exercise 4 Solution: Unity Point Cloud Visualization

### Problem
Create a Unity script that visualizes point cloud data as individual points in the scene.

### Solution
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

    [Header("Performance")]
    public bool useObjectPooling = true;
    public int poolSize = 1000;

    private List<GameObject> pointPool;
    private List<GameObject> activePoints;
    private int poolIndex = 0;

    void Start()
    {
        InitializePointPool();
    }

    void InitializePointPool()
    {
        if (useObjectPooling)
        {
            pointPool = new List<GameObject>();
            activePoints = new List<GameObject>();

            // Create pooled objects
            for (int i = 0; i < poolSize; i++)
            {
                GameObject point = CreatePoint();
                point.SetActive(false);
                pointPool.Add(point);
            }
        }
    }

    GameObject CreatePoint()
    {
        GameObject point;
        if (pointPrefab != null)
        {
            point = Instantiate(pointPrefab, Vector3.zero, Quaternion.identity);
        }
        else
        {
            // Create default sphere if no prefab provided
            point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            point.GetComponent<Renderer>().material = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            point.GetComponent<Renderer>().material.color = pointColor;
        }

        point.transform.SetParent(transform);
        point.transform.localScale = Vector3.one * pointSize;
        return point;
    }

    public void UpdatePointCloud(List<Vector3> points)
    {
        if (useObjectPooling)
        {
            UpdateWithPooling(points);
        }
        else
        {
            UpdateWithoutPooling(points);
        }
    }

    void UpdateWithPooling(List<Vector3> points)
    {
        // Deactivate all previously active points
        foreach (GameObject activePoint in activePoints)
        {
            activePoint.SetActive(false);
        }
        activePoints.Clear();

        // Activate and position pooled points
        int pointsToShow = Mathf.Min(points.Count, poolSize);
        for (int i = 0; i < pointsToShow; i++)
        {
            GameObject point = pointPool[poolIndex];
            point.SetActive(true);
            point.transform.position = points[i];
            point.GetComponent<Renderer>().material.color = GetColorForHeight(points[i].y);

            activePoints.Add(point);
            poolIndex = (poolIndex + 1) % poolSize;
        }
    }

    void UpdateWithoutPooling(List<Vector3> points)
    {
        // Destroy all previous points
        foreach (Transform child in transform)
        {
            Destroy(child.gameObject);
        }

        // Create new points
        int pointsToShow = Mathf.Min(points.Count, maxPoints);
        for (int i = 0; i < pointsToShow; i++)
        {
            GameObject point = CreatePoint();
            point.transform.position = points[i];
            point.GetComponent<Renderer>().material.color = GetColorForHeight(points[i].y);
        }
    }

    Color GetColorForHeight(float height)
    {
        // Map height to color gradient (blue to red)
        float normalizedHeight = Mathf.InverseLerp(-5f, 5f, height);
        return Color.Lerp(Color.blue, Color.red, normalizedHeight);
    }

    // Helper method to generate sample point cloud
    public void GenerateSamplePointCloud()
    {
        List<Vector3> points = new List<Vector3>();

        // Generate a spherical point cloud
        for (int i = 0; i < 1000; i++)
        {
            float theta = Random.Range(0f, 2f * Mathf.PI);
            float phi = Random.Range(0f, Mathf.PI);
            float radius = Random.Range(1f, 3f);

            float x = radius * Mathf.Sin(phi) * Mathf.Cos(theta);
            float y = radius * Mathf.Sin(phi) * Mathf.Sin(theta);
            float z = radius * Mathf.Cos(phi);

            points.Add(new Vector3(x, y, z));
        }

        UpdatePointCloud(points);
    }
}
```

### Key Points:
- Object pooling for performance optimization
- Height-based coloring for visual clarity
- Configurable point size and color
- Efficient update mechanisms for large point clouds
- Sample generation for testing purposes

## Exercise 5 Solution: Sensor Fusion Implementation

### Problem
Implement a sensor fusion node that combines LiDAR and IMU data to improve robot localization.

### Solution
```python
#!/usr/bin/env python3
"""
Sensor Fusion Node for Robot Localization

This node combines LiDAR and IMU data to provide improved pose estimation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscriptions
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom_fused', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # State variables
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.robot_twist = np.array([0.0, 0.0, 0.0])  # vx, vy, omega
        self.last_imu_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion

        # Covariance matrices (simplified)
        self.pose_covariance = np.eye(6) * 0.1  # Initial uncertainty
        self.twist_covariance = np.eye(6) * 0.05

        # Timing
        self.last_update_time = self.get_clock().now()

        self.get_logger().info('Sensor fusion node initialized')

    def lidar_callback(self, msg):
        """Process LiDAR data for position estimation."""
        # Extract features from LiDAR scan (simplified approach)
        # In practice, this would involve feature extraction, ICP, etc.

        # For this example, we'll use a simple approach to estimate movement
        # based on changes in the environment
        self.process_lidar_data(msg)

    def imu_callback(self, msg):
        """Process IMU data for orientation and acceleration."""
        # Extract orientation from IMU
        quat = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])

        # Convert to Euler angles for simple processing
        rotation = R.from_quat(quat)
        euler_angles = rotation.as_euler('xyz')

        # Update orientation in state
        self.last_imu_orientation = quat
        self.robot_pose[2] = euler_angles[2]  # Update theta (yaw)

        # Extract angular velocity and linear acceleration
        angular_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        linear_acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Integrate to get velocity and position (simplified)
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time

        if dt > 0:
            # Update twist based on IMU acceleration
            self.robot_twist[2] = angular_vel[2]  # Angular velocity around Z

            # Integrate linear acceleration to get velocity (in robot frame)
            local_acc = np.array([linear_acc[0], linear_acc[1], 0])

            # Transform to global frame using current orientation
            cos_th = np.cos(self.robot_pose[2])
            sin_th = np.sin(self.robot_pose[2])
            R_global = np.array([[cos_th, -sin_th, 0],
                                [sin_th, cos_th, 0],
                                [0, 0, 1]])

            global_acc = R_global @ local_acc
            self.robot_twist[0:2] += global_acc[0:2] * dt

            # Update position based on velocity
            self.robot_pose[0:2] += self.robot_twist[0:2] * dt

    def process_lidar_data(self, scan_msg):
        """Process LiDAR data for position refinement."""
        # This is a simplified approach - in practice, you would:
        # 1. Extract features (corners, edges, planes) from the scan
        # 2. Match features with previous scan (ICP or similar)
        # 3. Estimate transformation between scans
        # 4. Fuse with IMU data using a Kalman filter or particle filter

        # For this example, we'll just use the data to adjust confidence
        # in our pose estimate based on environmental consistency

        # Calculate average distance to assess environment openness
        valid_ranges = [r for r in scan_msg.ranges if scan_msg.range_min <= r <= scan_msg.range_max]
        if valid_ranges:
            avg_distance = sum(valid_ranges) / len(valid_ranges)

            # Adjust covariance based on environmental features
            # More features generally mean better position confidence
            if avg_distance < 2.0:  # Close environment - more features available
                self.pose_covariance[:2, :2] *= 0.8  # Reduce position uncertainty
            else:  # Open environment - fewer features
                self.pose_covariance[:2, :2] *= 1.2  # Increase position uncertainty

    def publish_odometry(self):
        """Publish fused odometry data."""
        current_time = self.get_clock().now()
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set pose
        odom_msg.pose.pose.position.x = float(self.robot_pose[0])
        odom_msg.pose.pose.position.y = float(self.robot_pose[1])
        odom_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        yaw = self.robot_pose[2]
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = np.sin(yaw * 0.5)
        odom_msg.pose.pose.orientation.w = np.cos(yaw * 0.5)

        # Set pose covariance
        odom_msg.pose.covariance = self.pose_covariance.flatten().tolist()

        # Set twist
        odom_msg.twist.twist.linear.x = float(self.robot_twist[0])
        odom_msg.twist.twist.linear.y = float(self.robot_twist[1])
        odom_msg.twist.twist.angular.z = float(self.robot_twist[2])

        # Set twist covariance
        odom_msg.twist.covariance = self.twist_covariance.flatten().tolist()

        # Publish message
        self.odom_pub.publish(odom_msg)

        # Broadcast transform
        self.broadcast_transform(current_time, odom_msg.pose.pose)

    def broadcast_transform(self, stamp, pose):
        """Broadcast the transform from odom to base_link."""
        from geometry_msgs.msg import TransformStamped

        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z

        t.transform.rotation = pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
        """Periodic callback to publish fused data."""
        self.publish_odometry()


def main(args=None):
    rclpy.init(args=args)

    try:
        fusion_node = SensorFusionNode()

        # Create timer for periodic publishing
        timer = fusion_node.create_timer(0.05, fusion_node.timer_callback)  # 20 Hz

        rclpy.spin(fusion_node)

    except KeyboardInterrupt:
        pass
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Key Points:
- Combines LiDAR and IMU data for improved localization
- Implements basic sensor fusion algorithm
- Publishes fused odometry with covariance
- Uses proper ROS 2 message types and conventions
- Includes TF broadcasting for coordinate transforms
- Handles timing and data association properly

## Exercise 6 Solution: Unity Sensor Data Visualization

### Problem
Create a Unity script that visualizes different sensor modalities (LiDAR, camera, IMU) in a unified interface.

### Solution
```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections.Generic;

public class MultiSensorVisualizer : MonoBehaviour
{
    [Header("Sensor Data Sources")]
    public TextMeshProUGUI statusText;
    public TextMeshProUGUI poseText;
    public TextMeshProUGUI imuText;
    public TextMeshProUGUI lidarText;

    [Header("Visualization Elements")]
    public PointCloudVisualizer pointCloudVisualizer;
    public GameObject cameraFeedPanel;
    public RawImage cameraTexture;
    public GameObject imuIndicator;
    public LineRenderer trajectoryLine;

    [Header("Sensor Simulation")]
    public bool simulateSensors = true;
    public float simulationUpdateRate = 10f;

    // Internal state
    private float lastUpdateTime = 0f;
    private List<Vector3> trajectoryPoints;
    private Vector3 currentPose;
    private Vector3 currentImuData;
    private List<Vector3> simulatedPointCloud;

    void Start()
    {
        trajectoryPoints = new List<Vector3>();
        simulatedPointCloud = new List<Vector3>();

        if (trajectoryLine != null)
        {
            trajectoryLine.positionCount = 0;
        }

        if (simulateSensors)
        {
            GenerateInitialPointCloud();
        }
    }

    void Update()
    {
        if (simulateSensors && Time.time - lastUpdateTime > 1f / simulationUpdateRate)
        {
            UpdateSimulatedSensors();
            lastUpdateTime = Time.time;
        }

        UpdateVisualizations();
    }

    void GenerateInitialPointCloud()
    {
        // Generate a simple environment point cloud
        for (float x = -5f; x <= 5f; x += 0.5f)
        {
            for (float y = -5f; y <= 5f; y += 0.5f)
            {
                // Add ground points
                simulatedPointCloud.Add(new Vector3(x, -0.5f, y));

                // Add wall points occasionally
                if (Mathf.Abs(x) > 4.5f || Mathf.Abs(y) > 4.5f)
                {
                    for (float z = 0f; z <= 3f; z += 0.5f)
                    {
                        simulatedPointCloud.Add(new Vector3(x, z, y));
                    }
                }
            }
        }
    }

    void UpdateSimulatedSensors()
    {
        // Update pose with simulated movement
        currentPose += new Vector3(
            Mathf.Sin(Time.time) * 0.01f,
            0,
            Mathf.Cos(Time.time) * 0.01f
        );

        // Add current position to trajectory
        trajectoryPoints.Add(currentPose);
        if (trajectoryPoints.Count > 1000) // Limit trajectory length
        {
            trajectoryPoints.RemoveAt(0);
        }

        // Update IMU simulation
        currentImuData = new Vector3(
            Mathf.Sin(Time.time * 2) * 0.1f,  // Linear acceleration X
            Mathf.Cos(Time.time * 1.5f) * 0.1f,  // Linear acceleration Y
            Mathf.Sin(Time.time * 0.8f) * 9.81f + 0.1f  // Linear acceleration Z (gravity + noise)
        );

        // Simulate IMU orientation change
        if (imuIndicator != null)
        {
            imuIndicator.transform.rotation = Quaternion.Euler(
                Mathf.Sin(Time.time * 0.5f) * 5f,  // Roll
                Mathf.Cos(Time.time * 0.7f) * 5f,  // Pitch
                Mathf.Sin(Time.time * 0.3f) * 10f  // Yaw
            );
        }

        // Update point cloud with slight variations
        if (pointCloudVisualizer != null)
        {
            // Add some random points to simulate sensor noise
            List<Vector3> noisyPointCloud = new List<Vector3>(simulatedPointCloud);

            for (int i = 0; i < 50; i++)
            {
                Vector3 randomNoise = new Vector3(
                    Random.Range(-0.1f, 0.1f),
                    Random.Range(-0.1f, 0.1f),
                    Random.Range(-0.1f, 0.1f)
                );

                noisyPointCloud.Add(currentPose + randomNoise + new Vector3(
                    Random.Range(-2f, 2f),
                    Random.Range(-0.5f, 0.5f),
                    Random.Range(-2f, 2f)
                ));
            }

            pointCloudVisualizer.UpdatePointCloud(noisyPointCloud);
        }
    }

    void UpdateVisualizations()
    {
        // Update status text
        if (statusText != null)
        {
            statusText.text = $"Status: Active\n" +
                             $"Update Rate: {simulationUpdateRate:F1} Hz\n" +
                             $"Point Count: {(pointCloudVisualizer != null ? pointCloudVisualizer.GetPointCount() : 0)}";
        }

        // Update pose text
        if (poseText != null)
        {
            poseText.text = $"Pose:\n" +
                           $"X: {currentPose.x:F2} m\n" +
                           $"Y: {currentPose.y:F2} m\n" +
                           $"Z: {currentPose.z:F2} m";
        }

        // Update IMU text
        if (imuText != null)
        {
            imuText.text = $"IMU Data:\n" +
                          $"Acc X: {currentImuData.x:F3} m/s²\n" +
                          $"Acc Y: {currentImuData.y:F3} m/s²\n" +
                          $"Acc Z: {currentImuData.z:F3} m/s²";
        }

        // Update LiDAR text
        if (lidarText != null)
        {
            lidarText.text = $"LiDAR:\n" +
                            $"Points: {simulatedPointCloud.Count}\n" +
                            $"Range: 0.1 - 20.0 m\n" +
                            $"FOV: 360° H, 30° V";
        }

        // Update trajectory visualization
        if (trajectoryLine != null && trajectoryPoints.Count > 1)
        {
            trajectoryLine.positionCount = trajectoryPoints.Count;
            for (int i = 0; i < trajectoryPoints.Count; i++)
            {
                trajectoryLine.SetPosition(i, new Vector3(
                    trajectoryPoints[i].x,
                    0.1f,  // Slightly above ground for visibility
                    trajectoryPoints[i].z
                ));
            }
        }
    }

    public void ResetSimulation()
    {
        trajectoryPoints.Clear();
        currentPose = Vector3.zero;
        currentImuData = Vector3.zero;

        if (trajectoryLine != null)
        {
            trajectoryLine.positionCount = 0;
        }

        if (pointCloudVisualizer != null)
        {
            pointCloudVisualizer.ClearPointCloud();
        }
    }
}
```

### Key Points:
- Integrates multiple sensor modalities in a unified interface
- Visualizes point clouds, poses, and IMU data simultaneously
- Includes trajectory tracking and visualization
- Simulates realistic sensor behaviors with noise
- Provides comprehensive status monitoring
- Offers user controls for simulation reset

## Summary of Exercise Solutions

These solutions demonstrate:
1. **Proper sensor configuration** with realistic parameters
2. **Best practices** for sensor implementation in both Gazebo and Unity
3. **Integration techniques** for combining multiple sensor modalities
4. **Performance considerations** for real-time sensor processing
5. **Visualization methods** for sensor data interpretation

All solutions follow the educational contract standards with clear explanations, proper documentation, and realistic examples that match actual robotics applications.