# Isaac Sim Synthetic Data Generation

## Overview

Synthetic data generation is a core capability of Isaac Sim that enables the creation of large, diverse, and accurately labeled datasets for training AI models. Unlike real-world data collection, synthetic data generation provides unlimited data with perfect ground truth annotations, controlled environmental conditions, and diverse scenarios that would be difficult or impossible to capture in the real world.

## Key Benefits of Synthetic Data

- **Perfect Annotations**: Pixel-perfect segmentation, bounding boxes, and 3D annotations
- **Unlimited Data**: Generate as much data as needed without physical constraints
- **Controlled Environments**: Precise control over lighting, weather, and scene conditions
- **Safety**: Generate dangerous scenarios without risk to people or equipment
- **Cost-Effective**: No physical infrastructure or data collection teams required

## Synthetic Data Pipeline

The synthetic data generation process in Isaac Sim follows this general pipeline:

1. **Scene Creation**: Build virtual environments with diverse objects, lighting, and physics
2. **Sensor Configuration**: Set up virtual sensors (cameras, LiDAR, IMU, etc.)
3. **Data Collection**: Run simulations to generate sensor data
4. **Annotation Generation**: Automatically create ground truth annotations
5. **Dataset Export**: Export data in standard formats for training

## Scene Creation for Data Generation

### Environment Design
When creating scenes for synthetic data generation, consider:

- **Diversity**: Include multiple lighting conditions, weather patterns, and environmental variations
- **Realism**: Use physically accurate materials and lighting to match real-world conditions
- **Variability**: Vary object positions, orientations, and configurations
- **Annotation Readiness**: Ensure scenes are designed with annotation requirements in mind

### Asset Management
- Use high-quality 3D models with accurate physics properties
- Include a variety of object textures, materials, and appearances
- Implement domain randomization techniques to improve model generalization
- Organize assets in a structured manner for easy scene composition

## Sensor Configuration

### Camera Sensors
Configure camera sensors for different computer vision tasks:

```python
# Example: Configuring a RGB camera for object detection
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.sensor import Camera

# Create a camera prim
camera = Camera(
    prim_path="/World/Robot/Camera",
    frequency=30,  # 30 Hz capture rate
    resolution=(1920, 1080),  # Full HD resolution
    position=np.array([0, 0, 1.0]),
    orientation=usdrt.math.Quatf(0, 0, 0, 1)
)

# Enable RGB capture
camera.add_render_product("rgb", camera_resolution=(1920, 1080))
```

### LiDAR Sensors
For 3D object detection and mapping tasks:

```python
# Example: Configuring a LiDAR sensor
from omni.isaac.sensor import RotatingLidarPhysX

lidar = RotatingLidarPhysX(
    prim_path="/World/Robot/Lidar",
    translation=np.array([0, 0, 1.5]),
    config="Yosemite",
    min_range=0.1,
    max_range=25.0
)
```

## Annotation Types

### 2D Annotations
- **Bounding Boxes**: 2D bounding boxes for object detection
- **Semantic Segmentation**: Pixel-level class labels
- **Instance Segmentation**: Pixel-level instance labels
- **Keypoint Annotations**: 2D keypoint locations for pose estimation

### 3D Annotations
- **3D Bounding Boxes**: 3D cuboids for objects in the scene
- **Point Cloud Annotations**: Labeled point clouds from LiDAR
- **6D Pose**: Object position and orientation in 3D space
- **3D Keypoints**: 3D joint locations for articulated objects

## Domain Randomization

Domain randomization is a technique to improve the transfer of models trained on synthetic data to real-world applications:

- **Lighting Variation**: Randomize light positions, intensities, and colors
- **Material Randomization**: Vary surface properties, textures, and appearances
- **Camera Parameters**: Randomize camera intrinsics and extrinsics
- **Environmental Conditions**: Vary weather, fog, and atmospheric effects
- **Object Properties**: Randomize object textures, positions, and configurations

## Data Export Formats

Isaac Sim supports export to various standard formats:

### COCO Format
For object detection and segmentation tasks:
```json
{
  "info": {...},
  "licenses": [...],
  "categories": [...],
  "images": [...],
  "annotations": [...]
}
```

### KITTI Format
For 3D object detection:
```
Type,Truncated,Occluded,Alpha,Bbox_l,Bbox_t,Bbox_r,Bbox_b,Dim_h,Dim_w,Dim_l,Loc_x,Loc_y,Loc_z,Rot_y
Car,0.00,0,0.82,415.9,153.8,561.5,248.2,1.67,1.87,3.72,2.74,1.56,16.1,1.65
```

### Custom Formats
For specialized applications, custom export formats can be implemented using Isaac Sim's Python API.

## Best Practices

### Data Quality Assurance
- Validate annotations for accuracy and completeness
- Check for occlusions and visibility issues
- Verify sensor data quality and calibration
- Implement automated quality checks in the pipeline

### Performance Optimization
- Optimize scene complexity for desired frame rates
- Use appropriate resolution settings for target applications
- Implement efficient rendering techniques
- Balance realism with computational requirements

### Transfer Learning Considerations
- Include real-world data in training to bridge the sim-to-real gap
- Use domain adaptation techniques
- Validate model performance on real-world data
- Implement gradual domain adaptation strategies

## Example Workflow: Object Detection Dataset

1. **Create Scene**: Design a warehouse environment with various objects
2. **Configure Camera**: Set up RGB camera with appropriate parameters
3. **Randomize Environment**: Apply domain randomization techniques
4. **Run Simulation**: Generate data with different object configurations
5. **Export Annotations**: Export bounding box annotations in COCO format
6. **Validate Data**: Check data quality and annotation accuracy

## Troubleshooting

### Annotation Quality Issues
- Check sensor calibration and positioning
- Verify object visibility and occlusion handling
- Validate annotation pipeline settings
- Adjust rendering quality settings if needed

### Performance Problems
- Reduce scene complexity or rendering resolution
- Optimize asset complexity and polygon counts
- Adjust simulation parameters for faster rendering
- Use distributed rendering for large datasets

## Next Steps

After implementing synthetic data generation:
1. [Isaac Sim Workflows](./workflows.md) for best practices in data generation
2. [Isaac ROS Integration](../isaac-ros/introduction.md) to learn how to process synthetic data with Isaac ROS
3. [Exercises](./exercises.md) to practice synthetic data generation