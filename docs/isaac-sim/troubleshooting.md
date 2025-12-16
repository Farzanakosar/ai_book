# Isaac Sim Troubleshooting Guide

## Common Issues and Solutions

### Installation and Launch Issues

#### Isaac Sim Won't Launch
**Symptoms**: Application fails to start or crashes immediately after launch

**Solutions**:
1. **Check GPU Drivers**: Ensure NVIDIA drivers are updated to version 510 or higher
   ```bash
   nvidia-smi
   ```

2. **Verify GPU Compatibility**: Check if your GPU meets minimum requirements
   ```bash
   nvidia-smi -q | grep "Product Name"
   ```

3. **Launch with Minimal Configuration**:
   ```bash
   cd ~/IsaacSim
   ./isaac-sim.sh --no-configure
   ```

4. **Check Disk Space**: Ensure at least 50GB free space in installation directory
   ```bash
   df -h ~/IsaacSim
   ```

5. **Run with Verbose Logging**:
   ```bash
   ./isaac-sim.sh --verbose
   ```

#### Performance Issues
**Symptoms**: Low frame rates, stuttering, or application freezing

**Solutions**:
1. **Reduce Rendering Quality**: Lower resolution or disable advanced rendering features
2. **Optimize Scene Complexity**: Remove unnecessary objects or simplify geometry
3. **Check System Resources**: Monitor CPU, GPU, and RAM usage
4. **Close Other Applications**: Free up system resources for Isaac Sim
5. **Update Graphics Drivers**: Ensure latest drivers are installed

#### GPU Not Detected
**Symptoms**: Isaac Sim reports no compatible GPU found

**Solutions**:
1. **Verify Driver Installation**:
   ```bash
   nvidia-smi
   glxinfo | grep "OpenGL renderer"
   ```

2. **Check GPU Power**: Ensure GPU is properly connected to power supply
3. **Verify Display Connection**: Connect monitor directly to GPU (not motherboard)
4. **Disable Integrated Graphics**: In BIOS, set primary graphics to PCIe/NVIDIA

### ROS 2 Integration Issues

#### ROS Bridge Connection Problems
**Symptoms**: Isaac Sim and ROS 2 nodes cannot communicate

**Solutions**:
1. **Verify ROS Domain ID**:
   ```bash
   echo $ROS_DOMAIN_ID
   # Should match between Isaac Sim and ROS 2 terminals
   ```

2. **Check Network Configuration**:
   ```bash
   # Verify network interfaces
   ip addr show

   # Check if Fast DDS is properly configured
   printenv | grep RMW
   ```

3. **Test Basic ROS Communication**:
   ```bash
   # In separate terminals
   ros2 topic list
   ros2 run demo_nodes_cpp talker
   ros2 run demo_nodes_cpp listener
   ```

4. **Launch Isaac Sim with ROS Support**:
   ```bash
   cd ~/IsaacSim
   ./isaac-sim.sh --enable-ros2-bridge
   ```

#### Sensor Data Issues
**Symptoms**: Sensor data not publishing or incorrect format

**Solutions**:
1. **Verify Sensor Configuration**: Check sensor parameters in Isaac Sim
2. **Check Topic Names**: Ensure topic names match between Isaac Sim and ROS nodes
3. **Validate Message Types**: Confirm message type compatibility
4. **Monitor Topic Traffic**:
   ```bash
   ros2 topic list
   ros2 topic echo /camera/rgb/image_raw
   ```

### Rendering and Graphics Issues

#### Black Screen or No Display
**Symptoms**: Isaac Sim window appears but shows black screen

**Solutions**:
1. **Check OpenGL Support**:
   ```bash
   glxinfo | grep "OpenGL version"
   glxinfo | grep "direct rendering"
   ```

2. **Verify Display Settings**: Ensure monitor is properly detected
3. **Launch with Software Rendering**:
   ```bash
   export MESA_GL_VERSION_OVERRIDE=3.3
   ./isaac-sim.sh
   ```

4. **Reset Isaac Sim Configuration**:
   ```bash
   mv ~/.nvidia-omniverse/config ~/.nvidia-omniverse/config.backup
   ```

#### Visual Artifacts
**Symptoms**: Incorrect lighting, missing textures, or geometry issues

**Solutions**:
1. **Update GPU Drivers**: Install latest NVIDIA Studio or Game Ready drivers
2. **Check VRAM Usage**: Monitor GPU memory usage during simulation
3. **Adjust Rendering Settings**: Lower quality settings to reduce GPU load
4. **Verify Asset Integrity**: Check if custom assets are properly formatted

### Data Generation Issues

#### Synthetic Data Quality Problems
**Symptoms**: Poor annotation quality, missing annotations, or incorrect data format

**Solutions**:
1. **Verify Sensor Configuration**: Check camera/LiDAR parameters
2. **Check Lighting Conditions**: Ensure adequate lighting for visibility
3. **Validate Annotation Pipeline**: Test annotation generation separately
4. **Monitor Data Export**: Verify exported data format and quality

#### Performance During Data Generation
**Symptoms**: Slow data generation or frame drops during capture

**Solutions**:
1. **Reduce Scene Complexity**: Simplify geometry or reduce object count
2. **Optimize Capture Frequency**: Lower frame rate if real-time capture isn't needed
3. **Use Headless Mode**: Generate data without GUI for better performance
4. **Check Storage Speed**: Ensure fast storage for data writing

### Python API Issues

#### Script Execution Problems
**Symptoms**: Python scripts fail to execute or Isaac Sim API calls fail

**Solutions**:
1. **Verify Python Environment**: Ensure correct Python version and packages
2. **Check Isaac Sim Python Path**:
   ```bash
   export PYTHONPATH=$ISAACSIM_PATH/python:$PYTHONPATH
   ```

3. **Test Basic API Access**:
   ```python
   from omni.isaac.core import World
   world = World()
   world.reset()
   ```

4. **Check Permissions**: Ensure write permissions for required directories

#### Memory Management Issues
**Symptoms**: Application crashes due to memory exhaustion during long runs

**Solutions**:
1. **Monitor Memory Usage**: Use system monitoring tools during execution
2. **Implement Proper Cleanup**: Clear objects and reset world state periodically
3. **Reduce Batch Sizes**: Process smaller chunks of data at a time
4. **Use Memory Profiling**: Identify memory leaks in custom scripts

## Diagnostic Tools

### System Information Commands
```bash
# GPU information
nvidia-smi

# System resources
htop
nvidia-ml-py3 # For GPU monitoring in Python

# Graphics information
glxinfo | grep -i opengl
vulkaninfo # If Vulkan is supported
```

### Isaac Sim Logging
```bash
# Enable verbose logging
./isaac-sim.sh --verbose --log-level info

# Check log files
ls ~/.nvidia-omniverse/logs/
tail -f ~/.nvidia-omniverse/logs/latest.log
```

### ROS 2 Diagnostic Commands
```bash
# Check ROS network
ros2 doctor

# Monitor topics
ros2 topic list -t
ros2 topic hz /camera/rgb/image_raw

# Check services
ros2 service list
```

## Advanced Troubleshooting

### Debugging with Isaac Sim Console
1. Launch Isaac Sim with console enabled
2. Use the built-in scripting console to test API calls
3. Monitor internal logs and error messages
4. Use the scene debugger to inspect scene objects

### Performance Profiling
1. **GPU Profiling**: Use NVIDIA Nsight Systems for GPU analysis
2. **CPU Profiling**: Monitor CPU usage and identify bottlenecks
3. **Memory Profiling**: Track memory allocation and leaks
4. **Network Profiling**: For distributed simulation setups

### Log Analysis
```bash
# Isaac Sim logs location
~/.nvidia-omniverse/logs/

# ROS 2 logs location
~/.ros/log/

# Isaac ROS logs
~/isaac_ros_ws/log/
```

## Common Error Messages

### "Failed to initialize GPU"
- **Cause**: Incompatible or outdated GPU drivers
- **Solution**: Update to latest NVIDIA drivers

### "Out of memory"
- **Cause**: Insufficient GPU or system memory
- **Solution**: Reduce scene complexity or increase system resources

### "Could not connect to ROS master"
- **Cause**: ROS network configuration issue
- **Solution**: Verify ROS_DOMAIN_ID and network settings

### "Python module not found"
- **Cause**: Incorrect Python path or missing packages
- **Solution**: Set PYTHONPATH and install required packages

### "Failed to load asset"
- **Cause**: Missing or corrupted asset files
- **Solution**: Verify asset paths and file integrity

## Prevention Strategies

### Regular Maintenance
- Update GPU drivers regularly
- Clean up temporary files and logs periodically
- Monitor system health and resource usage
- Keep Isaac Sim updated to latest stable version

### Best Practices
- Test changes incrementally
- Maintain backup configurations
- Document working setups
- Use version control for scenes and scripts

## Getting Help

### Official Resources
- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
- [NVIDIA Developer Forums](https://forums.developer.nvidia.com/)
- [Isaac ROS GitHub Issues](https://github.com/NVIDIA-ISAAC-ROS)

### Community Support
- ROS Discourse forums
- Isaac Sim community Discord
- Robotics Stack Exchange

## Next Steps

If troubleshooting doesn't resolve your issue:
1. Check the [Performance Optimization Guide](./performance.md) for advanced techniques
2. Review the [Isaac ROS Integration](../isaac-ros/troubleshooting.md) troubleshooting guide
3. Consider creating a minimal reproduction case for support