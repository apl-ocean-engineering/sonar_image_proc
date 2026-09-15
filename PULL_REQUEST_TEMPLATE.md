# Pull Request Template for APL Ocean Engineering

## Title
**Port sonar_image_proc to ROS 2 (Humble/Jazzy)**

---

## Description
This pull request ports the ROS 1 `sonar_image_proc` package to **ROS 2 (Humble/Jazzy)**. The primary goal is to maintain the original functionality while adapting to ROS 2 APIs and conventions.

### Key Changes
1. **ROS 2 Migration**:
   - Replaced `ros::NodeHandle` with `rclcpp::Node` and `rclcpp::LifecycleNode` for lifecycle management.
   - Updated message dependencies to use ROS 2 versions (e.g., `marine_acoustic_msgs`).
   - Replaced ROS 1 nodelets with ROS 2 components.

2. **API Adaptations**:
   - Used `rclcpp_lifecycle` for node lifecycle transitions (configure, activate, deactivate, cleanup, shutdown).
   - Updated parameter handling to use `rclcpp::Parameter` and dynamic parameter callbacks.
   - Replaced `cv_bridge` and `image_transport` with their ROS 2 equivalents.

3. **Removed Deprecated Features**:
   - Removed ROS 1-specific nodelets (e.g., `sonar_postprocessor_node.cpp` is deprecated in ROS 2).
   - Removed dependencies on `dynamic_reconfigure` (replaced with ROS 2 parameter APIs).

4. **Preserved Functionality**:
   - All original features (sonar image drawing) are preserved.
   - Supports the same input/output topics and parameters as the ROS 1 version.

5. **Dependencies**:
   - Updated `package.xml` to use ROS 2 dependencies (e.g., `rclcpp`, `rclcpp_lifecycle`, `lifecycle_msgs`).
   - Retained `OpenCV` for image processing.

---

## Testing
- **Build Status**: The package builds successfully with `colcon build` in a ROS 2 Humble/Jazzy environment.
- **Functionality**: Tested with simulated sonar data to ensure:
  - Sonar images are correctly processed and published.
  - Parameters can be dynamically reconfigured.
  - Lifecycle transitions work as expected.
- **Compatibility**: Verified compatibility with `marine_acoustic_msgs` and OpenCV.

---

## Beispiel Command
To run the node with default parameters:
```bash
ros2 run sonar_image_proc draw_sonar_node --ros-args -p max_range:=30.0
```

---

## Example Parameter Overrides
```bash
ros2 run sonar_image_proc draw_sonar_node --ros-args \
  -p max_range:=50.0 \
  -p color_map:="mitchell" \
  -p publish_histogram:=true
```

---

## Limitations
- The input topic name (`sonar_image`) is currently hardcoded in the node. To use a different topic, modify the subscription in `draw_sonar_lifecycle_node.cpp:195`.
- Some legacy API features (e.g., `old_api` drawing) are retained for backward compatibility but may be deprecated in future versions.

---

## References
- Original ROS 1 package: [apl-ocean-engineering/sonar_image_proc](https://github.com/apl-ocean-engineering/sonar_image_proc)
- ROS 2 Lifecycle documentation: [ROS 2 Lifecycle](https://docs.ros.org/en/rolling/Concepts/About-Lifecycle-Nodes.html)

---

## Checklist
- [x] Code builds without errors in ROS 2 Humble/Jazzy.
- [x] All original features are preserved.
- [x] README updated to reflect ROS 2 changes.
- [x] License (BSD) remains unchanged.
- [ ] CI tests pass (if applicable).

---

## Additional Notes
- This PR is a **direct port** of the ROS 1 package to ROS 2. No new features are added, but the code is structured to facilitate future enhancements.
- Contributors are welcome to test and provide feedback on this port.