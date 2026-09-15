# sonar_image_proc

A **ROS 2** package for processing and visualizing data from **forward-looking imaging sonars**. This package ports and adapts the [APL Ocean Engineering ROS 1 package](https://github.com/apl-ocean-engineering/sonar_image_proc) to ROS 2.

## Maintainers

- [Ankita Jadhav](mailto:ajadhav@marum.de)
- [Christian Meurer](mailto:cmeurer@uni-bremen.de)

---

## Features

> **Note:** This package is designed to work with **ROS 2 Humble and Jazzy**. It has been tested with `marine_acoustic_msgs` and OpenCV for sonar data processing.

- **Real-time sonar image processing** for forward-looking imaging sonars.
- **ROS 2 Lifecycle Node** (`draw_sonar_lifecycle_node`) for managing node state transitions (configure, activate, deactivate, cleanup, shutdown).
- **Sonar image drawing** with configurable parameters:
  - Max range
  - Color maps (e.g., `InfernoColorMap`, `MitchellColorMap`, `InfernoSaturationColorMap`)
  - Line alpha, thickness, and spacing for overlays
  - Logarithmic scaling for intensity values
- **Multiple output topics**:
  - `/drawn_sonar`: Processed sonar image (polar remapped).
  - `/drawn_sonar_rect`: Rectangular sonar image.
  - `/drawn_sonar_osd`: Sonar image with overlay (e.g., range/bearing lines).
  - `/sonar_image_proc_timing`: Timing metrics for performance analysis.
- **Dynamic parameter reconfiguration** at runtime.
- **Compatibility** with `marine_acoustic_msgs` for sonar data input.

---

## Dependencies

### Build Dependencies
- **ROS 2 Humble/Jazzy**
- `ament_cmake`
- `rclcpp`
- `rclcpp_components`
- `rclcpp_lifecycle`
- `lifecycle_msgs`
- `cv_bridge`
- `image_transport`
- `marine_acoustic_msgs`
- `std_msgs`
- `OpenCV` (for image processing)

### Runtime Dependencies (In addition to build dependencies)
- `rclcpp_lifecycle`
- `lifecycle_msgs`

---

## Installation

### From Source
1. Clone this repository into your ROS 2 workspace:
   ```bash
   git clone <repository_url> sonar_image_proc
   ```
2. Build the package:
   ```bash
   colcon build --packages-select sonar_image_proc
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

---

## Usage

### Running the Node
Launch the `draw_sonar_lifecycle_node` to process sonar images:
```bash
ros2 run sonar_image_proc draw_sonar_node --ros-args -p max_range:=30.0
```

### Parameters
The node supports the following parameters (configurable via ROS 2 parameter overrides or dynamic reconfiguration):

| Parameter            | Type    | Default Value | Description |
|----------------------|---------|---------------|-------------|
| `max_range`          | double  | 30.0          | Maximum range (meters) to visualize. |
| `publish_old`        | bool    | false         | Enable publishing legacy API output. |
| `publish_timing`     | bool    | true          | Publish timing metrics. |
| `publish_histogram`  | bool    | false         | Publish histogram data. |
| `color_map`          | string  | "inferno"     | Color map for visualization (e.g., "inferno", "mitchell"). |
| `line_alpha`         | double  | 0.5           | Transparency of overlay lines. |
| `line_thickness`     | int     | 1             | Thickness of overlay lines. |
| `range_spacing`      | double  | 0.0           | Spacing between range lines (0 = auto). |
| `bearing_spacing`    | double  | 20.0          | Spacing between bearing lines (degrees). |
| `log_scale`          | bool    | false         | Enable logarithmic scaling for intensity. |
| `min_db`             | double  | 0.0           | Minimum dB for logarithmic scaling. |
| `max_db`             | double  | 0.0           | Maximum dB for logarithmic scaling. |

Example parameter override:
```bash
ros2 run sonar_image_proc draw_sonar_node --ros-args -p max_range:=50.0 
```

### Topics
#### Subscribed Topics
| Topic         | Type                                  | Description |
|---------------|---------------------------------------|-------------|
| `sonar_image` | `marine_acoustic_msgs/msg/ProjectedSonarImage` | Input sonar image data. |

#### Published Topics
| Topic                     | Type                                  | Description |
|---------------------------|---------------------------------------|-------------|
| `drawn_sonar`             | `sensor_msgs/msg/Image`               | Processed sonar image (polar remapped). |
| `drawn_sonar_rect`        | `sensor_msgs/msg/Image`               | Rectangular sonar image. |
| `drawn_sonar_osd`         | `sensor_msgs/msg/Image`               | Sonar image with overlay. |
| `old_drawn_sonar`         | `sensor_msgs/msg/Image`               | Legacy API output. |
| `histogram`               | `std_msgs/msg/UInt32MultiArray`       | Histogram data. |
| `sonar_image_proc_timing` | `std_msgs/msg/String`                | Timing metrics (JSON format). |

---

## Architecture

### Key Classes
| Class | File | Description |
|-------|------|-------------|
| `DrawSonarLifecycleNode` | `ros/include/sonar_image_proc/draw_sonar_lifecycle_node.hpp` | ROS 2 Lifecycle Node for sonar image processing. |
| `SonarDrawer` | `include/sonar_image_proc/SonarDrawer.h` | Draws sonar images in rectangular and polar formats. |
| `SonarColorMap` | `include/sonar_image_proc/ColorMaps.h` | Color maps for sonar intensity visualization. |
| `HistogramGenerator` | `include/sonar_image_proc/HistogramGenerator.h` | Generates histograms for sonar data. |
| `SonarImageMsgInterface` | `ros/include/sonar_image_proc/sonar_image_msg_interface.h` | Adapts `ProjectedSonarImage` messages to `AbstractSonarInterface`. |

### Nodelets (Legacy)
- `sonar_postprocessor_node.cpp`: ROS 1 nodelet for backward compatibility (deprecated in ROS 2).

---

## License
This package is licensed under the **BSD** license. See the [LICENSE](LICENSE) file for details.

---
## Contributing
Contributions are welcome! Please open an issue or submit a pull request.

---
## References
- Original ROS 1 package: [apl-ocean-engineering/sonar_image_proc](https://github.com/apl-ocean-engineering/sonar_image_proc)
- Colormap inspiration: [BIDS/colormap](https://github.com/BIDS/colormap)
- OpenCV documentation: [Overlay Image Example](https://docs.opencv.org/4.x/d3/d63/classcv_1_1Mat.html)
