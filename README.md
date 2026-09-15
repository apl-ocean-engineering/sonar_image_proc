# sonar_image_proc

A **ROS 2** package for processing and visualizing data from **forward-looking imaging sonars**. This package is a ROS 2 adaptation of the original [APL Ocean Engineering `sonar_image_proc` ROS 1 package](https://github.com/apl-ocean-engineering/sonar_image_proc).

The ROS 2 version has been tested with **Blueprint Oculus M1200d** and **M750d** imaging sonars.

## Acknowledgements

This package is based on the original **`sonar_image_proc`** package developed by **APL Ocean Engineering**. We acknowledge and thank the APL Ocean Engineering team for their original work and for making the ROS 1 package available as an open-source project.

**Original ROS 1 repository:** [APL Ocean Engineering `sonar_image_proc`](https://github.com/apl-ocean-engineering/sonar_image_proc)

---

## Features

> **Note:** This package is compatible with **ROS 2 Jazzy** and has been tested with `marine_acoustic_msgs` and OpenCV for sonar image processing.

* **ROS 2 Lifecycle Node** (`draw_sonar_lifecycle_node`) for managing node state transitions, including:

  * Configure
  * Activate
  * Deactivate
  * Cleanup
  * Shutdown

* **Sonar image visualization** with configurable parameters:

  * Maximum visualization range
  * Overlay line transparency, thickness, and spacing
  * Range and bearing line spacing
  * Logarithmic intensity scaling
  * Configurable color maps

* **Multiple output topics**:

  * `drawn_sonar`: Processed sonar image with polar remapping.
  * `drawn_sonar_rect`: Rectangular sonar image.
  * `drawn_sonar_osd`: Sonar image with visualization overlays, such as range and bearing lines.
  * `sonar_image_proc_timing`: Timing information for performance analysis.

* **Runtime parameter updates** for adjusting visualization settings without restarting the node.

* **Integration with `oculus_driver_package`**: The node subscribes to sonar data published by the driver package and processes the data when available.

* **Compatibility with `marine_acoustic_msgs`** for receiving sonar image data.

---

## Dependencies

### Build Dependencies

* **ROS 2 Jazzy**
* `ament_cmake`
* `rclcpp`
* `rclcpp_components`
* `rclcpp_lifecycle`
* `lifecycle_msgs`
* `cv_bridge`
* `image_transport`
* `marine_acoustic_msgs`
* `sensor_msgs`
* `std_msgs`
* OpenCV

### Runtime Dependencies

In addition to the build dependencies:

* `rclcpp_lifecycle`
* `lifecycle_msgs`
* `marine_acoustic_msgs`
* `oculus_driver_package`

---

## Installation

### From Source

Clone the repository into the `src` directory of your ROS 2 workspace:

```bash
cd ~/your_ros2_workspace/src
git clone https://github.com/apl-ocean-engineering/sonar_image_proc.git
```

Build the package:

```bash
cd ~/your_ros2_workspace
colcon build --packages-select sonar_image_proc
```

Source the workspace:

```bash
source install/setup.bash
```

---

## Usage

### Launch the Draw Sonar Node

Start the Draw Sonar node using:

```bash
ros2 launch sonar_image_proc draw_sonar.launch.py
```

### Configure and Activate the Node

After launching the node, configure and activate it:

```bash
ros2 lifecycle set /oculus/draw_sonar_node configure
ros2 lifecycle set /oculus/draw_sonar_node activate
```

### Deactivate and Shut Down the Node

To stop the node, first deactivate it, then clean up and shut it down:

```bash
ros2 lifecycle set /oculus/draw_sonar_node deactivate
ros2 lifecycle set /oculus/draw_sonar_node cleanup
ros2 lifecycle set /oculus/draw_sonar_node shutdown
```

### Update Visualization Parameters

Visualization parameters can be updated at runtime. For example, to change the spacing between range lines:

```bash
ros2 param set /oculus/draw_sonar_node range_spacing 2.0
```

---

## Parameters

The node supports the following parameters, which can be configured through ROS 2 parameter overrides or updated at runtime.

| Parameter           | Type   |     Default | Description                                                                       |
| ------------------- | ------ | ----------: | --------------------------------------------------------------------------------- |
| `max_range`         | double |      `40.0` | Maximum range, in meters, to visualize.                                           |
| `publish_old`       | bool   |     `false` | Enable publishing of the legacy API output.                                       |
| `publish_timing`    | bool   |      `true` | Enable publishing of timing metrics.                                              |
| `publish_histogram` | bool   |     `false` | Enable publishing of histogram data.                                              |
| `color_map`         | string | `"inferno"` | Color map used for visualization.                                                 |
| `line_alpha`        | double |       `0.2` | Transparency of visualization overlay lines.                                      |
| `line_thickness`    | int    |         `1` | Thickness of visualization overlay lines.                                         |
| `range_spacing`     | double |       `1.0` | Spacing between range lines, in meters. A value of `0` enables automatic spacing. |
| `bearing_spacing`   | double |      `45.0` | Spacing between bearing lines, in degrees.                                        |
| `log_scale`         | bool   |     `false` | Enable logarithmic scaling of intensity values.                                   |
| `min_db`            | double |       `0.0` | Minimum dB value used for logarithmic scaling.                                    |
| `max_db`            | double |       `0.0` | Maximum dB value used for logarithmic scaling.                                    |

### Parameter Override Example

Parameters can also be set when starting the node:

```bash
ros2 run sonar_image_proc draw_sonar_node \
  --ros-args -p max_range:=10.0
```

---

## Topics

### Subscribed Topics

| Topic         | Type                                           | Description             |
| ------------- | ---------------------------------------------- | ----------------------- |
| `sonar_image` | `marine_acoustic_msgs/msg/ProjectedSonarImage` | Input sonar image data. |

### Published Topics

| Topic                     | Type                    | Description                                 |
| ------------------------- | ----------------------- | ------------------------------------------- |
| `drawn_sonar`             | `sensor_msgs/msg/Image` | Processed sonar image with polar remapping. |
| `drawn_sonar_rect`        | `sensor_msgs/msg/Image` | Rectangular sonar image.                    |
| `drawn_sonar_osd`         | `sensor_msgs/msg/Image` | Sonar image with visualization overlays.    |
| `old_drawn_sonar`         | `sensor_msgs/msg/Image` | Legacy API output.                          |
| `sonar_image_proc_timing` | `std_msgs/msg/String`   | Processing timing metrics in JSON format.   |

---

## Architecture

### Key Classes

| Class                    | File                                                         | Description                                                              |
| ------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------------------ |
| `DrawSonarLifecycleNode` | `ros/include/sonar_image_proc/draw_sonar_lifecycle_node.hpp` | ROS 2 Lifecycle Node responsible for sonar image processing.             |
| `SonarDrawer`            | `include/sonar_image_proc/SonarDrawer.h`                     | Draws sonar images in rectangular and polar formats.                     |
| `SonarColorMap`          | `include/sonar_image_proc/ColorMaps.h`                       | Provides color maps for sonar intensity visualization.                   |
| `HistogramGenerator`     | `include/sonar_image_proc/HistogramGenerator.h`              | Generates histogram data for sonar images.                               |
| `SonarImageMsgInterface` | `ros/include/sonar_image_proc/sonar_image_msg_interface.h`   | Adapts `ProjectedSonarImage` messages to the sonar processing interface. |

### Legacy ROS 1 Components

Legacy ROS 1 components are retained in the repository for reference but are not used by the ROS 2 implementation.

---

## License

This package is licensed under the **BSD License**. See the [LICENSE](LICENSE) file for details.

---

## References

* **Original ROS 1 package:** [APL Ocean Engineering `sonar_image_proc`](https://github.com/apl-ocean-engineering/sonar_image_proc)
