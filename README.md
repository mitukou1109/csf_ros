# csf_ros

A **ROS 2 wrapper** for the _Cloth Simulation Filter (CSF)_<sup>[1]</sup>, providing ground and non-ground point cloud separation.

Internally, this package uses [**csf_pcl**](https://github.com/mitukou1109/csf_pcl.git).

## 🚀 Usage

Clone the repositories, install dependencies, build, and run the node:

```bash
cd ~/csf_ws/src  # Move to your workspace's source directory
git clone https://github.com/mitukou1109/csf_pcl.git
git clone https://github.com/mitukou1109/csf_ros.git
cd ..
rosdep install -iyr --from-paths src
colcon build  # Add options as needed
source ~/csf_ws/install/local_setup.bash
ros2 run csf_ros ground_filter_node
```

## 🔄 Interface

### Subscribed Topics

| Name      | Type                      | Description       |
| --------- | ------------------------- | ----------------- |
| `/points` | `sensor_msgs/PointCloud2` | Input point cloud |

### Published Topics

| Name                  | Type                      | Description            |
| --------------------- | ------------------------- | ---------------------- |
| `~/ground_points`     | `sensor_msgs/PointCloud2` | Ground point cloud     |
| `~/off_ground_points` | `sensor_msgs/PointCloud2` | Off-ground point cloud |

## ⚙️ Parameters

| Name                              | Type           | Default value | Description                                                                                                                                                      |
| --------------------------------- | -------------- | ------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `gravity_aligned_frame`           | string         | `""`          | The frame whose z-axis is aligned with gravity. If provided, the input point cloud is transformed into this frame before filtering.                              |
| `crop_range.[min\|max]`           | vector<double> | `[]`          | Cropping range for the input point cloud. Specify `[x, y, z]` for min/max points, or leave empty to disable cropping.                                            |
| `cloth_resolution`                | double         | 1.0           | Cloth grid resolution.                                                                                                                                           |
| `cloth_margin`                    | double         | 2.0           | Margin around the area occupied by the input point cloud.                                                                                                        |
| `cloth_rigidness`                 | int            | 3             | Cloth rigidness (see the paper for details). Higher values make the cloth less sensitive to small bumps.                                                         |
| `cloth_initial_z_offset`          | double         | 0.05          | Height offset below the lowest position (minimum z value) of the point cloud, defining where the cloth starts to fall.                                           |
| `gravity`                         | double         | 9.81          | Gravity used in the cloth simulation.                                                                                                                            |
| `time_step`                       | double         | 0.1           | Time step for each iteration of the simulation.                                                                                                                  |
| `max_iterations`                  | int            | 500           | Maximum number of iterations. The simulation stops if this count is reached or when the height variation becomes smaller than `iteration_termination_threshold`. |
| `iteration_termination_threshold` | double         | 0.005         | Height variation threshold for terminating the simulation.                                                                                                       |
| `enable_post_processing`          | bool           | true          | Enable or disable post-processing (see the paper for details).                                                                                                   |
| `slope_fitting_threshold`         | double         | 0.3           | Threshold for slope fitting (corresponds to $h_{\mathrm{cp}}$ in the paper).                                                                                     |
| `classification_threshold`        | double         | 0.5           | Threshold for classifying ground vs. off-ground points (corresponds to $h_{\mathrm{cc}}$ in the paper).                                                          |

## 🧩 Reference

[1] W. Zhang et al., “An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation,” Remote Sensing, vol. 8, no. 6, p. 501, June 2016, doi: 10.3390/rs8060501.

## 📝 License

This project is licensed under the [BSD License](LICENSE).
