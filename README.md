# csf_ros

ROS 2 wrapper for [CSF](https://github.com/jianboqi/CSF) (Cloth Simulation Filtering).

## Usage

### Install CSF

```
git clone https://github.com/jianboqi/CSF.git
cd CSF
mkdir build && cd build
cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON # the option is required to link the library with shared one (ROS node)
make -j
sudo make install
```

### Build

```
cd ~/csf_ws/src # example
git clone https://github.com/mitukou1109/csf_ros.git
cd ..
rosdep install -iyr --from-paths src
colcon build # add options as you like
source ~/csf_ws/install/local_setup.bash
ros2 run csf_ros ground_filter_node --ros-args --params-file ~/csf_ws/install/csf_ros/share/csf_ros/config/ground_filter.yaml
```

## Interface

### Subscribed topics

| Name      | Type                      | Description       |
| --------- | ------------------------- | ----------------- |
| `/points` | `sensor_msgs/PointCloud2` | Input point cloud |

### Published topics

| Name                  | Type                      | Description            |
| --------------------- | ------------------------- | ---------------------- |
| `~/ground_points`     | `sensor_msgs/PointCloud2` | Ground point cloud     |
| `~/off_ground_points` | `sensor_msgs/PointCloud2` | Off-ground point cloud |

### Parameters

| Name                     | Type     | Description |
| ------------------------ | -------- | ----------- |
| `enable_post_processing` | `bool`   |             |
| `class_threshold`        | `double` |             |
| `cloth_resolution`       | `double` |             |
| `max_iterations`         | `int`    |             |
| `rigidness`              | `int`    |             |
| `time_step`              | `double` |             |
