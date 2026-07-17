# gradient_cost_plugin

This package is a ROS2 navigation2 costmap layer plugin to populate a layer with a cost that depends on the gradient of the landscape in front of the robot.

The data used is either a `sensor_msgs::msg::PointCloud2`, e.g. created by an RGBD camera, or a `sensor_msgs::msg::Image` depth image (plus its matching `sensor_msgs::msg::CameraInfo`), e.g. published directly by an RGBD camera driver.  This is selected with the `input_type` parameter (see below).  When a depth image is used, it is deprojected into a pointcloud (using `depth_image_proc`'s conversion code) before being processed exactly as if it had been received as a `PointCloud2`.

The plugin uses the GridMap package (http://github.com/anybotics/grid_map).  This turns the pointcloud2 into a grid of elevations, which are then processed.  All the processing is done in the global frame (set by `global_frame`, see below).

Two pieces of information are calculated: a step size and a gradient.

## Step size

The step size is the maximum absolute value difference in elevation between a cell of the grid map and its 8 neighbours.  If this step size is larger than the threshold `max_step`, the cell is marked as `LETHAL_OBSTACLE`.

## Gradient

The gradient is calculated using a Sobel Operator (https://en.wikipedia.org/wiki/Sobel_operator).  This gives a gradient in the two directions of the map and is used to compute the slope of the landscape at this location.

The slope is then turned into a cost by linearly mapping [`min_angle`, `max_angle`] to cost [`FREE_SPACE`, `LETHAL_OBSTACLE`].  Slopes outside the source range are clamped to the ends of the target range.  Only the absolute value of the angle is considered (same cost whether we are going up or down).

## Parameters

| Parameter | Description | Default values | Dynamic? |
|-----------|-------------|----------------|----------|
| `enabled` | Whether the plugin is enabled or not. | True | Yes |
| `max_step` | Maximum step allowed (in m), above which a cell is marked as `LETHAL_OBSTACLE`. | 0.2 | Yes |
| `max_angle` | Maximum angle from horizontal (in degrees) of the landscape above which the cell is marked as `LETHAL_OBSTACLE`. | 40 | Yes |
| `min_angle` | Minimum angle from horizontal (in degrees) of the landscape, below which the cell is marked as `min_cost`. | 10 | Yes |
| `min_cost` | Minimum cost to use.  Some planners like having a cost! | 10 | Yes |
| `input_type` | Type of data received on `topic`: `"pointcloud"` (a `PointCloud2`) or `"depth_image"` (an `Image`). | "pointcloud" | No |
| `topic` | Topic name where to get the pointclouds (or depth images) from. | empty string | No |
| `camera_info_topic` | Topic name for the `CameraInfo` matching the depth image.  Only used when `input_type` is `"depth_image"`.  If left empty, it is derived from `topic` by replacing its last path component with `camera_info` (the usual `image_transport` convention), e.g. `/camera/depth/image_raw` -> `/camera/depth/camera_info`. | empty string | No |
| `depth_frame_is_optical` | Whether the depth image's frame is a proper camera `_optical_frame` (X right, Y down, Z forward), as used by real camera drivers.  Set to `false` if it is instead tagged with a plain REP103 body/link frame (X forward, Y left, Z up) that already expects the optical-to-body axis remap applied in software -- this is the case for Gazebo's simulated depth camera, for example.  Getting this wrong produces a pointcloud that looks rotated relative to its own declared frame.  Only used when `input_type` is `"depth_image"`. | True | No |
| `obstacle_max_range` | Maximum range in the frame of the sensor to be considered in the pointcloud (also used to set the size of the GridMap). | 5.0 | No |
| `obstacle_min_range` | Minimum range in the frame of the sensor to be considered in the pointcloud. | 0.0 | No |
| `sensor_frame` | Name of the frame of the sensor, to overide or set a missing frame from the data. | empty string | No |
| `elevation_combination` | Specifies how multiple pointcloud points are combined in one cell of the GridMap (see below). | "last" | Yes |
| `no_data_timeout` | Timeout on data to consider the produced costmap current (in seconds).  A zero or negative value disables the check. | 0.0 | No |

When `input_type` is `"depth_image"`, the depth image must use either the `16UC1` (depth in mm, e.g. RealSense-style) or `32FC1` (depth in m) encoding; the frame used for the transform to `global_frame` is the depth image's `header.frame_id` (or `sensor_frame` if set).

The GridMap size is set by the parameter `obstacle_max_range`.  The resolution of the GridMap is that of the corresponding map (parameter `resolution` of the costmap).  The maximum allowed step size should obviously not only depend on the physical capabilities of the robot but also the distance between neighbouring cells, e.g. the resolution of the GridMap/costmap.

The `costmap_2d` parameter `transform_tolerance` is also used.

When multiple points from the pointcloud fall in the same cell, they can be combined in a numer of ways, specified by the parameter `elevation_combination`:

- "first": uses the first elevation;
- "last": uses the last elevation.  This is probably the cheapest method;
- "min": uses the minimum elevation.  This is good if negative obstacles are expected, but amplifies the importance of noise;
- "max": uses the maximum elevation.  This is good if positive obstacles are expected, but amplifies the importance of noise;
- "average": calculates the average of all elevation values falling in each cell.  This is good to remove noise, but is expensive.

## yaml config file

Below is an example of the yaml entry for the plugin, here as part of the global map.

```
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: odom
      robot_base_frame: base_link
      footprint: "[ [0.28, 0.14], [0.28, -0.14], [-0.243, -0.14], [-0.243, 0.14] ]"
      rolling_window: False
      width: 500
      height: 500
      resolution: 0.5
      origin_x: -1.0
      origin_y: -250.0
      track_unknown_space: false
      plugins: ["gradient_layer"]
      gradient_layer:
        plugin: "gradient_cost_plugin::GradientCostLayer"
        enabled: True
        footprint_clearing_enabled: True
        max_step: 0.3
        min_angle: 10
        max_angle: 40
        topic: /oak/points
        obstacle_max_range: 5.0
        obstacle_min_range: 0.2
        elevation_combination: "last"
        no_data_timeout: 0.5
```

To use a depth image instead of a pointcloud, set `input_type` to `"depth_image"` and point `topic` at the depth image (the matching `CameraInfo` topic is auto-derived unless `camera_info_topic` is set):

```
      gradient_layer:
        plugin: "gradient_cost_plugin::GradientCostLayer"
        enabled: True
        max_step: 0.3
        min_angle: 10
        max_angle: 40
        input_type: "depth_image"
        topic: /oak/stereo/image_raw
        obstacle_max_range: 5.0
        obstacle_min_range: 0.2
```

In Gazebo simulation, the simulated depth camera typically tags the depth image with its plain link frame rather than a proper `_optical_frame`, so `depth_frame_is_optical` needs to be set to `false`:

```
        input_type: "depth_image"
        topic: /camera/depth/image_raw
        depth_frame_is_optical: false
```
