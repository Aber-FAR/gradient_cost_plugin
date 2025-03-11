# gradient_cost_plugin

This package is a ROS2 navigation2 costmap layer plugin to populate a layer with a cost that depends on the gradient of the landscape in front of the robot.

The data used is a `sensor_msgs::msg::PointCloud2`, e.g. created by an RGBD camera.

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
| `footprint_clearing_enabled` | If true, the robot footprint will clear (mark as free) the space in which it travels. | True | Yes |
| `max_step` | Maximum step allowed (in m), above which a cell is marked as `LETHAL_OBSTACLE`. | 0.2 | Yes |
| `max_angle` | Maximum angle from horizontal (in degrees) of the landscape above which the cell is marked as `LETHAL_OBSTACLE`. | 40 | Yes |
| `min_angle` | Minimum angle from horizontal (in degrees) of the landscape, below which the cell is marked as `min_cost`. | 10 | Yes |
| `min_cost` | Minimum cost to use.  Some planners like having a cost! | 10 | Yes |
| `topic` | Topic name where to get the pointclouds from. | empty string | No |
| `obstacle_max_range` | Maximum range in the frame of the sensor to be considered in the pointcloud (also used to set the size of the GridMap). | 5.0 | No |
| `obstacle_min_range` | Minimum range in the frame of the sensor to be considered in the pointcloud. | 0.0 | No |
| `sensor_frame` | Name of the frame of the sensor, to overide or set a missing frame from the data. | empty string | No |

The GridMap size is set by the parameter `obstacle_max_range`.  The resolution of the GridMap is that of the corresponding map (parameter `resolution` of the costmap).  The maximum allowed step size should obviously not only depend on the physical capabilities of the robot but also the distance between neighbouring cells, e.g. the resolution of the GridMap.

The `costmap_2d` parameter `transform_tolerance` is also used.

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
```
