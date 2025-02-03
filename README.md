# gradient_cost_plugin

This package is a ROS2 navigation2 costmap layer plugin to populate a layer with a cost that depends on the gardient of the landscape in front of a robot.

The data used is a pointcloud2, eg created by an RGBD camera.

The plugin uses the Grid Map package (http://github.com/anybotics/grid_map).  This turns the pointcloud2 into a grid of elevations., which are then processed.

Two pieces of information are calculated: a step size and a gradient.

## Step size

The step size is the maximum absolute value difference in elevation between a cell of the grid map and its 8 neighbours.  If this step size is larger than the threshold `max_step`, the cell is marked as `LETHAL_OBSTACLE`.

## Gradient

The gradient is calculated using a Sobel Operator (https://en.wikipedia.org/wiki/Sobel_operator).  This gives a gradient in the two directions of the map and is used to compute the slope of the landscape at this location.

The slope is then turned into a cost by linearly mapping [`min_angle`,`max_angle`] to cost [0, `LETHAL_OBSTACLE`].  Slopes outside the source range are clamped to the ends of the range.

## Parameters

| Parameter | Description |
|-----------|-------------|
| `max_step` | Maximum step allowed, above which a cell is marked as `LEHAL_OBSTACLE`. |
| `min_angle` and `max_angle` | Minimum and maximum angles, mapped to costs [0, `LETHAL_OBSTACLE`]. |