/*
 * gradient_cost_costmap_plugin
 * 
 * Copyright 2025 Aberystwyth University
 * 
 * This library and program are free software: you can redistribute them
 * and/or modify them under the terms of the GNU General Public License as 
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 * 
 * The program and library are distributed in the hope that they will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef GRADIENT_COST_LAYER_HPP
#define GRADIENT_COST_LAYER_HPP

#include <memory>
#include <string>
#include <vector>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_costmap_2d/costmap_2d_converter.hpp>



#include <rclcpp/rclcpp.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#include <tf2_ros/message_filter.h>
#pragma GCC diagnostic pop
#include <message_filters/subscriber.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/observation_buffer.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <tf2_ros/buffer.h>

namespace gradient_cost_plugin
{

/*!
 * \class GradientCostLayer
 * \brief This plugin populates a costmap_2d with a cost calculated from the
 * gradient of a surface extracted from a pointcloud in some reference frame.
 */
class GradientCostLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  /*!
   * \brief Constructor
   */
  GradientCostLayer()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
  }

  /*!
   * \brief Destructor
   */
  virtual ~GradientCostLayer();

  /*!
   * \brief Initialization process of layer on startup
   */
  virtual void onInitialize();

  /*!
   * \brief Update the bounds of the master costmap by this layer's update
   * dimensions
   * \param robot_x X pose of robot
   * \param robot_y Y pose of robot
   * \param robot_yaw Robot orientation
   * \param min_x X min map coord of the window to update
   * \param min_y Y min map coord of the window to update
   * \param max_x X max map coord of the window to update
   * \param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y,
                            double* max_x, double* max_y);

  /*!
   * \brief Update the costs in the master costmap in the window
   * \param master_grid The master costmap grid to update
   * \param min_x X min map coord of the window to update
   * \param min_y Y min map coord of the window to update
   * \param max_x X max map coord of the window to update
   * \param max_y Y max map coord of the window to update
   */
  virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                           int min_i, int min_j, int max_i, int max_j);

  /*!
   * \brief Reset this costmap
   */
  virtual void reset();

  /*!
   * \brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable() {return true;}

  /*!
   * \brief Callback executed when a parameter change is detected
   * \param parameters ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  /*!
   * \brief  A callback to handle buffering PointCloud2 messages
   * \param cloud The cloud (message) returned from a message notifier
   */
  void pointCloud2Callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

protected:

  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
  bool footprint_clearing_enabled_ = false;

  /*!
   * \brief Clear costmap layer info below the robot's footprint.
   * \param robot_x X pose of robot
   * \param robot_y Y pose of robot
   * \param robot_yaw Robot orientation
   * \param min_x X min map coord of the window to update
   * \param min_y Y min map coord of the window to update
   * \param max_x X max map coord of the window to update
   * \param max_y Y max map coord of the window to update
   */
  void updateFootprint(double robot_x, double robot_y, double robot_yaw,
                       double* min_x, double* min_y,
                       double* max_x, double* max_y);

  std::string global_frame_;  ///< \brief The global frame for the costmap

  /// \brief Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  bool rolling_window_;
  bool was_reset_;

  /*!
   * \brief Pointcloud2 subscriber.
   */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr PC2_sub_ = nullptr;

  /*!
   * \brief Received point cloud transformed to the global_frame.
   */
  sensor_msgs::msg::PointCloud2 cloud_transf_;

  /*!
   * \brief Origin of the sensor, transformed to global frame.
   */
  geometry_msgs::msg::PointStamped orig_transf_point_;

  /*!
   * \brief GridMap where the received pointcloud2 is stored.
   *
   * This is done after filtering and frame change.
   */
  grid_map::GridMap grid_map_;

  /*!
   * \brief Max coords of the bounding box of the pointcloud in the gridmap.
   */
  grid_map::Index BB_max_;

  /*!
   * \brief Min coords of the bounding box of the pointcloud in the gridmap.
   */
  grid_map::Index BB_min_;

  /*!
   * \brief Transformed observation cloud, filtered and transformed from the cloud
   * received in the callback.
   */
  sensor_msgs::msg::PointCloud2 global_frame_cloud_;

  /*
   * \brief PCL pointcloud2 to convert from the ROS pointcloud2.
   */
//   pcl::PCLPointCloud2 pclCloud2_;

  /*
   * \brief PCL pointcloud pointer to convert from the pointcloud2 and manipulate later.
   */
//   pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudPtr_ = nullptr;

  /*!
   * \brief Buffer for the TFs received.
   */
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_ = nullptr;

  /*!
   * \brief Transform listener for the pointclouds.
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;

  /*!
   * \brief Time tolerance in the transform, in s.
   */
  tf2::Duration tf_tolerance_;

  /*
   * \brief GridMap PCL loader.
   */
//   std::shared_ptr<grid_map::GridMapPclLoader> gridMapPclLoader_ = nullptr;

  /*!
   * \brief Max range for looking at obstacles.
   * This is used to filter out points from the pointcloud that are too far away.
   */
  float obstacle_max_range_ = 5.0;

  /*!
   * \brief Min range for looking at obstacles.
   * This is used to filter out points from the pointcloud that are too close.
   */
  float obstacle_min_range_ = 0.0;

  /*!
   * \brief Maximum gradient over which the ground is marked as lethal.
   */
  double maxGradient_;

  /*!
   * \brief Grid map publisher.
   */
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_ = nullptr;

  /*!
   * \brief Minimun cost to use.
   *
   * For some planners, having a cost (rather than 0) helps.
   */
  float min_cost_ = 10;

  /*!
   * \brief Maximum size allowed for a step (m).
   */
  float max_step_ = 0.20;

  /*!
   * \brief Minimum angle of the slope (rad).
   *
   * Any angle below that maps to a cost of 0.
   */
  float min_angle_ = 10;

  /*!
   * \brief Maximum angle of the slope (rad).
   *
   * Any angle above that maps to a cost of LETHAL_OBSTACLE.
   */
  float max_angle_ = 40;

  /*!
   * \brief Name of the frame of the sensor.
   */
  std::string sensor_frame_;

};

}  // namespace gradient_cost_plugin

#endif  // GRADIENT_COST_LAYER_HPP
