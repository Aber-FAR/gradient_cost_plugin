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

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_pcl/helpers.hpp>
namespace gm = ::grid_map::grid_map_pcl;
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_costmap_2d/costmap_2d_converter.hpp>



#include <rclcpp/rclcpp.hpp>
#include <laser_geometry/laser_geometry.hpp>
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

//#define DO_DEBUG

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
   * \brief Update the bounds of the master costmap by this layer's update dimensions
   * \param robot_x X pose of robot
   * \param robot_y Y pose of robot
   * \param robot_yaw Robot orientation
   * \param min_x X min map coord of the window to update
   * \param min_y Y min map coord of the window to update
   * \param max_x X max map coord of the window to update
   * \param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double * min_x, double * min_y,
                            double * max_x, double * max_y);

  /*!
   * \brief Update the costs in the master costmap in the window
   * \param master_grid The master costmap grid to update
   * \param min_x X min map coord of the window to update
   * \param min_y Y min map coord of the window to update
   * \param max_x X max map coord of the window to update
   * \param max_y Y max map coord of the window to update
   */
  virtual void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                           int min_i, int min_j, int max_i, int max_j);

  /*!
   * \brief Deactivate the layer
   */
  // virtual void deactivate();

  /*!
   * \brief Activate the layer
   */
  // virtual void activate();

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
   * \param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  /*
   * \brief triggers the update of observations buffer
   */
  // void resetBuffersLastUpdated();

  /*!
   * \brief  A callback to handle buffering PointCloud2 messages
   * \param cloud The cloud (message) returned from a message notifier
   */
  void pointCloud2Callback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

  // for testing purposes
  // void addStaticObservation(nav2_costmap_2d::Observation& obs,
  //                           bool marking, bool clearing);
  // void clearStaticObservations(bool marking, bool clearing);

protected:
  /*
   * \brief  Get the observations used to mark space
   * \param marking_observations A reference to a vector that will be populated
   * with the observations
   * \return True if all the observation buffers are current, false otherwise
   */
  // bool getMarkingObservations(
  //   std::vector<nav2_costmap_2d::Observation>& marking_observations) const;

  /*
   * \brief  Get the observations used to clear space
   * \param clearing_observations A reference to a vector that will be
   * populated with the observations
   * \return True if all the observation buffers are current, false otherwise
   */
  // bool getClearingObservations(
  //   std::vector<nav2_costmap_2d::Observation>& clearing_observations) const;

  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
  bool footprint_clearing_enabled_;

  /*!
   * \brief Clear costmap layer info below the robot's footprint
   */
  void updateFootprint(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y);

  std::string global_frame_;  ///< \brief The global frame for the costmap
  double min_obstacle_height_;  ///< \brief Max Obstacle Height
  double max_obstacle_height_;  ///< \brief Max Obstacle Height

  /// \brief Used to project laser scans into point clouds
  laser_geometry::LaserProjection projector_;
  // \brief Used for the observation message filters
  // std::vector<std::shared_ptr<message_filters::SubscriberBase<rclcpp_lifecycle::LifecycleNode>>>
  // observation_subscribers_;
  // \brief Used to make sure that transforms are available for each sensor
  // std::vector<std::shared_ptr<tf2_ros::MessageFilterBase>> observation_notifiers_;
  // \brief Used to store observations from various sensors
  // std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> observation_buffers_;
  // \brief Used to store observation buffers used for marking obstacles
  // std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> marking_buffers_;
  // \brief Used to store observation buffers used for clearing obstacles
  // std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> clearing_buffers_;

  /// \brief Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Used only for testing purposes
  // std::vector<nav2_costmap_2d::Observation> static_clearing_observations_;
  // std::vector<nav2_costmap_2d::Observation> static_marking_observations_;

  bool rolling_window_;
  bool was_reset_;
//   int combination_method_;

#ifdef DO_DEBUG
  /*!
   * \brief pointcloud2 publisher.
   */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pc2_pub_ = nullptr;
#endif // DO_DEBUG

  /*!
   * \brief Pointcloud2 subscriber.
   */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr PC2_sub_ = nullptr;

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

  /*!
   * \brief PCL pointcloud2 to convert from the ROS pointcloud2.
   */
  pcl::PCLPointCloud2 pclCloud2_;
  
  /*!
   * \brief PCL pointcloud pointer to convert from the pointcloud2 and manipulate later.
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudPtr_ = nullptr;

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

  /*!
   * \brief GridMap PCL loader.
   */
  std::shared_ptr<grid_map::GridMapPclLoader> gridMapPclLoader_ = nullptr;
  
  /*!
   * \brief Max range for looking at obstacles.
   * This is used to filter out points from the pointcloud that are too far away.
   */
  float obstacle_max_range_;

  /*!
   * \brief Min range for looking at obstacles.
   * This is used to filter out points from the pointcloud that are too close.
   */
  float obstacle_min_range_;

  /*!
   * \brief Maximum gradient over which the ground is marked as lethal.
   */
  double maxGradient_;

  /*!
   * \brief Grid map publisher.
   */
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridMapPub_ = nullptr;

  /*!
   * \brief Name of the GridMap PCL loader config file.
   */
  std::string grid_map_config_file_;

  /*!
   * \brief Maximum size allowed for a step.
   */
  float maxStep_ = 0.05;

  /*!
   * \brief Name of the frame of the sensor.
   */
  std::string sensor_frame_;

};

}  // namespace gradient_cost_plugin

#endif  // GRADIENT_COST_LAYER_HPP
