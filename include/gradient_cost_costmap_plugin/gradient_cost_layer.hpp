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
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
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

  /*!
   * \brief A callback handling depth images, when using "depth_image" as
   * input_type.
   *
   * This only stores the received image and, exactly like
   * pointCloud2Callback(), sets data_received_ to true and data_processed_
   * to false to flag that new, as yet unprocessed, data has arrived.  It
   * deliberately does not convert the image to a pointcloud itself:
   * deprojecting a depth image is comparatively expensive, and the depth
   * image may be published faster than the costmap actually consumes it, or
   * while this layer is disabled.  The conversion is instead done lazily by
   * convertPendingDepthImage(), called from updateBounds() using the same
   * data_received_/data_processed_ guard as the rest of that function, so
   * that it only ever happens (at most once per update cycle) when the
   * result is actually going to be used.
   * \param depth_msg The depth image.
   */
  void depthImageCallback(sensor_msgs::msg::Image::ConstSharedPtr depth_msg);

  /*!
   * \brief A callback to keep track of the CameraInfo matching the depth
   * images, when using "depth_image" as input_type.
   * \param info_msg The camera info.
   */
  void cameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);

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

  /*!
   * \brief Possible method of combining several elevation values in the GridMap.
   */
  enum elev_combination_methods
  {
    first = 0,
    last,
    min,
    max,
    average
  };

  /*!
   * \brief Convert a string to the equivalent elevation combination code.
   *
   * If the provided string is not valid, then the code is not changed.
   *
   * \param[in] name The string representing the method.
   * \param[out] code The code (from elev_combination_methods).
   * \param[in] do_exit If true, then we exit in case of a unknown string.
   * \return true if the provided string was valid, false otherwise.
   */
  bool from_string(const std::string& name,
                   elev_combination_methods* code,
                   bool do_exit = false)
  {
    bool ok = true;
    if (name == "first")
      *code = first;
    else if (name == "last")
      *code = last;
    else if (name == "min")
      *code = min;
    else if (name == "max")
      *code = max;
    else if (name == "average")
      *code = average;
    else
    {
      ok = false;
      RCLCPP_ERROR_STREAM(logger_,
                          "Unknown elevation combination method: "
                          << name);
      if (do_exit)
        exit(EXIT_FAILURE);
    }

    return(ok);
  }

  /*!
   * \brief The ombination method for when more than one point cloud point fall
   * in the same cell of the GridMap.
   */
  elev_combination_methods elev_comb_ = last;

  /*!
   * \brief Derive the default CameraInfo topic from a depth image topic,
   * following the usual image_transport convention, e.g.
   * "/camera/depth/image_raw" -> "/camera/depth/camera_info".
   * \param[in] image_topic The depth image topic.
   * \return The derived camera_info topic.
   */
  static std::string deriveCameraInfoTopic(const std::string& image_topic);

  /*!
   * \brief Convert latest_depth_msg_ into a PointCloud2 and feed it to
   * pointCloud2Callback().
   *
   * Only called from updateBounds(), guarded there by the same
   * data_received_/data_processed_ check used for the "pointcloud"
   * input_type, so the (expensive) deprojection only happens when new depth
   * data has arrived and a costmap update is actually about to consume it.
   * Logs a throttled warning and does nothing if no CameraInfo has been
   * received yet.
   */
  void convertPendingDepthImage();

  /*!
   * \brief The global frame for the costmap
   */
  std::string global_frame_;

  /*!
   * \brief Dynamic parameters handler
   */
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  bool rolling_window_;
  bool was_reset_;

  /*!
   * \brief Source of data used to populate the GridMap: either "pointcloud"
   * (a sensor_msgs::msg::PointCloud2) or "depth_image" (a
   * sensor_msgs::msg::Image plus its matching sensor_msgs::msg::CameraInfo).
   */
  std::string input_type_ = "pointcloud";

  /*!
   * \brief Pointcloud2 subscriber.  Only used when input_type is
   * "pointcloud".
   */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr PC2_sub_ = nullptr;

  /*!
   * \brief Depth image subscriber.  Only used when input_type is
   * "depth_image".
   */
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_ = nullptr;

  /*!
   * \brief Topic on which the CameraInfo matching the depth image is
   * received.  Only used when input_type is "depth_image".
   */
  std::string camera_info_topic_;

  /*!
   * \brief CameraInfo subscriber.  Only used when input_type is
   * "depth_image".
   */
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_
                                                                = nullptr;

  /*!
   * \brief Latest CameraInfo received, used to project the depth image into
   * a pointcloud.  Only used when input_type is "depth_image".
   */
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_ = nullptr;

  /*!
   * \brief Latest depth image received.  Only used when input_type is
   * "depth_image".
   *
   * Whether it still needs converting to a pointcloud is tracked by
   * data_received_/data_processed_ (set directly in depthImageCallback()),
   * exactly as for the "pointcloud" input_type; there is no separate
   * bookkeeping for the depth image path.
   */
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_msg_ = nullptr;

  /*!
   * \brief Pinhole camera model built from latest_camera_info_.  Only used
   * when input_type is "depth_image".
   */
  image_geometry::PinholeCameraModel camera_model_;

  /*!
   * \brief Whether the depth image's own frame (depth_msg->header.frame_id,
   * overridden by sensor_frame_ if set) is a proper camera "_optical_frame"
   * (X right, Y down, Z forward into the scene), as used by
   * depth_image_proc::convertDepth() and by real camera drivers.
   *
   * Some sources (notably Gazebo's simulated depth camera) instead tag the
   * depth image with the sensor's REP103 body frame (X forward, Y left, Z
   * up) and bake the optical-to-body axis remap into their own point
   * generation.  Set this to false in that case so
   * convertPendingDepthImage() applies the same remap; otherwise the
   * resulting pointcloud ends up rotated relative to that frame.  Only used
   * when input_type is "depth_image".
   */
  bool depth_frame_is_optical_ = true;

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

  /*!
   * \brief Time of the last data.
   */
  rclcpp::Time last_data_time_;

  /*!
   * \brief Timeout on data (in seconds).
   *
   * If the data becomes older than this, the costmap is declared not current.
   * A zero (default) or negative timeout disables this.
   */
  double no_data_timeout_ = 0;

  /*!
   * \brief Tells if the last arrived data (pointcloud) has been processed.
   */
  bool data_processed_ = false;

  /*!
   * \brief Tells if we have received some data, in case we try to process it
   * before then.
   */
  bool data_received_ = false;
};

}  // namespace gradient_cost_plugin

#endif  // GRADIENT_COST_LAYER_HPP
