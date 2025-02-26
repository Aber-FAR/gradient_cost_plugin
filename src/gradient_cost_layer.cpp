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

#include "gradient_cost_costmap_plugin/gradient_cost_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav2_costmap_2d/costmap_math.hpp>

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

using rcl_interfaces::msg::ParameterType;

using std::placeholders::_1;

namespace gradient_cost_plugin
{

  /*!
   * \brief Destructor
   */
  GradientCostLayer::~GradientCostLayer()
  {
    dyn_params_handler_.reset();

  }

  /*!
   * \brief Initialization process of layer on startup
   */
  void GradientCostLayer::onInitialize()
  {
    double transform_tolerance;

    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
    declareParameter("min_obstacle_height", rclcpp::ParameterValue(0.0));
    declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));
    declareParameter("observation_sources",
                     rclcpp::ParameterValue(std::string("")));
    declareParameter("max_step",
                     rclcpp::ParameterValue(max_step_));
    int tmp_angle = static_cast<int>(min_angle_ + 0.5);
    declareParameter("min_angle",
                     rclcpp::ParameterValue(tmp_angle));
    tmp_angle = static_cast<int>(max_angle_ + 0.5);
    declareParameter("max_angle",
                     rclcpp::ParameterValue(tmp_angle));

    auto node = node_.lock();
    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }

    node->get_parameter(name_ + "." + "enabled", enabled_);
//     node->get_parameter(name_ + "." + "footprint_clearing_enabled",
//                         footprint_clearing_enabled_);
    node->get_parameter("transform_tolerance", transform_tolerance);
    tf_tolerance_ = tf2::durationFromSec(transform_tolerance);
    node->get_parameter(name_ + "." + "max_step", max_step_);
    node->get_parameter(name_ + "." + "min_angle", tmp_angle);
    min_angle_ = static_cast<float>(tmp_angle) / 180.0 * M_PI;
    node->get_parameter(name_ + "." + "max_angle", tmp_angle);
    max_angle_ = static_cast<float>(tmp_angle) / 180.0 * M_PI;

    if (rcutils_logging_set_logger_level("gradient_cost_layer",
                                         RCUTILS_LOG_SEVERITY_ERROR)
            != RCUTILS_RET_OK)
    {
      RCLCPP_DEBUG(logger_,
        "Could not change logging level of logger 'gradient_cost_layer'");
    }

    dyn_params_handler_ = node->add_on_set_parameters_callback(
        std::bind(
            &GradientCostLayer::dynamicParametersCallback,
            this,
            std::placeholders::_1));

    rolling_window_ = layered_costmap_->isRolling();

    default_value_ = FREE_SPACE;

    matchSize();

    current_ = true;
    was_reset_ = false;

    global_frame_ = layered_costmap_->getGlobalFrameID();

    std::string topic;

    declareParameter("topic", rclcpp::ParameterValue(""));
    declareParameter("sensor_frame", rclcpp::ParameterValue(std::string("")));
    declareParameter("obstacle_max_range",
                     rclcpp::ParameterValue(obstacle_max_range_));
    declareParameter("obstacle_min_range",
                     rclcpp::ParameterValue(obstacle_min_range_));

    node->get_parameter(name_ + "." + "topic", topic);
    node->get_parameter(name_ + "." + "sensor_frame", sensor_frame_);

    node->get_parameter(name_ + "." + "obstacle_max_range", obstacle_max_range_);
    node->get_parameter(name_ + "." + "obstacle_min_range", obstacle_min_range_);

    std::string nodeName(node->get_name());
    grid_map_pub_ = node->create_publisher<grid_map_msgs::msg::GridMap>(
        nodeName + "/grid_map_from_pointcloud",
        10);

    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    PC2_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, qos, std::bind(&GradientCostLayer::pointCloud2Callback, this, _1));

    //
    // Initialise the grid map.
    //
    // We make the gridmap the size of what interests us.  We make it big
    // enough so that we don't have to resize it all the time, whatever the
    // orientation of the robot is.  We therefore need a square with side equal
    // to 2 * obstacle_max_range.  This is wastefull in memory and clearing time,
    // and could lead to going over cells that are definitely not covered by the
    // pointclouds.  However resizing the grid (or creating a new grid) at every
    // new pointcloud is expensive.  So as the gridmap is populated we keep the
    // bounding box of the pointcloud (as index in the grid map) and
    // only process this.
    grid_map::Length length = grid_map::Length(2 * obstacle_max_range_,
                                               2 * obstacle_max_range_);

    // we put the center of the grid map where the sensor is.  At the moment we
    // don't know so will set it to (0,0) and setPosition() the grid map when we
    // get pointclouds.
    grid_map::Position position = grid_map::Position(0.0, 0.0);

    grid_map_.setGeometry(length, resolution_, position);
    grid_map_.setFrameId(global_frame_);
    grid_map_.add("elevation");
    grid_map_.add("cost");
  }


  /*!
   * \brief Callback executed when a parameter change is detected
   * \param parameters ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  GradientCostLayer::dynamicParametersCallback(
                              std::vector<rclcpp::Parameter> parameters)
  {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    rcl_interfaces::msg::SetParametersResult result;

    for (auto parameter : parameters)
    {
      const auto &param_type = parameter.get_type();
      const auto &param_name = parameter.get_name();

      if (param_type == ParameterType::PARAMETER_DOUBLE)
      {
        if (param_name == name_ + "." + "max_step")
        {
          max_step_ = parameter.as_double();
        }
        else if (param_name == name_ + "." + "max_angle")
        {
          max_angle_ = parameter.as_double();
        }
        else if (param_name == name_ + "." + "min_angle")
        {
          min_angle_ = parameter.as_double();
        }
      }
      else if (param_type == ParameterType::PARAMETER_BOOL)
      {
        if (param_name == name_ + "." + "enabled" && enabled_ != parameter.as_bool())
        {
          enabled_ = parameter.as_bool();
          if (enabled_)
          {
            current_ = false;
          }
        }
//         else if (param_name == name_ + "." + "footprint_clearing_enabled")
//         {
//           footprint_clearing_enabled_ = parameter.as_bool();
//         }
      }
      else if (param_type == ParameterType::PARAMETER_INTEGER)
      {
        // if (param_name == name_ + "." + "combination_method")
        // {
        //   combination_method_ = parameter.as_int();
        // }
      }
    }

    result.successful = true;
    return result;
  }

  /*!
   * \brief  A callback to handle buffering PointCloud2 messages
   * \param cloud The cloud (message) returned from a message notifier
   */
  void GradientCostLayer::pointCloud2Callback(
                    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

    auto node = node_.lock();

    //
    // We transform the pointcloud to the global frame.  This makes a copy of
    // the pointcloud.
    std::string origin_frame = (sensor_frame_ == "" ?
      cloud->header.frame_id : sensor_frame_);

    cloud_transf_.header.stamp = cloud->header.stamp;
    cloud_transf_.header.frame_id = origin_frame;
    try
    {
      tf2_buffer_->transform(*cloud, cloud_transf_,
                             global_frame_, tf_tolerance_);
    }
    catch (tf2::TransformException & ex)
    {
      // if an exception occurs, we need to remove the empty observation from the list
      RCLCPP_ERROR(
        logger_,
        "TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s",
        origin_frame.c_str(),
        cloud->header.frame_id.c_str(), ex.what());
      return;
    }

    // We save the origin of the sensor, in the global frame.
    geometry_msgs::msg::PointStamped orig_point;
    orig_point.header.stamp = cloud->header.stamp;
    orig_point.header.frame_id = cloud->header.frame_id;
    orig_point.point.x = 0;
    orig_point.point.y = 0;
    orig_point.point.z = 0;
    try
    {
      tf2_buffer_->transform(orig_point, orig_transf_point_,
                             global_frame_, tf_tolerance_);
    }
    catch (tf2::TransformException & ex)
    {
      // if an exception occurs, we need to remove the empty observation from the list
      RCLCPP_ERROR(
        logger_,
        "TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s",
        cloud->header.frame_id.c_str(),
        cloud->header.frame_id.c_str(), ex.what());
      return;
    }

    // We have new data to process so we are not current any more.
    current_ = false;

  }



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
  void GradientCostLayer::updateBounds(double robot_x, double robot_y,
                                       double robot_yaw,
                                       double* min_x, double* min_y,
                                       double* max_x, double* max_y)
  {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());  

    auto node = node_.lock();

    if (current_)
    {
      // There is nothing to do so we return.
      return;
    }

    if (rolling_window_)
    {
      updateOrigin(robot_x - getSizeInMetersX() / 2,
                   robot_y - getSizeInMetersY() / 2);
    }
    if (!enabled_)
    {
      return;
    }
    useExtraBounds(min_x, min_y, max_x, max_y);

    // update the global current status
    current_ = true;

    //
    // Initialise the grid map.
    //
    grid_map_.clearAll();

    // And reset the bounding box of the PC in the gridmap.
    BB_max_(0) = BB_max_(1) = std::numeric_limits<int>::min();
    BB_min_(0) = BB_min_(1) = std::numeric_limits<int>::max();

    // Point from the original pointcloud and a point for the gridmap.
    geometry_msgs::msg::PointStamped pc_point;
    geometry_msgs::msg::PointStamped gm_point;
    pc_point.header.stamp = cloud_transf_.header.stamp;
    pc_point.header.frame_id = cloud_transf_.header.frame_id;

    grid_map_.setPosition(grid_map::Position(orig_transf_point_.point.x,
                                             orig_transf_point_.point.y));

    // Filter the transformed PC.
    // We filter out points too close or too far from the sensor.
    // The points that are kept are directly put in the gridmap.

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_transf_, "x");
    std::vector<unsigned char>::const_iterator iter_cloud
                                          = cloud_transf_.data.begin();
    std::vector<unsigned char>::const_iterator iter_cloud_end
                                          = cloud_transf_.data.end();
    for (; iter_cloud != iter_cloud_end;
         ++iter_x, iter_cloud += cloud_transf_.point_step)
    {
      double pointDepth2 = ((iter_x[0] - orig_transf_point_.point.x)
                            * (iter_x[0] - orig_transf_point_.point.x))
                           + ((iter_x[1] - orig_transf_point_.point.y)
                            * (iter_x[1] - orig_transf_point_.point.y))
                           + ((iter_x[2] - orig_transf_point_.point.z)
                            * (iter_x[2] - orig_transf_point_.point.z));

      if ((pointDepth2 <= (obstacle_max_range_ * obstacle_max_range_))
          && (pointDepth2 >= (obstacle_min_range_ * obstacle_min_range_)))
      {
        // Make a point from the PC
        gm_point.point.x = iter_x[0];
        gm_point.point.y = iter_x[1];
        gm_point.point.z = iter_x[2];

        // Add the point to the gridmap.
        grid_map::Index mapIdx;
        bool res = grid_map_.getIndex(grid_map::Position(gm_point.point.x,
                                                         gm_point.point.y),
                                      mapIdx);
        if (res)
        {
          // If a value already exist at this location, we keep the highest.
          double current_elev = grid_map_.at("elevation", mapIdx);
          grid_map_.at("elevation", mapIdx) = std::max(gm_point.point.z,
                                                       current_elev);

          // And update the bounding box
          BB_max_(0) = std::max(BB_max_(0), mapIdx(0));
          BB_max_(1) = std::max(BB_max_(1), mapIdx(1));
          BB_min_(0) = std::min(BB_min_(0), mapIdx(0));
          BB_min_(1) = std::min(BB_min_(1), mapIdx(1));
        }
      }
    }

    //
    // Create the costs in the gridmap and the costmap.
    //
    grid_map::Index mapIdx, subMapIdx;
    unsigned int mx, my;
    grid_map::Position mapPos;
    unsigned int costmapIdx;
    int indX, indY;
    for (indX = BB_min_(0) + 1; indX <= BB_max_(0) - 1; indX++)
    {
      for (indY = BB_min_(1) + 1; indY <= BB_max_(1) - 1; indY++)
      {
        mapIdx(0) = indX;
        mapIdx(1) = indY;

        // Remove locations that do not have data
        float centreElev = grid_map_.at("elevation", mapIdx);
        if (std::isnan(centreElev))
        {
          // No data here, we might as well ignore the point.
          continue;
        }

        // Remove locations that are outside of the local map.
        grid_map_.getPosition(mapIdx, mapPos);
        bool point_ok = worldToMap(mapPos(0), mapPos(1), mx, my);
        if (!point_ok)
        {
          RCLCPP_DEBUG_STREAM(logger_,
            "Computing map coords failed: (" << mapPos(0) << "," << mapPos(1)
            << ")");
          continue;
        }

        float cost = FREE_SPACE;
        // float cost = 100;
        costmapIdx = getIndex(mx, my);
        costmap_[costmapIdx] = cost;
        grid_map_.at("cost", mapIdx) = cost;

        // First a look at the 8 neighbours to check step size.
        if ((fabs(grid_map_.at("elevation", grid_map::Index(indX-1,indY-1))
                  - centreElev) > max_step_)
            || (fabs(grid_map_.at("elevation", grid_map::Index(indX-0,indY-1))
                      - centreElev) > max_step_)
            || (fabs(grid_map_.at("elevation", grid_map::Index(indX+1,indY-1))
                      - centreElev) > max_step_)
            || (fabs(grid_map_.at("elevation", grid_map::Index(indX-1,indY-0))
                      - centreElev) > max_step_)
            || (fabs(grid_map_.at("elevation", grid_map::Index(indX+1,indY-0))
                      - centreElev) > max_step_)
            || (fabs(grid_map_.at("elevation", grid_map::Index(indX-1,indY+1))
                      - centreElev) > max_step_)
            || (fabs(grid_map_.at("elevation", grid_map::Index(indX-0,indY+1))
                      - centreElev) > max_step_)
            || (fabs(grid_map_.at("elevation", grid_map::Index(indX+1,indY+1))
                      - centreElev) > max_step_))
        {
          grid_map_.at("cost", mapIdx) = LETHAL_OBSTACLE;
          costmap_[costmapIdx] = LETHAL_OBSTACLE;
          touch(mapPos(0), mapPos(1), min_x, min_y, max_x, max_y);
          continue;
        }

        // Then we calculate the local gradient using a Sobel operator.
        const float devide = (4.0 * 2.0 * resolution_);
               // 4.0 for the weighting, 2.0 for 2 increments on x or y
        float grad_x =
          ( 1.0 * (grid_map_.at("elevation", grid_map::Index(indX+1,indY-1))
                  - grid_map_.at("elevation", grid_map::Index(indX-1,indY-1)))
          + 2.0 * (grid_map_.at("elevation", grid_map::Index(indX+1,indY-0))
                  - grid_map_.at("elevation", grid_map::Index(indX-1,indY-0)))
          + 1.0 * (grid_map_.at("elevation", grid_map::Index(indX+1,indY+1))
                  - grid_map_.at("elevation", grid_map::Index(indX-1,indY+1))))
          / devide;
        float grad_y =
          (  1.0 * (grid_map_.at("elevation", grid_map::Index(indX-1,indY+1))
                   - grid_map_.at("elevation", grid_map::Index(indX-1,indY-1)))
           + 2.0 * (grid_map_.at("elevation", grid_map::Index(indX+0,indY+1))
                   - grid_map_.at("elevation", grid_map::Index(indX+0,indY-1)))
           + 1.0 * (grid_map_.at("elevation", grid_map::Index(indX+1,indY+1))
                   - grid_map_.at("elevation", grid_map::Index(indX+1,indY-1))))
           / devide;

        // And the angle to horizontal, in deg.  The normal pointing up to the
        // grid is N = (grad_x, grad_y, 1).  The vertical direction is
        // V = (0, 0, 1).  The angle to the vertical is
        // angle_v = acos((N.V)/sqrt(mag(N)*mag(V))), where '.' is the dot
        // product.  Here N.V = 1 and mag(V) = 1.  he angle N to V is what we
        // need to compare to min_angle_ and max_angle_.
        float angle = acos(1.0
                          / sqrt(grad_x * grad_x + grad_y * grad_y + 1));
        // Angles in [pi/2, pi] need to be mirrored to fall in [0, pi/2] as
        // they correspond to an upside down face which seems to happen on
        // (near) horizontal faces
        if (angle > M_PI_2)
        {
          angle = M_PI - angle;
        }
        // We only care about the absolute value of the angle.
        angle = fabs(angle);
        if (angle <= min_angle_)
          cost = 0;
        else if (angle >= max_angle_)
          cost = LETHAL_OBSTACLE;
        else
          cost = ((angle - min_angle_)
                 * LETHAL_OBSTACLE / (max_angle_ - min_angle_));

        // Set the cost in the gris and costmap.
        grid_map_.at("cost", mapIdx) = cost;
        costmap_[costmapIdx] = cost;
        touch(mapPos(0), mapPos(1), min_x, min_y, max_x, max_y);
      }
    }

    if (grid_map_pub_->get_subscription_count() > 0)
    {
      auto msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
      grid_map_pub_->publish(std::move(msg));
    }

    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  }



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
  void GradientCostLayer::updateFootprint(double robot_x, double robot_y,
                                          double robot_yaw,
                                          double *min_x, double *min_y,
                                          double *max_x,
                                          double *max_y)
  {
    if (!footprint_clearing_enabled_)
    {
      return;
    }
    nav2_costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw,
                                        getFootprint(), transformed_footprint_);

    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
      touch(transformed_footprint_[i].x, transformed_footprint_[i].y,
            min_x, min_y, max_x, max_y);
    }
  }


  /*!
   * \brief Update the costs in the master costmap in the window
   * \param master_grid The master costmap grid to update
   * \param min_x X min map coord of the window to update
   * \param min_y Y min map coord of the window to update
   * \param max_x X max map coord of the window to update
   * \param max_y Y max map coord of the window to update
   */
  void GradientCostLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                      int min_i, int min_j, int max_i, int max_j)
  {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    if (!enabled_)
    {
      return;
    }

    // if not current due to reset, set current now after clearing
    if (!current_ && was_reset_)
    {
      was_reset_ = false;
      current_ = true;
    }

    if (footprint_clearing_enabled_)
    {
      setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
    }

    // We always overwrite.
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  }


  void
  GradientCostLayer::reset()
  {
    resetMaps();
    // resetBuffersLastUpdated();
    current_ = false;
    was_reset_ = true;
  }

} // namespace gradient_cost_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(gradient_cost_plugin::GradientCostLayer,
                       nav2_costmap_2d::Layer)
