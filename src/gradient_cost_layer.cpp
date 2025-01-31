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

#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_pcl/helpers.hpp>
namespace gm = ::grid_map::grid_map_pcl;
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_costmap_2d/costmap_2d_converter.hpp>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

using nav2_costmap_2d::Observation;
using nav2_costmap_2d::ObservationBuffer;
using rcl_interfaces::msg::ParameterType;

namespace gradient_cost_plugin
{

  GradientCostLayer::~GradientCostLayer()
  {
    dyn_params_handler_.reset();
    for (auto &notifier : observation_notifiers_)
    {
      notifier.reset();
    }
  }

  void GradientCostLayer::onInitialize()
  {
    bool track_unknown_space;
    double transform_tolerance;

    // The topics that we'll subscribe to from the parameter server
    std::string topics_string;

    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
    declareParameter("min_obstacle_height", rclcpp::ParameterValue(0.0));
    declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));
    // declareParameter("combination_method", rclcpp::ParameterValue(1));
    declareParameter("observation_sources",
                     rclcpp::ParameterValue(std::string("")));
    declareParameter("max_gradient",
                     rclcpp::ParameterValue(45.0 / 180.0 * M_PI));
    declareParameter("max_step",
                     rclcpp::ParameterValue(maxStep_));
    declareParameter("grid_map_config_file",
                     rclcpp::ParameterValue(std::string("")));

    auto node = node_.lock();
    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }

    node->get_parameter(name_ + "." + "enabled", enabled_);
    node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
    node->get_parameter(name_ + "." + "min_obstacle_height", min_obstacle_height_);
    node->get_parameter(name_ + "." + "max_obstacle_height", max_obstacle_height_);
    // node->get_parameter(name_ + "." + "combination_method", combination_method_);
    node->get_parameter("track_unknown_space", track_unknown_space);
    node->get_parameter("transform_tolerance", transform_tolerance);
    node->get_parameter(name_ + "." + "observation_sources", topics_string);
    node->get_parameter(name_ + "." + "max_gradient", maxGradient_);
    node->get_parameter(name_ + "." + "max_step", maxStep_);
    node->get_parameter(name_ + "." + "grid_map_config_file",
                        grid_map_config_file_);

    dyn_params_handler_ = node->add_on_set_parameters_callback(
        std::bind(
            &GradientCostLayer::dynamicParametersCallback,
            this,
            std::placeholders::_1));

    RCLCPP_INFO(
        logger_,
        "Subscribed to Topics: %s", topics_string.c_str());

    rolling_window_ = layered_costmap_->isRolling();

    if (track_unknown_space)
    {
      default_value_ = NO_INFORMATION;
    }
    else
    {
      default_value_ = FREE_SPACE;
    }

    matchSize();

    current_ = true;
    was_reset_ = false;

    global_frame_ = layered_costmap_->getGlobalFrameID();

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;

    // now we need to split the topics based on whitespace which we can use a stringstream for
    std::stringstream ss(topics_string);

    std::string source;
    while (ss >> source)
    {
      // get the parameters for the specific topic
      double observation_keep_time, expected_update_rate,
          min_obstacle_height, max_obstacle_height;
      std::string topic, sensor_frame, data_type;
      bool inf_is_valid;
      // clearing, marking;

      declareParameter(source + "." + "topic", rclcpp::ParameterValue(source));
      declareParameter(source + "." + "sensor_frame", rclcpp::ParameterValue(std::string("")));
      declareParameter(source + "." + "observation_persistence", rclcpp::ParameterValue(0.0));
      declareParameter(source + "." + "expected_update_rate", rclcpp::ParameterValue(0.0));
      declareParameter(source + "." + "data_type", rclcpp::ParameterValue(std::string("LaserScan")));
      declareParameter(source + "." + "min_obstacle_height", rclcpp::ParameterValue(0.0));
      declareParameter(source + "." + "max_obstacle_height", rclcpp::ParameterValue(0.0));
      declareParameter(source + "." + "inf_is_valid", rclcpp::ParameterValue(false));
      // declareParameter(source + "." + "marking", rclcpp::ParameterValue(true));
      // declareParameter(source + "." + "clearing", rclcpp::ParameterValue(false));
      declareParameter(source + "." + "obstacle_max_range", rclcpp::ParameterValue(2.5));
      declareParameter(source + "." + "obstacle_min_range", rclcpp::ParameterValue(0.0));
      // declareParameter(source + "." + "raytrace_max_range", rclcpp::ParameterValue(3.0));
      // declareParameter(source + "." + "raytrace_min_range", rclcpp::ParameterValue(0.0));

      node->get_parameter(name_ + "." + source + "." + "topic", topic);
      node->get_parameter(name_ + "." + source + "." + "sensor_frame", sensor_frame);
      node->get_parameter(
          name_ + "." + source + "." + "observation_persistence",
          observation_keep_time);
      node->get_parameter(
          name_ + "." + source + "." + "expected_update_rate",
          expected_update_rate);
      node->get_parameter(name_ + "." + source + "." + "data_type", data_type);
      node->get_parameter(name_ + "." + source + "." + "min_obstacle_height", min_obstacle_height);
      node->get_parameter(name_ + "." + source + "." + "max_obstacle_height", max_obstacle_height);
      node->get_parameter(name_ + "." + source + "." + "inf_is_valid", inf_is_valid);
      // node->get_parameter(name_ + "." + source + "." + "marking", marking);
      // node->get_parameter(name_ + "." + source + "." + "clearing", clearing);

      if (data_type != "PointCloud2")
      {
        RCLCPP_FATAL(
            logger_,
            "Only pointcloud2 topics are currently supported");
        throw std::runtime_error(
            "Only pointcloud2 topics are currently supported");
      }

      // get the obstacle range for the sensor
      double obstacle_max_range, obstacle_min_range;
      node->get_parameter(name_ + "." + source + "." + "obstacle_max_range", obstacle_max_range);
      node->get_parameter(name_ + "." + source + "." + "obstacle_min_range", obstacle_min_range);

      // get the raytrace ranges for the sensor
      // double raytrace_max_range, raytrace_min_range; // These are ignored
      // node->get_parameter(name_ + "." + source + "." + "raytrace_min_range", raytrace_min_range);
      // node->get_parameter(name_ + "." + source + "." + "raytrace_max_range", raytrace_max_range);

      RCLCPP_DEBUG(
          logger_,
          "Creating an observation buffer for source %s, topic %s, frame %s",
          source.c_str(), topic.c_str(),
          sensor_frame.c_str());

      // create an observation buffer
      observation_buffers_.push_back(
          std::shared_ptr<ObservationBuffer>(
              new ObservationBuffer(
                  node, topic, observation_keep_time, expected_update_rate,
                  min_obstacle_height, max_obstacle_height,
                  obstacle_max_range, obstacle_min_range,
                  // raytrace_max_range, raytrace_min_range,
                  0.0, 0.0, // Unused raytrace parameters
                  *tf_,
                  global_frame_,
                  sensor_frame,
                  tf2::durationFromSec(transform_tolerance))));

      marking_buffers_.push_back(observation_buffers_.back());

      // check if we'll also add this buffer to our clearing observation buffers
      // if (clearing)
      // {
      //   clearing_buffers_.push_back(observation_buffers_.back());
      // }

      RCLCPP_DEBUG(
          logger_,
          "Created an observation buffer for source %s, topic %s, "
          "global frame: %s, "
          "expected update rate: %.2f, observation persistence: %.2f",
          source.c_str(), topic.c_str(),
          global_frame_.c_str(), expected_update_rate, observation_keep_time);

      rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
      custom_qos_profile.depth = 50;

      // create a callback for the topic
      auto sub = std::make_shared<message_filters::Subscriber<
          sensor_msgs::msg::PointCloud2, rclcpp_lifecycle::LifecycleNode>>(node,
                                            topic, custom_qos_profile, sub_opt);
      sub->unsubscribe();

      if (inf_is_valid)
      {
        RCLCPP_WARN(
            logger_,
            "obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      auto filter = std::make_shared<tf2_ros::MessageFilter<
                                      sensor_msgs::msg::PointCloud2>>(
          *sub, *tf_, global_frame_, 50,
          node->get_node_logging_interface(),
          node->get_node_clock_interface(),
          tf2::durationFromSec(transform_tolerance));

      filter->registerCallback(
          std::bind(
              &GradientCostLayer::pointCloud2Callback, this,
              std::placeholders::_1,
              observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);

      if (sensor_frame != "")
      {
        std::vector<std::string> target_frames;
        target_frames.push_back(global_frame_);
        target_frames.push_back(sensor_frame);
        observation_notifiers_.back()->setTargetFrames(target_frames);
      }
    }

    std::string nodeName(node->get_name());
    gridMapPub_ = node->create_publisher<grid_map_msgs::msg::GridMap>(
        nodeName + "/grid_map_from_raw_pointcloud",
        10);

  }

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
        if (param_name == name_ + "." + "min_obstacle_height")
        {
          min_obstacle_height_ = parameter.as_double();
        }
        else if (param_name == name_ + "." + "max_obstacle_height")
        {
          max_obstacle_height_ = parameter.as_double();
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
        else if (param_name == name_ + "." + "footprint_clearing_enabled")
        {
          footprint_clearing_enabled_ = parameter.as_bool();
        }
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

  void GradientCostLayer::pointCloud2Callback(
                    sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
                    const std::shared_ptr<ObservationBuffer> &buffer)
  {
    // buffer the point cloud
    buffer->lock();
    buffer->bufferCloud(*message);
    buffer->unlock();
  }

  void GradientCostLayer::updateBounds(
                    double robot_x, double robot_y, double robot_yaw,
                    double *min_x, double *min_y, double *max_x, double *max_y)
  {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());  

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

    bool current = true;
    std::vector<Observation> observations;

    // get the marking observations
    current = current && getMarkingObservations(observations);

    // update the global current status
    current_ = current;

    pcl::PCLPointCloud2 pclCloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudPtr(
                                new pcl::PointCloud<pcl::PointXYZ>);

    if (rcutils_logging_set_logger_level("gradient_cost_layer",
                                         RCUTILS_LOG_SEVERITY_ERROR)
            != RCUTILS_RET_OK)
    {
      RCLCPP_DEBUG(logger_,
        "Could not change logging level of logger 'gradient_cost_layer'");
    }
    grid_map::GridMapPclLoader gridMapPclLoader(
                    rclcpp::get_logger("gradient_cost_layer"));
    // std::cout << "Config file: " << grid_map_config_file_ << std::endl;
    gridMapPclLoader.loadParameters(grid_map_config_file_);

    // place the new obstacles into a priority queue... each with a priority of
    // zero to begin with
    for (std::vector<Observation>::const_iterator it = observations.begin();
         it != observations.end(); ++it)
    {
      const Observation &obs = *it;

      const sensor_msgs::msg::PointCloud2 &cloud = *(obs.cloud_);

      pcl_conversions::toPCL(cloud, pclCloud2);
      pcl::fromPCLPointCloud2(pclCloud2,*pclCloudPtr);

      gridMapPclLoader.setInputCloud(pclCloudPtr);
      gridMapPclLoader.initializeGridMapGeometryFromInputCloud();
      gridMapPclLoader.addLayerFromInputCloud("elevation");
      grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
      gridMap.setFrameId(global_frame_);

      // We add a layer for the calculated cost.  This is of type float but
      // we will stick to values [0,255] for easy conversion to
      // ROS costmap_2d.
      gridMap.add("cost");

      //! \todo neighbourhoodSize needs to be an option.
      int neighbourhoodSize = 3;
      int nhs2 = neighbourhoodSize / 2;

      // double sqObstacleMaxRange = obs.obstacle_max_range_
      //                                * obs.obstacle_max_range_;
      // double sqObstacleMinRange = obs.obstacle_min_range_
      //                                * obs.obstacle_min_range_;

      grid_map::Position startSubmap(nhs2, nhs2);
      grid_map::Size submapSize = gridMap.getSize();
      submapSize = submapSize - grid_map::Size(nhs2, nhs2);

      auto node = node_.lock();
      // std::cout << node->get_name()
      //           << " (" << startSubmap(0) << "," << startSubmap(1) << ")"
      //           << "  (" << submapSize(0) << "," << submapSize(1) << ")"
      //           << std::endl;

      // for (grid_map::SubmapIterator mapIter(gridMap, startIdx, submapSize);
      // for (grid_map::GridMapIterator mapIter(gridMap);
      //      !mapIter.isPastEnd();
      //      ++mapIter)
      grid_map::Index mapIdx, subMapIdx;
      unsigned int mx, my;
      grid_map::Position mapPos;
      unsigned int costmapIdx;
      int indX, indY;
      for (indX = startSubmap(0); indX < submapSize(0); indX++)
      {
        for (indY = startSubmap(1); indY < submapSize(1); indY++)
        {
          mapIdx(0) = indX;
          mapIdx(1) = indY;

          // Remove locations that do not have data
          float centreElev = gridMap.at("elevation", mapIdx);
          if (std::isnan(centreElev))
          {
            // No data here, we might as well ignore the point.
            continue;
          }

          // Remove locations that are outside of the local map.
          gridMap.getPosition(mapIdx, mapPos);
          if (!worldToMap(mapPos(0), mapPos(1), mx, my))
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
          gridMap.at("cost", mapIdx) = cost;

          // std::cout << node->get_name()
          //           << " costmap pos (" << mx << "," << my << ") size ("
          //           << getSizeInCellsX() << "," << getSizeInCellsY()
          //           << ") index " << costmapIdx << "\n";

          // First a loop over the 3x3 neighbourhood to check for steps.
          for (int subIndX = -1; subIndX <= 1; subIndX++)
          {
            for (int subIndY = -1; subIndY <= 1; subIndY++)
            {
              subMapIdx(0) = indX + subIndX;
              subMapIdx(1) = indY + subIndY;
              float elev = gridMap.at("elevation", subMapIdx);
              if (!std::isnan(elev) && (fabs(elev - centreElev) > maxStep_))
              {
                gridMap.at("cost", mapIdx) = LETHAL_OBSTACLE;
                costmap_[costmapIdx] = LETHAL_OBSTACLE;
              }
            }
          }

          if (costmap_[costmapIdx] != default_value_)
            touch(mapPos(0), mapPos(1), min_x, min_y, max_x, max_y);

        // for (grid_map::SubmapIterator neighIter(gridMap, 
        //                                         startIdx, submapSize);
        //      !neighIter.isPastEnd();
        //      ++neighIter)
        // {
        //   float elev = gridMap.at("elevation", *neighIter);
        //   if (fabs(elev - centreElev) > maxStep)
        //   {
        //     gridMap.at("cost", *mapIter) = LETHAL_OBSTACLE;
        //   }
        // }

        }
      }

      // std::cout << node->get_name() << " gridmap last pos " << indX << " " << indY << std::endl;

      auto msg = grid_map::GridMapRosConverter::toMessage(gridMap);
      gridMapPub_->publish(std::move(msg));
      



      // - iterate through the calculated normals and add a cost to the layer
      //   if the normal is too horizontal.

      // double sqObstacleMaxRange = obs.obstacle_max_range_
      //                                * obs.obstacle_max_range_;
      // double sqObstacleMinRange = obs.obstacle_min_range_
      //                                * obs.obstacle_min_range_;

      // pcl::PointCloud<pcl::Normal>::iterator normalIter;
      // pcl::PointCloud<pcl::PointXYZ>::iterator cloudIter
      //     = pclCloudPtr->points.begin();
      // for (normalIter = cloudNormals->points.begin();
      //      normalIter < cloudNormals->points.end();
      //      normalIter++, cloudIter++)
      // {
      //   double px = cloudIter->x;
      //   double py = cloudIter->y;
      //   double pz = cloudIter->z;
      //   double nx = normalIter->normal_x;
      //   double ny = normalIter->normal_y;
      //   double nz = normalIter->normal_z;

      //   // Ignore points too far or too close.
      //   double sqDist =
      //       (px - obs.origin_.x) * (px - obs.origin_.x) +
      //       (py - obs.origin_.y) * (py - obs.origin_.y) +
      //       (pz - obs.origin_.z) * (pz - obs.origin_.z);

      //   if (sqDist >= sqObstacleMaxRange)
      //   {
      //     RCLCPP_DEBUG(logger_, "The point is too far away");
      //     continue;
      //   }
      //   if (sqDist < sqObstacleMinRange)
      //   {
      //     RCLCPP_DEBUG(logger_, "The point is too close");
      //     continue;
      //   }

      //   // Angle between the normal (nx,ny,nz) and the vertical (0,0,1).
      //   double angle = acos((0 + 0 + nz*1)
      //         / (sqrt(nx*nx + ny*ny + nz*nz)));
      //   // Angles in [pi/2, pi] need to be mirrored to fall in [0, pi/2] as
      //   // they correspond to an upside down face which seems to happen on
      //   // (near) horizontal faces
      //   if (angle > M_PI_2)
      //   {
      //     angle = M_PI - angle; 
      //   }

      //   // The cost is a char.  Maximum cost is for a maximum allowed
      //   // angle.  Minimum value is for a horizontal
      //   // gradient (angle 0).
      //   unsigned char cost = fmin(angle, maxGradient_) / maxGradient_ * 255;

      //   // if (cost != 0)
      //     // std::cout << nx << " " << ny << " " << nz << " " << angle << " "
      //     //   << int(cost)
      //     //   << std::endl;

      //   unsigned int mx, my;
      //   if (!worldToMap(px, py, mx, my))
      //   {
      //     RCLCPP_DEBUG(logger_, "Computing map coords failed");
      //     continue;
      //   }
      //   unsigned int index = getIndex(mx, my);
      //   // costmap_[index] = LETHAL_OBSTACLE;
      //   costmap_[index] = cost;
      //   touch(px, py, min_x, min_y, max_x, max_y);
      // }







      // sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
      // sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
      // sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

      // for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
      // {
      //   double px = *iter_x, py = *iter_y, pz = *iter_z;

      //   // if the obstacle is too low, we won't add it
      //   if (pz < min_obstacle_height_)
      //   {
      //     RCLCPP_DEBUG(logger_, "The point is too low");
      //     continue;
      //   }

      //   // if the obstacle is too high we won't add it
      //   if (pz > max_obstacle_height_)
      //   {
      //     RCLCPP_DEBUG(logger_, "The point is too high");
      //     continue;
      //   }


      //   // now we need to compute the map coordinates for the observation
      //   unsigned int mx, my;
      //   if (!worldToMap(px, py, mx, my))
      //   {
      //     RCLCPP_DEBUG(logger_, "Computing map coords failed");
      //     continue;
      //   }

      //   unsigned int index = getIndex(mx, my);
      //   costmap_[index] = LETHAL_OBSTACLE;
      //   touch(px, py, min_x, min_y, max_x, max_y);
      // }
    }
    
    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  }

  void
  GradientCostLayer::updateFootprint(
      double robot_x, double robot_y, double robot_yaw,
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

  void
  GradientCostLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
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

  // void
  // GradientCostLayer::addStaticObservation(nav2_costmap_2d::Observation &obs,
  //                                         bool marking, bool clearing)
  // {
  //   if (marking)
  //   {
  //     static_marking_observations_.push_back(obs);
  //   }
  //   if (clearing)
  //   {
  //     static_clearing_observations_.push_back(obs);
  //   }
  // }

  // void
  // GradientCostLayer::clearStaticObservations(bool marking, bool clearing)
  // {
  //   if (marking)
  //   {
  //     static_marking_observations_.clear();
  //   }
  //   if (clearing)
  //   {
  //     static_clearing_observations_.clear();
  //   }
  // }

  bool
  GradientCostLayer::getMarkingObservations(std::vector<Observation> &marking_observations) const
  {
    bool current = true;
    // get the marking observations
    for (unsigned int i = 0; i < marking_buffers_.size(); ++i)
    {
      marking_buffers_[i]->lock();
      marking_buffers_[i]->getObservations(marking_observations);
      current = marking_buffers_[i]->isCurrent() && current;
      marking_buffers_[i]->unlock();
    }
    marking_observations.insert(
        marking_observations.end(),
        static_marking_observations_.begin(), static_marking_observations_.end());
    return current;
  }

  bool
  GradientCostLayer::getClearingObservations(std::vector<Observation> &clearing_observations) const
  {
    bool current = true;
    // get the clearing observations
    for (unsigned int i = 0; i < clearing_buffers_.size(); ++i)
    {
      clearing_buffers_[i]->lock();
      clearing_buffers_[i]->getObservations(clearing_observations);
      current = clearing_buffers_[i]->isCurrent() && current;
      clearing_buffers_[i]->unlock();
    }
    clearing_observations.insert(
        clearing_observations.end(),
        static_clearing_observations_.begin(), static_clearing_observations_.end());
    return current;
  }

  // void
  // GradientCostLayer::raytraceFreespace(
  //     const Observation& /*clearing_observation*/,
  //     double* /*min_x*/, double* /*min_y*/,
  //     double* /*max_x*/, double* /*max_y*/)
  // {
    // We do not rayrace anything in this plugin.

  //   double ox = clearing_observation.origin_.x;
  //   double oy = clearing_observation.origin_.y;
  //   const sensor_msgs::msg::PointCloud2 &cloud = *(clearing_observation.cloud_);

  //   // get the map coordinates of the origin of the sensor
  //   unsigned int x0, y0;
  //   if (!worldToMap(ox, oy, x0, y0))
  //   {
  //     RCLCPP_WARN(
  //         logger_,
  //         "Sensor origin at (%.2f, %.2f) is out of map bounds (%.2f, %.2f) to (%.2f, %.2f). "
  //         "The costmap cannot raytrace for it.",
  //         ox, oy,
  //         origin_x_, origin_y_,
  //         origin_x_ + getSizeInMetersX(), origin_y_ + getSizeInMetersY());
  //     return;
  //   }

  //   // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  //   double origin_x = origin_x_, origin_y = origin_y_;
  //   double map_end_x = origin_x + size_x_ * resolution_;
  //   double map_end_y = origin_y + size_y_ * resolution_;

  //   touch(ox, oy, min_x, min_y, max_x, max_y);

  //   // for each point in the cloud, we want to trace a line from the origin
  //   // and clear obstacles along it
  //   sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  //   sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

  //   for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
  //   {
  //     double wx = *iter_x;
  //     double wy = *iter_y;

  //     // now we also need to make sure that the enpoint we're raytracing
  //     // to isn't off the costmap and scale if necessary
  //     double a = wx - ox;
  //     double b = wy - oy;

  //     // the minimum value to raytrace from is the origin
  //     if (wx < origin_x)
  //     {
  //       double t = (origin_x - ox) / a;
  //       wx = origin_x;
  //       wy = oy + b * t;
  //     }
  //     if (wy < origin_y)
  //     {
  //       double t = (origin_y - oy) / b;
  //       wx = ox + a * t;
  //       wy = origin_y;
  //     }

  //     // the maximum value to raytrace to is the end of the map
  //     if (wx > map_end_x)
  //     {
  //       double t = (map_end_x - ox) / a;
  //       wx = map_end_x - .001;
  //       wy = oy + b * t;
  //     }
  //     if (wy > map_end_y)
  //     {
  //       double t = (map_end_y - oy) / b;
  //       wx = ox + a * t;
  //       wy = map_end_y - .001;
  //     }

  //     // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
  //     unsigned int x1, y1;

  //     // check for legality just in case
  //     if (!worldToMap(wx, wy, x1, y1))
  //     {
  //       continue;
  //     }

  //     unsigned int cell_raytrace_max_range = cellDistance(clearing_observation.raytrace_max_range_);
  //     unsigned int cell_raytrace_min_range = cellDistance(clearing_observation.raytrace_min_range_);
  //     MarkCell marker(costmap_, FREE_SPACE);
  //     // and finally... we can execute our trace to clear obstacles along that line
  //     raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_max_range, cell_raytrace_min_range);

  //     updateRaytraceBounds(
  //         ox, oy, wx, wy, clearing_observation.raytrace_max_range_,
  //         clearing_observation.raytrace_min_range_, min_x, min_y, max_x,
  //         max_y);
  //   }
  // }

  void
  GradientCostLayer::activate()
  {
    for (auto &notifier : observation_notifiers_)
    {
      notifier->clear();
    }

    // if we're stopped we need to re-subscribe to topics
    for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
    {
      if (observation_subscribers_[i] != NULL)
      {
        observation_subscribers_[i]->subscribe();
      }
    }
    resetBuffersLastUpdated();
  }

  void
  GradientCostLayer::deactivate()
  {
    for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
    {
      if (observation_subscribers_[i] != NULL)
      {
        observation_subscribers_[i]->unsubscribe();
      }
    }
  }

  // void
  // GradientCostLayer::updateRaytraceBounds(
  //     double /*ox*/, double /*oy*/, double /*wx*/, double /*wy*/,
  //     double /*max_range*/, double /*min_range*/,
  //     double* /*min_x*/, double* /*min_y*/, double* /*max_x*/, double* /*max_y*/)
  // {
  //   // double dx = wx - ox, dy = wy - oy;
    // double full_distance = hypot(dx, dy);
    // if (full_distance < min_range)
    // {
    //   return;
    // }
    // double scale = std::min(1.0, max_range / full_distance);
    // double ex = ox + dx * scale, ey = oy + dy * scale;
    // touch(ex, ey, min_x, min_y, max_x, max_y);
  // }

  void
  GradientCostLayer::reset()
  {
    resetMaps();
    resetBuffersLastUpdated();
    current_ = false;
    was_reset_ = true;
  }

  void
  GradientCostLayer::resetBuffersLastUpdated()
  {
    for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
    {
      if (observation_buffers_[i])
      {
        observation_buffers_[i]->resetLastUpdated();
      }
    }
  }

} // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(gradient_cost_plugin::GradientCostLayer,
                       nav2_costmap_2d::Layer)
