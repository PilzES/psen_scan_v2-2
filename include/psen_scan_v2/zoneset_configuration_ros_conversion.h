// Copyright (c) 2021 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef PSEN_SCAN_V2_ZONESET_CONFIGURATION_ROS_CONVERSION_H
#define PSEN_SCAN_V2_ZONESET_CONFIGURATION_ROS_CONVERSION_H

#include <geometry_msgs/msg/polygon.hpp>

#include "psen_scan_v2/default_ros_parameters.h"
#include "psen_scan_v2/msg/zone_set.hpp"
#include "psen_scan_v2/msg/zone_set_configuration.hpp"
#include "psen_scan_v2/zoneset_msg_builder.h"
#include "psen_scan_v2_standalone/configuration/zoneset_configuration.h"
#include "psen_scan_v2_standalone/configuration/default_parameters.h"
#include "psen_scan_v2_standalone/util/tenth_of_degree.h"

using ZoneSetStandalone = psen_scan_v2_standalone::configuration::ZoneSet;
using ZoneSetConfigurationStandalone = psen_scan_v2_standalone::configuration::ZoneSetConfiguration;
using TenthOfDegree = psen_scan_v2_standalone::util::TenthOfDegree;

static inline double deg_to_rad(double deg)
{
  return deg * M_PI / 180.0;
}

geometry_msgs::msg::Polygon fromPolar(const std::vector<unsigned long>& radii_in_mm,
                                 const TenthOfDegree& phi_step,
                                 const double& x_axis_rotation)
{
  geometry_msgs::msg::Polygon polygon;
  size_t i = 0;
  for (const auto& r : radii_in_mm)
  {
    geometry_msgs::msg::Point32 point;

    const auto angle = phi_step.toRad() * i - x_axis_rotation;
    point.x = (r / 1000.) * std::cos(angle);
    point.y = (r / 1000.) * std::sin(angle);
    point.z = 0;

    polygon.points.push_back(point);

    ++i;
  }

  return polygon;
}

psen_scan_v2::msg::ZoneSet toRosMsg(const ZoneSetStandalone& zoneset,
                               const std::string& frame_id,
                               rclcpp::Time stamp = rclcpp::Clock().now())
{
  psen_scan_v2::ZoneSetMsgBuilder zoneset_msg_builder;
  zoneset_msg_builder.headerStamp(stamp)
      .headerFrameId(frame_id)
      .safety1(fromPolar(zoneset.safety1_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .safety2(fromPolar(zoneset.safety2_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .safety3(fromPolar(zoneset.safety3_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .warn1(fromPolar(zoneset.warn1_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .warn2(fromPolar(zoneset.warn2_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .muting1(fromPolar(zoneset.muting1_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .muting2(fromPolar(zoneset.muting2_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .safety1_sub0(fromPolar(zoneset.safety1_sub0_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .safety2_sub0(fromPolar(zoneset.safety2_sub0_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .safety3_sub0(fromPolar(zoneset.safety3_sub0_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .warn1_sub0(fromPolar(zoneset.warn1_sub0_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .warn2_sub0(fromPolar(zoneset.warn2_sub0_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .muting1_sub0(fromPolar(zoneset.muting1_sub0_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .muting2_sub0(fromPolar(zoneset.muting2_sub0_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .safety1_sub1(fromPolar(zoneset.safety1_sub1_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .safety2_sub1(fromPolar(zoneset.safety2_sub1_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .safety3_sub1(fromPolar(zoneset.safety3_sub1_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .warn1_sub1(fromPolar(zoneset.warn1_sub1_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .warn2_sub1(fromPolar(zoneset.warn2_sub1_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .muting1_sub1(fromPolar(zoneset.muting1_sub1_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .muting2_sub1(fromPolar(zoneset.muting2_sub1_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .safety1_sub2(fromPolar(zoneset.safety1_sub2_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .safety2_sub2(fromPolar(zoneset.safety2_sub2_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .safety3_sub2(fromPolar(zoneset.safety3_sub2_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .warn1_sub2(fromPolar(zoneset.warn1_sub2_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .warn2_sub2(fromPolar(zoneset.warn2_sub2_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .muting1_sub2(fromPolar(zoneset.muting1_sub2_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION))
      .muting2_sub2(fromPolar(zoneset.muting2_sub2_, zoneset.resolution_, psen_scan_v2::DEFAULT_X_AXIS_ROTATION));

  if (zoneset.speed_range_)
  {
    zoneset_msg_builder.speedLower(zoneset.speed_range_->min_).speedUpper(zoneset.speed_range_->max_);
  }

  return zoneset_msg_builder.build();
}

psen_scan_v2::msg::ZoneSetConfiguration toRosMsg(const ZoneSetConfigurationStandalone& zoneset_configuration,
                                            const std::string& frame_id,
                                            rclcpp::Time stamp = rclcpp::Clock().now())
{
  psen_scan_v2::msg::ZoneSetConfiguration zoneset_config_msg;

  for (const auto& z : zoneset_configuration.zonesets_)
  {
    zoneset_config_msg.zonesets.push_back(toRosMsg(z, frame_id, stamp));
  }
  return zoneset_config_msg;
}

#endif  // PSEN_SCAN_V2_ZONESET_CONFIGURATION_ROS_CONVERSION_H