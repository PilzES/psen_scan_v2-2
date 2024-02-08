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

#ifndef PSEN_SCAN_V2_ACTIVE_ZONESET_NODE_H
#define PSEN_SCAN_V2_ACTIVE_ZONESET_NODE_H

#include <string>
#include <vector>
#include <boost/optional.hpp>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "psen_scan_v2/msg/zone_set_configuration.hpp"

namespace psen_scan_v2
{
static const std::string DEFAULT_ACTIVE_ZONESET_TOPIC = "active_zoneset";
static const std::string DEFAULT_ZONECONFIGURATION_TOPIC = "zoneconfiguration";
static const std::string DEFAULT_ZONESET_MARKER_ARRAY_TOPIC = "active_zoneset_markers";

/**
 * @brief ROS Node that continuously publishes a marker for the active_zoneset.
 *
 * subscribes to: ns/active_zoneset
 * subscribes to: ns/zoneconfiguration
 *
 * advertises: ns/active_zoneset_markers
 */
class ActiveZonesetNode
{
public:
  /**
   * @brief Constructor.
   *
   * @param node Node handle for the ROS node on which the scanner topic is advertised.
   */
  ActiveZonesetNode(const rclcpp::Node::SharedPtr& node);

public:
  void zonesetCallback(const psen_scan_v2::msg::ZoneSetConfiguration::SharedPtr zoneset_config);
  void activeZonesetCallback(const std_msgs::msg::UInt8::SharedPtr active_zoneset_id);

private:
  void updateMarkers();
  bool isAllInformationAvailable() const;

  psen_scan_v2::msg::ZoneSet activeZoneset() const;

  void addMarkers(std::vector<visualization_msgs::msg::Marker>& new_markers);
  void publishCurrentMarkers();
  void addDeleteMessageForUnusedLastMarkers();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<psen_scan_v2::msg::ZoneSetConfiguration>::SharedPtr zoneset_subscriber_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr active_zoneset_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zoneset_markers_;

  boost::optional<psen_scan_v2::msg::ZoneSetConfiguration> zoneset_config_;
  boost::optional<std_msgs::msg::UInt8> active_zoneset_id_;
  std::vector<visualization_msgs::msg::Marker> last_markers_;
  std::vector<visualization_msgs::msg::Marker> current_markers_;
};

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_ACTIVE_ZONESET_NODE_H
