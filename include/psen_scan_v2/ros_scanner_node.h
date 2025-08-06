// Copyright (c) 2020-2021 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_ROS_SCANNER_NODE_H
#define PSEN_SCAN_V2_ROS_SCANNER_NODE_H

#include <stdexcept>
#include <string>
#include <atomic>
#include <chrono>
#include <future>
#include <algorithm>

#include <fmt/format.h>

#include <gtest/gtest_prod.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "psen_scan_v2_standalone/scanner_v2.h"

#include "psen_scan_v2/laserscan_ros_conversions.h"
#include "psen_scan_v2/io_state_ros_conversion.h"
#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/util/logging.h"

/**
 * @brief Root namespace for the ROS part
 */
namespace psen_scan_v2
{
using namespace psen_scan_v2_standalone;
using namespace std::chrono_literals;

/**
 * @brief ROS Node that continuously publishes scan data of a single PSENscan laser scanner.
 *
 */
template <typename S = ScannerV2>
class ROSScannerNodeT
{
public:
  /**
   * @brief Constructor.
   *
   * @param node rclcpp::Node instance.
   * @param topic Name of the ROS topic under which the scanner data are published.
   * @param tf_prefix Prefix for the frame ids.
   * @param x_axis_rotation Rotation of 2D scan around the z-axis.
   * @param scanner_config Scanner configuration.
   */
  ROSScannerNodeT(const rclcpp::Node::SharedPtr& node,
                  const std::string& topic,
                  const std::string& tf_prefix,
                  const double& x_axis_rotation,
                  const ScannerConfiguration& scanner_config);

  //! @brief Continuously fetches data from the scanner and publishes the data as ROS scanner message.
  void run();
  //! @brief Terminates the fetching and publishing of scanner data.
  void terminate();

private:
  void laserScanCallback(const LaserScan& scan);
  void publishChangedIOStates(const std::vector<psen_scan_v2_standalone::IOState>& io_states);

private:
  rclcpp::Node::SharedPtr node_;
  using ScannerId = psen_scan_v2_standalone::configuration::ScannerId;
  std::unordered_map<ScannerId, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> pubs_scan_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_zone_;
  rclcpp::Publisher<psen_scan_v2::msg::IOState>::SharedPtr pub_io_;
  std::unordered_map<ScannerId, std::string> tf_prefixes_;
  double x_axis_rotation_;
  S scanner_;
  std::atomic_bool terminate_{ false };

  psen_scan_v2_standalone::IOState last_io_state_{};

 friend class RosScannerNodeTests;
  FRIEND_TEST(RosScannerNodeTests, shouldStartAndStopSuccessfullyIfScannerRespondsToRequests);
  FRIEND_TEST(RosScannerNodeTests, shouldPublishScansWhenLaserScanCallbackIsInvoked);
  FRIEND_TEST(RosScannerNodeTests, shouldPublishActiveZonesetWhenLaserScanCallbackIsInvoked);
  FRIEND_TEST(RosScannerNodeTests, shouldWaitWhenStopRequestResponseIsMissing);
  FRIEND_TEST(RosScannerNodeTests, shouldProvideScanTopic);
  FRIEND_TEST(RosScannerNodeTests, shouldProvideActiveZonesetTopic);
  FRIEND_TEST(RosScannerNodeTests, shouldPublishScanEqualToConversionOfSuppliedLaserScan);
  FRIEND_TEST(RosScannerNodeTests, shouldThrowExceptionSetInScannerStartFuture);
  FRIEND_TEST(RosScannerNodeTests, shouldThrowExceptionSetInScannerStopFuture);
  FRIEND_TEST(RosScannerNodeTests, shouldPublishChangedIOStatesEqualToConversionOfSuppliedStandaloneIOStates);
  FRIEND_TEST(RosScannerNodeTests, shouldPublishLatchedOnIOStatesTopic);
  FRIEND_TEST(RosScannerNodeTests, shouldLogChangedIOStates);
};

typedef ROSScannerNodeT<> ROSScannerNode;

template <typename S>
ROSScannerNodeT<S>::ROSScannerNodeT(const rclcpp::Node::SharedPtr& node,
                                    const std::string& topic,
                                    const std::string& tf_prefix,
                                    const double& x_axis_rotation,
                                    const ScannerConfiguration& scanner_config)
  : node_(node)
  , x_axis_rotation_(x_axis_rotation)
  , scanner_(scanner_config, std::bind(&ROSScannerNodeT<S>::laserScanCallback, this, std::placeholders::_1))
{
  pubs_scan_.insert(std::make_pair(ScannerId::master, node_->create_publisher<sensor_msgs::msg::LaserScan>(tf_prefix + "/" + topic, 1)));
  tf_prefixes_.insert(std::make_pair(ScannerId::master, tf_prefix));
  for (int i = 0; i < scanner_config.nrSubscribers(); i++)
  {
    ScannerId id = psen_scan_v2_standalone::configuration::subscriber_number_to_scanner_id(i);
    std::string topic_subscriber = topic + "_" + psen_scan_v2_standalone::configuration::SCANNER_ID_TO_STRING.at(id);
    pubs_scan_.insert(std::make_pair(id, node_->create_publisher<sensor_msgs::msg::LaserScan>(tf_prefix + "/" + topic_subscriber, 1)));
    std::string tf_prefix_subscriber =
        tf_prefix + "_" + psen_scan_v2_standalone::configuration::SCANNER_ID_TO_STRING.at(id);
    tf_prefixes_.insert(std::make_pair(id, tf_prefix_subscriber));
  }
  pub_zone_ = node->create_publisher<std_msgs::msg::UInt8>("/active_zoneset", 1);
  pub_io_ = node->create_publisher<psen_scan_v2::msg::IOState>("/io_state", 6);
}

template <typename S>
void ROSScannerNodeT<S>::laserScanCallback(const LaserScan& scan)
{
  try
  {
    std::string tf_prefix_with_subscriber_ = tf_prefixes_.at(scan.scannerId());
    const auto laser_scan_msg = toLaserScanMsg(scan, tf_prefix_with_subscriber_, x_axis_rotation_);
    PSENSCAN_INFO_ONCE(
        "ScannerNode",
        "Publishing laser scan with angle_min={:.1f} angle_max={:.1f} angle_increment={:.1f} degrees. {} angle values.",
        data_conversion_layer::radianToDegree(laser_scan_msg.angle_min),
        data_conversion_layer::radianToDegree(laser_scan_msg.angle_max),
        data_conversion_layer::radianToDegree(laser_scan_msg.angle_increment),
        laser_scan_msg.ranges.size());
    pubs_scan_.at(scan.scannerId())->publish(laser_scan_msg);

    std_msgs::msg::UInt8 active_zoneset;
    active_zoneset.data = scan.activeZoneset();
    pub_zone_->publish(active_zoneset);

    publishChangedIOStates(scan.ioStates());
  }
  // LCOV_EXCL_START
  catch (const std::invalid_argument& e)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e.what());
  }
  // LCOV_EXCL_STOP
}

template <typename S>
void ROSScannerNodeT<S>::publishChangedIOStates(const std::vector<psen_scan_v2_standalone::IOState>& io_states)
{
  std::string tf_prefix_with_subscriber_ = tf_prefixes_.at(ScannerId::master);
  for (const auto& io : io_states)
  {
    if (last_io_state_ != io)
    {
      pub_io_->publish(toIOStateMsg(io, tf_prefix_with_subscriber_));

      PSENSCAN_INFO("RosScannerNode",
                    "IOs changed, new input: {}, new output: {}",
                    formatPinStates(io.changedInputStates(last_io_state_)),
                    formatPinStates(io.changedOutputStates(last_io_state_)));
      last_io_state_ = io;
    }
  }
}

template <typename S>
void ROSScannerNodeT<S>::terminate()
{
  terminate_ = true;
}

template <typename S>
void ROSScannerNodeT<S>::run()
{
  rclcpp::Rate r(10);
  scanner_.start();
  while (rclcpp::ok() && !terminate_)
  {
    rclcpp::spin_some(node_);
    r.sleep();
  }
  const auto stop_future = scanner_.stop();
  const auto stop_status = stop_future.wait_for(3s);
  if (stop_status == std::future_status::timeout)
  {
    RCLCPP_ERROR(node_->get_logger(), "Scanner did not finish properly");
  }
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_ROS_SCANNER_NODE_H
