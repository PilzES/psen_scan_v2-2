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

#include <rclcpp/rclcpp.hpp>

#include "psen_scan_v2/config_server_node.h"
#include "psen_scan_v2/ros_parameter_handler.h"

const std::string CONFIG_FILE{ "config_file" };
const std::string FRAME_ID{ "frame_id" };

using namespace psen_scan_v2;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("config_server_node", node_options);

  try
  {
    psen_scan_v2::ConfigServerNode config_server_node(node,
                                                      getRequiredParamFromServer<std::string>(*node, CONFIG_FILE).c_str(),
                                                      getRequiredParamFromServer<std::string>(*node, FRAME_ID));

    rclcpp::spin(node);
  }
  // LCOV_EXCL_START
  catch (std::exception& e)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), e.what());
  }
  // LCOV_EXCL_STOP

  return 0;
}