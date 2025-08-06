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

#ifndef PSEN_SCAN_V2_ZONESET_MSG_BUILDER_H
#define PSEN_SCAN_V2_ZONESET_MSG_BUILDER_H

#include <cstdint>
#include <string>

#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/header.hpp>

#include "psen_scan_v2/msg/zone_set.hpp"

namespace psen_scan_v2
{
class ZoneSetMsgBuilder
{
public:
  psen_scan_v2::msg::ZoneSet build() const;

public:
  ZoneSetMsgBuilder& headerSeq(uint32_t header_seq);
  ZoneSetMsgBuilder& headerStamp(const rclcpp::Time& header_stamp);
  ZoneSetMsgBuilder& headerFrameId(const std::string& header_frame_id);
  ZoneSetMsgBuilder& safety1(const geometry_msgs::msg::Polygon& safety1);
  ZoneSetMsgBuilder& safety2(const geometry_msgs::msg::Polygon& safety2);
  ZoneSetMsgBuilder& safety3(const geometry_msgs::msg::Polygon& safety3);
  ZoneSetMsgBuilder& warn1(const geometry_msgs::msg::Polygon& warn1);
  ZoneSetMsgBuilder& warn2(const geometry_msgs::msg::Polygon& warn2);
  ZoneSetMsgBuilder& muting1(const geometry_msgs::msg::Polygon& muting1);
  ZoneSetMsgBuilder& muting2(const geometry_msgs::msg::Polygon& muting2);
  ZoneSetMsgBuilder& safety1_sub0(const geometry_msgs::msg::Polygon& safety1_sub0);
  ZoneSetMsgBuilder& safety2_sub0(const geometry_msgs::msg::Polygon& safety2_sub0);
  ZoneSetMsgBuilder& safety3_sub0(const geometry_msgs::msg::Polygon& safety3_sub0);
  ZoneSetMsgBuilder& warn1_sub0(const geometry_msgs::msg::Polygon& warn1_sub0);
  ZoneSetMsgBuilder& warn2_sub0(const geometry_msgs::msg::Polygon& warn2_sub0);
  ZoneSetMsgBuilder& muting1_sub0(const geometry_msgs::msg::Polygon& muting1_sub0);
  ZoneSetMsgBuilder& muting2_sub0(const geometry_msgs::msg::Polygon& muting2_sub0);
  ZoneSetMsgBuilder& safety1_sub1(const geometry_msgs::msg::Polygon& safety1_sub1);
  ZoneSetMsgBuilder& safety2_sub1(const geometry_msgs::msg::Polygon& safety2_sub1);
  ZoneSetMsgBuilder& safety3_sub1(const geometry_msgs::msg::Polygon& safety3_sub1);
  ZoneSetMsgBuilder& warn1_sub1(const geometry_msgs::msg::Polygon& warn1_sub1);
  ZoneSetMsgBuilder& warn2_sub1(const geometry_msgs::msg::Polygon& warn2_sub1);
  ZoneSetMsgBuilder& muting1_sub1(const geometry_msgs::msg::Polygon& muting1_sub1);
  ZoneSetMsgBuilder& muting2_sub1(const geometry_msgs::msg::Polygon& muting2_sub1);
  ZoneSetMsgBuilder& safety1_sub2(const geometry_msgs::msg::Polygon& safety1_sub2);
  ZoneSetMsgBuilder& safety2_sub2(const geometry_msgs::msg::Polygon& safety2_sub2);
  ZoneSetMsgBuilder& safety3_sub2(const geometry_msgs::msg::Polygon& safety3_sub2);
  ZoneSetMsgBuilder& warn1_sub2(const geometry_msgs::msg::Polygon& warn1_sub2);
  ZoneSetMsgBuilder& warn2_sub2(const geometry_msgs::msg::Polygon& warn2_sub2);
  ZoneSetMsgBuilder& muting1_sub2(const geometry_msgs::msg::Polygon& muting1_sub2);
  ZoneSetMsgBuilder& muting2_sub2(const geometry_msgs::msg::Polygon& muting2_sub2);
  ZoneSetMsgBuilder& speedLower(const float& speed_lower);
  ZoneSetMsgBuilder& speedUpper(const float& speed_upper);

private:
  psen_scan_v2::msg::ZoneSet zoneset_msg_;
};

inline psen_scan_v2::msg::ZoneSet ZoneSetMsgBuilder::build() const
{
  return zoneset_msg_;
}

// inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::headerSeq(uint32_t seq)
// {
//   zoneset_msg_.header.seq = seq;
//   return *this;
// }

inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::headerStamp(const rclcpp::Time& stamp)
{
  zoneset_msg_.header.stamp = stamp;
  return *this;
}

inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::headerFrameId(const std::string& frame_id)
{
  zoneset_msg_.header.frame_id = frame_id;
  return *this;
}

// ZoneSetMsgBuilder Master
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety1(const geometry_msgs::msg::Polygon& safety1)
{
  zoneset_msg_.safety1 = safety1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety2(const geometry_msgs::msg::Polygon& safety2)
{
  zoneset_msg_.safety2 = safety2;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety3(const geometry_msgs::msg::Polygon& safety3)
{
  zoneset_msg_.safety3 = safety3;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::warn1(const geometry_msgs::msg::Polygon& warn1)
{
  zoneset_msg_.warn1 = warn1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::warn2(const geometry_msgs::msg::Polygon& warn2)
{
  zoneset_msg_.warn2 = warn2;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::muting1(const geometry_msgs::msg::Polygon& muting1)
{
  zoneset_msg_.muting1 = muting1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::muting2(const geometry_msgs::msg::Polygon& muting2)
{
  zoneset_msg_.muting2 = muting2;
  return *this;
}

// ZoneSetMsgBuilder subscriber0
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety1_sub0(const geometry_msgs::msg::Polygon& safety1_sub0)
{
  zoneset_msg_.safety1_sub0 = safety1_sub0;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety2_sub0(const geometry_msgs::msg::Polygon& safety2_sub0)
{
  zoneset_msg_.safety2_sub0 = safety2_sub0;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety3_sub0(const geometry_msgs::msg::Polygon& safety3_sub0)
{
  zoneset_msg_.safety3_sub0 = safety3_sub0;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::warn1_sub0(const geometry_msgs::msg::Polygon& warn1_sub0)
{
  zoneset_msg_.warn1_sub0 = warn1_sub0;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::warn2_sub0(const geometry_msgs::msg::Polygon& warn2_sub0)
{
  zoneset_msg_.warn2_sub0 = warn2_sub0;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::muting1_sub0(const geometry_msgs::msg::Polygon& muting1_sub0)
{
  zoneset_msg_.muting1_sub0 = muting1_sub0;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::muting2_sub0(const geometry_msgs::msg::Polygon& muting2_sub0)
{
  zoneset_msg_.muting2_sub0 = muting2_sub0;
  return *this;
}

// ZoneSetMsgBuilder Subscriber1
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety1_sub1(const geometry_msgs::msg::Polygon& safety1_sub1)
{
  zoneset_msg_.safety1_sub1 = safety1_sub1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety2_sub1(const geometry_msgs::msg::Polygon& safety2_sub1)
{
  zoneset_msg_.safety2_sub1 = safety2_sub1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety3_sub1(const geometry_msgs::msg::Polygon& safety3_sub1)
{
  zoneset_msg_.safety3_sub1 = safety3_sub1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::warn1_sub1(const geometry_msgs::msg::Polygon& warn1_sub1)
{
  zoneset_msg_.warn1_sub1 = warn1_sub1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::warn2_sub1(const geometry_msgs::msg::Polygon& warn2_sub1)
{
  zoneset_msg_.warn2_sub1 = warn2_sub1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::muting1_sub1(const geometry_msgs::msg::Polygon& muting1_sub1)
{
  zoneset_msg_.muting1_sub1 = muting1_sub1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::muting2_sub1(const geometry_msgs::msg::Polygon& muting2_sub1)
{
  zoneset_msg_.muting2_sub1 = muting2_sub1;
  return *this;
}

// ZoneSetMsgBuilder Subscriber2
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety1_sub2(const geometry_msgs::msg::Polygon& safety1_sub2)
{
  zoneset_msg_.safety1_sub2 = safety1_sub2;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety2_sub2(const geometry_msgs::msg::Polygon& safety2_sub2)
{
  zoneset_msg_.safety2_sub2 = safety2_sub2;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety3_sub2(const geometry_msgs::msg::Polygon& safety3_sub2)
{
  zoneset_msg_.safety3_sub2 = safety3_sub2;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::warn1_sub2(const geometry_msgs::msg::Polygon& warn1_sub2)
{
  zoneset_msg_.warn1_sub2 = warn1_sub2;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::warn2_sub2(const geometry_msgs::msg::Polygon& warn2_sub2)
{
  zoneset_msg_.warn2_sub2 = warn2_sub2;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::muting1_sub2(const geometry_msgs::msg::Polygon& muting1_sub2)
{
  zoneset_msg_.muting1_sub2 = muting1_sub2;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::muting2_sub2(const geometry_msgs::msg::Polygon& muting2_sub2)
{
  zoneset_msg_.muting2_sub2 = muting2_sub2;
  return *this;
}


inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::speedLower(const float& speed_lower)
{
  zoneset_msg_.speed_lower = speed_lower;
  return *this;
}

inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::speedUpper(const float& speed_upper)
{
  zoneset_msg_.speed_upper = speed_upper;
  return *this;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_ZONESET_MSG_BUILDER_H
