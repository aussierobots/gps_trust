// Copyright 2024 Australian Robotics Supplies & Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef GPS_TRUST_NODE__NODE_PARAMS_HPP_
#define GPS_TRUST_NODE__NODE_PARAMS_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"

enum ParamStatus
{
  PARAM_INITIAL = 0,  // Initial VALUE loaded from node
  PARAM_CHANGED = 1,  // Paremeter Event updated changed value
  PARAM_DELETED = 2,  // Paramter Event deleted the value
};

enum ParamNotify
{
  NOTI_NOT_ACTIONED,    // Parameter has been updated but nothing actioned
  NOTI_GPS_TRUST,       // Parameter value has been delivered
  NOTI_NODE_UPDATE,    // Parameter needs to be updated
};

enum NodeParamType
{
  NPT_NTRIP_CLIENT = 0,
  NPT_UBX_CFG = 1,
};

// Function to convert enum to string
std::string paramTypeToString(NodeParamType type)
{
  switch (type) {
    case NodeParamType::NPT_NTRIP_CLIENT: return "NTRIP";
    case NodeParamType::NPT_UBX_CFG: return "UBX_CFG";
    default: return "UNKNOWN";
  }
}

// Function to convert string to enum (if needed)
NodeParamType stringToParamType(const std::string & str)
{
  if (str == "NTRIP") {return NodeParamType::NPT_NTRIP_CLIENT;}
  if (str == "UBX_CFG") {return NodeParamType::NPT_UBX_CFG;}
  throw std::invalid_argument("Unknown NodeParamType string");
}

struct param_key_t
{
  std::string name;
  NodeParamType node_type;
};

bool operator<(const param_key_t & lhs, const param_key_t & rhs)
{
  if (lhs.name < rhs.name) {
    return true;
  }
  if (lhs.name > rhs.name) {
    return false;
  }
  // If names are equal, compare ParamType
  return lhs.node_type < rhs.node_type;
}

struct param_state_t
{
  rclcpp::ParameterValue value;
  rclcpp::ParameterType type;
  ParamStatus status;
  ParamNotify noti;
};

#endif  // GPS_TRUST_NODE__NODE_PARAMS_HPP_
