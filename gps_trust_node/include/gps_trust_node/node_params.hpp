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
#include <optional>
#include "rclcpp/rclcpp.hpp"

enum ParamType {
    Null = 0,
    Bool = 1,
    Int = 2,
    Double = 3,
    String = 4,
};

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
std::string param_type_to_string(NodeParamType type)
{
  switch (type) {
    case NodeParamType::NPT_NTRIP_CLIENT: return "NTRIP";
    case NodeParamType::NPT_UBX_CFG: return "UBX_CFG";
    default: return "UNKNOWN";
  }
}

// Function to convert string to enum (if needed)
NodeParamType string_to_param_type(const std::string & str)
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

// Helper class for parameter cache operations
class ParamCacheHelper
{
public:
  // Constructor taking a ROS logger and mutex
  ParamCacheHelper(
    rclcpp::Logger logger,
    std::mutex & mutex,
    std::map<param_key_t, param_state_t> & cache_map)
  : logger_(logger), mutex_(mutex), params_cache_map_(cache_map) {}

  // Check if a parameter exists in the cache
  bool check_param_in_cache(const std::string & param_name, NodeParamType node_type)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    param_key_t key{param_name, node_type};
    if (params_cache_map_.find(key) == params_cache_map_.end()) {
      RCLCPP_WARN(
        logger_,
        "Parameter '%s' of type %s not found in cache map",
        param_name.c_str(),
        param_type_to_string(node_type).c_str()
      );
      return false;
    }
    return true;
  }

  // Get parameter state from cache
  std::optional<param_state_t> get_param_from_cache(
    const std::string & param_name,
    NodeParamType node_type)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    param_key_t key{param_name, node_type};
    auto it = params_cache_map_.find(key);
    if (it == params_cache_map_.end()) {
      RCLCPP_WARN(
        logger_,
        "Parameter '%s' of type %s not found in cache map",
        param_name.c_str(),
        param_type_to_string(node_type).c_str()
      );
      return std::nullopt;
    }
    return it->second;
  }

  // Update parameter state in cache
  bool update_param_in_cache(
    const std::string & param_name,
    NodeParamType node_type,
    const param_state_t & state)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    param_key_t key{param_name, node_type};
    auto it = params_cache_map_.find(key);
    if (it == params_cache_map_.end()) {
      RCLCPP_WARN(
        logger_,
        "Cannot update: Parameter '%s' of type %s not found in cache map",
        param_name.c_str(),
        param_type_to_string(node_type).c_str()
      );
      return false;
    }
    params_cache_map_[key] = state;
    return true;
  }

  // Update parameter notification state
  bool update_param_notification(
    const std::string & param_name,
    NodeParamType node_type,
    ParamNotify noti)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    param_key_t key{param_name, node_type};
    auto it = params_cache_map_.find(key);
    if (it == params_cache_map_.end()) {
      RCLCPP_WARN(
        logger_,
        "Cannot update notification: Parameter '%s' of type %s not found in cache map",
        param_name.c_str(),
        param_type_to_string(node_type).c_str()
      );
      return false;
    }
    it->second.noti = noti;
    return true;
  }

private:
  rclcpp::Logger logger_;
  std::mutex & mutex_;
  std::map<param_key_t, param_state_t> & params_cache_map_;
};

#endif   // GPS_TRUST_NODE__NODE_PARAMS_HPP_
