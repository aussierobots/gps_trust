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

#include <cstdio>
#include <curl/curl.h>
#include <json/json.h>
#include <sstream>
#include <zlib.h>
#include <vector>
#include <string>
#include <chrono>
#include <memory>
#include <mutex>
#include <algorithm>

#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/header.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_hp_pos_llh.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_orb.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_sat.hpp"
#include "ublox_ubx_msgs/msg/ubx_sec_sig.hpp"
#include "ublox_ubx_msgs/msg/ubx_rxm_rawx.hpp"
#include "rtcm_msgs/msg/message.hpp"
#include "gps_trust_node/visibility_control.h"
#include "gps_trust_node/node_params.hpp"
#include "gps_trust_msgs/msg/gps_trust_indicator.hpp"

using namespace std::chrono_literals;
using SetParametersResult =
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace gps_trust
{

class GPSTrustNode : public rclcpp::Node
{
public:
  GPS_TRUST_NODE_PUBLIC
  GPSTrustNode(const rclcpp::NodeOptions & options)
  : Node("gps_trust_node", options)
  {
    RCLCPP_INFO(this->get_logger(), "starting %s", get_name());

    auto qos = rclcpp::SensorDataQoS();

    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

    rclcpp::SubscriptionOptions sub_options;
    sub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    // Subscriber
    nav_llh_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavHPPosLLH>(
      "/ubx_nav_hp_pos_llh", qos,
      std::bind(&GPSTrustNode::ubx_nav_llh_callback, this, std::placeholders::_1),
      sub_options);

    sec_sig_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXSecSig>(
      "/ubx_sec_sig", qos,
      std::bind(&GPSTrustNode::ubx_sec_sig_callback, this, std::placeholders::_1),
      sub_options);

    nav_orb_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavOrb>(
      "/ubx_nav_orb", qos,
      std::bind(&GPSTrustNode::ubx_nav_orb_callback, this, std::placeholders::_1),
      sub_options);

    nav_sat_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavSat>(
      "/ubx_nav_sat", qos,
      std::bind(&GPSTrustNode::ubx_nav_sat_callback, this, std::placeholders::_1),
      sub_options);

    rxm_rawx_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXRxmRawx>(
      "/ubx_rxm_rawx", qos,
      std::bind(&GPSTrustNode::ubx_rxm_rawx_callback, this, std::placeholders::_1),
      sub_options);

    // Publisher
    pub_ =
      this->create_publisher<gps_trust_msgs::msg::GPSTrustIndicator>(
      "gps_trust_indicator", qos, pub_options);

    param_cache_helper_ = std::make_unique<ParamCacheHelper>(
      this->get_logger(),
      params_cache_mutex_,
      params_cache_map_
    );

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          get_logger(), "Interrupted while waiting for parameter client service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_WARN(get_logger(), "parameter client service not available, waiting again...");
    }

    // Get GPS Trust Host URL
    declare_parameter(
      "GPS_TRUST_API_URL",
      "http://mbp.local:9000/lambda-url/gps-trust-api"
      // "https://gtapi.aussierobots.com.au/gps-trust-api"
    );

    get_parameter("GPS_TRUST_API_URL", api_url_);

    RCLCPP_INFO(this->get_logger(), "api_url: '%s'", api_url_.c_str());

    // Get API Key from parameter
    declare_parameter("GPS_TRUST_DEVICE_API_KEY", "demo_api_key");
    get_parameter("GPS_TRUST_DEVICE_API_KEY", api_key_);

    declare_parameter("NTRIP_CLIENT_NODE", "/ntrip_client");
    get_parameter("NTRIP_CLIENT_NODE", ntrip_client_node_);

    declare_parameter("UBLOX_DGNSS_NODE", "/ublox_dgnss");
    get_parameter("UBLOX_DGNSS_NODE", ublox_dgnss_node_);

    declare_parameter("maxage_conn", 30);
    maxage_conn_ = get_parameter("maxage_conn").as_int();

    declare_parameter("log_level", "INFO");
    log_level_ = get_parameter("log_level").as_string();

    ntrip_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this,
      ntrip_client_node_);
    ublox_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this,
      ublox_dgnss_node_);

    ntrip_parameter_event_sub_ = ntrip_parameters_client_->on_parameter_event(
      std::bind(
        &GPSTrustNode::on_parameter_event_callback,
        this, _1));

    ublox_parameter_event_sub_ = ublox_parameters_client_->on_parameter_event(
      std::bind(
        &GPSTrustNode::on_parameter_event_callback,
        this, _1));

    while (!ntrip_parameters_client_->wait_for_service(1s) &&
      !ublox_parameters_client_->wait_for_service(1s))
    {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "interrupted while waiting for the service. exiting.");
        rclcpp::shutdown();
        return;
      }
      RCLCPP_WARN(this->get_logger(), "paramter client service not available, waiting again...");
    }

    initialise_ntrip_parameters();
    initialise_ublox_parameters();

    RCLCPP_INFO(get_logger(), "api_key: '%s'", api_key_.c_str());

    RCLCPP_INFO(get_logger(), "params_cache_map: %s", get_params_cache_output().c_str());
  }

private:
  rclcpp::AsyncParametersClient::SharedPtr ntrip_parameters_client_;
  rclcpp::AsyncParametersClient::SharedPtr ublox_parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr ntrip_parameter_event_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr ublox_parameter_event_sub_;

  std::map<param_key_t, param_state_t> params_cache_map_;
  std::mutex params_cache_mutex_;  // Mutex to protect access to the map
  std::unique_ptr<ParamCacheHelper> param_cache_helper_;

  std::string ntrip_client_node_;
  std::string ublox_dgnss_node_;

  std::string log_level_;
  long maxage_conn_;

  GPS_TRUST_NODE_LOCAL
  void initialise_ntrip_parameters()
  {
    // list existing parameters
    auto ntrip_parameter_list_future = ntrip_parameters_client_->list_parameters(
      {"host", "port", "mountpoint"},
      2);

    // Wait for the future to be ready
    if (rclcpp::spin_until_future_complete(
        this->get_node_base_interface(),
        ntrip_parameter_list_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get ntrip parameter list");
      return;
    }
    auto ntrip_parameter_list = ntrip_parameter_list_future.get();


    auto parameters_future = ntrip_parameters_client_->get_parameters(ntrip_parameter_list.names);
    // Wait for the future to be ready
    if (rclcpp::spin_until_future_complete(
        this->get_node_base_interface(),
        parameters_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get ntrip parameters");
      return;
    }
    auto parameters = parameters_future.get();

    {
      // Lock the mutex to ensure thread safety
      std::lock_guard<std::mutex> lock(params_cache_mutex_);
      std::stringstream ss;

      ss << "\nNTRIP Parameter names:";
      for (auto & p :parameters) {
        auto name = p.get_name();
        auto value = p.get_parameter_value();
        auto type = p.get_type();

        ss << "\n " << name;

        auto key = param_key_t {name, NPT_NTRIP_CLIENT};
        params_cache_map_[key] = {value, type, PARAM_INITIAL, NOTI_NOT_ACTIONED};

      }

      RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
    }
  }

  GPS_TRUST_NODE_LOCAL
  void initialise_ublox_parameters()
  {
    auto ublox_parameter_list_future = ublox_parameters_client_->list_parameters(
      {},
      2);

    // Wait for the future to be ready
    if (rclcpp::spin_until_future_complete(
        this->get_node_base_interface(),
        ublox_parameter_list_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get  ublox parameter list");
      return;
    }
    auto ublox_parameter_list = ublox_parameter_list_future.get();

    // List of parameter names to exclude
    std::set<std::string> excluded_names = {"start_type_description_service", "use_sim_time"};

    // Filter out excluded parameter names
    std::vector<std::string> filtered_parameter_names;
    for (const auto & name : ublox_parameter_list.names) {
      if (excluded_names.find(name) == excluded_names.end()) {
        filtered_parameter_names.push_back(name);
      }
    }

    auto parameters_future = ublox_parameters_client_->get_parameters(filtered_parameter_names);
    // Wait for the future to be ready
    if (rclcpp::spin_until_future_complete(
        this->get_node_base_interface(),
        parameters_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get ublox parameters");
      return;
    }

    auto parameters = parameters_future.get();
    {
      // Lock the mutex to ensure thread safety
      std::lock_guard<std::mutex> lock(params_cache_mutex_);

      std::stringstream ss;
      ss << "\nUBLOX Parameter names:";
      for (auto & p : parameters) {
        auto name = p.get_name();
        auto value = p.get_parameter_value();
        auto type = p.get_type();

        ss << "\n " << name;

        auto key = param_key_t {name, NPT_UBX_CFG};
        params_cache_map_[key] = {value, type, PARAM_INITIAL, NOTI_NOT_ACTIONED};
      }

      RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
    }
  }

  GPS_TRUST_NODE_LOCAL
  std::string get_params_cache_output()
  {
    std::stringstream ss;
    ss << std::endl;

    // Iterate over the map and extract key-value pairs
    for (const auto & entry : params_cache_map_) {
      const param_key_t & key = entry.first;
      const param_state_t & state = entry.second;

      // Output key (name and type)
      ss << "Name: " << key.name << ", Type: ";
      switch (key.node_type) {
        case NPT_NTRIP_CLIENT:
          ss << "NTRIP Client";
          break;
        case NPT_UBX_CFG:
          ss << "UBX Config";
          break;
      }

      // Output parameter value (as string) using rclcpp::ParameterValue's get types
      ss << ", Value: ";
      if (state.value.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        ss << state.value.get<bool>();
      } else if (state.value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        ss << state.value.get<int64_t>();
      } else if (state.value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        ss << state.value.get<double>();
      } else if (state.value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        ss << state.value.get<std::string>();
      } else {
        ss << "Unknown type";
      }

      // Output ParamStatus
      ss << ", Status: ";
      switch (state.status) {
        case PARAM_INITIAL:
          ss << "Initial";
          break;
        case PARAM_CHANGED:
          ss << "Changed";
          break;
        case PARAM_DELETED:
          ss << "Deleted";
          break;
      }

      // Output ParamNotify
      ss << ", Notify: ";
      switch (state.noti) {
        case NOTI_NOT_ACTIONED:
          ss << "Not Actioned";
          break;
        case NOTI_GPS_TRUST:
          ss << "GPS Trust";
          break;
        case NOTI_NODE_UPDATE:
          ss << "Node Update";
          break;
      }

      // End line after each entry
      ss << std::endl;
    }

    // Return the accumulated output as a string
    return ss.str();
  }

  GPS_TRUST_NODE_LOCAL
  rcl_interfaces::msg::SetParametersResult on_parameter_event_callback(
    rcl_interfaces::msg::ParameterEvent::UniquePtr event)
  {
    // Check if event_name is not one of the expected parameters
    if (event->node != ntrip_client_node_ && event->node != ublox_dgnss_node_) {
      // Return or handle the event when it's not one of the specified names
      return rcl_interfaces::msg::SetParametersResult();
    }

    // ignore qos overrides
    event->new_parameters.erase(
      std::remove_if(
        event->new_parameters.begin(),
        event->new_parameters.end(),
        [](const auto & item) {
          const char * param_override_prefix = "qos_overrides.";
          return std::strncmp(
            item.name.c_str(), param_override_prefix, sizeof(param_override_prefix) - 1) == 0u;
        }),
      event->new_parameters.end());
    if (
      !event->new_parameters.size() && !event->changed_parameters.size() &&
      !event->deleted_parameters.size())
    {
      return rcl_interfaces::msg::SetParametersResult();
    }


    auto get_node_type = [&](const std::string & node_str) -> NodeParamType {
      if (node_str == ntrip_client_node_) {
        return NPT_NTRIP_CLIENT;
      } else if (node_str == ublox_dgnss_node_) {
        return NPT_UBX_CFG;
      } else {
        throw std::invalid_argument("Unknown node string: " + node_str);
      }
    };

    // lambda function to update the parameter cache map.
    auto update_params_cache = [&](const rcl_interfaces::msg::Parameter & msg_p,
      NodeParamType node_type,
      ParamStatus status) {
      rclcpp::Parameter p = rclcpp::Parameter::from_parameter_msg(msg_p);

      // Create the key and update the params_cache_map_ with the passed node_type and status
      auto key = param_key_t {p.get_name(), node_type};
      params_cache_map_[key] = {p.get_parameter_value(), p.get_type(), status, NOTI_NODE_UPDATE};
    };

    {
      // Lock the mutex to ensure thread safety
      std::lock_guard<std::mutex> lock(params_cache_mutex_);

      std::stringstream ss;
      ss << "\nParameter event: ";
      // ss << event->node;

      auto node_type = get_node_type(event->node);
      ss << param_type_to_string(node_type);

      ss << "\n new parameters:";
      for (auto & new_parameter : event->new_parameters) {
        ss << "\n  " << new_parameter.name;
        update_params_cache(new_parameter, node_type, PARAM_INITIAL);
      }
      ss << "\n changed parameters:";
      for (auto & changed_parameter : event->changed_parameters) {
        ss << "\n  " << changed_parameter.name;
        update_params_cache(changed_parameter, node_type, PARAM_CHANGED);
      }
      ss << "\n deleted parameters:";
      for (auto & deleted_parameter : event->deleted_parameters) {
        ss << "\n  " << deleted_parameter.name;
        update_params_cache(deleted_parameter, node_type, PARAM_DELETED);
      }
      ss << "\n";
      RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    return rcl_interfaces::msg::SetParametersResult();
  }

  GPS_TRUST_NODE_LOCAL
  Json::Value json_from_params_cache_map()
  {
    Json::Value ntypes;
    Json::Value names;
    Json::Value values;
    Json::Value ptypes;
    Json::Value statuses;

    // Iterate over the params_cache_map_ and process each entry
    for (const auto & entry : params_cache_map_) {
      // Extract key and state
      const param_key_t & key = entry.first;
      const param_state_t & state = entry.second;

      // only want to send the intitial or udpated - keep chatter low
      if (state.noti != NOTI_NOT_ACTIONED && state.noti != NOTI_NODE_UPDATE) {
        continue;
      }

      // Append key components
      ntypes.append(static_cast<int>(key.node_type));
      names.append(key.name);

      // Append state components
      switch (state.value.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          values.append(state.value.get<bool>());
          break;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          values.append(state.value.get<int64_t>());
          break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          values.append(state.value.get<double>());
          break;
        case rclcpp::ParameterType::PARAMETER_STRING:
          values.append(state.value.get<std::string>());
          break;
        default:
          RCLCPP_ERROR(
            get_logger(), "parameter: %s has an unknown value: %s", key.name.c_str(),
            state.value.get<std::string>().c_str());
          values.append("UNKNOWN");
          break;
      }

      ptypes.append(static_cast<int>(state.type));        // Convert ParameterType to integer
      statuses.append(static_cast<int>(state.status));          // Convert ParamStatus to integer
    }

    Json::Value nparams; // short for node params
    nparams["ntypes"] = ntypes;
    nparams["names"] = names;
    nparams["values"] = values;
    nparams["ptypes"] = ptypes;
    nparams["statuses"] = statuses;

    return nparams;
  }

  GPS_TRUST_NODE_LOCAL
  Json::Value json_from_ubx_nav_llh(const ublox_ubx_msgs::msg::UBXNavHPPosLLH::SharedPtr msg)
  {
    // Extract the LLH and high-precision components
    double lat = (msg->lat + msg->lat_hp * 1e-2) * 1e-7;
    double lon = (msg->lon + msg->lon_hp * 1e-2) * 1e-7;
    double height = (msg->height + msg->height_hp * 0.1) * 1e-3; // meters
    double hmsl = (msg->hmsl + msg->hmsl_hp * 0.1) * 1e-3; // meters
    double h_acc = msg->h_acc * 1e-4; // meters
    double v_acc = msg->v_acc * 1e-4; // meters

    Json::Value json_stamp;
    json_stamp["sec"] = msg->header.stamp.sec;
    json_stamp["nanosec"] = msg->header.stamp.nanosec;

    Json::Value llh;
    llh["timestamp"] = json_stamp;
    llh["frame_id"] = msg->header.frame_id;
    llh["lon"] = lon;
    llh["lat"] = lat;
    llh["height"] = height;
    llh["hmsl"] = hmsl;
    llh["h_acc"] = h_acc;
    llh["v_acc"] = v_acc;

    return llh;
  }

  GPS_TRUST_NODE_LOCAL
  void ubx_nav_llh_callback(const ublox_ubx_msgs::msg::UBXNavHPPosLLH::SharedPtr msg)
  {
    // Create JSON object to send to the API
    Json::Value json_request;

    json_request["llh"] = json_from_ubx_nav_llh(msg);

    RCLCPP_DEBUG(get_logger(), "sec_sig_json_.use_count(): %ld", sec_sig_json_.use_count());
    if (sec_sig_json_.use_count() != 0) {
      json_request["sec_sig"] = *sec_sig_json_.get();
      sec_sig_json_.reset();
    }

    RCLCPP_DEBUG(get_logger(), "nav_orb_json_.use_count(): %ld", nav_orb_json_.use_count());
    if (nav_orb_json_.use_count() != 0) {
      json_request["nav_orb"] = *nav_orb_json_.get();
      nav_orb_json_.reset();
    }

    RCLCPP_DEBUG(get_logger(), "nav_sat_json_.use_count(): %ld", nav_sat_json_.use_count());
    if (nav_sat_json_.use_count() != 0) {
      json_request["nav_sat"] = *nav_sat_json_.get();
      nav_sat_json_.reset();
    }

    RCLCPP_DEBUG(get_logger(), "rxm_rawx_json_.use_count(): %ld", rxm_rawx_json_.use_count());
    if (rxm_rawx_json_.use_count() != 0) {
      json_request["rxm_rawx"] = *rxm_rawx_json_.get();
      rxm_rawx_json_.reset();
    }

    // count how many params need to be actioned
    size_t pcm_len = 0;
    // Iterate through the map
    for (const auto & entry : params_cache_map_) {
      const param_state_t & state = entry.second;   // Access the param_state_t struct

      // Check if the notification status is either NOTI_NOT_ACTIONED or NOTI_NODE_UPDATE
      if (state.noti == NOTI_NOT_ACTIONED || state.noti == NOTI_NODE_UPDATE) {
        ++pcm_len;      // Increment the counter if the condition is satisfied
      }
    }

    RCLCPP_DEBUG(
      get_logger(), "params_cache_map_ to be actioned or have been updated len: %ld",
      pcm_len);

    if (pcm_len > 0) {
      // Lock the mutex to ensure thread safety
      std::lock_guard<std::mutex> lock(params_cache_mutex_);

      json_request["nparams"] = json_from_params_cache_map();

      for (auto & entry : params_cache_map_) {
        auto & state = entry.second;
        if (state.noti == NOTI_NOT_ACTIONED || state.noti == NOTI_NODE_UPDATE) {
          state.noti = NOTI_GPS_TRUST;
        }
      }

    }

    Json::StreamWriterBuilder builder;
    builder.settings_["indentation"] = "";
    RCLCPP_DEBUG(
      get_logger(), "json_request: %s",
      Json::writeString(builder, json_request).c_str());


    // Call API and get the response
    Json::Value data;
    data["data"] = json_request;
    RCLCPP_INFO(
      get_logger(), "json data: %s",
      Json::writeString(builder, data).c_str());
    Json::Value json_response = callAPI(data, false);

    if (json_response.empty()) {
      RCLCPP_ERROR(
        get_logger(), "error calling GPS Trust API: %s",
        api_url_.c_str());

      // Decode and publish the GPS trust indicator
      gps_trust_msgs::msg::GPSTrustIndicator gps_trust_msg;
      gps_trust_msg.header.frame_id = this->get_name();
      gps_trust_msg.header.stamp = this->now();
      gps_trust_msg.trust_level = 0;
      gps_trust_msg.status = "ERROR with GPS Trust API connection";
      pub_->publish(gps_trust_msg);

    } else {
      RCLCPP_INFO(
        get_logger(), "json_response: %s",
        Json::writeString(builder, json_response).c_str());

      // Decode and publish the GPS trust indicator
      gps_trust_msgs::msg::GPSTrustIndicator gps_trust_msg;
      // gps_trust_msg.header.stamp = this->now();
      gps_trust_msg.header.frame_id = json_response["frameId"].asString();
      Json::Value stamp = json_response["timestamp"];
      gps_trust_msg.header.stamp.sec = stamp["sec"].asInt64();
      gps_trust_msg.header.stamp.nanosec = stamp["nanosec"].asUInt64();
      gps_trust_msg.trust_level = json_response["trustLevel"].asInt();
      gps_trust_msg.status = json_response["status"].asString();
      pub_->publish(gps_trust_msg);

      // RCLCPP_INFO(
      //   get_logger(), "json_response.isMember(\"action\"): %d",
      //   json_response.isMember("action"));

      // First check if we have a non-null action
      if (json_response.isMember("action") && !json_response["action"].isNull()) {
        // Process the EntityAction
        const auto & action_json = json_response["action"];

        // Additional safety checks for the action structure
        if (action_json.isObject() &&
          action_json.isMember("names") && action_json.isMember("values") &&
          action_json.isMember("ptypes") &&
          action_json["names"].isArray() && action_json["values"].isArray() &&
          action_json["ptypes"].isArray())
        {

          const auto & names = action_json["names"];
          const auto & values = action_json["values"];
          const auto & ptypes = action_json["ptypes"];

          // Process the action data here
          for (Json::Value::ArrayIndex i = 0; i < names.size(); i++) {
            auto name = names[i].asString();
            auto value = values[i].asString();
            auto ptype = ptypes[i].asUInt();

            RCLCPP_INFO(
              get_logger(), "Action param: %s = %s (type: %d)",
              name.c_str(),
              value.c_str(),
              ptype);
            // Add your action processing logic here

            if (!param_cache_helper_->check_param_in_cache(name, NodeParamType::NPT_UBX_CFG)) {
              RCLCPP_WARN(
                get_logger(), "Action name: %s not in param_cache .. doing nothing!",
                name.c_str());

            } else {
              param_cache_helper_->update_param_notification(name, NPT_UBX_CFG, NOTI_NODE_UPDATE);

              auto param_optional = param_cache_helper_->get_param_from_cache(
                name,
                NodeParamType::NPT_UBX_CFG);
              if (param_optional) {
                auto param = *param_optional;
                std::vector<rclcpp::Parameter> parameters;

                try {
                  switch (param.type) {
                    case ParamType::Bool: {
                        std::string bool_str = value;
                        // Convert to lowercase for case-insensitive comparison
                        std::transform(
                          bool_str.begin(), bool_str.end(), bool_str.begin(),
                          ::tolower);
                        parameters.emplace_back(name, bool_str == "true" || bool_str == "1");
                        break;
                      }
                    case ParamType::Int:
                      parameters.emplace_back(name, std::stoi(value));
                      break;
                    case ParamType::Double:
                      parameters.emplace_back(name, std::stod(value));
                      break;
                    case ParamType::String:
                      parameters.emplace_back(name, value);
                      break;
                    case ParamType::Null:
                      RCLCPP_WARN(this->get_logger(), "Parameter '%s' has Null type", name.c_str());
                      return;
                    default:
                      RCLCPP_ERROR(
                        this->get_logger(), "Unknown parameter type for '%s'",
                        name.c_str());
                      return;
                  }

                  // For the whole parameters vector
                  for (const auto & param : parameters) {
                    RCLCPP_INFO(
                      get_logger(),
                      "Parameter: name='%s' type=%d value=%s",
                      param.get_name().c_str(),
                      param.get_type(),
                      param.value_to_string().c_str()
                    );
                  }

                  auto callback = [this, name, parameters](
                    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future)
                  {
                    auto results = future.get();
                    for (const auto & result : results) {
                      if (!result.successful) {
                        RCLCPP_ERROR(
                          get_logger(),
                          "Failed to set parameter '%s' (value: %s): %s",
                          name.c_str(),
                          parameters.front().value_to_string().c_str(),
                          result.reason.c_str()
                        );
                      } else {
                        RCLCPP_INFO(
                          get_logger(),
                          "Successfully set parameter '%s' to %s",
                          name.c_str(),
                          parameters.front().value_to_string().c_str()
                        );
                      }
                    }
                  };

                  ublox_parameters_client_->set_parameters(parameters, callback);

                } catch (const std::exception & e) {
                  RCLCPP_ERROR(
                    get_logger(),
                    "Error converting parameter '%s': %s",
                    name.c_str(),
                    e.what()
                  );
                }
              }
            }
          }
        }
      }
    }
}

GPS_TRUST_NODE_LOCAL
Json::Value json_from_ubx_sec_sec(const ublox_ubx_msgs::msg::UBXSecSig::SharedPtr msg)
{
  Json::Value json_stamp;
  json_stamp["sec"] = msg->header.stamp.sec;
  json_stamp["nanosec"] = msg->header.stamp.nanosec;

  Json::Value ss;
  ss["timestamp"] = json_stamp;
  ss["frame_id"] = msg->header.frame_id;

  ss["jam_det_enabled"] = static_cast<bool>(msg->jam_det_enabled);
  ss["jamming_state"] = msg->jamming_state;

  ss["spf_det_enabled"] = static_cast<bool>(msg->spf_det_enabled);
  ss["spoofing_state"] = msg->spoofing_state;

  if (msg->version >= 2) {
    ss["jam_num_cent_freqs"] = msg->jam_num_cent_freqs;

    Json::Value cent_freqs;
    Json::Value jammed_states;

    for (size_t i = 0; i < msg->jam_num_cent_freqs; i++) {
      auto jm = msg->jam_state_cent_freqs[i];
      cent_freqs.append(jm.cent_freq);
      jammed_states.append(jm.jammed);
    }

    ss["cent_freqs"] = cent_freqs;
    ss["jammed_states"] = jammed_states;
  }

  return ss;
}

GPS_TRUST_NODE_LOCAL
void ubx_sec_sig_callback(const ublox_ubx_msgs::msg::UBXSecSig::SharedPtr msg)
{
  sec_sig_json_ = std::make_shared<Json::Value>(json_from_ubx_sec_sec(msg));

  RCLCPP_DEBUG(get_logger(), "sec_sig_json: %s", sec_sig_json_->toStyledString().c_str());
}

GPS_TRUST_NODE_LOCAL
Json::Value json_from_ubx_nav_orb(const ublox_ubx_msgs::msg::UBXNavOrb::SharedPtr msg)
{
  Json::Value json_stamp;
  json_stamp["sec"] = msg->header.stamp.sec;
  json_stamp["nanosec"] = msg->header.stamp.nanosec;

  Json::Value no;
  no["timestamp"] = json_stamp;
  no["frame_id"] = msg->header.frame_id;

  no["itow"] = msg->itow;
  no["num_sv"] = msg->num_sv;

  Json::Value gnss_ids;
  Json::Value sv_ids;
  Json::Value healths;
  Json::Value visibilities;
  Json::Value eph_usabilities;
  Json::Value eph_sources;
  Json::Value alm_usabilities;
  Json::Value alm_sources;
  Json::Value ano_aop_usabilities;
  Json::Value orb_types;

  for (size_t i = 0; i < msg->num_sv; i++) {
    auto sv = msg->sv_info[i];
    gnss_ids.append(sv.gnss_id);
    sv_ids.append(sv.sv_id);
    healths.append(sv.sv_flag.health);
    visibilities.append(sv.sv_flag.visibility);
    eph_usabilities.append(sv.eph.eph_usability);
    eph_sources.append(sv.eph.eph_source);
    alm_usabilities.append(sv.alm.alm_usability);
    alm_sources.append(sv.alm.alm_source);
    ano_aop_usabilities.append(sv.other_orb.ano_aop_usability);
    orb_types.append(sv.other_orb.orb_type);
  }

  no["gnss_ids"] = gnss_ids;
  no["sv_ids"] = sv_ids;
  no["healths"] = healths;
  no["visibilities"] = visibilities;
  no["eph_usabilities"] = eph_usabilities;
  no["eph_sources"] = eph_sources;
  no["alm_usabilities"] = alm_usabilities;
  no["alm_sources"] = alm_sources;
  no["ano_aop_usabilities"] = ano_aop_usabilities;
  no["orb_types"] = orb_types;

  return no;
}

GPS_TRUST_NODE_LOCAL

std::string flatten_json_array(std::string name, Json::Value array)
{
  std::ostringstream oss;
  oss << std::quoted(name) << " : [";
  size_t i = 0;
  for (auto id: array) {
    if (i++ > 0) {oss << ",";}
    oss << id;
  }
  oss << "]";
  return oss.str();
}

GPS_TRUST_NODE_LOCAL
void ubx_nav_orb_callback(const ublox_ubx_msgs::msg::UBXNavOrb::SharedPtr msg)
{
  nav_orb_json_ = std::make_shared<Json::Value>(json_from_ubx_nav_orb(msg));

  auto json = *nav_orb_json_.get();

  using std::endl;

  // need to format our own json response here due to the arrayas
  std::ostringstream oss;
  oss << "{" << endl;
  oss << "\t" << std::quoted("frame_id") << " : " << json["frame_id"].toStyledString();
  oss << "\t" << std::quoted("timestamp") << " : " << json["timestamp"].toStyledString();
  oss << "\t" << std::quoted("itow") << " : " << json["itow"].toStyledString();
  oss << "\t" << std::quoted("num_sv") << " : " << json["num_sv"].toStyledString();

  oss << "\t" << flatten_json_array("gnss_ids", json["gnss_ids"]) << endl;
  oss << "\t" << flatten_json_array("sv_ids", json["sv_ids"]) << endl;
  oss << "\t" << flatten_json_array("healths", json["healths"]) << endl;
  oss << "\t" << flatten_json_array("visibilities", json["visibilities"]) << endl;
  oss << "\t" << flatten_json_array("eph_usabilities", json["eph_usabilities"]) << endl;
  oss << "\t" << flatten_json_array("eph_sources", json["eph_sources"]) << endl;
  oss << "\t" << flatten_json_array("alm_usabilities", json["alm_usabilities"]) << endl;
  oss << "\t" << flatten_json_array("alm_sources", json["alm_sources"]) << endl;
  oss << "\t" << flatten_json_array("ano_aop_usabilities", json["ano_aop_usabilities"]) << endl;
  oss << "\t" << flatten_json_array("orb_types", json["orb_types"]) << endl;

  oss << "}" << endl;

  RCLCPP_DEBUG(get_logger(), "nav_orb_json: %s", oss.str().c_str());
}

GPS_TRUST_NODE_LOCAL
Json::Value json_from_ubx_nav_sat(const ublox_ubx_msgs::msg::UBXNavSat::SharedPtr msg)
{
  Json::Value json_stamp;
  json_stamp["sec"] = msg->header.stamp.sec;
  json_stamp["nanosec"] = msg->header.stamp.nanosec;

  Json::Value ns;
  ns["timestamp"] = json_stamp;
  ns["frame_id"] = msg->header.frame_id;

  ns["itow"] = msg->itow;
  ns["num_svs"] = msg->num_svs;

  Json::Value gnss_ids;
  Json::Value sv_ids;
  Json::Value cnos;
  Json::Value elevs;
  Json::Value azims;
  Json::Value pr_ress;
  Json::Value quality_inds;
  Json::Value sv_useds;
  Json::Value healths;
  Json::Value diff_corrs;
  Json::Value smoothedes;
  Json::Value orbit_sources;
  Json::Value eph_avails;
  Json::Value alm_avails;
  Json::Value ano_avails;
  Json::Value aop_avails;
  Json::Value sbas_corr_useds;
  Json::Value rtcm_corr_useds;
  Json::Value slas_corr_useds;
  Json::Value spartn_corr_useds;
  Json::Value pr_corr_useds;
  Json::Value cr_corr_useds;
  Json::Value do_corr_useds;
  Json::Value clas_corr_useds;

  for (size_t i = 0; i < msg->num_svs; i++) {
    auto sv = msg->sv_info[i];
    gnss_ids.append(sv.gnss_id);
    sv_ids.append(sv.sv_id);
    cnos.append(sv.cno);
    elevs.append(sv.elev);
    azims.append(sv.azim);
    pr_ress.append(sv.pr_res * 0.1);
    quality_inds.append(sv.flags.quality_ind);
    sv_useds.append(sv.flags.sv_used);
    healths.append(sv.flags.health);
    diff_corrs.append(sv.flags.diff_corr);
    smoothedes.append(sv.flags.smoothed);
    orbit_sources.append(sv.flags.orbit_source);
    eph_avails.append(sv.flags.eph_avail);
    alm_avails.append(sv.flags.alm_avail);
    ano_avails.append(sv.flags.ano_avail);
    aop_avails.append(sv.flags.aop_avail);
    sbas_corr_useds.append(sv.flags.sbas_corr_used);
    rtcm_corr_useds.append(sv.flags.rtcm_corr_used);
    slas_corr_useds.append(sv.flags.slas_corr_used);
    spartn_corr_useds.append(sv.flags.spartn_corr_used);
    pr_corr_useds.append(sv.flags.pr_corr_used);
    cr_corr_useds.append(sv.flags.cr_corr_used);
    do_corr_useds.append(sv.flags.do_corr_used);
    clas_corr_useds.append(sv.flags.clas_corr_used);
  }

  ns["gnss_ids"] = gnss_ids;
  ns["sv_ids"] = sv_ids;
  ns["cnos"] = cnos;
  ns["elevs"] = elevs;
  ns["azims"] = azims;
  ns["pr_ress"] = pr_ress;
  ns["quality_inds"] = quality_inds;
  ns["sv_useds"] = sv_useds;
  ns["healths"] = healths;
  ns["diff_corrs"] = diff_corrs;
  ns["smoothedes"] = smoothedes;
  ns["orbit_sources"] = orbit_sources;
  ns["eph_avails"] = eph_avails;
  ns["alm_avails"] = alm_avails;
  ns["ano_avails"] = ano_avails;
  ns["aop_avails"] = aop_avails;
  ns["sbas_corr_useds"] = sbas_corr_useds;
  ns["rtcm_corr_useds"] = rtcm_corr_useds;
  ns["slas_corr_useds"] = slas_corr_useds;
  ns["spartn_corr_useds"] = spartn_corr_useds;
  ns["pr_corr_useds"] = pr_corr_useds;
  ns["cr_corr_useds"] = cr_corr_useds;
  ns["do_corr_useds"] = do_corr_useds;
  ns["clas_corr_useds"] = clas_corr_useds;

  return ns;
}

GPS_TRUST_NODE_LOCAL
void ubx_nav_sat_callback(const ublox_ubx_msgs::msg::UBXNavSat::SharedPtr msg)
{
  nav_sat_json_ = std::make_shared<Json::Value>(json_from_ubx_nav_sat(msg));

  auto json = *nav_sat_json_.get();

  using std::endl;

  // need to format our own json response here due to the arrayas
  std::ostringstream oss;
  oss << "{" << endl;
  oss << "\t" << std::quoted("frame_id") << " : " << json["frame_id"].toStyledString();
  oss << "\t" << std::quoted("timestamp") << " : " << json["timestamp"].toStyledString();
  oss << "\t" << std::quoted("itow") << " : " << json["itow"].toStyledString();
  oss << "\t" << std::quoted("num_svs") << " : " << json["num_svs"].toStyledString();

  oss << "\t" << flatten_json_array("gnss_ids", json["gnss_ids"]) << endl;
  oss << "\t" << flatten_json_array("sv_ids", json["sv_ids"]) << endl;
  oss << "\t" << flatten_json_array("cnos", json["cnos"]) << endl;
  oss << "\t" << flatten_json_array("elevs", json["elevs"]) << endl;
  oss << "\t" << flatten_json_array("azims", json["azims"]) << endl;
  oss << "\t" << flatten_json_array("pr_ress", json["azims"]) << endl;
  oss << "\t" << flatten_json_array("quality_inds", json["quality_inds"]) << endl;
  oss << "\t" << flatten_json_array("sv_useds", json["sv_useds"]) << endl;
  oss << "\t" << flatten_json_array("healths", json["healths"]) << endl;
  oss << "\t" << flatten_json_array("diff_corrs", json["diff_corrs"]) << endl;
  oss << "\t" << flatten_json_array("smoothedes", json["smoothedes"]) << endl;
  oss << "\t" << flatten_json_array("orbit_sources", json["orbit_sources"]) << endl;
  oss << "\t" << flatten_json_array("eph_avails", json["eph_avails"]) << endl;
  oss << "\t" << flatten_json_array("alm_avails", json["alm_avails"]) << endl;
  oss << "\t" << flatten_json_array("ano_avails", json["ano_avails"]) << endl;
  oss << "\t" << flatten_json_array("aop_avails", json["aop_avails"]) << endl;
  oss << "\t" << flatten_json_array("sbas_corr_useds", json["sbas_corr_useds"]) << endl;
  oss << "\t" << flatten_json_array("rtcm_corr_useds", json["rtcm_corr_useds"]) << endl;
  oss << "\t" << flatten_json_array("slas_corr_useds", json["slas_corr_useds"]) << endl;
  oss << "\t" << flatten_json_array("spartn_corr_useds", json["spartn_corr_useds"]) << endl;
  oss << "\t" << flatten_json_array("pr_corr_useds", json["pr_corr_useds"]) << endl;
  oss << "\t" << flatten_json_array("cr_corr_useds", json["cr_corr_useds"]) << endl;
  oss << "\t" << flatten_json_array("do_corr_useds", json["do_corr_useds"]) << endl;
  oss << "\t" << flatten_json_array("clas_corr_useds", json["clas_corr_useds"]) << endl;

  oss << "}" << endl;

  RCLCPP_DEBUG(get_logger(), "nav_sat_json: %s", oss.str().c_str());
}

GPS_TRUST_NODE_LOCAL
Json::Value json_from_ubx_rxm_rawx(const ublox_ubx_msgs::msg::UBXRxmRawx::SharedPtr msg)
{
  Json::Value json_stamp;
  json_stamp["sec"] = msg->header.stamp.sec;
  json_stamp["nanosec"] = msg->header.stamp.nanosec;

  Json::Value rr;
  rr["timestamp"] = json_stamp;
  rr["frame_id"] = msg->header.frame_id;

  rr["rcv_tow"] = msg->rcv_tow;
  rr["week"] = msg->week;
  rr["leap_s"] = msg->leap_s;
  rr["num_meas"] = msg->num_meas;

  Json::Value rec_stat;
  rec_stat["leap_sec"] = msg->rec_stat.leap_sec;
  rec_stat["clk_rest"] = msg->rec_stat.clk_reset;
  rr["rec_stat"] = rec_stat;

  rr["version"] = msg->version;

  Json::Value pr_mess;
  Json::Value cp_mess;
  Json::Value do_mess;
  Json::Value gnss_ids;
  Json::Value sv_ids;
  Json::Value sig_ids;
  Json::Value freq_ids;
  Json::Value locktimes;
  Json::Value c_nos;
  Json::Value pr_stdevs;
  Json::Value cp_stdevs;
  Json::Value do_stdevs;
  Json::Value pr_valids;
  Json::Value cp_valids;
  Json::Value half_cycs;
  Json::Value sub_half_cycs;


  for (size_t i = 0; i < msg->num_meas; i++) {
    auto rawx_data = msg->rawx_data[i];
    pr_mess.append(rawx_data.pr_mes);
    cp_mess.append(rawx_data.cp_mes);
    do_mess.append(rawx_data.do_mes);
    gnss_ids.append(rawx_data.gnss_id);
    sv_ids.append(rawx_data.sv_id);
    sig_ids.append(rawx_data.sig_id);
    freq_ids.append(rawx_data.freq_id);
    locktimes.append(rawx_data.locktime);
    c_nos.append(rawx_data.c_no);
    pr_stdevs.append(rawx_data.pr_stdev);
    cp_stdevs.append(rawx_data.cp_stdev);
    do_stdevs.append(rawx_data.do_stdev);
    pr_valids.append(rawx_data.trk_stat.pr_valid);
    cp_valids.append(rawx_data.trk_stat.cp_valid);
    half_cycs.append(rawx_data.trk_stat.half_cyc);
    sub_half_cycs.append(rawx_data.trk_stat.sub_half_cyc);
  }

  rr["pr_mess"] = pr_mess;
  rr["cp_mess"] = cp_mess;
  rr["do_mess"] = do_mess;
  rr["gnss_ids"] = gnss_ids;
  rr["sv_ids"] = sv_ids;
  rr["sig_ids"] = sig_ids;
  rr["freq_ids"] = freq_ids;
  rr["locktimes"] = locktimes;
  rr["c_nos"] = c_nos;
  rr["pr_stdevs"] = pr_stdevs;
  rr["cp_stdevs"] = cp_stdevs;
  rr["do_stdevs"] = do_stdevs;
  rr["pr_valids"] = pr_valids;
  rr["cp_valids"] = cp_valids;
  rr["half_cycs"] = half_cycs;
  rr["sub_half_cycs"] = sub_half_cycs;

  return rr;
}

GPS_TRUST_NODE_LOCAL
void ubx_rxm_rawx_callback(const ublox_ubx_msgs::msg::UBXRxmRawx::SharedPtr msg)
{
  rxm_rawx_json_ = std::make_shared<Json::Value>(json_from_ubx_rxm_rawx(msg));

  auto json = *rxm_rawx_json_.get();

  using std::endl;

  // need to format our own json response here due to the arrayas
  std::ostringstream oss;
  oss << "{" << endl;
  oss << "\t" << std::quoted("frame_id") << " : " << json["frame_id"].toStyledString();
  oss << "\t" << std::quoted("timestamp") << " : " << json["timestamp"].toStyledString();
  oss << "\t" << std::quoted("rcv_tow") << " : " << json["rcv_tow"].toStyledString();
  oss << "\t" << std::quoted("week") << " : " << json["week"].toStyledString();
  oss << "\t" << std::quoted("leap_s") << " : " << json["leap_s"].toStyledString();
  oss << "\t" << std::quoted("num_meas") << " : " << json["num_meas"].toStyledString();
  oss << "\t" << std::quoted("rec_stat") << " : " << json["rec_stat"].toStyledString();
  oss << "\t" << std::quoted("version") << " : " << json["version"].toStyledString();

  oss << "\t" << flatten_json_array("pr_mess", json["pr_mess"]) << endl;
  oss << "\t" << flatten_json_array("cp_mess", json["cp_mess"]) << endl;
  oss << "\t" << flatten_json_array("do_mess", json["do_mess"]) << endl;
  oss << "\t" << flatten_json_array("gnss_ids", json["gnss_ids"]) << endl;
  oss << "\t" << flatten_json_array("sv_ids", json["sv_ids"]) << endl;
  oss << "\t" << flatten_json_array("sig_ids", json["sig_ids"]) << endl;
  oss << "\t" << flatten_json_array("freq_ids", json["freq_ids"]) << endl;
  oss << "\t" << flatten_json_array("locktimes", json["locktimes"]) << endl;
  oss << "\t" << flatten_json_array("c_nos", json["c_nos"]) << endl;
  oss << "\t" << flatten_json_array("pr_stdevs", json["pr_stdevs"]) << endl;
  oss << "\t" << flatten_json_array("cp_stdevs", json["cp_stdevs"]) << endl;
  oss << "\t" << flatten_json_array("do_stdevs", json["do_stdevs"]) << endl;
  oss << "\t" << flatten_json_array("pr_valids", json["pr_valids"]) << endl;
  oss << "\t" << flatten_json_array("cp_valids", json["cp_valids"]) << endl;
  oss << "\t" << flatten_json_array("half_cycs", json["half_cycs"]) << endl;
  oss << "\t" << flatten_json_array("sub_half_cycs", json["sub_half_cycs"]) << endl;

  oss << "}" << endl;

  RCLCPP_DEBUG(get_logger(), "rxm_rawx_json: %s", oss.str().c_str());
}

GPS_TRUST_NODE_LOCAL
std::string compressString(const std::string & str)
{
  z_stream zs;
  zs.zalloc = Z_NULL;
  zs.zfree = Z_NULL;
  zs.opaque = Z_NULL;
  zs.avail_in = str.size();
  zs.next_in = reinterpret_cast<Bytef *>(const_cast<char *>(str.data()));

  if (deflateInit(&zs, Z_DEFAULT_COMPRESSION) != Z_OK) {
    throw std::runtime_error("Failed to initialize zlib deflate");
  }

  std::vector<Bytef> compressed_data;
  const size_t BUFSIZE = 128;
  Bytef outbuffer[BUFSIZE];

  do {
    zs.avail_out = BUFSIZE;
    zs.next_out = outbuffer;
    deflate(&zs, Z_FINISH);

    for (size_t i = 0; i < BUFSIZE - zs.avail_out; ++i) {
      compressed_data.push_back(outbuffer[i]);
    }
  } while (zs.avail_out == 0);

  deflateEnd(&zs);

  return std::string(compressed_data.begin(), compressed_data.end());
}

GPS_TRUST_NODE_LOCAL
// Function to decompress a gzip-compressed string
std::string decompressGzip(const std::string & compressed)
{
  std::vector<Bytef> decompressed;
  z_stream zs;
  zs.zalloc = Z_NULL;
  zs.zfree = Z_NULL;
  zs.opaque = Z_NULL;
  zs.avail_in = compressed.size();
  zs.next_in = reinterpret_cast<Bytef *>(const_cast<char *>(compressed.data()));

  if (inflateInit2(&zs, MAX_WBITS + 16) != Z_OK) {
    throw std::runtime_error("Failed to initialize zlib inflate");
  }

  const size_t BUFSIZE = 128;
  Bytef outbuffer[BUFSIZE];

  do {
    zs.avail_out = BUFSIZE;
    zs.next_out = outbuffer;
    inflate(&zs, Z_SYNC_FLUSH);

    for (size_t i = 0; i < BUFSIZE - zs.avail_out; ++i) {
      decompressed.push_back(outbuffer[i]);
    }
  } while (zs.avail_out == 0);

  inflateEnd(&zs);

  return std::string(decompressed.begin(), decompressed.end());
}

GPS_TRUST_NODE_LOCAL
Json::Value callAPI(Json::Value & json_request, bool use_gzip_encoding)
{
  CURL * curl;
  CURLcode res;
  std::string readBuffer;
  bool is_gzip = false;

  curl = curl_easy_init();
  if (curl) {
    // Enable gzip compressed response
    curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "gzip");

    curl_easy_setopt(curl, CURLOPT_URL, api_url_.c_str());
    // Add headers such as API Key
    struct curl_slist * headers = NULL;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    if (use_gzip_encoding) {
      headers = curl_slist_append(headers, "Content-Encoding: gzip");
    }

    curl_easy_setopt(curl, CURLOPT_MAXAGE_CONN, maxage_conn_);

    // Prepare the API key header
    std::string api_key_header = "x-api-key:" + api_key_;

    // Set the custom headers
    headers = curl_slist_append(headers, api_key_header.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    // Set the User-Agent
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "GPSTrustROS2Agent/1.0");

    // Set the POST option
    curl_easy_setopt(curl, CURLOPT_POST, 1L);

    // Add data
    std::string request_body = json_request.toStyledString();
    std::string compressed_body;

    if (use_gzip_encoding) {
      compressed_body = compressString(request_body);
      // Set compressed POST data
      curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, compressed_body.size());
      curl_easy_setopt(curl, CURLOPT_COPYPOSTFIELDS, compressed_body.data());
    } else {
      curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body.c_str());
    }

    // Receive data
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

    // Set header function and data
    curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, headerCallback);
    curl_easy_setopt(curl, CURLOPT_HEADERDATA, &is_gzip);

    res = curl_easy_perform(curl);

    if (res != CURLE_OK) {
      RCLCPP_ERROR(get_logger(), "curl_easy_perform() failed: %s", curl_easy_strerror(res));
    }

    curl_easy_cleanup(curl);

    if (readBuffer.empty()) {
      RCLCPP_WARN(get_logger(), "curl readBuffer: '%s'", readBuffer.c_str());
    } else {
      RCLCPP_DEBUG(get_logger(), "curl readBuffer: '%s'", readBuffer.c_str());
    }
  }


  Json::Value json_response;

  if (!readBuffer.empty()) {
    // Check the is_gzip flag to see if the content was gzip-encoded
    if (is_gzip) {
      // Decompress gzip-encoded content
      std::string decompressed_content = decompressGzip(readBuffer);
      std::istringstream response_stream(decompressed_content);
      response_stream >> json_response;
    } else {
      std::istringstream response_stream(readBuffer);
      response_stream >> json_response;
    }
  }

  return json_response;
}

GPS_TRUST_NODE_LOCAL
// Header callback function
static size_t headerCallback(char * buffer, size_t size, size_t nitems, void * userdata)
{
  // Calculate the real size of the incoming header
  size_t real_size = size * nitems;

  // Convert the header to a string for easier parsing
  std::string header(buffer, real_size);

  // Look for the Content-Encoding header
  if (header.find("Content-Encoding: gzip") != std::string::npos) {
    // Set flag or do something to indicate that the content is gzip-encoded
    bool * is_gzip = static_cast<bool *>(userdata);
    *is_gzip = true;
  }

  return real_size;
}

GPS_TRUST_NODE_LOCAL
// Function to be used with libcurl for writing received data into a string
static size_t writeCallback(void * contents, size_t size, size_t nmemb, void * userp)
{
  // Calculate the real size of the incoming data
  size_t real_size = size * nmemb;

  // Cast the pointer for the output buffer to std::string
  std::string * out_str = static_cast<std::string *>(userp);

  // Append the received data to the output string
  out_str->append(static_cast<char *>(contents), real_size);

  // Return the size of the data taken
  return real_size;
}

rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavHPPosLLH>::SharedPtr nav_llh_sub_;
rclcpp::Subscription<ublox_ubx_msgs::msg::UBXSecSig>::SharedPtr sec_sig_sub_;
rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavOrb>::SharedPtr nav_orb_sub_;
rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavSat>::SharedPtr nav_sat_sub_;
rclcpp::Subscription<ublox_ubx_msgs::msg::UBXRxmRawx>::SharedPtr rxm_rawx_sub_;

rclcpp::Publisher<gps_trust_msgs::msg::GPSTrustIndicator>::SharedPtr pub_;

std::string api_key_;
std::string api_url_;
std::string device_id_;

std::shared_ptr<Json::Value> sec_sig_json_;
std::shared_ptr<Json::Value> nav_orb_json_;
std::shared_ptr<Json::Value> nav_sat_json_;
std::shared_ptr<Json::Value> rxm_rawx_json_;

public:
GPS_TRUST_NODE_LOCAL
~GPSTrustNode()
{
  curl_global_cleanup();
  RCLCPP_INFO(this->get_logger(), "finished");
}
};
}  // namespace gps_trust

RCLCPP_COMPONENTS_REGISTER_NODE(gps_trust::GPSTrustNode)
