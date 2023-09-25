// Copyright 2023 Australian Robotics Supplies & Technology
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/header.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_hp_pos_llh.hpp"
#include "rtcm_msgs/msg/message.hpp"
#include "gps_trust_node/visibility_control.h"
#include "gps_trust_msgs/msg/gps_trust_indicator.hpp"

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
    // Subscriber
    sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavHPPosLLH>(
      "/ubx_nav_hp_pos_llh", 10,
      std::bind(&GPSTrustNode::ubxCallback, this, std::placeholders::_1));

    // Publisher
    pub_ =
      this->create_publisher<gps_trust_msgs::msg::GPSTrustIndicator>("gps_trust_indicator", 10);

    // Get GPS Trust Host URL
    this->declare_parameter("GPS_TRUST_API_URL", "http://localhost:8080/");
    this->get_parameter("GPS_TRUST_API_URL", api_url_);

    // Get API Key from parameter
    this->declare_parameter("GPS_TRUST_API_KEY", "demo_api_key");
    this->get_parameter("GPS_TRUST_API_KEY", api_key_);
  }

private:
  GPS_TRUST_NODE_LOCAL
  void ubxCallback(const ublox_ubx_msgs::msg::UBXNavHPPosLLH::SharedPtr msg)
  {

    // Extract the LLH and high-precision components
    double lat = msg->lat * 1e-7 + msg->lat_hp * 1e-9;
    double lon = msg->lon * 1e-7 + msg->lon_hp * 1e-9;
    double height = msg->height * 1e-3 + msg->height_hp * 1e-4;
    double hmsl = msg->hmsl * 1e-3 + msg->hmsl_hp * 1e-4;
    double h_acc = msg->h_acc * 1e-3;
    double v_acc = msg->v_acc * 1e-3;

    // Create JSON object to send to the API
    Json::Value json_request;

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

    json_request["llh"] = llh;

    RCLCPP_INFO(get_logger(), "json_request: %s", json_request.toStyledString().c_str());

    // Call API and get the response
    Json::Value json_response = callAPI(json_request, false);

    RCLCPP_INFO(get_logger(), "json_response: %s", json_response.toStyledString().c_str());


    // Decode and publish the GPS trust indicator
    gps_trust_msgs::msg::GPSTrustIndicator gps_trust_msg;
    gps_trust_msg.header.stamp = this->now();
    gps_trust_msg.trust_level = json_response["trust_level"].asInt();
    gps_trust_msg.status = json_response["status"].asString();
    pub_->publish(gps_trust_msg);
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

      std::string api_key_header = "API_KEY: " + api_key_;
      headers = curl_slist_append(headers, api_key_header.c_str());
      curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

      // Set the POST option
      curl_easy_setopt(curl, CURLOPT_POST, 1L);

      // Add data
      std::string request_body = json_request.toStyledString();
      std::string compressed_body = compressString(request_body);

      if (use_gzip_encoding) {
        compressed_body = compressString(request_body);
        // Set compressed POST data
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, compressed_body.size());
        curl_easy_setopt(curl, CURLOPT_COPYPOSTFIELDS, compressed_body.data());
      } else {
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body.c_str());
      }

      // Receive data
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

      // Set header function and data
      curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, headerCallback);
      curl_easy_setopt(curl, CURLOPT_HEADERDATA, &is_gzip);

      res = curl_easy_perform(curl);

      if (res != CURLE_OK) {
        fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
      }

      curl_easy_cleanup(curl);

      RCLCPP_INFO(get_logger(), "curl readBuffer: %s", readBuffer.c_str());
    }

    Json::Value json_response;

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
  static size_t WriteCallback(void * contents, size_t size, size_t nmemb, void * userp)
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

  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavHPPosLLH>::SharedPtr sub_;
  rclcpp::Publisher<gps_trust_msgs::msg::GPSTrustIndicator>::SharedPtr pub_;
  std::string api_key_;
  std::string api_url_;

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
