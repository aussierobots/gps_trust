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
#include "ublox_ubx_msgs/msg/ubx_nav_orb.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_sat.hpp"
#include "ublox_ubx_msgs/msg/ubx_sec_sig.hpp"
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

    auto qos = rclcpp::SensorDataQoS();
    // Subscriber
    nav_llh_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavHPPosLLH>(
      "/ubx_nav_hp_pos_llh", qos,
      std::bind(&GPSTrustNode::ubx_nav_llh_callback, this, std::placeholders::_1));

    sec_sig_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXSecSig>(
      "/ubx_sec_sig", qos,
      std::bind(&GPSTrustNode::ubx_sec_sig_callback, this, std::placeholders::_1));

    nav_orb_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavOrb>(
      "/ubx_nav_orb", qos,
      std::bind(&GPSTrustNode::ubx_nav_orb_callback, this, std::placeholders::_1));

    nav_sat_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavSat>(
      "/ubx_nav_sat", qos,
      std::bind(&GPSTrustNode::ubx_nav_sat_callback, this, std::placeholders::_1));

    // Publisher
    pub_ =
      this->create_publisher<gps_trust_msgs::msg::GPSTrustIndicator>("gps_trust_indicator", qos);

    // Get GPS Trust Host URL
    this->declare_parameter(
      "GPS_TRUST_API_URL",
      // "http://mbp.local:9000/lambda-url/gps-trust-api"
      "https://gtapi.aussierobots.com.au/gps-trust-api"
      );
    this->get_parameter("GPS_TRUST_API_URL", api_url_);

    RCLCPP_INFO(this->get_logger(), "api_url: '%s'", api_url_.c_str());

    // Get API Key from parameter
    this->declare_parameter("GPS_TRUST_DEVICE_API_KEY", "demo_api_key");
    this->get_parameter("GPS_TRUST_DEVICE_API_KEY", api_key_);

    RCLCPP_INFO(get_logger(), "api_key: '%s'", api_key_.c_str());
  }

private:
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

  rclcpp::Publisher<gps_trust_msgs::msg::GPSTrustIndicator>::SharedPtr pub_;

  std::string api_key_;
  std::string api_url_;
  std::string device_id_;

  std::shared_ptr<Json::Value> sec_sig_json_;
  std::shared_ptr<Json::Value> nav_orb_json_;
  std::shared_ptr<Json::Value> nav_sat_json_;

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
