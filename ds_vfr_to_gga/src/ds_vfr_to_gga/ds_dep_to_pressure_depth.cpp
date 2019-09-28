//
// Created by jvaccaro on 9/6/19.
//
/**
* Copyright 2019 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include "ds_vfr_to_gga/ds_dep_to_pressure_depth.h"

namespace ds_nav {

DepToDepthPressure::DepToDepthPressure()
    : ds_base::DsProcess() {

}

DepToDepthPressure::DepToDepthPressure(int argc, char *argv[], const std::string &name)
    : ds_base::DsProcess(argc, argv, name) {

}

DepToDepthPressure::~DepToDepthPressure() = default;

std::pair<bool, ds_sensor_msgs::DepthPressure>
DepToDepthPressure::parse_raw(const ds_core_msgs::RawData &msg) {
  auto dep_msg = ds_sensor_msgs::DepthPressure{};
  std::string dep_string(msg.data.data(), msg.data.data() + msg.data.size());
  std::vector<std::string> dep_list;
  boost::split(dep_list, dep_string, boost::is_any_of(" "));
  if (dep_list.size() != 7) {
    return {false, dep_msg};
  }
  if (dep_list[0] != "DEP") {
    return {false, dep_msg};
  }
  if (dep_list[3] != "Jason") {
    return {false, dep_msg};
  }
  if (dep_list[4] != "0") {
    return {false, dep_msg};
  }

  auto dep = std::stod(dep_list[5]);
  auto raw = std::stod(dep_list[7]);

  dep_msg.pressure_raw = raw;
  dep_msg.pressure = raw;
  dep_msg.tare = dep_msg.DEPTH_PRESSURE_NO_DATA;
  dep_msg.depth = dep;
  dep_msg.latitude = dep_msg.DEPTH_PRESSURE_NO_DATA;
  dep_msg.pressure_raw_unit = dep_msg.UNIT_PRESSURE_PSI;
  dep_msg.header = msg.header;
  dep_msg.header.stamp = ros::Time::now();
  dep_msg.ds_header = msg.ds_header;

  return {true, dep_msg};
}

void
DepToDepthPressure::_on_navest_msg(const ds_core_msgs::RawData &msg) {
  bool ok = false;
  ds_sensor_msgs::DepthPressure dep{};
  std::tie(ok, dep) = parse_raw(msg);
  if (ok) {
    m_depthpressure.publish(dep);
  }
}

void
DepToDepthPressure::setupConnections() {
  ds_base::DsProcess::setupConnections();
  m_navest_conn = addConnection("navest", boost::bind(&DepToDepthPressure::_on_navest_msg, this, _1));
  std::string depthpressure_topic = ros::param::param<std::string>("~depthpressure", "depthpressure");
  m_depthpressure = nodeHandle().advertise<ds_sensor_msgs::DepthPressure>(depthpressure_topic, 1000);
}

} //namespace ds_nav