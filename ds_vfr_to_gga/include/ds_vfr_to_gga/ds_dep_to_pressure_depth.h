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

#ifndef PROJECT_DS_DEP_TO_PRESSURE_DEPTH_H
#define PROJECT_DS_DEP_TO_PRESSURE_DEPTH_H

#include "ds_base/ds_process.h"
#include "ds_asio/ds_connection.h"
#include "ds_sensor_msgs/DepthPressure.h"

namespace ds_nav {

class DepToDepthPressure : public ds_base::DsProcess {

  DS_DISABLE_COPY(DepToDepthPressure)
 public:
  DepToDepthPressure();
  DepToDepthPressure(int argc, char *argv[], const std::string& name);
  ~DepToDepthPressure() override;
  static std::pair<bool, ds_sensor_msgs::DepthPressure> parse_raw(const ds_core_msgs::RawData &msg);

  void _on_navest_msg(const ds_core_msgs::RawData &msg);

 protected:
  void setupConnections() override;

 private:
  boost::shared_ptr<ds_asio::DsConnection> m_navest_conn;
  ros::Publisher m_depthpressure;
};
}

#endif //PROJECT_DS_DEP_TO_PRESSURE_DEPTH_H
