//
// Created by jvaccaro on 5/10/19.
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

#ifndef SENTRY_WS_KONGSBERGEM2040_PRIVATE_H
#define SENTRY_WS_KONGSBERGEM2040_PRIVATE_H

#include "ds_kongsberg/kongsberg_em2040.h"

namespace ds_kongsberg{
struct KongsbergEM2040Private{
  ros::ServiceServer ping_srv_;
  ros::ServiceServer power_srv_;
  ros::ServiceServer settings_srv_;
  ros::ServiceServer values_srv_;
  ros::ServiceServer bist_srv_;

  ros::Publisher mbraw_pub_;
  ros::Publisher mbfilter_pub_;
//  ros::Publisher mbgrid_pub_;
//  ros::Publisher mbgridstats_pub_;
  ros::Publisher watercolumn_pub_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher offset_pub_;

  ros::Publisher kmstatus_pub_;

  // UDP connection to data stream
  boost::shared_ptr<ds_asio::DsConnection> kmall_conn_;

  // UDP connection to kctrl for sending/receiving commands
  boost::shared_ptr<ds_asio::DsConnection> kctrl_conn_;

  std::string sounder_name_;

  int latest_soundspeed_;
  bool started_;

  //BIST VALUES... maybe there's a better way?
  bool bist_running = false;
  int bist_progress = 0;
  std::vector<std::string> bist_tests;
  std::string bist_filename;
  std::string bist_filename_base;
};
}
#endif //SENTRY_WS_KONGSBERGEM2040_PRIVATE_H
