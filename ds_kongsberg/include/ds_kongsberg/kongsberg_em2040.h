//
// Created by jvaccaro on 4/11/19.
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

#ifndef SENTRY_WS_KONGSBERGEM2040_H
#define SENTRY_WS_KONGSBERGEM2040_H

#include <ds_base/ds_process.h>

#include <ds_core_msgs/RawData.h>
#include "../../src/EM_datagrams/EMdgmFormat.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include "ds_kongsberg_msgs/PingCmd.h"
#include "ds_kongsberg_msgs/PowerCmd.h"
#include "ds_kongsberg_msgs/SettingsCmd.h"
#include "ds_kongsberg_msgs/BistCmd.h"
#include "ds_kongsberg_msgs/LoadXmlCmd.h"
#include "ds_kongsberg_msgs/KongsbergStatus.h"

#include <ds_multibeam_msgs/MultibeamFilterStats.h>
#include <ds_multibeam_msgs/MultibeamGrid.h>
#include <ds_multibeam_msgs/MultibeamGridStats.h>
#include <ds_multibeam_msgs/MultibeamRaw.h>

namespace ds_kongsberg{
struct KongsbergEM2040Private;

class KongsbergEM2040 : public ds_base::DsProcess {
  DS_DECLARE_PRIVATE(KongsbergEM2040)
  DS_DISABLE_COPY(KongsbergEM2040)

 public:
  KongsbergEM2040();
  KongsbergEM2040(int argc, char* argv[], const std::string& name);
  ~KongsbergEM2040() override;

  bool parse_data(ds_core_msgs::RawData& raw);
  bool parse_message(ds_core_msgs::RawData& raw);
  bool read_kmall_dgm_from_kctrl(int type, ds_core_msgs::RawData& raw);
  bool parse_ipu(std::vector<std::string> fields);

  static std::pair<bool, EMdgmMRZ> read_mrz(uint8_t* bytes, int max_length);
  bool read_bist_result(ds_core_msgs::RawData& raw);
  uint8_t read_good_bad_missing(std::string);
//  static EMdgmMWC read_mwc(uint8_t* bytes);
//
  static ds_multibeam_msgs::MultibeamRaw mrz_to_mb_raw(EMdgmMRZ* msg);
//  static sensor_msgs::Image mwc_to_image(EMdgmMWC* msg);
//  static sensor_msgs::PointCloud2 mb_raw_to_pointcloud(ds_multibeam_msgs::MultibeamRaw* msg);
  void mbraw_to_kmstatus(ds_multibeam_msgs::MultibeamRaw raw);

 protected:
  void setupServices() override;
  void setupPublishers() override;
  void setupParameters() override;
  void setupConnections() override;
  void setupTimers() override;

 private:
  bool _ping_cmd(ds_kongsberg_msgs::PingCmd::Request &req, ds_kongsberg_msgs::PingCmd::Response &res);
  bool _power_cmd(ds_kongsberg_msgs::PowerCmd::Request &req, ds_kongsberg_msgs::PowerCmd::Response &res);
  bool _settings_cmd(ds_kongsberg_msgs::SettingsCmd::Request &req, ds_kongsberg_msgs::SettingsCmd::Response &res);
  bool _bist_cmd(ds_kongsberg_msgs::BistCmd::Request &req, ds_kongsberg_msgs::BistCmd::Response &res);
  bool _load_xml_cmd(ds_kongsberg_msgs::LoadXmlCmd::Request &req, ds_kongsberg_msgs::LoadXmlCmd::Response &res);

  void _on_kmall_data(ds_core_msgs::RawData raw);
  void _on_kctrl_data(ds_core_msgs::RawData raw);

  std::string _send_kctrl_command(int cmd);
  template <class T1>
  std::string _send_kctrl_param(std::string param, T1 param_val);
  template <class T1>
  std::string _send_kctrl_param(std::vector<std::string> params, std::vector<T1> vals);
  void _startup_sequence();
  void _print_bist(std::string name, std::string status, std::string msg);
  void _run_next_bist();
  void _new_kmall_file();
  void _write_kmall_data(ds_core_msgs::RawData& raw);
  void _write_kctrl_xml(ds_core_msgs::RawData& raw);
  std::string _read_kctrl_xml(std::string filename);
  void _send_xml_param(ds_core_msgs::RawData& raw);
  void _on_kctrl_timeout(const ros::TimerEvent&);
  void _on_kmall_timeout(const ros::TimerEvent&);
  void _on_pu_powered_timeout(const ros::TimerEvent&);
  void _on_pu_connected_timeout(const ros::TimerEvent&);
  void _on_pinging_timeout(const ros::TimerEvent&);

  std::unique_ptr<KongsbergEM2040Private> d_ptr_;
};

}

#endif //SENTRY_WS_KONGSBERGEM2040_H
