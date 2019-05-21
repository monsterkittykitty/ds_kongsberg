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
//#include "EM_datagrams/EMdgmFormat.h"
#include <sensor_msgs/Image.h>

#include "ds_kongsberg_msgs/PingCmd.h"
#include "ds_kongsberg_msgs/PowerCmd.h"
#include "ds_kongsberg_msgs/SettingsCmd.h"
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

//  void parse_message(ds_core_msgs::RawData& raw);
//
//  static EMdgmMRZ read_mrz(uint8_t* bytes);
//  static EMdgmMWC read_mwc(uint8_t* bytes);
//
//  static ds_multibeam_msgs::MultibeamRaw mrz_to_mb_raw(EMdgmMRZ* msg);
//  static ds_multibeam_msgs::MultibeamGrid mb_raw_to_mb_grid(ds_multibeam_msgs::MultibeamRaw* msg);
//  static ds_multibeam_msgs::MultibeamGridStats mb_raw_to_mb_grid_stats(ds_multibeam_msgs::MultibeamRaw* msg);
//  static ds_multibeam_msgs::MultibeamFilterStats mb_raw_to_mb_filter_stats(ds_multibeam_msgs::MultibeamRaw* msg);
//  static sensor_msgs::Image mwc_to_image(EMdgmMWC* msg);

 protected:
  void setupServices() override;
  void setupSubscriptions() override;
  void setupPublishers() override;
  void setupParameters() override;
  void setupConnections() override;

 private:
  bool _ping_cmd(ds_kongsberg_msgs::PingCmd::Request req, ds_kongsberg_msgs::PingCmd::Response res);
  bool _power_cmd(ds_kongsberg_msgs::PowerCmd::Request req, ds_kongsberg_msgs::PowerCmd::Response res);
  bool _settings_cmd(ds_kongsberg_msgs::SettingsCmd::Request req, ds_kongsberg_msgs::SettingsCmd::Response res);
  void onKMallData(ds_core_msgs::RawData raw);
  void onKctrlData(ds_core_msgs::RawData raw);

  std::unique_ptr<KongsbergEM2040Private> d_ptr_;
};

}

#endif //SENTRY_WS_KONGSBERGEM2040_H
