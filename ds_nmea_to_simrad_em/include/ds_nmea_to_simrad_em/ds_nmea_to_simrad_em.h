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

//
// Created by jvaccaro on 9/2/19.
//

#ifndef PROJECT_DS_NMEA_TO_SIMRAD_EM_H
#define PROJECT_DS_NMEA_TO_SIMRAD_EM_H

#include "ds_base/ds_process.h"
#include <ds_asio/ds_connection.h>
#include "ds_sensor_parsers/SimradEM.h"

namespace ds_nav {

class NmeaToSimradEm : public ds_base::DsProcess {
  /// The purpose of this node is to receive Octans STD NMEA strings and convert them to Simrad EM.
  /// This node was created for the jason integration of the Kongsberg EM2040 multibeam
  /// Also publishes a Gyro message for a Jason nav solution
  DS_DISABLE_COPY(NmeaToSimradEm)

 public:
  NmeaToSimradEm();
  NmeaToSimradEm(int argc, char *argv[], const std::string &name);
  ~NmeaToSimradEm() override;

  static std::string serialize_simrad_em(ds_sensor_parsers::simrad_em);

  void _on_nmea_msg(const ds_core_msgs::RawData &msg);
  void _on_em2040_msg(const ds_core_msgs::RawData &msg);
  void _on_spitter_timer(const ros::TimerEvent&);

 protected:
  void setupConnections() override;

 private:
  boost::shared_ptr<ds_asio::DsConnection> m_nmea_conn;
  boost::shared_ptr<ds_asio::DsConnection> m_em2040_conn;
  ds_sensor_parsers::simrad_em m_simrad_em;
  bool m_ignore_status = false;
  bool m_heading_received = false;
  bool m_pitch_roll_received = false;
  bool m_is_spitter = false;
  ros::Timer m_spitter;
  ros::Publisher m_gyro_pub;
};

} //namespace

#endif //PROJECT_DS_NMEA_TO_SIMRAD_EM_H
