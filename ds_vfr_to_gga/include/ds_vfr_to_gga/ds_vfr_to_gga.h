//
// Created by jvaccaro on 8/20/19.
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

#ifndef PROJECT_DS_NAVSTATE_TO_GGA_H
#define PROJECT_DS_NAVSTATE_TO_GGA_H

#include <ds_base/ds_process.h>
#include <ds_asio/ds_connection.h>
#include <ds_core_msgs/RawData.h>

namespace ds_nav{

class VfrToGga : public ds_base::DsProcess {
  /// The purpose of this node is to receive Navest VFR messages over a connection,
  /// convert and pack them into GPGGA strings,
  /// then send them via UDP.
  /// Written originally for Kongsberg EM2040 integration with Jason
  /// Also publishes a NavAgg solution for later comparison
  DS_DISABLE_COPY(VfrToGga)

 public:
  VfrToGga();
  VfrToGga(int argc, char* argv[], const std::string& name);
  ~VfrToGga() override;

  static std::pair<bool, std::string> pack_vfr_message(const ds_core_msgs::RawData& msg
      , std::string soln="SOLN_DEADRECK", std::string beacon1="9", std::string beacon2="0");

  void _on_vfr_msg(const ds_core_msgs::RawData& msg);
  void _on_gga_msg(const ds_core_msgs::RawData& msg);

 protected:
  void setupConnections() override;

 private:
  boost::shared_ptr<ds_asio::DsConnection> m_gga_conn, m_vfr_conn;
  ros::Timer m_spitter;
  std::string m_soln;
  std::string m_beacon1;
  std::string m_beacon2;
  ros::Publisher m_nav_agg_pub;
//  bool m_is_spitter;
//  ros:Timer m_spitter_timer;
};

}



#endif //PROJECT_DS_NAVSTATE_TO_GGA_H
