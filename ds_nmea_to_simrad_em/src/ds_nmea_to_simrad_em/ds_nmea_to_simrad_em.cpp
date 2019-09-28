/**
* Copyright 2018 Woods Hole Oceanographic Institution
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

#include "ds_nmea_to_simrad_em/ds_nmea_to_simrad_em.h"
#include "ds_nmea_parsers/util.h"
#include "ds_nmea_parsers/Tro.h"
#include "ds_nmea_parsers/Hdt.h"
#include "ds_nmea_parsers/Inf.h"
#include "ds_nmea_msgs/Tro.h"
#include "ds_nmea_msgs/Hdt.h"
#include "ds_nmea_msgs/Inf.h"

namespace ds_nav{

NmeaToSimradEm::NmeaToSimradEm()
    : ds_base::DsProcess()
{
}

NmeaToSimradEm::NmeaToSimradEm(int argc, char* argv[], const std::string& name)
    : ds_base::DsProcess(argc, argv, name)
{
}

NmeaToSimradEm::~NmeaToSimradEm()
{
  m_spitter.stop();
}

std::string
NmeaToSimradEm::serialize_simrad_em(ds_sensor_parsers::simrad_em em)
{
  ds_sensor_parsers::simrad_em* em_ptr(&em);
  auto size = sizeof(em);
  auto ptr = reinterpret_cast<uint8_t*>(em_ptr);
  std::string str(ptr, ptr+size);
//  ROS_ERROR_STREAM("STRING OUTPUT: " << str);
  return str;
}

void
NmeaToSimradEm::_on_nmea_msg(const ds_core_msgs::RawData& msg)
{
  // Convert to string
  std::string nmea_string(msg.data.data(), msg.data.data()+msg.data.size());
//  ROS_ERROR_STREAM("RECEIVED: " << nmea_string);
  // CHECK FOR HEADING
  auto hdt = ds_nmea_msgs::Hdt{};
  if (ds_nmea_msgs::from_string(hdt, nmea_string)){
    m_simrad_em.heading = 100*hdt.heading;
    m_heading_received = true;
//    ROS_ERROR_STREAM("HEADING RECEIVED!");
  }

  // CHECK FOR PITCH/ROLL
  auto tro = ds_nmea_msgs::Tro{};
  if (ds_nmea_msgs::from_string(tro, nmea_string)){
    m_simrad_em.pitch = 100*tro.pitch_deg;
    m_simrad_em.roll = 100*tro.roll_deg;
    m_pitch_roll_received = true;
//    ROS_ERROR_STREAM("PITCH ROLL RECEIVED!");
  }

  // CHECK FOR STATUS
  auto inf = ds_nmea_msgs::Inf{};
  if (ds_nmea_msgs::from_string(inf, nmea_string)){
//    ROS_ERROR_STREAM("STATUS RECEIVED!");
    if (!m_ignore_status){
      m_heading_received &= !inf.heading_invalid;
      m_pitch_roll_received &= !inf.pitch_invalid;
      m_pitch_roll_received &= !inf.roll_invalid;
//      ROS_ERROR_STREAM("STATUS READ: HDG "<< !inf.heading_invalid << " PITCH "
//                                          << !inf.pitch_invalid << " ROLL " << !inf.roll_invalid);
    }
    if (m_heading_received && m_pitch_roll_received){
      auto out_message = serialize_simrad_em(m_simrad_em);
//      ROS_ERROR_STREAM("HPR: "<< m_simrad_em.heading/100.0 << " " << m_simrad_em.pitch/100.0 << " " << m_simrad_em.roll/100.0);
      m_em2040_conn->send(out_message);
      auto gyro = ds_sensor_parsers::simrad_to_gyro(m_simrad_em);
      gyro.header = msg.header;
      gyro.ds_header = msg.ds_header;
      m_gyro_pub.publish(gyro);
    } else {
//      ROS_ERROR_STREAM("HEADING OR PITCH NOT RECEIVED, NOT PUBLISHING");
    }
    m_heading_received = false;
    m_pitch_roll_received = false;
//    ROS_ERROR_STREAM("RESET HEADING AND PITCH TO RECEIVE");
  }

  // Other messages?
}

void
NmeaToSimradEm::_on_em2040_msg(const ds_core_msgs::RawData& msg)
{
  // DO NOTHING
}

void
NmeaToSimradEm::setupConnections()
{
  ds_base::DsProcess::setupConnections();
  m_nmea_conn = addConnection("nmea", boost::bind(&NmeaToSimradEm::_on_nmea_msg, this, _1));
  m_em2040_conn = addConnection("em2040", boost::bind(&NmeaToSimradEm::_on_em2040_msg, this, _1));
  m_simrad_em = ds_sensor_parsers::simrad_em{};
  m_simrad_em.sensor_status = 0x90;
  m_simrad_em.synch_byte = 0x90;
  m_ignore_status = ros::param::param<bool>("ignore_status", true);
  m_is_spitter = ros::param::param<bool>("~is_spitter", false);
  m_spitter = nodeHandle().createTimer(ros::Duration(0.05), &NmeaToSimradEm::_on_spitter_timer, this);
  m_spitter.start();
  auto gyro_topic = ros::param::param<std::string>("~gyro_topic", "gyro");
  m_gyro_pub = nodeHandle().advertise<ds_sensor_msgs::Gyro>(gyro_topic, 1000);
}

void
NmeaToSimradEm::_on_spitter_timer(const ros::TimerEvent&){
  if (m_is_spitter){
    m_simrad_em.heading = 0;
    m_simrad_em.pitch = 50;
    m_simrad_em.roll = -50;
    ROS_ERROR_STREAM("SPITTER: "<< m_simrad_em.heading/100.0 << " " << m_simrad_em.pitch/100.0 << " " << m_simrad_em.roll/100.0);
    auto out_message = serialize_simrad_em(m_simrad_em);
    m_em2040_conn->send(out_message);
  }
}

} //namespace
