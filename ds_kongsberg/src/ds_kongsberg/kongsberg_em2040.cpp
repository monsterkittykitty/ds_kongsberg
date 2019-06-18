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

#include "ds_kongsberg/kongsberg_em2040.h"
#include "kongsberg_em2040_private.h"
#include "ds_kongsberg_msgs/KongsbergKSSIS.h"
#include "kongsberg_em2040_strings.h"
#include "ds_core_msgs/ClockOffset.h"
#include <regex>
#include "boost/date_time/posix_time/posix_time_types.hpp"
#include "ds_util/int_to_hex.h"

namespace ds_kongsberg{

KongsbergEM2040::KongsbergEM2040()
    : DsProcess()
    , d_ptr_(std::unique_ptr<KongsbergEM2040Private>(new KongsbergEM2040Private))
{
}

KongsbergEM2040::KongsbergEM2040(int argc, char* argv[], const std::string& name)
    : DsProcess(argc, argv, name)
    , d_ptr_(std::unique_ptr<KongsbergEM2040Private>(new KongsbergEM2040Private))
{
}

KongsbergEM2040::~KongsbergEM2040() = default;

// XXXXXXXXXXXXX
// KCtrl Parsers
// -------------
bool
KongsbergEM2040::parse_message(ds_core_msgs::RawData& raw)
{
  DS_D(KongsbergEM2040);
  ds_kongsberg_msgs::KongsbergKSSIS msg;
  // Split on commas and pull specific indexes
  auto str = std::string{ reinterpret_cast<const char*>(raw.data.data()), raw.data.size() };
  auto buf = std::istringstream{ str };
  std::vector<std::string> fields;

  while( buf.good() ) {
    std::string substr;
    getline( buf, substr, ',' );
    fields.push_back( substr );
  }

  if (fields.size() < 3){
    ROS_ERROR_STREAM("Msg too short : "<<str);
    return false;
  }
  if (fields[0] != "$KSSIS"){
    ROS_ERROR_STREAM("Msg doesn't contain $KSSIS : "<<fields[0]);
    return false;
  }

  msg.type = std::stoi(fields[1]);
  switch (msg.type){
    case K_TO_SIS::KCTRL_VERSION :
      if (fields[2] == "KCTRL_VER="){
        break;
      }
    case K_TO_SIS::STATUS_IPI : {
      std::stringstream ipi(fields[2]);
      fields.pop_back();
      while( ipi.good() )
      {
        std::string substr;
        getline( ipi, substr, '~' );
        if (substr.length()>0)
          fields.push_back( substr );
      }
      if (!d->started_){
        _startup_sequence();
      }
      break;
    }
    case K_TO_SIS::BIST_RESULT : {
      read_bist_result(raw);
    }
    default :
      if (fields[2] != d->sounder_name_){
        ROS_ERROR_STREAM("Sounder name doesn't match : "<<fields[2]);
        return false;
      }
      break;
  }

  msg.header = raw.header;
  msg.ds_header = raw.ds_header;
  msg.payload.resize(fields.size() - 2);
  for (int i=0; i<msg.payload.size(); i++){
    msg.payload[i] = fields[i+2];
  }
  d->kmstatus_pub_.publish(msg);
  return true;
}
bool
KongsbergEM2040::read_bist_result(ds_core_msgs::RawData& raw)
{
  bist_result r;
  uint8_t* ptr = raw.data.data();

  DS_D(KongsbergEM2040);
  std::string front = "$KSSIS," + std::to_string(K_TO_SIS::BIST_RESULT) + "," + d->sounder_name_ + ",";
  auto index = front.length();
  auto max_index = raw.data.size();

  r.result_code = *reinterpret_cast<uint16_t*>(ptr + index);
  index += sizeof(r.result_code);
  r.zeros = *reinterpret_cast<uint16_t*>(ptr + index);
  index += sizeof(r.zeros);
  uint8_t ibr[4];
  for (int i=0; i<4; i++)
    ibr[i] = (ptr + index)[i];
  r.ibr = std::string(ibr, ibr + sizeof(ibr)/sizeof(ibr[0]));
  index += sizeof(ibr);
  for (int i=0; i<18; i++)
    r.code[i] = (ptr + index)[i];
  index += sizeof(r.code);

  auto str = std::string{ reinterpret_cast<const char*>(ptr+index), max_index - index };
  auto buf = std::istringstream{ str };
  std::vector<std::string> fields;//=

  while( buf.good() )
  {
    std::string substr;
    getline( buf, substr, '\n' );
    r.msg_lines.push_back( substr );
  }

  if (d->bist_running){
    r.bist_type = d->bist_tests[d->bist_progress];
  } else {
    r.bist_type = r.msg_lines[0];
  }

  r.read_result_code();

  ROS_ERROR_STREAM(" >> Result " << r.bist_type << " : "<< r.pass << " code: "
                                 << ds_util::int_to_32_hex(r.result_code));

  if (d->bist_running){
    r.print_msg(d->bist_filename);
    _run_next_bist();
  }
}

// XXXXXXXXXXXXX
// KMAll parsers
// -------------
bool
KongsbergEM2040::parse_data(ds_core_msgs::RawData& raw)
{
  DS_D(KongsbergEM2040);
  int data_size = raw.data.size();
  int min_size = sizeof(EMdgmHeader);
  if (data_size < min_size){
    ROS_ERROR_STREAM("Raw Data received less than minimum size "<<min_size<<" bytes");
    return false;
  }
  uint8_t* bytes_ptr = raw.data.data();
  auto hdr = reinterpret_cast<EMdgmHeader*>(bytes_ptr);
  if (data_size != hdr->numBytesDgm){
    ROS_ERROR_STREAM("Raw Data size received ("<<data_size<<") != hdr->numBytes ("<<hdr->numBytesDgm<<")");
//    return false;
  }

  std::string msg_type(hdr->dgmType, hdr->dgmType + sizeof(hdr->dgmType)/sizeof(hdr->dgmType[0]));
  uint8_t* raw_array = reinterpret_cast<uint8_t*>(raw.data.data());
  std::string raw_str(raw_array, raw_array + sizeof(raw_array));
  if (msg_type==EM_DGM_I_INSTALLATION_PARAM)
  {
    ROS_INFO_STREAM("EM_DGM_I_INSTALLATION_PARAM");
    auto msg = reinterpret_cast<dgm_IIP*>(bytes_ptr);
    return true;
  }
  else if (msg_type==EM_DGM_I_OP_RUNTIME)
  {
    ROS_ERROR_STREAM("EM_DGM_I_OP_RUNTIME");
    auto msg = reinterpret_cast<dgm_IOP*>(bytes_ptr);
    ROS_ERROR_STREAM("  RAW: " << raw_str);
    return true;
  }
  else if (msg_type==EM_DGM_S_POSITION){
    ROS_ERROR_STREAM("EM_DGM_S_POSITION");
    auto msg = reinterpret_cast<EMdgmSPO*>(bytes_ptr);
    return true;
  }
  else if (msg_type==EM_DGM_S_KM_BINARY){
    ROS_ERROR_STREAM("EM_DGM_S_KM_BINARY");
    auto msg = reinterpret_cast<EMdgmSKM*>(bytes_ptr);
    return true;
  }
  else if (msg_type==EM_DGM_S_SOUND_VELOCITY_PROFILE){
    ROS_ERROR_STREAM("EM_DGM_S_SOUND_VELOCITY_PROFILE");
    auto msg = reinterpret_cast<EMdgmSVP*>(bytes_ptr);
    return true;
  }
  else if (msg_type==EM_DGM_S_SOUND_VELOCITY_TRANSDUCER){
    ROS_ERROR_STREAM("EM_DGM_S_SOUND_VELOCITY_TRANSDUCER");
  }
  else if (msg_type==EM_DGM_S_CLOCK){
    ROS_INFO_STREAM("EM_DGM_S_CLOCK");
    auto msg = reinterpret_cast<EMdgmSCL*>(bytes_ptr);
    ds_core_msgs::ClockOffset offset;
    offset.header.stamp = ros::Time::now();
    offset.ds_header = raw.ds_header;
    offset.device_stamp_minus_ros_stamp_sec = msg->sensData.offset_sec;
    d->offset_pub_.publish(offset);
    return true;
  }
  else if (msg_type==EM_DGM_S_DEPTH){
    ROS_ERROR_STREAM("EM_DGM_S_DEPTH");
    return false;
  }
  else if (msg_type==EM_DGM_S_HEIGHT){
    ROS_ERROR_STREAM("EM_DGM_S_HEIGHT");
    return false;
  }
  else if (msg_type==EM_DGM_M_RANGE_AND_DEPTH){
    ROS_ERROR_STREAM("EM_DGM_M_RANGE_AND_DEPTH");
    EMdgmMRZ mrz;
    bool ok = false;
    std::tie(ok, mrz) = read_mrz(bytes_ptr,data_size);
    if (ok){
      auto mbr = mrz_to_mb_raw(&mrz);
      mbr.ds_header = raw.ds_header;
      d->mbraw_pub_.publish(mbr);
    }
    return ok;
  }
  else if (msg_type==EM_DGM_M_WATER_COLUMN){
    ROS_ERROR_STREAM("EM_DGM_M_WATER_COLUMN");
    return false;
  }
  else if (msg_type==EM_DGM_C_POSITION){
    ROS_ERROR_STREAM("EM_DGM_C_POSITION");
    auto msg = reinterpret_cast<EMdgmCPO*>(bytes_ptr);
    return true;
  }
  else if (msg_type==EM_DGM_C_HEAVE){
    ROS_ERROR_STREAM("EM_DGM_C_HEAVE");
    return true;
  }
  ROS_ERROR_STREAM("TYPE : " << msg_type << " NOT PARSED");
  return false;
}

std::pair<bool, EMdgmMRZ>
KongsbergEM2040::read_mrz(uint8_t* ptr, int max_length)
{
  ROS_ERROR_STREAM("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  EMdgmMRZ mrz;
  int count = 0;

  mrz.header = *(reinterpret_cast<EMdgmHeader*>(ptr + count));
  count += sizeof(mrz.header);
  if (count>max_length){
    ROS_ERROR_STREAM("*After header* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
    return {false, {}};
  }
  ROS_ERROR_STREAM("#MRZ Header");
  ROS_ERROR_STREAM("  echoSounderID: "<<mrz.header.echoSounderID);
  ROS_ERROR_STREAM("  time_sec: "<<mrz.header.time_sec);
  ROS_ERROR_STREAM("  time_sec: "<<mrz.header.time_nanosec);
  mrz.partition = *(reinterpret_cast<EMdgmMpartition*>(ptr + count));
  count += sizeof(mrz.partition);
  if (count>max_length){
    ROS_ERROR_STREAM("*After partition* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
    return {false, {}};
  }
  ROS_ERROR_STREAM("#MRZ Partition");
  ROS_ERROR_STREAM("  numOfDgms: "<<mrz.partition.numOfDgms);
  ROS_ERROR_STREAM("  dgmNum: "<<mrz.partition.dgmNum);
  mrz.cmnPart = *(reinterpret_cast<EMdgmMbody*>(ptr + count));
  count += mrz.cmnPart.numBytesCmnPart;
  if (count>max_length){
    ROS_ERROR_STREAM("*After Common Part* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
    return {false, {}};
  }
  ROS_ERROR_STREAM("#MRZ Body");
  ROS_ERROR_STREAM("  numBytesCmnPart: "<<mrz.cmnPart.numBytesCmnPart);
  ROS_ERROR_STREAM("  pingCnt: "<<mrz.cmnPart.pingCnt);
  ROS_ERROR_STREAM("  rxFansPerPing: "<<mrz.cmnPart.rxFansPerPing);
  mrz.pingInfo = *(reinterpret_cast<EMdgmMRZ_pingInfo*>(ptr + count));
  count += mrz.pingInfo.numBytesInfoData;
  if (count>max_length){
    ROS_ERROR_STREAM("*After Ping Info* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
    return {false, {}};
  }
  ROS_ERROR_STREAM("#MRZ Ping Info");
  ROS_ERROR_STREAM("  numBytesInfoData: "<<mrz.pingInfo.numBytesInfoData);
  ROS_ERROR_STREAM("  numTxSectors: "<<mrz.pingInfo.numTxSectors);
  ROS_ERROR_STREAM("  numBytesPerTxSector: "<<mrz.pingInfo.numBytesPerTxSector);

  ROS_ERROR_STREAM("#MRZ Tx Sectors");
  for (int i=0; i<mrz.pingInfo.numTxSectors; i++){
    mrz.sectorInfo[i] = *(reinterpret_cast<EMdgmMRZ_txSectorInfo*>(ptr + count));
    ROS_ERROR_STREAM("  sectorInfo["<<i<<"]tx.ArrNumber: "<< (int)(mrz.sectorInfo[i].txArrNumber));
    count += mrz.pingInfo.numBytesPerTxSector;
    if (count>max_length){
      ROS_ERROR_STREAM("*After Sector["<<i<<"]* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
      return {false, {}};
    }
  }
  ROS_ERROR_STREAM("#MRZ Rx Info");
  mrz.rxInfo = *(reinterpret_cast<EMdgmMRZ_rxInfo*>(ptr + count));
  count += mrz.rxInfo.numBytesRxInfo;
  if (count>max_length){
    ROS_ERROR_STREAM("*After RXInfo* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
    return {false, {}};
  }
  ROS_ERROR_STREAM("  mrz.rxInfo.numSoundingsValidMain: "<< mrz.rxInfo.numSoundingsValidMain);
  ROS_ERROR_STREAM("  mrz.rxInfo.numSoundingsMaxMain: "<<mrz.rxInfo.numSoundingsMaxMain);
  ROS_ERROR_STREAM("  mrz.rxInfo.numExtraDetectionClasses: "<<mrz.rxInfo.numExtraDetectionClasses);
  ROS_ERROR_STREAM("#MRZ Extra detections");
  for (int i=0; i<mrz.rxInfo.numExtraDetectionClasses; i++){
    mrz.extraDetClassInfo[i] = *(reinterpret_cast<EMdgmMRZ_extraDetClassInfo*>(ptr + count));
    count += mrz.rxInfo.numBytesPerClass;
//    ROS_ERROR_STREAM("  mrz.extraDetClassInfo["<<i<<"].numExtraDetInClass"<<mrz.extraDetClassInfo[i].numExtraDetInClass);
    if (count>max_length){
      ROS_ERROR_STREAM("*After Extra Det["<<i<<"]* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
      return {false, {}};
    }
  }
  ROS_ERROR_STREAM("#MRZ soundings");
  int SIsamples = 0;
  for (int i=0; i<mrz.rxInfo.numSoundingsMaxMain+mrz.rxInfo.numExtraDetections; i++){
    mrz.sounding[i] = *(reinterpret_cast<EMdgmMRZ_sounding*>(ptr + count));
    count += mrz.rxInfo.numBytesPerSounding;
    SIsamples +=mrz.sounding[i].SInumSamples;
//    ROS_ERROR_STREAM("  mrz.sounding["<<i<<"].SInumSamples: "<<(int)(mrz.sounding[i].SInumSamples));
//    ROS_ERROR_STREAM("  mrz.sounding["<<i<<"].sectorNumb: "<<(int)(mrz.sounding[i].txSectorNumb));
    if (count>max_length){
      ROS_ERROR_STREAM("*After sounding["<<i<<"]* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
      return {false, {}};
    }
  }
  ROS_ERROR_STREAM("#MRZ Seafloor Image: " << SIsamples);
  // Actual number of seabed image samples (SIsample_desidB) to be found by summing
  // parameter SInumSamples in struct EMdgmMRZ_sounding_def for all beams.
  for (int i=0; i<SIsamples; i++){
    mrz.SIsample_desidB[i] = *(reinterpret_cast<uint16_t*>(ptr + count));
    count += sizeof(uint16_t);
    if (count>max_length){
      ROS_ERROR_STREAM("*After SI Samp["<<i<<"]* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
      return {false, {}};
    }
  }
  ROS_ERROR_STREAM("Success! MRZ read completely "<<count<<"/"<<max_length);
  return {true, mrz};
}

ds_multibeam_msgs::MultibeamRaw
KongsbergEM2040::mrz_to_mb_raw(EMdgmMRZ* msg)
{
//  return {};
////  struct EMdgmHeader_def 	header
  ds_multibeam_msgs::MultibeamRaw mb{};
  ros::Time t;
  mb.header.stamp = t.fromSec(msg->header.time_sec + msg->header.time_nanosec / 1e9);

  int num_soundings = msg->rxInfo.numSoundingsMaxMain + msg->rxInfo.numExtraDetections;
  mb.beamflag.resize(num_soundings);
  mb.twowayTravelTime.resize(num_soundings);
  mb.txDelay.resize(num_soundings);
  mb.intensity.resize(num_soundings);
//  mb.angleAlongTrack.resize(num_soundings);
  mb.angleAcrossTrack.resize(num_soundings);
//  mb.beamwidthAlongTrack.resize(num_soundings);
  mb.beamwidthAcrossTrack.resize(num_soundings);
  for (int i = 0; i < num_soundings; i++) {
    mb.beamflag[i] = (msg->sounding[i].detectionType == 2 ? mb.BEAM_BAD_SONAR : mb.BEAM_OK);
    mb.twowayTravelTime[i] = msg->sounding[i].twoWayTravelTime_sec;
    mb.txDelay[i] = msg->sounding[i].twoWayTravelTimeCorrection_sec;
    mb.intensity[i] = msg->sounding[i].reflectivity1_dB;
    ROS_ERROR_STREAM("Sounding num : "<<msg->sounding[i].soundingIndex);
//    ROS_ERROR_STREAM("Sector "<<msg->sounding[i].txSectorNumb);//<<" / "<<msg->pingInfo.numTxSectors);//Tilt angle deg : "<<msg->sectorInfo[msg->sounding[i].txSectorNumb].tiltAngleReTx_deg);
    int sector = msg->sounding[i].txSectorNumb;
    ROS_ERROR_STREAM("Sector num : "<<sector<<"/"<<msg->pingInfo.numTxSectors);
//    if (sector < msg->pingInfo.numTxSectors){
      mb.angleAlongTrack[i] = 3.14159 * msg->sectorInfo[sector].tiltAngleReTx_deg/180; // use sector index to get tilt angle, then convert to rad
//    }
    mb.angleAcrossTrack[i] = 3.14159 * msg->sounding[i].WCNomBeamAngleAcross_deg / 180.0; // convert deg to rad
//    mb.beamwidthAlongTrack[i] = 0;
    mb.beamwidthAcrossTrack[i] = 3.14159 * msg->sounding[i].WCNomBeamAngleAcross_deg / 180.0;
  }

  mb.soundspeed = msg->pingInfo.soundSpeedAtTxDepth_mPerSec;
  return mb;
}

sensor_msgs::Image
KongsbergEM2040::mwc_to_image(EMdgmMWC* msg)
{
  return {};
}


// XXXXXXXXXX
// Setup Fxns
// ----------
void
KongsbergEM2040::setupConnections()
{
  ds_base::DsProcess::setupConnections();
  DS_D(KongsbergEM2040);
  d->kmall_conn_ = addConnection("kmall_connection", boost::bind(&KongsbergEM2040::_on_kmall_data, this, _1));
  d->kctrl_conn_ = addConnection("kctrl_connection", boost::bind(&KongsbergEM2040::_on_kctrl_data, this, _1));
}
void
KongsbergEM2040::setupServices()
{
  ds_base::DsProcess::setupServices();
  DS_D(KongsbergEM2040);
  auto nh = nodeHandle();

  std::string ping_srv = ros::param::param<std::string>("~ping_service", "ping_cmd");
  d->ping_srv_ = nh.advertiseService<ds_kongsberg_msgs::PingCmd::Request, ds_kongsberg_msgs::PingCmd::Response>
      (ping_srv, boost::bind(&KongsbergEM2040::_ping_cmd, this, _1, _2));
  std::string power_srv = ros::param::param<std::string>("~power_service", "power_cmd");
  d->power_srv_ = nh.advertiseService<ds_kongsberg_msgs::PowerCmd::Request, ds_kongsberg_msgs::PowerCmd::Response>
      (power_srv, boost::bind(&KongsbergEM2040::_power_cmd, this, _1, _2));
  std::string settings_srv = ros::param::param<std::string>("~settings_service", "settings_cmd");
  d->settings_srv_ = nh.advertiseService<ds_kongsberg_msgs::SettingsCmd::Request,
                                         ds_kongsberg_msgs::SettingsCmd::Response>
      (settings_srv, boost::bind(&KongsbergEM2040::_settings_cmd, this, _1, _2));
  std::string bist_srv = ros::param::param<std::string>("~bist_service", "bist_cmd");
  d->bist_srv_ = nh.advertiseService<ds_kongsberg_msgs::BistCmd::Request, ds_kongsberg_msgs::BistCmd::Response>
      (bist_srv, boost::bind(&KongsbergEM2040::_bist_cmd, this, _1, _2));
}
void
KongsbergEM2040::setupParameters()
{
  ds_base::DsProcess::setupParameters();
  DS_D(KongsbergEM2040);
  d->sounder_name_ = ros::param::param<std::string>("~sounder_name", "EM2040_40");
  d->started_ = !ros::param::param<bool>("~run_startup", true);
  d->bist_filename_base = ros::param::param<std::string>("~bist_file_dir", "/home/jvaccaro/");
}
void
KongsbergEM2040::setupSubscriptions()
{
  ds_base::DsProcess::setupSubscriptions();
}
void
KongsbergEM2040::setupPublishers()
{
  DS_D(KongsbergEM2040);
  auto nh = nodeHandle();
  ds_base::DsProcess::setupPublishers();
  auto mbraw_topic = ros::param::param<std::string>("~mbraw_topic", "mbraw");
  d->mbraw_pub_ = nh.advertise<ds_multibeam_msgs::MultibeamRaw>(mbraw_topic, 1000);
  auto watercolumn_topic = ros::param::param<std::string>("~watercolumn_topic", "watercolumn");
  d->watercolumn_pub_ = nh.advertise<sensor_msgs::Image>(watercolumn_topic, 1000);
  auto pointcloud_topic = ros::param::param<std::string>("~pointcloud_topic", "pointcloud");
  d->pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 1000);
  auto kmstatus_topic = ros::param::param<std::string>("~kmstatus_topic", "kmstatus");
  d->kmstatus_pub_ = nh.advertise<ds_kongsberg_msgs::KongsbergKSSIS>(kmstatus_topic, 1000);
  auto offset_topic = ros::param::param<std::string>("~offset_topic", "pu_offset");
  d->offset_pub_ = nh.advertise<ds_core_msgs::ClockOffset>(offset_topic, 1000);
}

// XXXXXXXXXXXXX
// Service Calls
// -------------
bool
KongsbergEM2040::_ping_cmd(ds_kongsberg_msgs::PingCmd::Request &req, ds_kongsberg_msgs::PingCmd::Response &res)
{
  ROS_ERROR_STREAM("PING COMMAND");
  return true;
}
bool
KongsbergEM2040::_power_cmd(ds_kongsberg_msgs::PowerCmd::Request &req, ds_kongsberg_msgs::PowerCmd::Response &res)
{
  res.command_sent = _send_kctrl_command(req.power);
  return true;
}
bool
KongsbergEM2040::_settings_cmd(ds_kongsberg_msgs::SettingsCmd::Request &req, ds_kongsberg_msgs::SettingsCmd::Response &res)
{
  res.command_sent = _send_kctrl_param(req.setting_name, req.setting_value);
  return true;
}
bool
KongsbergEM2040::_bist_cmd(ds_kongsberg_msgs::BistCmd::Request &req, ds_kongsberg_msgs::BistCmd::Response &res)
{
  DS_D(KongsbergEM2040);
  bist_lists foo;
  switch (req.bist_command){
    case ds_kongsberg_msgs::BistCmd::Request::BIST_ONDECK :
      if (d->bist_running){
        res.action = "BIST already in progress... cancel it first";
        return false;
      } else {
        d->bist_running = true;
        d->bist_tests = foo.get_ondeck();
        d->bist_filename = d->bist_filename_base + "_bist_ondeck.txt";
      }
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_INWATER :
      if (d->bist_running){
        res.action = "BIST already in progress... cancel it first";
        return false;
      } else {
        d->bist_running = true;
        d->bist_tests = foo.get_inwater();
        d->bist_filename = d->bist_filename_base + "_bist_inwater.txt";
      }
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_CANCEL :
      if (d->bist_running){
        d->bist_tests.resize(d->bist_progress+1); // this index means it will stop neatly after the next test returns
        res.action = "BIST commanded to cancel!";
        return true;
      } else {
        res.action = "No BIST in progress, ready to start a new BIST";
        return false;
      }
    default:
      res.action = "Unknown BIST command... failure!";
      return false;
  }
  res.action = "BIST started... check progress in : " + d->bist_filename;
  d->bist_progress = 0;
  _run_next_bist();
  return true;
}

// XXXXXXXXXXXXXXXX
// Connection calls
// ----------------
void
KongsbergEM2040::_on_kmall_data(ds_core_msgs::RawData raw)
{
  if (!parse_data(raw)){
    ROS_ERROR_STREAM("KMAll data parse failed");
  }
}
void
KongsbergEM2040::_on_kctrl_data(ds_core_msgs::RawData raw)
{
  if (!parse_message(raw)){
    ROS_ERROR_STREAM("KCtrl message parse failed");
  }
}

// XXXXXXXXXXXXXXXXXXXXXX
// Command send shortcuts
// ----------------------
void
KongsbergEM2040::_startup_sequence()
{
  _send_kctrl_command(SIS_TO_K::START);
  _send_kctrl_command(SIS_TO_K::SET_READY);
  DS_D(KongsbergEM2040);
  d->started_ = true;
}
std::string
KongsbergEM2040::_send_kctrl_command(int cmd)
{
  DS_D(KongsbergEM2040);
  std::stringstream ss;
  ss << "$KSSIS,"
      << cmd << ","
      << d->sounder_name_;
  auto msg = ss.str();
  d->kctrl_conn_->send(msg);
  return msg;
}
template <class T1>
std::string
KongsbergEM2040::_send_kctrl_param(std::string param_name, T1 param_value)
{
  DS_D(KongsbergEM2040);
  std::stringstream ss;
  ss << "$KSSIS,"
     << SIS_TO_K::SETVALUES << ","
     << d->sounder_name_ << ","
     << param_name << "="
     << param_value;
  auto msg = ss.str();
  d->kctrl_conn_->send(msg);
  return msg;
}
void
KongsbergEM2040::_run_next_bist()
{
  DS_D(KongsbergEM2040);
  int num = d->bist_progress + 1;
  if (num < d->bist_tests.size() ){
    ROS_ERROR_STREAM("Running BIST ... "<< d->bist_tests[num]);
    // Send the next command
    _send_kctrl_param("INST_PARAM_BIST_DO", d->bist_tests[num]);
    d->bist_progress = num;
  } else {
    d->bist_running = false;
    ROS_ERROR_STREAM("BIST done! Check "<<d->bist_filename<<" for results!");
  }
}

} //namespace