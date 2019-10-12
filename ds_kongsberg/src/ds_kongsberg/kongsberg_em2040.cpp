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
#include "kongsberg_em2040_util.h"
#include "ds_kongsberg_msgs/KongsbergKSSIS.h"
#include "kongsberg_em2040_strings.h"
#include "ds_core_msgs/ClockOffset.h"
#include <regex>
#include "ds_util/int_to_hex.h"
#include "ds_kongsberg_msgs/KongsbergKMAllRecord.h"

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

KongsbergEM2040::~KongsbergEM2040(){
  DS_D(KongsbergEM2040);
  if (d->kmall_stream) {
    d->kmall_stream->close();
  }
};

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
    case K_TO_SIS::STATUS_IPI :
      break;
    case K_TO_SIS::STATUS_IPI_PU : {
      d->pu_powered_timer.stop();
      d->pu_powered_timer.start();
      d->m_status.pu_powered = true;
//      std::stringstream ipi(fields[2]);
//      fields.pop_back();
//      while( ipi.good() )
//      {
//        std::string substr;
//        getline( ipi, substr, '~' );
//        if (substr.length()>0)
//          fields.push_back( substr );
//      }
      if (!d->m_status.pu_connected){
        _startup_sequence();
      }
      break;
    }
    case K_TO_SIS::PU_DISCONNECTED : {
      ROS_ERROR_STREAM("PU POWERDOWN DETECTED");
      d->m_status.pu_powered = false;
      break;
    }
    case K_TO_SIS::STATUS_IPU : {
      parse_ipu(fields);
    }
    default :
      if (msg.type>800 && msg.type<900){
        read_kmall_dgm_from_kctrl(msg.type, raw);
        break;
      }
      if (fields[2] != d->m_status.sounder_name){
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
  d->kssis_pub_.publish(msg);
  return true;
}
bool
KongsbergEM2040::parse_ipu(std::vector<std::string> fields)
{
  if (fields.size() != 15) {
    ROS_ERROR_STREAM("IPU size too small " << fields.size());
    return false;
  }
  DS_D(KongsbergEM2040);
  if (fields[2] != d->m_status.sounder_name){
    ROS_ERROR_STREAM("IPU sounder name doesn't match");
    return false;
  }
//  d->m_status.cpu_temperature = read_good_bad_missing(fields[4]);
//  d->m_status.position_1 = read_good_bad_missing(fields[5]);
//  d->m_status.position_2 = read_good_bad_missing(fields[6]);
//  d->m_status.position_3 = read_good_bad_missing(fields[7]);
//  d->m_status.attitude_1 = read_good_bad_missing(fields[8]);
//  d->m_status.attitude_2 = read_good_bad_missing(fields[9]);
//  d->m_status.depth = read_good_bad_missing(fields[10]);
//  d->m_status.svp = read_good_bad_missing(fields[11]);
//  d->m_status.time_sync = read_good_bad_missing(fields[12]);
//  d->m_status.pps = read_good_bad_missing(fields[13]);
  d->m_status.cpu_temperature = fields[4];
  d->m_status.position_1 = fields[5];
  d->m_status.position_2 = fields[6];
  d->m_status.position_3 = fields[7];
  d->m_status.attitude_1 = fields[8];
  d->m_status.attitude_2 = fields[9];
  d->m_status.depth = fields[10];
  d->m_status.svp = fields[11];
  d->m_status.time_sync = fields[12];
  d->m_status.pps = fields[13];

  d->pu_connected_timer.stop();
  d->pu_connected_timer.start();
  d->m_status.pu_connected = true;
  d->kmstatus_pub_.publish(d->m_status);
  return true;
}

uint8_t
KongsbergEM2040::read_good_bad_missing(std::string msg)
{
//  boost::smatch results;
//  auto good = boost::regex{ "=OK;G" };
//  auto missing = boost::regex{ "=MISSING;R" };
//  auto activebad = boost::regex{ "=ACTIVE-BAD;R" };
//  auto bad = boost::regex{ "=BAD;R" };
//  auto off = boost::regex{ "=OFF;B" };
//  if (boost::regex_search(msg, results, good)){
//    return ds_kongsberg_msgs::KongsbergStatus::SENSOR_OK;
//  }
//  if (boost::regex_search(msg, results, missing)){
//    return ds_kongsberg_msgs::KongsbergStatus::SENSOR_MISSING;
//  }
//  if (boost::regex_search(msg, results, activebad)
//      || boost::regex_search(msg, results, bad)){
//    return ds_kongsberg_msgs::KongsbergStatus::SENSOR_BAD;
//  }
//  if (boost::regex_search(msg, results, off)){
//    return ds_kongsberg_msgs::KongsbergStatus::SENSOR_OFF;
//  }
  return 0;
}

bool
KongsbergEM2040::read_bist_result(ds_core_msgs::RawData& raw)
{
  dgm_IB r{};
  uint8_t* ptr = raw.data.data();

  r = *reinterpret_cast<dgm_IB*>(ptr);
  int index = sizeof(r) - 2;
  auto max_index = raw.data.size();
  if (max_index < index){
    return false;
  }

  DS_D(KongsbergEM2040);
  auto message = std::string{ reinterpret_cast<const char*>(ptr + index), max_index - index };
  std::string name;
  bist_strings b;
  name = b.get_name_from_code(r.BISTNumber);
  ROS_ERROR_STREAM("RECEIVED BIST "<<name);
  std::string status;
  if (r.BISTStatus == 0)
    status = "PASS";
  else if (r.BISTStatus < 0)
    status = "ERROR";
  else if (r.BISTStatus > 0)
    status = "WARNING";

  if (d->m_status.bist_running){
    _print_bist(name, status, message);
    _run_next_bist();
  }
  return true;
}
bool
KongsbergEM2040::read_kmall_dgm_from_kctrl(int type, ds_core_msgs::RawData& raw)
{
  ds_core_msgs::RawData stripped_raw{};
  stripped_raw.header = raw.header;
  stripped_raw.ds_header = raw.ds_header;

  DS_D(KongsbergEM2040);
  std::string front = "$KSSIS," + std::to_string(type) + "," + d->m_status.sounder_name + ",";
  auto index = front.length();
  auto max_index = raw.data.size();
  if (index > max_index){
    return false;
  }
  stripped_raw.data.resize(max_index-index);

  for (int i=0; i<max_index-index; i++){
    stripped_raw.data[i] = raw.data[i+index];
  }
  switch (type){
    case K_TO_SIS::XML :
      _write_kctrl_xml(stripped_raw);
      return true;
    case K_TO_SIS::BIST_RESULT :
      read_bist_result(stripped_raw);
      return true;
    default:
      return false;
  }
}

// XXXXXXXXXXXXX
// KMAll parsers
// -------------
bool
KongsbergEM2040::parse_data(ds_core_msgs::RawData& raw)
{
  DS_D(KongsbergEM2040);
  auto data_size = raw.data.size();
  auto min_size = sizeof(EMdgmHeader);
  if (data_size < min_size){
    ROS_ERROR_STREAM("Raw Data received less than minimum size "<<min_size<<" bytes");
    return false;
  }

  uint8_t* bytes_ptr = raw.data.data();
  auto hdr = reinterpret_cast<EMdgmHeader*>(bytes_ptr);
  //ROS_ERROR_STREAM("TIME: "<<hdr->time_sec);
  std::string msg_type(hdr->dgmType, hdr->dgmType + sizeof(hdr->dgmType)/sizeof(hdr->dgmType[0]));

  ds_kongsberg_msgs::KongsbergKMAllRecord record;
  record.ds_header = raw.ds_header;
  record.header.stamp = ros::Time(hdr->time_sec, hdr->time_nanosec);
  record.record_type = msg_type;
  record.kmall_filename = d->m_status.kmall_filename;
  record.record_size = data_size;

  if (data_size != hdr->numBytesDgm){
    ROS_ERROR_STREAM("hdr->dgmType ("<< record.record_type << ") data.size() ("<<data_size<<") != hdr->numBytes ("<<hdr->numBytesDgm<<")");
    return false;
  }

  if (msg_type==EM_DGM_I_INSTALLATION_PARAM) {
    record.record_name = "EM_DGM_I_INSTALLATION_PARAM";
    _new_kmall_file();
    _write_kmall_data(raw);
  }
  else if (msg_type==EM_DGM_I_OP_RUNTIME) {
    record.record_name = "EM_DGM_I_OP_RUNTIME";
    _write_kmall_data(raw);
  }
  else if (msg_type==EM_DGM_S_POSITION){
    record.record_name = "EM_DGM_S_POSITION";
    _write_kmall_data(raw);
  }
  else if (msg_type==EM_DGM_S_KM_BINARY){
    record.record_name = "EM_DGM_S_KM_BINARY";
    _write_kmall_data(raw);
  }
  else if (msg_type==EM_DGM_S_SOUND_VELOCITY_PROFILE){
    record.record_name = "EM_DGM_S_SOUND_VELOCITY_PROFILE";
    _write_kmall_data(raw);
  }
  else if (msg_type==EM_DGM_S_SOUND_VELOCITY_TRANSDUCER){
    record.record_name = "EM_DGM_S_SOUND_VELOCITY_TRANSDUCER";
    _write_kmall_data(raw);
  }
  else if (msg_type==EM_DGM_S_CLOCK){
    record.record_name = "EM_DGM_S_CLOCK";
    ds_core_msgs::ClockOffset offset;
    offset.header = record.header;
    offset.ds_header = record.ds_header;
    offset.device_stamp_minus_ros_stamp_sec = offset.header.stamp.toSec() - offset.ds_header.io_time.toSec();
    d->offset_pub_.publish(offset);
    _write_kmall_data(raw);
  }
  else if (msg_type==EM_DGM_S_DEPTH){
    record.record_name = "EM_DGM_S_DEPTH";
    _write_kmall_data(raw);
  }
  else if (msg_type==EM_DGM_S_HEIGHT){
    record.record_name = "EM_DGM_S_HEIGHT";
    _write_kmall_data(raw);
  }
  else if (msg_type==EM_DGM_M_RANGE_AND_DEPTH){
    d->pinging_timer.stop();
    d->pinging_timer.start();
    d->m_status.pinging = true;
    record.record_name = "EM_DGM_M_RANGE_AND_DEPTH";
    bool full_data = false;
    ds_core_msgs::RawData logme{};
    std::tie(full_data, logme) = check_and_append_mpartition(raw);
    if (full_data){
      _write_kmall_data(logme);
      EMdgmMRZ mrz;
      bool ok = false;
      std::tie(ok, mrz) = read_mrz(logme.data.data(),logme.data.size());
      if (ok){
        auto mbr = mrz_to_mb_raw(&mrz);
        mbr.header = record.header;
        mbr.ds_header = record.ds_header;
        d->mbraw_pub_.publish(mbr);
        d->m_status.ping_num = mrz.cmnPart.pingCnt;
        ROS_ERROR_STREAM("Ping num: "<<d->m_status.ping_num);
        mbraw_to_kmstatus(mbr);
      }
//      d->kmall_record_pub_.publish(record);
//      return true;
    } else {
      record.record_name = "EM_DGM_M_RANGE_AND_DEPTH P";
      ROS_ERROR_STREAM("PING PARTITION");
    }
  }
  else if (msg_type==EM_DGM_M_WATER_COLUMN){
    record.record_name = "EM_DGM_M_WATER_COLUMN";
    bool full_data = false;
    ds_core_msgs::RawData logme{};
    std::tie(full_data, logme) = check_and_append_mpartition(raw);
    if (full_data) {
      _write_kmall_data(logme);
    }
  }
  else if (msg_type==EM_DGM_C_POSITION){
    record.record_name = "EM_DGM_C_POSITION";
    _write_kmall_data(raw);
  }
  else if (msg_type==EM_DGM_C_HEAVE){
    record.record_name = "EM_DGM_C_HEAVE";
    _write_kmall_data(raw);
  }
  if (record.record_name.empty()){
    ROS_ERROR_STREAM("TYPE : " << msg_type << " not parsed, but recorded anyways");
    record.record_name = "Unknown";
    _write_kmall_data(raw);
  }
  d->kmall_record_pub_.publish(record);
  return true;
}

std::pair<bool, ds_core_msgs::RawData>
KongsbergEM2040::check_and_append_mpartition(ds_core_msgs::RawData raw_p)
{
  // Returns a bool and a RawData msg.
  // If the datagram is not partitioned, then it returns the datagram.
  // If the datagram is partitioned, then
  // True means that the datagram is complete (SHOULD BE LOGGED AND PARSED)
  // False means that the datagram is incomplete (NO LOGGING)
  DS_D(KongsbergEM2040);
  auto ptr = raw_p.data.data();
  auto max_length = raw_p.data.size();
  int count = 0;
  auto hdr = reinterpret_cast<EMdgmHeader*>(ptr + count);
  count += sizeof(EMdgmHeader);
  // ROS_ERROR_STREAM("PARTITIONED TIME: "<<hdr->time_sec);
  auto partition = reinterpret_cast<EMdgmMpartition*>(ptr + count);
  count += sizeof(EMdgmMpartition);
  // If the datagram isn't partitioned, then return itself immediately!
  if (partition->dgmNum == 1 && partition->numOfDgms == 1){
    return {true, raw_p};
  }
  // If it's the first in a sequence, then clear out the buffer and resize it to the current size.
  // Set its partition values to 1 and 1 respectively.
  else if (partition->dgmNum == 1 && partition->numOfDgms > 1){
    partition->dgmNum = 1;
    partition->numOfDgms = 1;
    d->kmall_partitioned.data.resize(max_length);
    d->kmall_partitioned.data = raw_p.data;
    ROS_ERROR_STREAM("Found partition piece "<<partition->dgmNum<< " of "<<partition->numOfDgms << " with size "<<max_length);
    return {false, {}};
  }
  // If it's a following datagram, then append it starting AFTER the partition.
  else if (partition->dgmNum > 1){
    auto current_length = d->kmall_partitioned.data.size();
    if (current_length == 0){
      ROS_ERROR_STREAM("MISSED EARLIER PARTITION PACKET... IGNORING LATER PACKETS");
    }
    // Resize and copy everything except the new header/partition
    // Overwrite the ending values for the length
    d->kmall_partitioned.data.resize(current_length + max_length - count - 4);
    memcpy(ptr + count, d->kmall_partitioned.data.data() + current_length - 4, max_length - count);
    ROS_ERROR_STREAM("Found partition piece "<<partition->dgmNum<< " of "<<partition->numOfDgms << " with size "<<max_length);
  }
  // If the datagram has completed transmission, then return the partitioned data message.
  if (partition->dgmNum == partition->numOfDgms){
    auto data_ptr = d->kmall_partitioned.data.data();
    auto data_size = d->kmall_partitioned.data.size();
    auto starting_size_ptr = reinterpret_cast<uint32_t*>(data_ptr);
    *starting_size_ptr = data_size;
    auto ending_size_ptr = reinterpret_cast<uint32_t*>(d->kmall_partitioned.data.data() + d->kmall_partitioned.data.size() - 4);
    *ending_size_ptr = data_size;
    ROS_ERROR_STREAM("Partition complete! Total size "<< data_size<<" bytes");
    return {true, d->kmall_partitioned};
  }
  // Otherwise, assume transmission has not completed and we are waiting for more data.
  return {false, {}};
};

std::pair<bool, EMdgmMRZ>
KongsbergEM2040::read_mrz(uint8_t* ptr, int max_length)
{

  EMdgmMRZ mrz;
  memset(&mrz, 0, sizeof(mrz));
  int count = 0;

  mrz.header = *(reinterpret_cast<EMdgmHeader*>(ptr + count));
  count += sizeof(mrz.header);
  if (count>max_length){
    ROS_ERROR_STREAM("*After header* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
    return {false, {}};
  }
  mrz.partition = *(reinterpret_cast<EMdgmMpartition*>(ptr + count));
  count += sizeof(mrz.partition);
  if (count>max_length){
    ROS_ERROR_STREAM("*After partition* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
    return {false, {}};
  }
  mrz.cmnPart = *(reinterpret_cast<EMdgmMbody*>(ptr + count));
  count += mrz.cmnPart.numBytesCmnPart;
  if (count>max_length){
    ROS_ERROR_STREAM("*After Common Part* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
    return {false, {}};
  }
  mrz.pingInfo = *(reinterpret_cast<EMdgmMRZ_pingInfo*>(ptr + count));
  count += mrz.pingInfo.numBytesInfoData;
  if (count>max_length){
    ROS_ERROR_STREAM("*After Ping Info* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
    return {false, {}};
  }
  for (int i=0; i<mrz.pingInfo.numTxSectors; i++){
    mrz.sectorInfo[i] = *(reinterpret_cast<EMdgmMRZ_txSectorInfo*>(ptr + count));
    count += mrz.pingInfo.numBytesPerTxSector;
    if (count>max_length){
      ROS_ERROR_STREAM("*After Sector["<<i<<"]* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
      return {false, {}};
    }
  }
  mrz.rxInfo = *(reinterpret_cast<EMdgmMRZ_rxInfo*>(ptr + count));
  count += mrz.rxInfo.numBytesRxInfo;
  if (count>max_length){
    ROS_ERROR_STREAM("*After RXInfo* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
    return {false, {}};
  }
  for (int i=0; i<mrz.rxInfo.numExtraDetectionClasses; i++){
    mrz.extraDetClassInfo[i] = *(reinterpret_cast<EMdgmMRZ_extraDetClassInfo*>(ptr + count));
    count += mrz.rxInfo.numBytesPerClass;
    if (count>max_length){
      ROS_ERROR_STREAM("*After Extra Det["<<i<<"]* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
      return {false, {}};
    }
  }
  int SIsamples = 0;
  for (int i=0; i<mrz.rxInfo.numSoundingsMaxMain+mrz.rxInfo.numExtraDetections; i++){
    mrz.sounding[i] = *(reinterpret_cast<EMdgmMRZ_sounding*>(ptr + count));
    count += mrz.rxInfo.numBytesPerSounding;
    SIsamples +=mrz.sounding[i].SInumSamples;
    if (count>max_length){
      ROS_ERROR_STREAM("*After sounding["<<i<<"]* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
      return {false, {}};
    }
  }
  for (int i=0; i<SIsamples; i++){
    mrz.SIsample_desidB[i] = *(reinterpret_cast<uint16_t*>(ptr + count));
    count += sizeof(uint16_t);
    if (count>max_length){
      ROS_ERROR_STREAM("*After SI Samp["<<i<<"]* In read_mrz, count="<<count<<" exceeded max_length="<<max_length);
      return {false, {}};
    }
  }
  auto check_length = *(reinterpret_cast<uint32_t*>(ptr + count));
  count += sizeof(uint32_t);
  ROS_ERROR_STREAM("Count: " << count << " max_length: "<<max_length);
  ROS_ERROR_STREAM("Header len: " << mrz.header.numBytesDgm << " Check len: "<<check_length);
  ROS_ERROR_STREAM("PING TIME: "<<mrz.header.time_sec);
  return {true, mrz};
}

ds_multibeam_msgs::MultibeamRaw
KongsbergEM2040::mrz_to_mb_raw(EMdgmMRZ* msg)
{
  ds_multibeam_msgs::MultibeamRaw mb{};
  ros::Time t;
  mb.header.stamp = t.fromSec(msg->header.time_sec + msg->header.time_nanosec / 1.0e9);

  int num_soundings = msg->rxInfo.numSoundingsMaxMain + msg->rxInfo.numExtraDetections;
  mb.beamflag.resize(num_soundings);
  mb.twowayTravelTime.resize(num_soundings);
  mb.txDelay.resize(num_soundings);
  mb.intensity.resize(num_soundings);
  mb.angleAlongTrack.resize(num_soundings);
  mb.angleAcrossTrack.resize(num_soundings);
  mb.beamwidthAlongTrack.resize(num_soundings);
  mb.beamwidthAcrossTrack.resize(num_soundings);
  for (int i = 0; i < num_soundings; i++) {
    mb.beamflag[i] = (msg->sounding[i].detectionType == 2 ? mb.BEAM_BAD_SONAR : mb.BEAM_OK);
    mb.twowayTravelTime[i] = msg->sounding[i].twoWayTravelTime_sec;
    mb.txDelay[i] = msg->sounding[i].twoWayTravelTimeCorrection_sec;
    mb.intensity[i] = msg->sounding[i].reflectivity1_dB;
    int sector = msg->sounding[i].txSectorNumb;
    if (sector < msg->pingInfo.numTxSectors){
      mb.angleAlongTrack[i] = deg_to_rad(msg->sectorInfo[sector].tiltAngleReTx_deg); // use sector index to get tilt angle, then convert to rad
    }
    mb.angleAcrossTrack[i] = deg_to_rad(msg->sounding[i].beamAngleReRx_deg); // convert deg to rad
    mb.beamwidthAlongTrack[i] = 0;
    mb.beamwidthAcrossTrack[i] = deg_to_rad(msg->sounding[i].WCNomBeamAngleAcross_deg);
  }

  mb.soundspeed = msg->pingInfo.soundSpeedAtTxDepth_mPerSec;
  return mb;
}

void
KongsbergEM2040::mbraw_to_kmstatus(ds_multibeam_msgs::MultibeamRaw raw)
{
  int num_soundings = raw.beamflag.size();
  int num_good = 0;
  std::vector<float> acrosstrack_angles_raw, ranges, depths;
  float min_range = 100;
  float max_range = 0;
  float min_depth = 100;
  float max_depth = 0;
  float center_angle = 100;
  float center_range = 0;
  float center_depth = 0;
  ranges.resize(num_soundings);
  depths.resize(num_soundings);
  for (int i=0; i<num_soundings; i++){
    if (raw.beamflag[i] != raw.BEAM_BAD_SONAR){
      num_good ++;
    }
    ranges[i] = raw.soundspeed*raw.twowayTravelTime[i] / 2.0;
    depths[i] = ranges[i]*cos(raw.angleAlongTrack[i])*cos(raw.angleAcrossTrack[i]);
    if (abs(raw.angleAcrossTrack[i]) < abs(center_angle)){
      center_angle = raw.angleAcrossTrack[i];
      center_range = ranges[i];
      center_depth = depths[i];
    }
    min_range = (ranges[i] < min_range ? ranges[i] : min_range);
    max_range = (ranges[i] > max_range ? ranges[i] : max_range);
    min_depth = (depths[i] < min_depth ? depths[i] : min_depth);
    max_depth = (depths[i] > max_depth ? depths[i] : max_depth);
  }
  DS_D(KongsbergEM2040);
  // ping num populated earlier
  d->m_status.num_soundings = num_soundings;
  d->m_status.percent_good = 100.0 * num_good / static_cast<float>(num_soundings);
  d->m_status.min_depth = min_depth;
  d->m_status.max_depth = max_depth;
  d->m_status.center_depth = center_depth;
  d->m_status.min_range = min_range;
  d->m_status.max_range = max_range;
  d->m_status.center_range = center_range;
  d->kmstatus_pub_.publish(d->m_status);
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
  const auto name = ros::this_node::getName();

  std::string ping_srv = ros::param::param<std::string>("~ping_service", "ping_cmd");
  d->ping_srv_ = nh.advertiseService<ds_kongsberg_msgs::PingCmd::Request, ds_kongsberg_msgs::PingCmd::Response>
      (name + "/" + ping_srv, boost::bind(&KongsbergEM2040::_ping_cmd, this, _1, _2));
  std::string power_srv = ros::param::param<std::string>("~power_service", "power_cmd");
  d->power_srv_ = nh.advertiseService<ds_kongsberg_msgs::PowerCmd::Request, ds_kongsberg_msgs::PowerCmd::Response>
      (name + "/" + power_srv, boost::bind(&KongsbergEM2040::_power_cmd, this, _1, _2));
  std::string settings_srv = ros::param::param<std::string>("~settings_service", "settings_cmd");
  d->settings_srv_ = nh.advertiseService<ds_kongsberg_msgs::SettingsCmd::Request,
                                         ds_kongsberg_msgs::SettingsCmd::Response>
      (name + "/" + settings_srv, boost::bind(&KongsbergEM2040::_settings_cmd, this, _1, _2));
  std::string bist_srv = ros::param::param<std::string>("~bist_service", "bist_cmd");
  d->bist_srv_ = nh.advertiseService<ds_kongsberg_msgs::BistCmd::Request, ds_kongsberg_msgs::BistCmd::Response>
      (name + "/" + bist_srv, boost::bind(&KongsbergEM2040::_bist_cmd, this, _1, _2));
  std::string xml_srv = ros::param::param<std::string>("~xml_service", "load_xml_cmd");
  d->xml_srv_ = nh.advertiseService<ds_kongsberg_msgs::LoadXmlCmd::Request, ds_kongsberg_msgs::LoadXmlCmd::Response>
      (name + "/" + xml_srv, boost::bind(&KongsbergEM2040::_load_xml_cmd, this, _1, _2));
}
void
KongsbergEM2040::setupParameters()
{
  ds_base::DsProcess::setupParameters();
  DS_D(KongsbergEM2040);
  // m_status parameters
  d->m_status = ds_kongsberg_msgs::KongsbergStatus{};
  d->m_status.sounder_name = ros::param::param<std::string>("~sounder_name", "EM2040_40");
  d->m_status.ship_name = ros::param::param<std::string>("~ship_name", "ShipName");
  d->m_status.pu_connected = !ros::param::param<bool>("~run_startup", true);
  d->m_status.bist_directory = ros::param::param<std::string>("~bist_dir", "/home/jvaccaro/");
  d->m_status.kmall_directory = ros::param::param<std::string>("~kmall_dir", "/home/jvaccaro/");
  d->m_status.xml_directory = ros::param::param<std::string>("~xml_dir", "/home/jvaccaro/");
  d->m_status.cpu_temperature = "";
  d->m_status.position_1 =  "";
  d->m_status.position_2 =  "";
  d->m_status.position_3 = "";
  d->m_status.attitude_1 = "";
  d->m_status.attitude_2 = "";
  d->m_status.depth = "";
  d->m_status.svp = "";
  d->m_status.time_sync = "";
  d->m_status.pps = "";
  d->m_status.rt_force_depth = "0";
  d->m_status.rt_min_depth = "0";
  d->m_status.rt_max_depth = "0";
  d->m_status.rt_ping_freq = "0";
  d->m_status.rt_depth_mode = "0";
  d->m_status.rt_detector_mode = "0";
  d->m_status.rt_fm_disable = "0";
  d->m_status.rt_log_watercol = "0";
  d->m_status.rt_extra_detect = "0";
  d->m_status.rt_pitch_stab = "0";
  d->m_status.rt_tx_angle_along = "0";
  d->m_status.rt_yaw_stab = "0";
  d->m_status.rt_max_ping_rate = "0";
  d->m_status.rt_min_swath_distance = "0";
  d->m_status.rt_trigger = "0";
  // other parameters
  d->kmall_max_buffer_size = 1e3*ros::param::param<int>("~max_kmall_buffer_kB", 30);
  d->kmall_max_file_size = 1e6*ros::param::param<int>("~max_kmall_file_MB", 400);
  d->kmall_partitioned = ds_core_msgs::RawData{};
  d->kmall_partitioned.data.resize(0);
  _new_kmall_file();
}

void
KongsbergEM2040::setupPublishers()
{
  ds_base::DsProcess::setupPublishers();
  DS_D(KongsbergEM2040);
  auto nh = nodeHandle();
  const auto name = ros::this_node::getName();

  auto mbraw_topic = ros::param::param<std::string>("~mbraw_topic", "mbraw");
  d->mbraw_pub_ = nh.advertise<ds_multibeam_msgs::MultibeamRaw>(name + "/" + mbraw_topic, 1000);

  auto pointcloud_topic = ros::param::param<std::string>("~pointcloud_topic", "pointcloud");
  d->pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(name + "/" + pointcloud_topic, 1000);

  auto kssis_topic = ros::param::param<std::string>("~kssis_topic", "kssis");
  d->kssis_pub_ = nh.advertise<ds_kongsberg_msgs::KongsbergKSSIS>(name + "/" + kssis_topic, 1000);

  auto offset_topic = ros::param::param<std::string>("~offset_topic", "pu_offset");
  d->offset_pub_ = nh.advertise<ds_core_msgs::ClockOffset>(name + "/" + offset_topic, 1000);

  auto kmall_record_topic = ros::param::param<std::string>("~kmall_records_topic", "kmall_records");
  d->kmall_record_pub_ = nh.advertise<ds_kongsberg_msgs::KongsbergKMAllRecord>(name + "/" + kmall_record_topic, 1000);

  auto kmstatus_topic = ros::param::param<std::string>("~kmstatus_topic", "kmstatus");
  d->kmstatus_pub_ = nh.advertise<ds_kongsberg_msgs::KongsbergStatus>(name + "/" + kmstatus_topic, 1000);
}

void
KongsbergEM2040::setupTimers()
{
  ds_base::DsProcess::setupTimers();
  DS_D(KongsbergEM2040);
  auto nh = nodeHandle();
  auto kmall_to = ros::param::param<double>("~kmall_timeout", 3.0);
  d->kmall_timer = nh.createTimer(ros::Duration(kmall_to),
                                  &KongsbergEM2040::_on_kmall_timeout, this);

  auto kctrl_to = ros::param::param<double>("~kctrl_timeout", 3.0);
  d->kctrl_timer = nh.createTimer(ros::Duration(3),
                                  &KongsbergEM2040::_on_kctrl_timeout, this);

  auto pu_powered_to = ros::param::param<double>("~pu_powered_timeout", 50.0);
  d->pu_powered_timer = nh.createTimer(ros::Duration(pu_powered_to),
                                       &KongsbergEM2040::_on_pu_powered_timeout, this);

  auto pu_connected_to = ros::param::param<double>("~pu_connected_timeout", 50.0);
  d->pu_connected_timer = nh.createTimer(ros::Duration(pu_connected_to),
                                         &KongsbergEM2040::_on_pu_connected_timeout, this);

  auto pinging_to = ros::param::param<double>("~pinging_timeout", 3.0);
  d->pinging_timer = nh.createTimer(ros::Duration(pinging_to),
                                    &KongsbergEM2040::_on_pinging_timeout, this);

  d->kmall_timer.start();
  d->kctrl_timer.start();
  d->pu_powered_timer.start();
  d->pu_connected_timer.start();
  d->pinging_timer.start();
}

// XXXXXXXXXXXXX
// Service Calls
// -------------
bool
KongsbergEM2040::_ping_cmd(ds_kongsberg_msgs::PingCmd::Request &req, ds_kongsberg_msgs::PingCmd::Response &res)
{
  DS_D(KongsbergEM2040);
  if (!d->m_status.kctrl_connected){
    res.action += "KCtrl not connected... ";
  }
  if (!d->m_status.pu_powered) {
    res.action += "PU not powered... ";
  }
  if (!d->m_status.pu_connected && req.ping!=req.PING_STARTUP){
    res.action += "PU not connected... ";
  }
  switch (req.ping){
    case ds_kongsberg_msgs::PingCmd::Request::PING_START :
//      _new_kmall_file();
      _send_kctrl_command(SIS_TO_K::LOG_IOP_SVP);
      _send_kctrl_command(SIS_TO_K::START_PING);
      d->m_status.commanded_pinging = true;
      res.action = "Commanded ping start, created new kmall file, logged IOP and SVP information";
      break;
    case ds_kongsberg_msgs::PingCmd::Request::PING_STOP :
      _send_kctrl_command(SIS_TO_K::STOP_PING);
      d->m_status.commanded_pinging = false;
      res.action = "Commanded ping stop";
      break;
    case ds_kongsberg_msgs::PingCmd::Request::PING_NEWFILE :
//      _send_kctrl_command(SIS_TO_K::STOP_PING);
//      _new_kmall_file();
      _send_kctrl_command(SIS_TO_K::LOG_IOP_SVP);
//      _send_kctrl_command(SIS_TO_K::START_PING);
      res.action = "Requested IOP SVP Header";
      break;
    case ds_kongsberg_msgs::PingCmd::Request::PING_STARTUP :
      _startup_sequence();
      res.action = "Commanded startup sequence";
      break;
    case ds_kongsberg_msgs::PingCmd::Request::PING_NEWFILE_FORCE :
      _new_kmall_file();
      res.action = "Forced open newfile";
      break;
    default:
      res.action = "Ping command not recognized";
  }
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
  if (!d->m_status.kctrl_connected
      || !d->m_status.pu_powered
      || !d->m_status.pu_connected){
    ROS_ERROR_STREAM("Sounder not connected, no BIST started.");
    res.action = "Sounder not connected, no BIST started.";
    return true;
  }

  if (d->m_status.bist_running
      && req.bist_command != ds_kongsberg_msgs::BistCmd::Request::BIST_CANCEL){
    res.action = "BIST already in progress... cancel it first";
    return true;
  }
  bist_strings bs;
  std::string bist_name;
  switch (req.bist_command){
    case ds_kongsberg_msgs::BistCmd::Request::BIST_ONDECK :
      d->bist_tests = bs.get_ondeck();
      bist_name = "bist_ondeck";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_INWATER :
      d->bist_tests = bs.get_inwater();
      bist_name = "bist_inwater";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_CANCEL :
      if (d->m_status.bist_running){
        d->m_status.bist_progress = d->bist_tests.size()-1;
        _run_next_bist();
        res.action = "BIST commanded to cancel!";
        return true;
      } else {
        res.action = "No BIST in progress, ready to start a new BIST";
        return true;
      }
    case ds_kongsberg_msgs::BistCmd::Request::BIST_CPU :
      d->bist_tests = std::vector<std::string>({bs.CPU});
      bist_name = "bist_cpu";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_TX_UNIT :
      d->bist_tests = std::vector<std::string>({bs.TX_UNIT});
      bist_name = "bist_tx_unit";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_RX_CHANNELS :
      d->bist_tests = std::vector<std::string>({bs.RX_CHANNELS});
      bist_name = "bist_rx_channels";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_RX_NOISE_SPECTRUM :
      d->bist_tests = std::vector<std::string>({bs.RX_NOISE_SPECTRUM});
      bist_name = "bist_rx_noise_spectrum";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_CBMF :
      d->bist_tests = std::vector<std::string>({bs.CBMF});
      bist_name = "bist_cbmf";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_CBMF_CPU :
      d->bist_tests = std::vector<std::string>({bs.CBMF_CPU});
      bist_name = "bist_cbmf_cpu";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_TX_VIA_RX :
      d->bist_tests = std::vector<std::string>({bs.TX_VIA_RX});
      bist_name = "bist_tx_via_rx";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_SOFTWARE_VERSIONS :
      d->bist_tests = std::vector<std::string>({bs.SOFTWARE_VERSIONS});
      bist_name = "bist_software_versions";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_RX_UNIT :
      d->bist_tests = std::vector<std::string>({bs.RX_UNIT});
      bist_name = "bist_rx_unit";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_RX_CBMF :
      d->bist_tests = std::vector<std::string>({bs.RX_CBMF});
      bist_name = "bist_rx_cbmf";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_RX_NOISE_LEVEL :
      d->bist_tests = std::vector<std::string>({bs.RX_NOISE_LEVEL});
      bist_name = "bist_rx_noise_level";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_SYSTEM_INFO :
      d->bist_tests = std::vector<std::string>({bs.SYSTEM_INFO});
      bist_name = "bist_system_info";
      break;
    default:
      res.action = "Unknown BIST command... failure!";
      return true;
  }
  d->m_status.bist_count ++;
  d->m_status.bist_running = true;
  d->m_status.bist_progress = -1;
  d->m_status.bist_filename = filename(d->m_status.bist_directory,
                                       d->m_status.bist_count,
                                       bist_name,
                                       ".txt");
  _run_next_bist();
  res.action = "BIST started... check progress in : " + d->m_status.bist_filename;
  return true;
}
bool
KongsbergEM2040::_load_xml_cmd(ds_kongsberg_msgs::LoadXmlCmd::Request &req, ds_kongsberg_msgs::LoadXmlCmd::Response &res)
{
  res.command = _read_kctrl_xml(req.xml_filename);
  return true;
}

// XXXXXXXXXXXXXXXX
// Connection calls
// ----------------
void
KongsbergEM2040::_on_kmall_data(ds_core_msgs::RawData raw)
{
  DS_D(KongsbergEM2040);

  if (!parse_data(raw)){
    ROS_ERROR_STREAM("KMAll data parse failed OR incomplete packet");
  } else {
    std::unique_lock<std::mutex> lck(d->m_status_mutex);
    d->kmall_timer.stop();
    d->kmall_timer.start();
    d->m_status.kmall_connected = true;
  }
}
void
KongsbergEM2040::_on_kctrl_data(ds_core_msgs::RawData raw)
{
  DS_D(KongsbergEM2040);
  if (!parse_message(raw)){
    ROS_ERROR_STREAM("KCtrl message parse failed");
  } else {
    std::unique_lock<std::mutex> lck(d->m_status_mutex);
    d->kctrl_timer.stop();
    d->kctrl_timer.start();
    d->m_status.kctrl_connected = true;
  }
}

// XXXXXXXXXXXXXXXXXXXXXX
// Command send shortcuts
// ----------------------
void
KongsbergEM2040::_startup_sequence()
{
  DS_D(KongsbergEM2040);
  _send_kctrl_command(SIS_TO_K::START);
  _send_kctrl_command(SIS_TO_K::SET_READY);
  if (d->m_status.commanded_pinging){
    _send_kctrl_command(SIS_TO_K::LOG_IOP_SVP);
    _send_kctrl_command(SIS_TO_K::START_PING);
  }
}
std::string
KongsbergEM2040::_send_kctrl_command(int cmd)
{
  DS_D(KongsbergEM2040);
  std::stringstream ss;
  ss << "$KSSIS,"
      << cmd << ","
      << d->m_status.sounder_name;
  auto msg = ss.str();
  d->kctrl_conn_->send(msg);
  return msg;
}

template <class T1>
std::string
KongsbergEM2040::_send_kctrl_param(std::string param, T1 val)
{
  return _send_kctrl_param(std::vector<std::string>{param}, std::vector<std::string>{val});
}

template <class T1>
std::string
KongsbergEM2040::_send_kctrl_param(std::vector<std::string> params, std::vector<T1> vals)
{
  DS_D(KongsbergEM2040);
  std::stringstream ss;
  ss << "$KSSIS,"
     << SIS_TO_K::SETVALUES << ","
     << d->m_status.sounder_name;
  for (int i=0; i<params.size(); i++){
    ss << ","
       << params[i] << "="
       << vals[i];
  }
  auto msg = ss.str();
  d->kctrl_conn_->send(msg);
  return msg;
}

void
KongsbergEM2040::_print_bist(std::string name, std::string status, std::string msg)
{
  DS_D(KongsbergEM2040);
  if (!d->m_status.bist_running){
    return;
  }
  d->bist_summary_stream << status << "\t" << name << "\n";
  std::ofstream fs;
  fs.open (d->m_status.bist_filename, std::ios::app);
  fs << "\n";
  fs << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n";
  fs << name << "\t" << status << "\n";
  fs << "--------------------------------\n";
  fs << msg;
  fs.close();
}

void
KongsbergEM2040::_run_next_bist()
{
  DS_D(KongsbergEM2040);
  if (!d->m_status.bist_running){
    return;
  }
  d->m_status.bist_progress++;
  if (d->m_status.bist_progress == 0){
    d->bist_summary_stream.str("");
    std::ofstream fs;
    fs.open (d->m_status.bist_filename, std::ios::app);
    fs << "BIST started at " << boost::posix_time::second_clock::universal_time() << "\n";
    fs << "Includes tests...\n";
    for (const auto test : d->bist_tests){
      fs << "\t" << test << "\n";
    }
    fs << "--------------------------------\n";
    fs.close();
  }
  if (d->m_status.bist_progress < d->bist_tests.size() ){
    ROS_INFO_STREAM("Running BIST ... "<< d->bist_tests[d->m_status.bist_progress]);
    d->m_status.bist_current = d->bist_tests[d->m_status.bist_progress];
    // Send the next command
    _send_kctrl_param("INST_PARAM_BIST_DO", d->bist_tests[d->m_status.bist_progress]);
  } else if (d->m_status.bist_progress >= d->bist_tests.size()){
    std::ofstream fs;
    fs.open (d->m_status.bist_filename, std::ios::app);
    fs << "\nXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n\n";
    fs << "BIST completed at " << boost::posix_time::second_clock::universal_time() << "\n";
    fs << "Ran tests...\n";
    fs << d->bist_summary_stream.str();
    fs.close();
    ROS_INFO_STREAM("BIST done! \n"<<d->bist_summary_stream.str()<<"Full results in "<<d->m_status.bist_filename);
    d->bist_tests.resize(0);
    d->m_status.bist_running = false;
    d->m_status.bist_current = "";
  }
}
void
KongsbergEM2040::_new_kmall_file()
{
  DS_D(KongsbergEM2040);
  d->m_status.kmall_filecount ++;
  d->m_status.kmall_filename = filename(d->m_status.kmall_directory,
                                        d->m_status.kmall_filecount,
                                        d->m_status.ship_name,
                                        ".kmall");
  if (d->kmall_stream == NULL){
    d->kmall_stream = new std::ofstream();
  } else {
    d->kmall_stream->close();
  }
  d->kmall_stream->open (d->m_status.kmall_filename, std::ios::out | std::ios::binary);
  d->kmall_buffer_size = 0;
  d->kmall_file_size = 0;
  d->m_status.kmall_filesize_kB = 0;
  ROS_ERROR_STREAM("New kmall file: " << d->m_status.kmall_filename);
}
void
KongsbergEM2040::_write_kmall_data(ds_core_msgs::RawData& raw)
{
  DS_D(KongsbergEM2040);
  if (d->kmall_stream->is_open()){
    auto size = raw.data.size();
    auto data = reinterpret_cast<const char*>(raw.data.data());
    d->kmall_stream->write(data, size);
    d->kmall_buffer_size += size;
    d->kmall_file_size += size;
    d->m_status.kmall_filesize_kB += d->kmall_file_size / 1e3;
    if (d->kmall_file_size > d->kmall_max_file_size){
      _send_kctrl_command(SIS_TO_K::LOG_IOP_SVP);
    } else if (d->kmall_buffer_size > d->kmall_max_buffer_size){
      d->kmall_stream->flush();
      d->kmall_buffer_size = 0;
    }
  }
}

void
KongsbergEM2040::_write_kctrl_xml(ds_core_msgs::RawData& raw)
{
  DS_D(KongsbergEM2040);
  auto xml_string = std::string{ reinterpret_cast<const char*>(raw.data.data()), raw.data.size() };
  d->m_status.xml_filecount++;
  d->m_status.xml_filename = filename(d->m_status.xml_directory,
                                      d->m_status.xml_filecount,
                                      d->m_status.sounder_name,
                                      ".xml");
  std::ofstream fs;
  fs.open (d->m_status.xml_filename, std::ios::binary);
  fs << xml_string;
  fs.close();
  ROS_ERROR_STREAM("Logged XML in "<<d->m_status.xml_filename);
  // Now look for some important key-value pairs
  std::vector<std::string> params, vals;
  std::tie(params, vals) = string_split_out_xml_params(xml_string);
  if (params.size() != vals.size()){
    ROS_ERROR_STREAM("Bad XML received, logged anyways");
    return;
  }
  for (int i=0; i<params.size(); i++){
    if (params[i] == "SDPM1") {
      ROS_INFO_STREAM("PING FREQ: " << params[i] << " val: " << vals[i]);
      d->m_status.rt_ping_freq = vals[i];
    } else if (params[i] == "SDFD") {
      ROS_INFO_STREAM("FORCE DEPTH param: "<<params[i]<<" val: "<<vals[i]);
      d->m_status.rt_force_depth = vals[i];
    } else if (params[i] == "SDMA") {
      ROS_INFO_STREAM("MAX DEPTH param: "<<params[i]<<" val: "<<vals[i]);
      d->m_status.rt_max_depth = vals[i];
    } else if (params[i] == "SDMI") {
      ROS_INFO_STREAM("MIN DEPTH param: " << params[i] << " val: " << vals[i]);
      d->m_status.rt_min_depth = vals[i];
    } else if (params[i] == "SDDM") {
      ROS_INFO_STREAM("DETECTOR MODE param: "<<params[i]<<" val: "<<vals[i]);
      d->m_status.rt_detector_mode = vals[i];
    } else if (params[i] == "SDFM") {
      ROS_INFO_STREAM("FM DISABLE param: " << params[i] << " val: " << vals[i]);
      d->m_status.rt_fm_disable = vals[i];
    } else if (params[i] == "SDED") {
      ROS_INFO_STREAM("EXTRA DETECT param: "<<params[i]<<" val: "<<vals[i]);
      d->m_status.rt_extra_detect = vals[i];
    } else if (params[i] == "SDPT1") {
      ROS_INFO_STREAM("DEPTH MODE param: "<<params[i]<<" val: "<<vals[i]);
      d->m_status.rt_depth_mode = vals[i];
    } else if (params[i] == "STET") {
      ROS_INFO_STREAM("TRIGGER param: "<<params[i]<<" val: "<<vals[i]);
      d->m_status.rt_trigger = vals[i];
    } else if (params[i] == "STYAMO") {
      ROS_INFO_STREAM("YAW STAB param: "<<params[i]<<" val: "<<vals[i]);
      d->m_status.rt_yaw_stab = vals[i];
    } else if (params[i] == "STXA") {
      ROS_INFO_STREAM("TX ANGLE param: " << params[i] << " val: " << vals[i]);
      d->m_status.rt_tx_angle_along = vals[i];
    } else if (params[i] == "STPS") {
      ROS_INFO_STREAM("PITCH STAB param: " << params[i] << " val: " << vals[i]);
      d->m_status.rt_pitch_stab = vals[i];
    } else if (params[i] == "STPF") {
      ROS_INFO_STREAM("PING RATE param: " << params[i] << " val: " << vals[i]);
      d->m_status.rt_max_ping_rate = vals[i];
    } else if (params[i] == "STPK") {
      ROS_INFO_STREAM("MIN SWATH param: " << params[i] << " val: " << vals[i]);
      d->m_status.rt_min_swath_distance = vals[i];
    }
  }
  d->kmstatus_pub_.publish(d->m_status);
}

std::string
KongsbergEM2040::_read_kctrl_xml(std::string filename)
{
  DS_D(KongsbergEM2040);
  std::vector<std::string> params, vals;
  std::tie(params, vals) = file_split_out_xml_params(filename);
  ROS_ERROR_STREAM(params.size() << " params found, " << vals.size() << " vals found");
  if (params.size() != vals.size()){
    return "No command sent, params and vals don't match";
  }
  return _send_kctrl_param(params, vals);
}

// XXXXXXXXXXXXXXX
// Status timeouts
// ---------------

void
KongsbergEM2040::_on_kctrl_timeout(const ros::TimerEvent&)
{
  DS_D(KongsbergEM2040);
  std::unique_lock<std::mutex> lck(d->m_status_mutex);
  if (d->m_status.kctrl_connected){
    ROS_ERROR_STREAM("Kctrl timed out... assume totally disconnected");
  }
  d->m_status.kctrl_connected = false;
  d->m_status.pu_powered = false;
  d->m_status.pu_connected = false;
  d->m_status.bist_running = false;
  d->kmstatus_pub_.publish(d->m_status);
}

void
KongsbergEM2040::_on_pu_powered_timeout(const ros::TimerEvent&)
{
  DS_D(KongsbergEM2040);
  std::unique_lock<std::mutex> lck(d->m_status_mutex);
  if (d->m_status.pu_powered){
    ROS_ERROR_STREAM("PU power timed out... assume powered off");
  }
  d->m_status.pu_powered = false;
  d->m_status.pu_connected = false;
  d->m_status.bist_running = false;
  d->kmstatus_pub_.publish(d->m_status);
}

void
KongsbergEM2040::_on_pu_connected_timeout(const ros::TimerEvent&)
{
  DS_D(KongsbergEM2040);
  std::unique_lock<std::mutex> lck(d->m_status_mutex);
  if (d->m_status.pu_connected){
    ROS_ERROR_STREAM("PU connection timed out... assume disconnected");
  }
  d->m_status.pu_connected = false;
  d->m_status.bist_running = false;
  d->kmstatus_pub_.publish(d->m_status);
}

void
KongsbergEM2040::_on_kmall_timeout(const ros::TimerEvent&)
{
  DS_D(KongsbergEM2040);
  std::unique_lock<std::mutex> lck(d->m_status_mutex);
  if (d->m_status.kmall_connected){
    ROS_ERROR_STREAM("Kmall stream timed out... assume totally disconnected");
  }
  d->m_status.kmall_connected = false;
  d->m_status.pinging = false;
  d->kmstatus_pub_.publish(d->m_status);
}

void
KongsbergEM2040::_on_pinging_timeout(const ros::TimerEvent&)
{
  DS_D(KongsbergEM2040);
  std::unique_lock<std::mutex> lck(d->m_status_mutex);
  if(d->m_status.commanded_pinging && d->m_status.kmall_connected){
    ROS_ERROR_STREAM("Ping timed out... attempt to restart pinging");
    _send_kctrl_command(SIS_TO_K::LOG_IOP_SVP);
    _send_kctrl_command(SIS_TO_K::START_PING);
  }
  d->m_status.pinging = false;
  d->kmstatus_pub_.publish(d->m_status);
}

} //namespace
