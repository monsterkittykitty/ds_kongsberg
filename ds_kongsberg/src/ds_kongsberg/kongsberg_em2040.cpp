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

KongsbergEM2040::~KongsbergEM2040()
{
  // shut down kctrl
  _send_kctrl_command(SIS_TO_K::TERMINATE);
}

bool
KongsbergEM2040::parse_message(ds_core_msgs::RawData& raw)
{
  DS_D(KongsbergEM2040);
  ds_kongsberg_msgs::KongsbergKSSIS msg;
  // Split on white space and pull specific indexes
  auto str = std::string{ reinterpret_cast<const char*>(raw.data.data()), raw.data.size() };
  auto buf = std::istringstream{ str };
  std::vector<std::string> fields;//=
//      std::vector<std::string>{ std::istream_iterator<std::string>(buf), std::istream_iterator<std::string>() };

  while( buf.good() )
  {
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
    return false;
  }

  std::string msg_type(hdr->dgmType, hdr->dgmType + sizeof(hdr->dgmType)/sizeof(hdr->dgmType[0]));
  uint8_t* raw_array = reinterpret_cast<uint8_t*>(raw.data.data());
  std::string raw_str(raw_array, raw_array + sizeof(raw_array));
  if (msg_type==EM_DGM_I_INSTALLATION_PARAM)
  {
    ROS_ERROR_STREAM("EM_DGM_I_INSTALLATION_PARAM");
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
    ROS_ERROR_STREAM("EM_DGM_S_CLOCK");
    auto msg = reinterpret_cast<EMdgmSCL*>(bytes_ptr);
    ds_core_msgs::ClockOffset offset;
    offset.header = raw.header;
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
//      auto mbf = mb_raw_to_mb_filter_stats(&mbr);
//      d->mbfilter_pub_.publish(mbf);
//      auto mbg = mb_raw_to_mb_grid(&mbr);
//      d->mbgrid_pub_.publish(mbg);
//      auto mbgs = mb_raw_to_mb_grid_stats(&mbr);
//      d->mbgridstats_pub_.publish(mbgs);
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
KongsbergEM2040::mrz_to_mb_raw(EMdgmMRZ* msg) {
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
////  struct EMdgmMpartition_def 	partition
////  struct EMdgmMbody_def 	cmnPart
////  struct EMdgmMRZ_pingInfo_def 	pingInfo
////  struct EMdgmMRZ_txSectorInfo_def 	sectorInfo [MAX_NUM_TX_PULSES]
////  struct EMdgmMRZ_rxInfo_def 	rxInfo
////  struct EMdgmMRZ_extraDetClassInfo_def 	extraDetClassInfo [MAX_EXTRA_DET_CLASSES]
////  struct EMdgmMRZ_sounding_def 	sounding [MAX_NUM_BEAMS+MAX_EXTRA_DET]
////  int16_t 	SIsample_desidB [MAX_SIDESCAN_SAMP]
//
//// already taken care of, but get the timing out
//  msg->header;
//    // uint32_t   numBytesDgm
////  uint8_t 	dgmType [4]
////  uint8_t 	dgmVersion
////  uint8_t 	systemID
////  uint16_t 	echoSounderID
////  uint32_t 	time_sec
////  uint32_t 	time_nanosec
//  msg->partition;
//// If a multibeam depth datagram (or any other large datagram) exceeds the limit of an UDP package (64 kB), the
//// datagram is split into several datagrams =< 64 kB before sending from the PU. The parameters in this struct will
//// give information of the partitioning of datagrams. K-Controller/SIS merges all UDP packets/datagram parts to one
//// datagram, and store it as one datagram in the .kmall files. Datagrams stored in .kmall files will therefore always
//// have numOfDgm = 1 and dgmNum = 1, and may have size > 64 kB. The maximum number of partitions from PU is given by
//// MAX_NUM_MWC_DGMS and MAX_NUM_MRZ_DGMS.
////  uint16_t 	numOfDgms
////  uint16_t 	dgmNum
//  msg->cmnPart;
////  uint16_t 	numBytesCmnPart
////  uint16_t 	pingCnt
////  uint8_t 	rxFansPerPing
////  uint8_t 	rxFanIndex
////  uint8_t 	swathsPerPing
////  uint8_t 	swathAlongPosition
////  uint8_t 	txTransducerInd
////  uint8_t 	rxTransducerInd
////  uint8_t 	numRxTransducers
////  uint8_t 	algorithmType
//  msg->pingInfo;
////    uint16_t numBytesInfoData;
////    uint16_t padding0;
////    float pingRate_Hz;
////    uint8_t beamSpacing;
////    uint8_t depthMode;
////    uint8_t subDepthMode;
////    uint8_t distanceBtwSwath;
////    uint8_t detectionMode;
////    uint8_t pulseForm;
////    uint16_t padding1;
////
////    float frequencyMode_Hz;
////    float freqRangeLowLim_Hz;
////    float freqRangeHighLim_Hz;
////    float maxTotalTxPulseLength_sec;
////    float maxEffTxPulseLength_sec;
////    float maxEffTxBandWidth_Hz;
////    float absCoeff_dBPerkm;
////    float portSectorEdge_deg;
////    float starbSectorEdge_deg;
////    float portMeanCov_deg;
////    float starbMeanCov_deg;
////    int16_t portMeanCov_m;
////    int16_t starbMeanCov_m;
////    uint8_t modeAndStabilisation;
////    uint8_t runtimeFilter1;
////    uint16_t runtimeFilter2;
////    uint32_t pipeTrackingStatus;
////    float transmitArraySizeUsed_deg;
////    float receiveArraySizeUsed_deg;
////    float transmitPower_dB;
////    uint16_t SLrampUpTimeRemaining;
////    uint16_t padding2;
////    float yawAngle_deg;
////    uint16_t numTxSectors;
////    uint16_t numBytesPerTxSector;
////    float headingVessel_deg;
////    float soundSpeedAtTxDepth_mPerSec;
////    float txTransducerDepth_m;
////    float z_waterLevelReRefPoint_m;
////    float x_kmallToall_m;
////    float y_kmallToall_m;
////    uint8_t latLongInfo;
////    uint8_t posSensorStatus;
////    uint8_t attitudeSensorStatus;
////    uint8_t padding3;
////    double latitude_deg;
////    double longitude_deg;
////    float ellipsoidHeightReRefPoint_m;
//  msg->sectorInfo[MAX_NUM_TX_PULSES];
////  uint8_t txSectorNumb;
////  uint8_t txArrNumber;
////  uint8_t txSubArray;
////  uint8_t padding0;
////  float sectorTransmitDelay_sec;
////  float tiltAngleReTx_deg;
////  float txNominalSourceLevel_dB;
////  float txFocusRange_m;
////  float centreFreq_Hz;
////  float signalBandWidth_Hz;
////  float totalSignalLength_sec;
////  uint8_t pulseShading;
////  uint8_t signalWaveForm;
////  uint16_t padding1;
//  msg->rxInfo;
////  uint16_t 	numBytesRxInfo
////  uint16_t 	numSoundingsMaxMain
////  uint16_t 	numSoundingsValidMain
////  uint16_t 	numBytesPerSounding
////  float 	WCSampleRate
////  float 	seabedImageSampleRate
////  float 	BSnormal_dB
////  float 	BSoblique_dB
////  uint16_t 	extraDetectionAlarmFlag
////  uint16_t 	numExtraDetections
////  uint16_t 	numExtraDetectionClasses
////  uint16_t 	numBytesPerClass
//  msg->extraDetClassInfo[MAX_EXTRA_DET_CLASSES];
////  uint16_t numExtraDetInClass;
////  int8_t padding;
////  uint8_t alarmFlag;
//  msg->sounding[MAX_NUM_BEAMS + MAX_EXTRA_DET];
////  uint16_t soundingIndex;
////  uint8_t txSectorNumb;
////  uint8_t detectionType;
////  uint8_t detectionMethod;
////  uint8_t rejectionInfo1;
////  uint8_t rejectionInfo2;
////  uint8_t postProcessingInfo;
////  uint8_t detectionClass;
////  uint8_t detectionConfidenceLevel;
////  uint16_t padding;
////  float rangeFactor;
////  float qualityFactor;
////  float detectionUncertaintyVer_m;
////  float detectionUncertaintyHor_m;
////  float detectionWindowLength_sec;
////  float echoLength_sec;
////  uint16_t WCBeamNumb;
////  uint16_t WCrange_samples;
////  float WCNomBeamAngleAcross_deg;
////  float meanAbsCoeff_dBPerkm;
////  float reflectivity1_dB;
////  float reflectivity2_dB;
////  float receiverSensitivityApplied_dB;
////  float sourceLevelApplied_dB;
////  float BScalibration_dB;
////  float TVG_dB;
////  float beamAngleReRx_deg;
////  float beamAngleCorrection_deg;
////  float twoWayTravelTime_sec;
////  float twoWayTravelTimeCorrection_sec;
////  float deltaLatitude_deg;
////  float deltaLongitude_deg;
////  float z_reRefPoint_m;
////  float y_reRefPoint_m;
////  float x_reRefPoint_m;
////  float beamIncAngleAdj_deg;
////  uint16_t realTimeCleanInfo;
////  uint16_t SIstartRange_samples;
////  uint16_t SIcentreSample;
////  uint16_t SInumSamples;
//  msg->SIsample_desidB[MAX_SIDESCAN_SAMP];
//}

sensor_msgs::Image
KongsbergEM2040::mwc_to_image(EMdgmMWC* msg)
{
  return {};
}

void
KongsbergEM2040::setupConnections()
{
  ds_base::DsProcess::setupConnections();
  DS_D(KongsbergEM2040);
  d->kmall_conn_ = addConnection("kmall_connection", boost::bind(&KongsbergEM2040::_on_kmall_data, this, _1));
  d->kctrl_conn_ = addConnection("kctrl_connection", boost::bind(&KongsbergEM2040::_on_kctrl_data, this, _1));
//  d->kmall_two_ = addConnection("kmall_multi_two", boost::bind(&KongsbergEM2040::_on_kmall_data, this, _1));
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
}

void
KongsbergEM2040::setupParameters()
{
  ds_base::DsProcess::setupParameters();
  DS_D(KongsbergEM2040);
  d->sounder_name_ = ros::param::param<std::string>("~sounder_name", "EM2040_40");
  d->started_ = false;

  // systemID
  // echoSounderID
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
//  auto mbfilter_topic = ros::param::param<std::string>("~mbfilter_topic", "mbfilter");
//  d->mbfilter_pub_ = nh.advertise<ds_multibeam_msgs::MultibeamFilterStats>(mbfilter_topic, 1000);
//  auto mbgrid_topic = ros::param::param<std::string>("~mbgrid_topic", "mbgrid");
//  d->mbgrid_pub_ = nh.advertise<ds_multibeam_msgs::MultibeamGrid>(mbgrid_topic, 1000);
//  auto mbgridstats_topic = ros::param::param<std::string>("~mbgridstats_topic", "mbgridstats");
//  d->mbgridstats_pub_ = nh.advertise<ds_multibeam_msgs::MultibeamGridStats>(mbgridstats_topic, 1000);
  auto watercolumn_topic = ros::param::param<std::string>("~watercolumn_topic", "watercolumn");
  d->watercolumn_pub_ = nh.advertise<sensor_msgs::Image>(watercolumn_topic, 1000);
  auto pointcloud_topic = ros::param::param<std::string>("~pointcloud_topic", "pointcloud");
  d->pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 1000);
  auto kmstatus_topic = ros::param::param<std::string>("~kmstatus_topic", "kmstatus");
  d->kmstatus_pub_ = nh.advertise<ds_kongsberg_msgs::KongsbergKSSIS>(kmstatus_topic, 1000);
  auto offset_topic = ros::param::param<std::string>("~offset_topic", "pu_offset");
  d->offset_pub_ = nh.advertise<ds_core_msgs::ClockOffset>(offset_topic, 1000);
}

bool
KongsbergEM2040::_ping_cmd(ds_kongsberg_msgs::PingCmd::Request req, ds_kongsberg_msgs::PingCmd::Response res)
{
  DS_D(KongsbergEM2040);
  d->kctrl_conn_->send("PING COMMAND");
  return true;
}

bool
KongsbergEM2040::_power_cmd(ds_kongsberg_msgs::PowerCmd::Request req, ds_kongsberg_msgs::PowerCmd::Response res)
{
//  _startup_sequence();
  DS_D(KongsbergEM2040);
//  d->kctrl_conn_->send("POWER COMMAND");
  _send_kctrl_command(req.power);
  d->kmall_conn_->send("POWER COMMAND VIA KMALL");
  return true;
}

bool
KongsbergEM2040::_settings_cmd(ds_kongsberg_msgs::SettingsCmd::Request req, ds_kongsberg_msgs::SettingsCmd::Response res)
{
  DS_D(KongsbergEM2040);
  d->kctrl_conn_->send("SETTINGS COMMAND");
  return true;
}

void
KongsbergEM2040::_on_kmall_data(ds_core_msgs::RawData raw)
{
  parse_data(raw);
  DS_D(KongsbergEM2040);
  d->kctrl_conn_->send("RECEIVED KMALL DATA");
}

void
KongsbergEM2040::_on_kctrl_data(ds_core_msgs::RawData raw)
{
  parse_message(raw);
}

void
KongsbergEM2040::_startup_sequence()
{

  _send_kctrl_command(SIS_TO_K::START);
  _send_kctrl_command(SIS_TO_K::SET_READY);
  DS_D(KongsbergEM2040);
  ROS_INFO_STREAM("Sounder \'"<<d->sounder_name_<<"\' started and set ready!");
  d->started_ = true;
}

void
KongsbergEM2040::_send_kctrl_command(int cmd)
{
  DS_D(KongsbergEM2040);
  std::stringstream msg;
  msg << "$KSSIS,"
      << cmd << ","
      << d->sounder_name_;
  d->kctrl_conn_->send(msg.str());
}

} //namespace