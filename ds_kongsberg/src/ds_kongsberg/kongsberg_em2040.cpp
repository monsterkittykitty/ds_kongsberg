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

//void
//KongsbergEM2040::parse_message(ds_core_msgs::RawData& raw)
//{
//  int min_size = sizeof(EMdgmHeader);
//  if (raw.data.size() < min_size){
//    ROS_ERROR_STREAM("Raw Data received less than minimum size "<<min_size<<" bytes");
//    return;
//  }
//  uint8_t* bytes_ptr = raw.data.data();
//  auto hdr = reinterpret_cast<EMdgmHeader*>(bytes_ptr);
//  if (raw.data.size() != hdr->numBytesDgm){
//    ROS_ERROR_STREAM("Raw Data size received ("<<raw.data.size()<<") != hdr->numBytes ("<<hdr.numBytesDgm<<")");
//    return;
//  }
//  if (hdr.dgmType==EM_DGM_I_INSTALLATION_PARAM){
//    ROS_INFO_STREAM("EM_DGM_I_INSTALLATION_PARAM");
//  } else if (hdr.dgmType==EM_DGM_I_OP_RUNTIME){
//    ROS_INFO_STREAM("EM_DGM_I_OP_RUNTIME");
//  } else if (hdr.dgmType==EM_DGM_S_POSITION){
//    ROS_INFO_STREAM("EM_DGM_S_POSITION");
//  } else if (hdr.dgmType==EM_DGM_S_KM_BINARY){
//    ROS_INFO_STREAM("EM_DGM_S_KM_BINARY");
//  } else if (hdr.dgmType==EM_DGM_S_SOUND_VELOCITY_PROFILE){
//    ROS_INFO_STREAM("EM_DGM_S_SOUND_VELOCITY_PROFILE");
//  } else if (hdr.dgmType==EM_DGM_S_SOUND_VELOCITY_TRANSDUCER){
//    ROS_INFO_STREAM("EM_DGM_S_SOUND_VELOCITY_TRANSDUCER");
//  } else if (hdr.dgmType==EM_DGM_S_CLOCK){
//    ROS_INFO_STREAM("EM_DGM_S_CLOCK");
//  } else if (hdr.dgmType==EM_DGM_S_DEPTH){
//    ROS_INFO_STREAM("EM_DGM_S_DEPTH");
//  } else if (hdr.dgmType==EM_DGM_S_HEIGHT){
//    ROS_INFO_STREAM("EM_DGM_S_HEIGHT");
//  } else
//  if (hdr.dgmType==EM_DGM_M_RANGE_AND_DEPTH){
//    ROS_INFO_STREAM("EM_DGM_M_RANGE_AND_DEPTH");
//    auto mrz = read_mrz(bytes_ptr);
////    auto mrz = reinterpret_cast<EMdgmMRZ*>(bytes_ptr);
//    auto mb_raw = mrz_to_mb_raw(mrz);
//  } else if (hdr.dgmType==EM_DGM_M_WATER_COLUMN){
//    ROS_INFO_STREAM("EM_DGM_M_WATER_COLUMN");
//
//  } else if (hdr.dgmType==EM_DGM_C_POSITION){
//    ROS_INFO_STREAM("EM_DGM_C_POSITION");
//  } else if (hdr.dgmType==EM_DGM_C_HEAVE){
//    ROS_INFO_STREAM("EM_DGM_C_HEAVE");
//  }
//}

//EMdgmMRZ read_mrz(uint8_t* bytes)
//{
//  EMdgmMRZ mrz;
//  int count = 0;
//  uint8_t* ptr = bytes;
//  mrz.header = *(reinterpret_cast<EMdgmHeader*>(ptr + count));
//  count += sizeof(mrz.header);
//  mrz.partition = *(reinterpret_cast<EMdgmpartition*>(ptr + count));
//  count += sizeof(mrz.partition);
//  mrz.cmnPart = *(reinterpret_cast<EMdgmMbody*>(ptr + count));
//  count += mrz.cmnPart.numBytesCmnPart;
//  mrz.pingInfo = *(reinterpret_cast<EMdgmMRZ_pingInfo*>(ptr + count));
//  count += mrz.pingInfo.numBytesInfoData;
//
//  for (int i=0; i<MAX_NUM_TX_PULSES; i++){
//    mrz.sectorInfo[i] = *(reinterpret_cast<EMdgmMRZ_pingInfo*>(ptr + count));
//    count += sizeof(mrz.sectorInfo[i]);
//  }
//
//  mrz.rxInfo = *(reinterpret_cast<EMdgmMRZ_rxInfo*>(ptr + count));
//  count += mrz.pingInfo.numBytesRxInfo;
//  for (i=0; i<MAX_EXTRA_DET_CLASSES; i++){
//    mrz.extraDetClassInfo[i] = *(reinterpret_cast<EMdgmMRZ_extraDetClassInfo*>(ptr + count));
//    count += sizeof(mrz.extraDetClassInfo[i]);
//  }
//  for (i=0; i<MAX_NUM_BEAMS+MAX_EXTRA_DET; i++){
//    mrz.sounding[i] = *(reinterpret_cast<EMdgmMRZ_sounding*>(ptr + count));
//    count += sizeof(mrz.sounding[i]);
//  }
//  for (i=0; i<MAX_SIDESCAN_SAMP; i++){
//    mrz.SIsample_desidB[i] = *(reinterpret_cast<uint16_t*>(ptr + count));;
//  }
//
//
//
//  return mrz;
//}

//ds_multibeam_msgs::MultibeamRaw
//KongsbergEM2040::mrz_to_mb_raw(EMdgmMRZ* msg)
//{
////  struct EMdgmHeader_def 	header
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

//ds_multibeam_msgs::MultibeamGrid
//KongsbergEM2040::mb_raw_to_mb_grid(ds_multibeam_msgs::MultibeamRaw* msg)
//{
//
//}
//
//ds_multibeam_msgs::MultibeamGridStats
//KongsbergEM2040::mb_raw_to_mb_grid_stats(ds_multibeam_msgs::MultibeamRaw* msg)
//{
//
//}
//
//ds_multibeam_msgs::MultibeamFilterStats
//KongsbergEM2040::mb_raw_to_mb_filter_stats(ds_multibeam_msgs::MultibeamRaw* msg)
//{
//
//}
//
//sensor_msgs::Image
//KongsbergEM2040::mwc_to_image(EMdgmMWC* msg)
//{
//
//}

void
KongsbergEM2040::setupConnections()
{
  ds_base::DsProcess::setupConnections();
  DS_D(KongsbergEM2040);
  d->kmall_conn_ = addConnection("kmall_connection", boost::bind(&KongsbergEM2040::onKMallData, this, _1));
  d->kctrl_conn_ = addConnection("kctrl_connection", boost::bind(&KongsbergEM2040::onKctrlData, this, _1));
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
  d->sounder_name_ = ros::param::param<std::string>("~sounder_name", "DEFAULT_SOUNDER");

  // systemID
  // echoSounderID
}
void
KongsbergEM2040::setupSubscriptions()
{
  ds_base::DsProcess::setupSubscriptions();
//  DS_D(KongsbergEM2040);

}
void
KongsbergEM2040::setupPublishers()
{
  ds_base::DsProcess::setupPublishers();
//  DS_D(KongsbergEM2040);
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
  DS_D(KongsbergEM2040);
  d->kctrl_conn_->send("POWER COMMAND");
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
KongsbergEM2040::onKMallData(ds_core_msgs::RawData raw)
{
  DS_D(KongsbergEM2040);
  d->kctrl_conn_->send("RECEIVED KMALL DATA");
}

void
KongsbergEM2040::onKctrlData(ds_core_msgs::RawData raw)
{
  DS_D(KongsbergEM2040);
  d->kctrl_conn_->send("RECEIVED KCTRL DATA");
}

} //namespace