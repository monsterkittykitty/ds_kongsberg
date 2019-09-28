//
// Created by jvaccaro on 5/30/19.
//
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

#include "ds_vfr_to_gga/ds_vfr_to_gga.h"
#include "ds_util/int_to_hex.h"
#include <gtest/gtest.h>
#include "ds_nmea_parsers/Gga.h"
#include "ds_nmea_parsers/util.h"

TEST(VFR_TO_GGA, valid) {
  
  
  const auto test_pairs =
      std::list<std::pair<std::string,const std::string>>{
          { "VFR 2018/03/23 00:00:00.821 9 0 SOLN_DEADRECK 179.035734 -34.905139 0.000 2.110 100 0.00 197.92 "
              , "$GPGGA,000000.000,3454.308340,S,17902.144040,E,2,08,1.0,0.000,M,0.000,M,1012*57"},
          { "VFR 2018/03/23 00:00:00.821 9 0 SOLN_DEADRECK -179.035734 -34.905139 0.000 2.110 100 0.00 197.92 "
              , "$GPGGA,000000.000,3454.308340,S,17902.144040,W,2,08,1.0,0.000,M,0.000,M,1012*45"},
          { "VFR 2018/03/23 00:00:00.821 9 0 SOLN_DEADRECK -179.035734 34.905139 0.000 2.110 100 0.00 197.92 "
              , "$GPGGA,000000.000,3454.308340,N,17902.144040,W,2,08,1.0,0.000,M,0.000,M,1012*58"},
          { "VFR 2018/03/23 00:00:00.821 9 0 SOLN_DEADRECK 179.035734 34.905139 0.000 2.110 100 0.00 197.92 "
              , "$GPGGA,000000.000,3454.308340,N,17902.144040,E,2,08,1.0,0.000,M,0.000,M,1012*74"}
      };
  for (const auto& test_pair : test_pairs)
  {
    auto test_str = test_pair.first;
    auto expected_str = test_pair.second;

    ds_core_msgs::RawData msg;
    msg.data = std::vector<unsigned char>(std::begin(test_str), std::end(test_str));

    ds_nmea_msgs::Gga expected;
    ASSERT_TRUE(ds_nmea_msgs::from_string(expected, expected_str));

    bool ok = false;
    std::string actual_str;
    std::tie(ok, actual_str) = ds_nav::VfrToGga::pack_vfr_message(msg);
    ds_nmea_msgs::Gga actual;
    ASSERT_TRUE(ds_nmea_msgs::from_string(actual, actual_str));

    EXPECT_FLOAT_EQ(expected.latitude, actual.latitude);
    EXPECT_FLOAT_EQ(expected.longitude, actual.longitude);
    EXPECT_FLOAT_EQ(expected.antenna_alt, actual.antenna_alt);
    EXPECT_FLOAT_EQ(expected.checksum, actual.checksum);
    EXPECT_FLOAT_EQ(ds_nmea_msgs::calculate_checksum(expected_str),
              ds_nmea_msgs::calculate_checksum(actual_str));
    ASSERT_EQ(expected.timestamp, actual.timestamp);

    ASSERT_TRUE(ok);
  }
}

TEST(VFR_TO_GGA, failing) {


  const auto test_strs =
      std::list<std::string>{
          "VFR 2018/03/23 00:00:00.821 9 0 SOLN_DEADRECK 179.035734 -34.905139 0.000 2.110 100 0.00 197.92 53 ", //too many
          "VFR 2018/03/23 00:00:00.821 9 0 SOLN_DEADRECK 179.035734 -34.905139 0.000 2.110 100 0.00 ", //not enough
          "VPR 2018/03/23 00:00:00.821 9 0 SOLN_DEADRECK 179.035734 -34.905139 0.000 2.110 100 0.00 197.92 ", // VPR
          "VFR 2018/03/23 00:00:00.821 9 0 SOLN_OTHER 179.035734 -34.905139 0.000 2.110 100 0.00 197.92 ", //wrong solution
      };
  for (const auto& test_str : test_strs)
  {
    ds_core_msgs::RawData msg;
    msg.data = std::vector<unsigned char>(std::begin(test_str), std::end(test_str));
    bool ok = false;
    std::string actual_str;
    std::tie(ok, actual_str) = ds_nav::VfrToGga::pack_vfr_message(msg);
    ASSERT_FALSE(ok);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}