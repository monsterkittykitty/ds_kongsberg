//
// Created by jvaccaro on 7/25/19.
//

#include "ds_kongsberg/kongsberg_em2040_util.h"

#include <list>
#include <gtest/gtest.h>
#include <ros/param.h>

TEST_F(Em2040Util, test_split_out_params_pass){
  std::string xml = "<TOKEN>\n<ID>ABC</ID>\n<VALUE>DEF</VALUE>\n</TOKEN>";
  auto res = ds_kongsberg::split_out_params(xml);
  ASSERT_STREQ(res.first[0], "ABC");
  ASSERT_STREQ(res.second[0], "DEF");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_em2040_utils");
  auto ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}