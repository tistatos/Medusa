/**
 * @File kinectTest.cpp
 *    Test file for kinect
 * @autor Erik Sandrén
 * @date 2015-02-27
 */
#include "kinect.h"
#include "gtest/gtest.h"

// Test sum function
TEST(KinectSum, generalTest) {
  EXPECT_EQ(1, Kinect::sum(1,0));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}