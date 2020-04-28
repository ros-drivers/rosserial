#include "ros_lib/ros/duration.h"
#include "ros_lib/ros/time.h"
#include <gtest/gtest.h>

class TestTime : public ::testing::Test {
public:
  static const ros::Time times[];
  static const ros::Duration durations[];
  static const int num_times;
  static const int num_durations;

  static const ros::Time additions[];

  static const ros::Time subtractions[];
};

const ros::Time TestTime::times[] = {
    {0L, 0L}, {0L, 500000000L}, {1L, 500000000L}};

const ros::Duration TestTime::durations[] = {
    {0L, 0L}, {0L, 500000000L}, {0L, 600000000L}, {1L, 600000000L}};

const int TestTime::num_times = sizeof(TestTime::times) / sizeof(ros::Time);
const int TestTime::num_durations =
    sizeof(TestTime::durations) / sizeof(ros::Duration);

const ros::Time TestTime::additions[] = {
    {0L, 0L},         {0L, 500000000L}, {0L, 600000000L}, {1L, 600000000L},
    {0L, 500000000L}, {1L, 0L},         {1L, 100000000L}, {2L, 100000000L},
    {1L, 500000000L}, {2L, 0L},         {2L, 100000000L}, {3L, 100000000L}};

const ros::Time TestTime::subtractions[] = {{0L, 0L},
                                            {0xFFFFFFFF, 500000000L},
                                            {0xFFFFFFFF, 400000000L},
                                            {0xFFFFFFFF - 1, 400000000L},
                                            {0L, 500000000L},
                                            {0L, 0L},
                                            {0xFFFFFFFF, 900000000L},
                                            {0xFFFFFFFF - 1, 900000000L},
                                            {1L, 500000000L},
                                            {1L, 0L},
                                            {0L, 900000000L},
                                            {0xFFFFFFFF, 900000000L}};

TEST_F(TestTime, testAddition) {
  for (int i = 0; i < num_times; ++i) {
    for (int j = 0; j < num_durations; ++j) {
      auto time = times[i];
      time += durations[j];
      auto idx = i * num_durations + j;
      EXPECT_EQ(time.sec, additions[idx].sec);
      EXPECT_EQ(time.nsec, additions[idx].nsec);
    }
  }
}

TEST_F(TestTime, testSubtraction) {
  for (int i = 0; i < num_times; ++i) {
    for (int j = 0; j < num_durations; ++j) {
      auto time = times[i];
      time -= durations[j];
      auto idx = i * num_durations + j;
      EXPECT_EQ(time.sec, additions[idx].sec);
      EXPECT_EQ(time.nsec, additions[idx].nsec);
    }
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
