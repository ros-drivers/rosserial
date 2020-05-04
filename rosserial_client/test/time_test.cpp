#include "ros/duration.h"
#include "ros/time.h"
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
    {0UL, 0UL}, {0UL, 500000000UL}, {1UL, 500000000UL}};

const ros::Duration TestTime::durations[] = {
    {0L, 0L}, {0L, 500000000L}, {0L, 600000000L}, {1L, 600000000L}};

const int TestTime::num_times = sizeof(TestTime::times) / sizeof(ros::Time);
const int TestTime::num_durations =
    sizeof(TestTime::durations) / sizeof(ros::Duration);

const ros::Time TestTime::additions[] = {
    {0UL, 0UL},        {0UL, 500000000UL}, {0L, 600000000UL},
    {1L, 600000000UL}, {0UL, 500000000UL}, {1UL, 0UL},
    {1L, 100000000UL}, {2L, 100000000UL},  {1UL, 500000000UL},
    {2UL, 0UL},        {2L, 100000000UL},  {3L, 100000000UL}};

const ros::Time TestTime::subtractions[] = {{0L, 0L},
                                            {0xFFFFFFFF, 500000000UL},
                                            {0xFFFFFFFF, 400000000UL},
                                            {0xFFFFFFFF - 1, 400000000UL},
                                            {0UL, 500000000UL},
                                            {0UL, 0UL},
                                            {0xFFFFFFFF, 900000000UL},
                                            {0xFFFFFFFF - 1, 900000000UL},
                                            {1UL, 500000000UL},
                                            {1UL, 0UL},
                                            {0UL, 900000000UL},
                                            {0xFFFFFFFF, 900000000UL}};

TEST_F(TestTime, testAddition) {
  for (int i = 0; i < num_times; ++i) {
    for (int j = 0; j < num_durations; ++j) {
      auto time = times[i];
      time += durations[j];
      auto idx = i * num_durations + j;
      EXPECT_EQ(time.sec, additions[idx].sec)
          << "Add " << durations[j].sec << "." << durations[j].nsec << " to "
          << times[i].sec << "." << times[i].nsec;
      EXPECT_EQ(time.nsec, additions[idx].nsec)
          << "Add " << durations[j].sec << "." << durations[j].nsec << " to "
          << times[i].sec << "." << times[i].nsec;
    }
  }
}

TEST_F(TestTime, testSubtraction) {
  for (int i = 0; i < num_times; ++i) {
    for (int j = 0; j < num_durations; ++j) {
      auto time = times[i];
      time -= durations[j];
      auto idx = i * num_durations + j;
      EXPECT_EQ(time.sec, subtractions[idx].sec)
          << "Subtract " << durations[j].sec << "." << durations[j].nsec
          << " from " << times[i].sec << "." << times[i].nsec;
      EXPECT_EQ(time.nsec, subtractions[idx].nsec)
          << "Subtract " << durations[j].sec << "." << durations[j].nsec
          << " from " << times[i].sec << "." << times[i].nsec;
    }
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
