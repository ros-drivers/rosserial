#include "ros/msg.h"
#include <gtest/gtest.h>
#include <math.h>

class TestFloat64 : public ::testing::Test
{
public:
  union
  {
    double val;
    unsigned char buffer[8];
  };

  static const double cases[];
  static const int num_cases;
};

const double TestFloat64::cases[] =
{
  0.0, 10.0, 15642.1, -50.2, 0.0001, -0.321,
  123456.789, -987.654321, 3.4e38, -3.4e38,
  0.0,        -0.0,         0.1,         -0.1,
  M_PI,       -M_PI,  123456.789,  -123456.789,
  INFINITY,   -INFINITY,         NAN, INFINITY - INFINITY,
  1e38,       -1e38,        1e39,        -1e39,
  1e-38,      -1e-38,       1e-39,       -1e-39,
  3.14159e-37,-3.14159e-37, 3.14159e-43, -3.14159e-43,
  1e-60,      -1e-60,       1e-45,       -1e-45,
  0.99999999999999, -0.99999999999999, 127.999999999999, -127.999999999999
};
const int TestFloat64::num_cases = sizeof(TestFloat64::cases) / sizeof(double);


TEST_F(TestFloat64, testRoundTrip)
{
  for (int i = 0; i < num_cases; i++)
  {
    memset(buffer, 0, sizeof(buffer));
    ros::Msg::serializeAvrFloat64(buffer, cases[i]);
    float deserialized = 0;
    ros::Msg::deserializeAvrFloat64(buffer, &deserialized);

    if (isnan(cases[i]))
    {
        // EXPECT_FLOAT_EQ will fail on nan, because nan != nan
        EXPECT_EQ(isnan(val), true);
        EXPECT_EQ(isnan(deserialized), true);
    }
    else
    {
        // Compare against C++ cast results.
        // In some of the test cases, truncation and loss of precision is expected
        // but it should be the same as what C++ compiler implements for (float).
        EXPECT_FLOAT_EQ(static_cast<float>(cases[i]), static_cast<float>(val));
        EXPECT_FLOAT_EQ(static_cast<float>(cases[i]), static_cast<float>(deserialized));
    }
  }
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
