/*
 * Package:   zone_lib
 * Filename:  test_zone_lib.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Voltron UTD
 * License:   MIT License
 */

#include <math.h> // abs
#include <gtest/gtest.h> // Testing framework
#include "zone_lib/zone.hpp"

using namespace navigator::zones_lib;
using PointMsg = geometry_msgs::msg::Point32;

class TestZoneLib : public ::testing::Test {
};

TEST_F(TestZoneLib, test_clockwise_comparator) {
    std::vector<PointMsg> msg_points;
    PointMsg p;
    p.x = 1;
    p.y = 1;
    msg_points.push_back(p);
    p.x = -1;
    p.y = 1;
    msg_points.push_back(p);
    p.x = -1;
    p.y = -1;
    msg_points.push_back(p);
    p.x = 1;
    p.y = -1;
    msg_points.push_back(p);


    ClockwiseComparator cmp(0,0);
    std::sort(msg_points.begin(), msg_points.end(), cmp);
    //points go clockwise
    ASSERT_FLOAT_EQ(-1, msg_points[0].x);
    ASSERT_FLOAT_EQ(1, msg_points[0].y);

    ASSERT_FLOAT_EQ(1, msg_points[1].x);
    ASSERT_FLOAT_EQ(1, msg_points[1].y);

    ASSERT_FLOAT_EQ(1, msg_points[2].x);
    ASSERT_FLOAT_EQ(-1, msg_points[2].y);

    ASSERT_FLOAT_EQ(-1, msg_points[3].x);
    ASSERT_FLOAT_EQ(-1, msg_points[3].y);
}
