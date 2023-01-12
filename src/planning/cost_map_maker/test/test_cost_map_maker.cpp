// /*
//  * Package:   motion_planner
//  * Filename:  test_OpenDriveUtils.hpp
//  * Author:    Egan Johnson
//  * Email:     egan.johnson@utdallas.edu
//  * Copyright: 2022, Nova UTD
//  * License:   MIT License
//  */

// #include <gtest/gtest.h>
// #include <boost/geometry.hpp>

// #include "zone_lib/zone.hpp"
// #include "motion_planner/MotionPlannerNode.hpp"
// using namespace navigator::motion_planner;
// using nova_msgs::msg::Trajectory;
// using nova_msgs::msg::TrajectoryPoint;
// using nova_msgs::msg::Zone;
// using nova_msgs::msg::ZoneArray;

// /**
//  * @brief Creates a simple trajectory
//  * with 10 points interspaced 1m apart
//  * on the x-axis, starting at 0.0. Points
//  * are set to an arbitrary speed of 1.0.
//  * 
//  * Creates an empty zone array.
//  * 
//  */
// class SimplePathTest : public ::testing::Test
// {
//     public:
//     Trajectory trajectory;
//     ZoneArray zone_array;

//     SimplePathTest(){
//         trajectory.points.resize(10);
//         for(int i = 0; i < 10; i++){
//             trajectory.points[i].x = i;
//             trajectory.points[i].y = 0.0;
//             trajectory.points[i].vx = 1.0;
//         }
//         zone_array.zones.resize(0);
//     }
// };

// TEST_F(SimplePathTest, test_zone_boundary){
//     // Creates a square with side length 1.0
//     // aligned with the cooridinate axis and centered
//     // at (5, 0). When the path is smoothed to this zone,
//     // we expect two additional points to be added to the
//     // path: (4.5, 0) and (5.5, 0). If we set the zone
//     // speed to 0, we expect the speeds to be 0 for points
//     // (4.5, 0), (5, 0), and (5.5, 0).
//     Zone zone;
//     zone.poly.points.resize(4);
//     zone.poly.points[0].x = 4.5;
//     zone.poly.points[0].y = 0.5;
//     zone.poly.points[1].x = 5.5;
//     zone.poly.points[1].y = 0.5;
//     zone.poly.points[2].x = 5.5;
//     zone.poly.points[2].y = -0.5;
//     zone.poly.points[3].x = 4.5;
//     zone.poly.points[3].y = -0.5;

//     zone.max_speed = 0.0;

//     zone_array.zones.push_back(zone);

//     Trajectory traj1(trajectory);
//     MotionPlannerNode::smooth(traj1, zone_array, 1.0, 1.0);
//     EXPECT_EQ(traj1.points.size(), 12ul);
//     EXPECT_NEAR(traj1.points[5].x, 4.5, 0.00001);
//     EXPECT_NEAR(traj1.points[5].y, 0.0, 0.00001);
//     EXPECT_NEAR(traj1.points[5].vx, 0.0, 0.00001);
//     EXPECT_NEAR(traj1.points[6].x, 5.0, 0.00001);
//     EXPECT_NEAR(traj1.points[6].y, 0.0, 0.00001);
//     EXPECT_NEAR(traj1.points[6].vx, 0.0, 0.00001);
//     EXPECT_NEAR(traj1.points[7].x, 5.5, 0.00001);
//     EXPECT_NEAR(traj1.points[7].y, 0.0, 0.00001);
//     EXPECT_NEAR(traj1.points[7].vx, 0.0, 0.00001);

//     for (TrajectoryPoint point : traj1.points){
//         std::cout << "(" <<point.x << ", " << point.y << ", " << point.vx << ")" << std::endl;
//     }

//     // If we change the acceleration speeds to 0.25 (0.5^2 for convenience),
//     // we expect the speeds for the points before and after the zone to be
//     // affected by the zone as well.

//     Trajectory traj2(trajectory);
//     MotionPlannerNode::smooth(traj2, zone_array, 0.25, 0.25);
//     EXPECT_EQ(traj2.points.size(), 12ul);
//     // Same in-zone behavior as above
//     EXPECT_NEAR(traj2.points[5].x, 4.5, 0.00001);
//     EXPECT_NEAR(traj2.points[5].y, 0.0, 0.00001);
//     EXPECT_NEAR(traj2.points[5].vx, 0.0, 0.00001);
//     EXPECT_NEAR(traj2.points[6].x, 5.0, 0.00001);
//     EXPECT_NEAR(traj2.points[6].y, 0.0, 0.00001);
//     EXPECT_NEAR(traj2.points[6].vx, 0.0, 0.00001);
//     EXPECT_NEAR(traj2.points[7].x, 5.5, 0.00001);
//     EXPECT_NEAR(traj2.points[7].y, 0.0, 0.00001);
//     EXPECT_NEAR(traj2.points[7].vx, 0.0, 0.00001);
//     // lower speeds approaching the zone boundary
//     EXPECT_EQ(traj2.points[4].vx, 0.5);
//     EXPECT_EQ(traj2.points[8].vx, 0.5);

//     std::cout << "traj2" << std::endl;
//     for (TrajectoryPoint point : traj2.points){
//         std::cout << "(" <<point.x << ", " << point.y << ", " << point.vx << ")" << std::endl;
//     }
// }

// TEST_F(SimplePathTest, test_duplicate_zones){
//     // Similar test to test_zone_boundary, but with
//     // the zone duplicated.
//     Zone zone;
//     zone.poly.points.resize(4);
//     zone.poly.points[0].x = 4.5;
//     zone.poly.points[0].y = 0.5;
//     zone.poly.points[1].x = 5.5;
//     zone.poly.points[1].y = 0.5;
//     zone.poly.points[2].x = 5.5;
//     zone.poly.points[2].y = -0.5;
//     zone.poly.points[3].x = 4.5;
//     zone.poly.points[3].y = -0.5;

//     zone.max_speed = 0.0;

//     zone_array.zones.push_back(zone);
//     zone_array.zones.push_back(zone);


//     Trajectory traj1(trajectory);
//     MotionPlannerNode::smooth(traj1, zone_array, 1.0, 1.0);
//     EXPECT_EQ(traj1.points.size(), 12ul);
//     EXPECT_NEAR(traj1.points[5].x, 4.5, 0.00001);
//     EXPECT_NEAR(traj1.points[5].y, 0.0, 0.00001);
//     EXPECT_NEAR(traj1.points[5].vx, 0.0, 0.00001);
//     EXPECT_NEAR(traj1.points[6].x, 5.0, 0.00001);
//     EXPECT_NEAR(traj1.points[6].y, 0.0, 0.00001);
//     EXPECT_NEAR(traj1.points[6].vx, 0.0, 0.00001);
//     EXPECT_NEAR(traj1.points[7].x, 5.5, 0.00001);
//     EXPECT_NEAR(traj1.points[7].y, 0.0, 0.00001);
//     EXPECT_NEAR(traj1.points[7].vx, 0.0, 0.00001);

//     for (TrajectoryPoint point : traj1.points){
//         std::cout << "(" <<point.x << ", " << point.y << ", " << point.vx << ")" << std::endl;
//     }
// }

// TEST_F(SimplePathTest, test_long_path){
//     // Test a path further than the horizon.
//     Trajectory traj1(trajectory);
//     // A 7.0 meter path with 1 meters between each point
//     // should have 8 points.
//     MotionPlannerNode::smooth(traj1, zone_array, 1.0, 1.0, 7.0);
//     EXPECT_EQ(traj1.points.size(), 8ul);
// }

// TEST_F(SimplePathTest, test_multiple_zones){
//     // Test a path with multiple distinct zones.
//     // this test will have a zone 3 meters wide
//     // and the same 1m zone from before, centered 
//     // at x = 5.0.
//     Zone z_small;
//     z_small.poly.points.resize(4);
//     z_small.poly.points[0].x = 5.0 - 0.5;
//     z_small.poly.points[0].y = 0.5;
//     z_small.poly.points[1].x = 5.0 + 0.5;
//     z_small.poly.points[1].y = 0.5;
//     z_small.poly.points[2].x = 5.0 + 0.5;
//     z_small.poly.points[2].y = -0.5;
//     z_small.poly.points[3].x = 5.0 - 0.5;
//     z_small.poly.points[3].y = -0.5;
//     z_small.max_speed = 0.0;

//     Zone z_big;
//     z_big.poly.points.resize(4);
//     z_big.poly.points[0].x = 5.0 - 1.5;
//     z_big.poly.points[0].y = 0.5;
//     z_big.poly.points[1].x = 5.0 + 1.5;
//     z_big.poly.points[1].y = 0.5;
//     z_big.poly.points[2].x = 5.0 + 1.5;
//     z_big.poly.points[2].y = -0.5;
//     z_big.poly.points[3].x = 5.0 - 1.5;
//     z_big.poly.points[3].y = -0.5;
//     z_big.max_speed = 0.1;

//     zone_array.zones.push_back(z_small);
//     zone_array.zones.push_back(z_big);

//     Trajectory traj1(trajectory);
//     MotionPlannerNode::smooth(traj1, zone_array, 1.0, 1.0);
//     EXPECT_EQ(traj1.points.size(), 14ul);
//     // Check points are in the right place
//     EXPECT_NEAR(traj1.points[3].x, 3.0, 0.00001);
//     EXPECT_NEAR(traj1.points[4].x, 3.5, 0.00001);
//     EXPECT_NEAR(traj1.points[5].x, 4.0, 0.00001);
//     EXPECT_NEAR(traj1.points[6].x, 4.5, 0.00001);
//     EXPECT_NEAR(traj1.points[7].x, 5.0, 0.00001);
//     EXPECT_NEAR(traj1.points[8].x, 5.5, 0.00001);
//     EXPECT_NEAR(traj1.points[9].x, 6.0, 0.00001);
//     EXPECT_NEAR(traj1.points[10].x, 6.5, 0.00001);
//     EXPECT_NEAR(traj1.points[11].x, 7.0, 0.00001);
//     // Check speeds are correct
//     EXPECT_NEAR(traj1.points[3].vx, 1.0, 0.00001);
//     EXPECT_NEAR(traj1.points[4].vx, 0.1, 0.00001);
//     EXPECT_NEAR(traj1.points[5].vx, 0.1, 0.00001);
//     EXPECT_NEAR(traj1.points[6].vx, 0.0, 0.00001);
//     EXPECT_NEAR(traj1.points[7].vx, 0.0, 0.00001);
//     EXPECT_NEAR(traj1.points[8].vx, 0.0, 0.00001);
//     EXPECT_NEAR(traj1.points[9].vx, 0.1, 0.00001);
//     EXPECT_NEAR(traj1.points[10].vx, 0.1, 0.00001);
//     EXPECT_NEAR(traj1.points[11].vx, 1.0, 0.00001);

//     for (TrajectoryPoint point : traj1.points){
//         std::cout << "(" <<point.x << ", " << point.y << ", " << point.vx << ")" << std::endl;
//     }
// }