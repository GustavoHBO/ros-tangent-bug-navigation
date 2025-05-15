#include <gtest/gtest.h>
#include "../src/TangentBug.h"
#include <ros/ros.h>
#include <memory>

/**
 * @brief Test fixture for TangentBug.
 *
 * Creates a TangentBug instance using a NodeHandle. The fixture
 * allows us to simulate sensor and odometry callbacks.
 */
class TangentBugTest : public ::testing::Test {
protected:
    ros::NodeHandle nh;
    std::unique_ptr<TangentBug> tangent_bug;

    TangentBugTest() {
        tangent_bug = std::make_unique<TangentBug>(nh);
    }
};

TEST_F(TangentBugTest, CalculateDistanceTest) {
    // Verify that the Euclidean distance between (0,0) and (3,4) is 5.
    double distance = tangent_bug->calculateDistance(0.0, 0.0, 3.0, 4.0);
    EXPECT_NEAR(distance, 5.0, 1e-6);
}

TEST_F(TangentBugTest, CalculateHeadingToGoalTest) {
    // Set the goal to (1,1) and current pose to (0,0) using callbacks.
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.pose.position.x = 1.0;
    goal_msg.pose.position.y = 1.0;
    tangent_bug->goalCallback(std::make_shared<geometry_msgs::PoseStamped>(goal_msg));

    nav_msgs::Odometry odom_msg;
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    tangent_bug->odomCallback(std::make_shared<nav_msgs::Odometry>(odom_msg));

    // The heading should be pi/4 (45 degrees) when moving from (0,0) to (1,1).
    double heading = tangent_bug->calculateHeadingToGoal();
    EXPECT_NEAR(heading, M_PI / 4.0, 1e-6);
}

TEST_F(TangentBugTest, IsPathClearTest) {
    // Create a fake laser scan message.
    sensor_msgs::LaserScan scan;
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 0.01;
    int num_readings = static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.ranges.resize(num_readings, 5.0);  // Initialize with a clear path (range=5.0)

    // Simulate the reception of the laser scan.
    tangent_bug->scanCallback(boost::make_shared<sensor_msgs::LaserScan>(scan));

    // For heading 0.0, the path should be clear.
    EXPECT_EQ(tangent_bug->isPathClear(0.0, 1.0), 1);

    // Now simulate an obstacle by setting a close range at heading 0.0.
    int index = static_cast<int>((0.0 - scan.angle_min) / scan.angle_increment);
    scan.ranges[index] = 0.5;  // Obstacle detected within threshold
    tangent_bug->scanCallback(boost::shared_ptr<sensor_msgs::LaserScan>(std::make_shared<sensor_msgs::LaserScan>(scan)));

    EXPECT_EQ(tangent_bug->isPathClear(0.0, 1.0), 0);
}

TEST_F(TangentBugTest, SelectBestBoundaryPointTest) {
    sensor_msgs::LaserScan scan;
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 0.01;
    int num_readings = static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.range_max = 10.0;
    scan.ranges.resize(num_readings, 5.0);

    // Inject a closer point with better heuristic
    int best_index = num_readings / 2 + 10; // off-center
    scan.ranges[best_index] = 2.0;

    tangent_bug->scanCallback(std::make_shared<sensor_msgs::LaserScan>(scan));

    // Position and goal required for cost calculation
    tangent_bug->current_pose_.position.x = 0.0;
    tangent_bug->current_pose_.position.y = 0.0;
    tangent_bug->goal_x_ = 5.0;
    tangent_bug->goal_y_ = 0.0;

    int index = tangent_bug->selectBestBoundaryPoint();
    EXPECT_EQ(index, best_index);
}

TEST_F(TangentBugTest, StateTransitionToContourTest) {
    // Fake goal directly ahead
    tangent_bug->goal_x_ = 10.0;
    tangent_bug->goal_y_ = 0.0;
    tangent_bug->current_pose_.position.x = 0.0;
    tangent_bug->current_pose_.position.y = 0.0;
    tangent_bug->current_pose_.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));

    // Block the path directly ahead
    sensor_msgs::LaserScan scan;
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 0.01;
    int num_readings = static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.range_max = 10.0;
    scan.ranges.resize(num_readings, 5.0);

    int center_idx = static_cast<int>((0.0 - scan.angle_min) / scan.angle_increment);
    scan.ranges[center_idx] = 0.5;  // Obstacle within threshold

    tangent_bug->scanCallback(std::make_shared<sensor_msgs::LaserScan>(scan));

    // Run compute control once
    tangent_bug->computeControl();

    EXPECT_EQ(tangent_bug->current_state_, TangentBug::FOLLOW_CONTOUR);
}

TEST_F(TangentBugTest, EmptyScanDoesNotCrash) {
    sensor_msgs::LaserScan scan;
    scan.ranges.clear();  // Empty ranges
    EXPECT_NO_THROW(tangent_bug->scanCallback(boost::make_shared<sensor_msgs::LaserScan>(scan)));
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "tangent_bug_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
