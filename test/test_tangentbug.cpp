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
