#include <gtest/gtest.h>
#include <sensor_msgs/LaserScan.h>
#include <limits>
#include "../src/TangentBug.h"

class TangentBugTest : public ::testing::Test
{
protected:
    ros::NodeHandle nh;
    std::unique_ptr<TangentBug> tangent_bug;

    TangentBugTest()
    {
        tangent_bug = std::make_unique<TangentBug>(nh);
    }
    sensor_msgs::LaserScan createScan(const std::vector<float> &ranges,
                                      float angle_min = -1.57,
                                      float angle_increment = 0.0175)
    {
        sensor_msgs::LaserScan scan;
        scan.angle_min = angle_min;
        scan.angle_increment = angle_increment;
        scan.ranges = ranges;
        return scan;
    }
};

// === ACCEPTANCE TESTS ===

TEST_F(TangentBugTest, DetectsSingleObstacleGap)
{
    std::vector<float> ranges(180, 1.0);
    for (int i = 85; i <= 95; ++i)
        ranges[i] = 5.0;

    auto scan = createScan(ranges);
    auto boundaries = tangent_bug->detectObstacleBoundaries2D(scan, 0.3);

    EXPECT_GE(boundaries.size(), 1);
}

TEST_F(TangentBugTest, DetectsMultipleGaps)
{
    std::vector<float> ranges(180, 1.0);
    for (int i = 30; i <= 40; ++i)
        ranges[i] = 4.0;
    for (int i = 90; i <= 100; ++i)
        ranges[i] = 5.0;
    for (int i = 150; i <= 160; ++i)
        ranges[i] = 3.5;

    auto scan = createScan(ranges);
    auto boundaries = tangent_bug->detectObstacleBoundaries2D(scan, 0.3);

    EXPECT_GE(boundaries.size(), 2);
}

// === REJECTION TESTS ===

TEST_F(TangentBugTest, NoGapsOnFlatWall)
{
    std::vector<float> ranges(180, 1.0);
    auto scan = createScan(ranges);
    auto boundaries = tangent_bug->detectObstacleBoundaries2D(scan, 0.3);

    EXPECT_TRUE(boundaries.empty());
}

TEST_F(TangentBugTest, IgnoresNaNs)
{
    std::vector<float> ranges(180, 1.0);
    ranges[50] = std::numeric_limits<float>::quiet_NaN();
    ranges[130] = std::numeric_limits<float>::quiet_NaN();

    auto scan = createScan(ranges);
    auto boundaries = tangent_bug->detectObstacleBoundaries2D(scan, 0.3);

    EXPECT_TRUE(boundaries.empty());
}

TEST_F(TangentBugTest, IgnoresInfValues)
{
    std::vector<float> ranges(180, 1.0);
    ranges[60] = std::numeric_limits<float>::infinity();
    ranges[120] = -std::numeric_limits<float>::infinity();

    auto scan = createScan(ranges);
    auto boundaries = tangent_bug->detectObstacleBoundaries2D(scan, 0.3);

    EXPECT_TRUE(boundaries.empty());
}

// === EDGE CASES ===

TEST_F(TangentBugTest, EmptyScanReturnsNoBoundaries)
{
    std::vector<float> ranges;
    auto scan = createScan(ranges);
    auto boundaries = tangent_bug->detectObstacleBoundaries2D(scan, 0.3);

    EXPECT_TRUE(boundaries.empty());
}

TEST_F(TangentBugTest, HandlesLargeRangeDifference)
{
    std::vector<float> ranges(180, 1.0);
    ranges[100] = 10.0; // A huge spike

    auto scan = createScan(ranges);
    auto boundaries = tangent_bug->detectObstacleBoundaries2D(scan, 0.3);

    EXPECT_GE(boundaries.size(), 1);
}

TEST_F(TangentBugTest, ThresholdSensitivity)
{
    std::vector<float> ranges(180, 1.0);
    ranges[90] = 1.4;

    auto scan = createScan(ranges);

    // Should detect if threshold is low
    auto low_threshold = tangent_bug->detectObstacleBoundaries2D(scan, 0.3);
    EXPECT_GE(low_threshold.size(), 1);

    // Should not detect if threshold is high
    auto high_threshold = tangent_bug->detectObstacleBoundaries2D(scan, 1.0);
    EXPECT_TRUE(high_threshold.empty());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_detect_obstacle_boundaries");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
