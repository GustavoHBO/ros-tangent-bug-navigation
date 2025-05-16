#include <gtest/gtest.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <limits>
#include "../src/TangentBug.h" // Replace with actual header path

class TangentBugSegmentTest : public ::testing::Test
{
protected:
    ros::NodeHandle nh_;
    TangentBug bug;

    TangentBugSegmentTest() : nh_("~"), bug(nh_) {}

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

TEST_F(TangentBugSegmentTest, SingleContinuousSegment)
{
    std::vector<float> ranges(10, 1.0); // All points at 1m
    auto scan = createScan(ranges);

    auto segments = bug.extractObstacleSegments2D(scan, 0.3);
    ASSERT_EQ(segments.size(), 1);
    EXPECT_EQ(segments[0].size(), 10);
}

TEST_F(TangentBugSegmentTest, SplitsSegmentsOnGap)
{
    std::vector<float> ranges = {1.0, 1.0, 1.0, 5.0, 5.0, 1.0, 1.0};
    auto scan = createScan(ranges);

    auto segments = bug.extractObstacleSegments2D(scan, 0.3);
    ASSERT_GE(segments.size(), 2);

    size_t total_points = 0;
    for (const auto &s : segments)
        total_points += s.size();

    EXPECT_EQ(total_points, 7);
}

TEST_F(TangentBugSegmentTest, MultipleSegments)
{
    std::vector<float> ranges = {4.64, 4.25, 4.90, 30.05, 3.90};
    auto scan = createScan(ranges);

    auto segments = bug.extractObstacleSegments2D(scan, 2.0);
    ASSERT_GE(segments.size(), 3);

    size_t total_points = 0;
    for (const auto &s : segments)
        total_points += s.size();

    EXPECT_EQ(total_points, 5);
}

// === REJECTION / EDGE CASE TESTS ===

TEST_F(TangentBugSegmentTest, IgnoresNaNsAndInfs)
{
    std::vector<float> ranges = {
        1.0,
        std::numeric_limits<float>::quiet_NaN(),
        1.0,
        std::numeric_limits<float>::infinity(),
        1.0};

    auto scan = createScan(ranges);
    auto segments = bug.extractObstacleSegments2D(scan, 0.3);

    size_t total_points = 0;
    for (const auto &seg : segments)
        total_points += seg.size();

    EXPECT_EQ(total_points, 3);
}

TEST_F(TangentBugSegmentTest, EmptyScanReturnsNoSegments)
{
    std::vector<float> ranges;
    auto scan = createScan(ranges);

    auto segments = bug.extractObstacleSegments2D(scan, 0.3);
    EXPECT_TRUE(segments.empty());
}

TEST_F(TangentBugSegmentTest, SensitivityToThreshold)
{
    std::vector<float> ranges = {1.0, 1.0, 1.5, 1.0};
    auto scan = createScan(ranges);

    auto low_thresh = bug.extractObstacleSegments2D(scan, 0.1);
    auto high_thresh = bug.extractObstacleSegments2D(scan, 1.0);

    EXPECT_GT(low_thresh.size(), high_thresh.size());
}

// === ENTRY POINT ===

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_extract_obstacle_segments");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
