#include <gtest/gtest.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <cmath>
#include "../src/utils/tangentbug_utils.h" // Adjust the path as necessary

using namespace tangentbug_utils;

geometry_msgs::Point makePoint(double x, double y)
{
    geometry_msgs::Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = 0;
    return pt;
}

TEST(SimplifySegmentTest, KeepsCriticalCorners)
{
    std::vector<geometry_msgs::Point> segment = {
        makePoint(0, 0), makePoint(1, 0), makePoint(1, 1), makePoint(2, 1)
    };

    auto result = simplifySegment(segment);

    ASSERT_EQ(result.size(), 4); // Should retain all because of 90-degree turn
}

TEST(SimplifySegmentTest, RemovesCollinearPoints)
{
    std::vector<geometry_msgs::Point> segment = {
        makePoint(0, 0), makePoint(1, 0), makePoint(2, 0), makePoint(3, 0)
    };

    auto result = simplifySegment(segment);

    ASSERT_EQ(result.size(), 2); // Only endpoints should remain
    EXPECT_EQ(result.front().x, 0);
    EXPECT_EQ(result.back().x, 3);
}

TEST(SimplifySegmentTest, KeepsMinimalPoints)
{
    std::vector<geometry_msgs::Point> segment = {
        makePoint(0, 0), makePoint(0.001, 0), makePoint(0.002, 0)
    };

    auto result = simplifySegment(segment, 0.005);

    ASSERT_EQ(result.size(), 2); // Remove midpoints due to distance filter
    EXPECT_EQ(result.front().x, 0);
    EXPECT_EQ(result.back().x, 0.002);
}

TEST(SimplifySegmentTest, NoSimplificationNeeded)
{
    std::vector<geometry_msgs::Point> segment = {
        makePoint(0, 0), makePoint(0.5, 0.5), makePoint(1, 1)
    };

    auto result = simplifySegment(segment);

    ASSERT_EQ(result.size(), 2); // Diagonal line, not collinear in discrete sense
}

// === ENTRY POINT ===
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}