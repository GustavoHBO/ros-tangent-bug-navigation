#include <gtest/gtest.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <limits>
#include "../src/TangentBug.h" // Replace with actual path

class FindClosestEndpointTest : public ::testing::Test {
protected:
    ros::NodeHandle nh_;
    TangentBug bug;

    FindClosestEndpointTest() : nh_("~"), bug(nh_) {}

    geometry_msgs::Point makePoint(double x, double y) {
        geometry_msgs::Point pt;
        pt.x = x;
        pt.y = y;
        pt.z = 0.0;
        return pt;
    }
};

TEST_F(FindClosestEndpointTest, ReturnsClosestFromSingleSegment)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(1, 1), makePoint(2, 2), makePoint(3, 3) }
    };

    geometry_msgs::Point target = makePoint(3.2, 3.1);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, 3.0, 1e-3);
    EXPECT_NEAR(closest.y, 3.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, SelectsFirstWhenCloserThanLast)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(0, 0), makePoint(5, 5) }
    };

    geometry_msgs::Point target = makePoint(0.1, 0.1);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, 0.0, 1e-3);
    EXPECT_NEAR(closest.y, 0.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, WorksWithMultipleSegments)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(10, 10), makePoint(11, 11) },
        { makePoint(0, 0), makePoint(1, 1) },
        { makePoint(5, 5), makePoint(6, 6) }
    };

    geometry_msgs::Point target = makePoint(0.5, 0.5);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, 0.0, 1e-3);
    EXPECT_NEAR(closest.y, 0.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesEmptySegmentsGracefully)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        {},
        { makePoint(2, 2), makePoint(3, 3) }
    };

    geometry_msgs::Point target = makePoint(2.1, 2.0);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, 2.0, 1e-3);
    EXPECT_NEAR(closest.y, 2.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, ReturnsFirstValidWhenEquidistant)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(1, 1), makePoint(4, 4) },
        { makePoint(1, 1), makePoint(4, 4) }
    };

    geometry_msgs::Point target = makePoint(1, 1);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, 1.0, 1e-3);
    EXPECT_NEAR(closest.y, 1.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, ReturnsZeroForAllEmpty)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        {}, {}, {}
    };

    geometry_msgs::Point target = makePoint(1, 1);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_EQ(closest.x, 0.0);
    EXPECT_EQ(closest.y, 0.0);
}

TEST_F(FindClosestEndpointTest, HandlesNegativeCoordinates)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(-1, -1), makePoint(-2, -2) }
    };

    geometry_msgs::Point target = makePoint(-1.5, -1.5);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, -1.0, 1e-3);
    EXPECT_NEAR(closest.y, -1.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesLargeCoordinates)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(10000, 10000), makePoint(20000, 20000) }
    };

    geometry_msgs::Point target = makePoint(15000, 15000);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, 10000.0, 1e-3);
    EXPECT_NEAR(closest.y, 10000.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesSinglePointSegment)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(1, 1) }
    };

    geometry_msgs::Point target = makePoint(1.1, 1.1);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, 1.0, 1e-3);
    EXPECT_NEAR(closest.y, 1.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesMultipleSegmentsWithSameDistance)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(1, 1), makePoint(2, 2) },
        { makePoint(3, 3), makePoint(4, 4) }
    };

    geometry_msgs::Point target = makePoint(2.5, 2.5);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, 2.0, 1e-3);
    EXPECT_NEAR(closest.y, 2.0, 1e-3);
    // Check if the first segment is chosen
    EXPECT_EQ(closest.x, segments[0][1].x); 
    EXPECT_EQ(closest.y, segments[0][1].y);
}

TEST_F(FindClosestEndpointTest, HandlesLongSegmentWithCloseTarget)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(0, 0), makePoint(100, 100) }
    };

    geometry_msgs::Point target = makePoint(50, 50);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, 0.0, 1e-3);
    EXPECT_NEAR(closest.y, 0.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesSegmentWithNegativeCoordinates)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(-10, -10), makePoint(-20, -20) }
    };

    geometry_msgs::Point target = makePoint(-15, -15);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, -10.0, 1e-3);
    EXPECT_NEAR(closest.y, -10.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesSegmentWithMixedCoordinates)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(-10, 10), makePoint(20, -20) }
    };

    geometry_msgs::Point target = makePoint(5, 5);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, -10.0, 1e-3);
    EXPECT_NEAR(closest.y, 10.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesSegmentWithZeroLength)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(1, 1), makePoint(1, 1) }
    };

    geometry_msgs::Point target = makePoint(1.1, 1.1);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, 1.0, 1e-3);
    EXPECT_NEAR(closest.y, 1.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesSegmentWithMultipleClosePoints)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(1, 1), makePoint(2, 2), makePoint(3, 3) }
    };

    geometry_msgs::Point target = makePoint(2.5, 2.5);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, 3.0, 1e-3);
    EXPECT_NEAR(closest.y, 3.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesSegmentWithMultipleClosePointsAndNegativeCoordinates)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(-1, -1), makePoint(-2, -2), makePoint(-3, -3) }
    };

    geometry_msgs::Point target = makePoint(-2.5, -2.5);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, -3.0, 1e-3);
    EXPECT_NEAR(closest.y, -3.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesSegmentWithMultipleClosePointsAndLargeCoordinates)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(1000, 1000), makePoint(2000, 2000), makePoint(3000, 3000) }
    };

    geometry_msgs::Point target = makePoint(1500, 1500);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, 1000.0, 1e-3);
    EXPECT_NEAR(closest.y, 1000.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesSegmentWithMultipleClosePointsAndNegativeLargeCoordinates)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(-1000, -1000), makePoint(-2000, -2000), makePoint(-3000, -3000) }
    };

    geometry_msgs::Point target = makePoint(-1500, -1500);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, -1000.0, 1e-3);
    EXPECT_NEAR(closest.y, -1000.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesSegmentWithMultipleClosePointsAndMixedCoordinates)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(-1000, 1000), makePoint(2000, -2000), makePoint(3000, 3000) }
    };

    geometry_msgs::Point target = makePoint(1500, 1500);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, 3000.0, 1e-3);
    EXPECT_NEAR(closest.y, 3000.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesSegmentWithMultipleClosePointsAndZeroCoordinates)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(0, 0), makePoint(1, 1), makePoint(2, 2) }
    };

    geometry_msgs::Point target = makePoint(0.5, 0.5);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, 0.0, 1e-3);
    EXPECT_NEAR(closest.y, 0.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesSegmenstWithMultiplePoints)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(1, 1), makePoint(2, 2), makePoint(3, 3), makePoint(4, 4) , makePoint(5, 5), makePoint(6, 6) },
        { makePoint(7, 7), makePoint(8, 8), makePoint(9, 9), makePoint(10, 10) },
        { makePoint(11, 11), makePoint(12, 12), makePoint(13, 13), makePoint(14, 14) },
        { makePoint(15, 15), makePoint(16, 16), makePoint(17, 17), makePoint(18, 18) }
    };
    geometry_msgs::Point target = makePoint(10, 10);
    auto closest = bug.findClosestEndpointToPoint(segments, target);
    EXPECT_NEAR(closest.x, 10.0, 1e-3);
    EXPECT_NEAR(closest.y, 10.0, 1e-3);
    // Check if the first segment is chosen
    EXPECT_EQ(closest.x, segments[1][3].x); 
    EXPECT_EQ(closest.y, segments[1][3].y);

    target = makePoint(5, 5);
    closest = bug.findClosestEndpointToPoint(segments, target);
    EXPECT_NEAR(closest.x, 6.0, 1e-3);
    EXPECT_NEAR(closest.y, 6.0, 1e-3);
    // Check if the first segment is chosen
    EXPECT_EQ(closest.x, segments[0][5].x);
    EXPECT_EQ(closest.y, segments[0][5].y);
}


// === ENTRY POINT ===

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_extract_obstacle_segments");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}