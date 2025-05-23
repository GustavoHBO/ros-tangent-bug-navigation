#include <gtest/gtest.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <limits>
#include "../src/TangentBug.h" // Replace with your actual header

class FindBestSegmentTest : public ::testing::Test {
protected:
    ros::NodeHandle nh_;
    TangentBug bug;

    FindBestSegmentTest() : nh_("~"), bug(nh_) {}

    geometry_msgs::Point makePoint(double x, double y)
    {
        geometry_msgs::Point pt;
        pt.x = x;
        pt.y = y;
        pt.z = 0.0;
        return pt;
    }
};

TEST_F(FindBestSegmentTest, ChoosesClosestStartPoint)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(10, 10), makePoint(11, 11) },
        { makePoint(1, 1), makePoint(2, 2) },
        { makePoint(5, 5), makePoint(6, 6) }
    };

    geometry_msgs::Point goal = makePoint(1.1, 1.0);
    auto best_segment = bug.findBestSegmentTowardGoal(segments, goal);

    ASSERT_EQ(best_segment.front().x, 1);
    ASSERT_EQ(best_segment.front().y, 1);
}

TEST_F(FindBestSegmentTest, ChoosesClosestEndPoint)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(0, 0), makePoint(5, 5) },
        { makePoint(10, 10), makePoint(11, 11) }
    };

    geometry_msgs::Point goal = makePoint(5.1, 5.1);
    auto best_segment = bug.findBestSegmentTowardGoal(segments, goal);

    ASSERT_EQ(best_segment.back().x, 5);
    ASSERT_EQ(best_segment.back().y, 5);
}

TEST_F(FindBestSegmentTest, HandlesEmptySegments)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        {},
        { makePoint(3, 3), makePoint(4, 4) },
        {}
    };

    geometry_msgs::Point goal = makePoint(3.2, 3.2);
    auto best_segment = bug.findBestSegmentTowardGoal(segments, goal);

    ASSERT_EQ(best_segment.front().x, 3);
    ASSERT_EQ(best_segment.front().y, 3);
}

TEST_F(FindBestSegmentTest, ReturnsEmptyIfAllEmpty)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = { {}, {}, {} };
    geometry_msgs::Point goal = makePoint(0, 0);

    auto best_segment = bug.findBestSegmentTowardGoal(segments, goal);
    EXPECT_TRUE(best_segment.empty());
}

TEST_F(FindBestSegmentTest, WorksWithNegativeCoordinates)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(-5, -5), makePoint(-6, -6) },
        { makePoint(-1, -1), makePoint(0, 0) },
    };

    geometry_msgs::Point goal = makePoint(-1.1, -1.2);
    auto best_segment = bug.findBestSegmentTowardGoal(segments, goal);

    ASSERT_EQ(best_segment.front().x, -1);
    ASSERT_EQ(best_segment.front().y, -1);
}

// === ENTRY POINT ===

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_extract_obstacle_segments");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}