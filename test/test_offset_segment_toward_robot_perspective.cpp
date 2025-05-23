#include <gtest/gtest.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <cmath>
#include "../src/TangentBug.h" // Replace with actual header path

class OffsetPerspectiveTest : public ::testing::Test {
protected:
    ros::NodeHandle nh_;
    TangentBug bug;

    OffsetPerspectiveTest() : nh_("~"), bug(nh_) {}

    geometry_msgs::Point makePoint(double x, double y) {
        geometry_msgs::Point pt;
        pt.x = x;
        pt.y = y;
        pt.z = 0.0;
        return pt;
    }

    bool almostEqual(double a, double b, double tol = 1e-3) {
        return std::abs(a - b) < tol;
    }
};

TEST_F(OffsetPerspectiveTest, OffsetCircleOutwardFromRobot)
{
    std::vector<geometry_msgs::Point> circle;
    int points = 20;
    double radius = 1.0;
    for (int i = 0; i < points; ++i)
    {
        double theta = 2 * M_PI * i / points;
        circle.push_back(makePoint(radius * cos(theta), radius * sin(theta)));
    }

    geometry_msgs::Point robot = makePoint(0, 0);
    auto result = bug.offsetSegmentTowardRobotPerspective(circle, robot, -10); // outward

    ASSERT_EQ(result.size(), circle.size());
    for (size_t i = 0; i < result.size(); ++i)
    {
        double original_dist = std::hypot(circle[i].x - robot.x, circle[i].y - robot.y);
        double shifted_dist = std::hypot(result[i].x - robot.x, result[i].y - robot.y);
        EXPECT_GT(shifted_dist, original_dist); // Each point should move outward
    }
}

TEST_F(OffsetPerspectiveTest, OffsetCircleInwardTowardRobot)
{
    std::vector<geometry_msgs::Point> circle;
    int points = 20;
    double radius = 1.0;
    for (int i = 0; i < points; ++i)
    {
        double theta = 2 * M_PI * i / points;
        circle.push_back(makePoint(radius * cos(theta), radius * sin(theta)));
    }

    geometry_msgs::Point robot = makePoint(0, 0);
    auto result = bug.offsetSegmentTowardRobotPerspective(circle, robot, 10); // inward

    ASSERT_EQ(result.size(), circle.size());
    for (size_t i = 0; i < result.size(); ++i)
    {
        double original_dist = std::hypot(circle[i].x - robot.x, circle[i].y - robot.y);
        double shifted_dist = std::hypot(result[i].x - robot.x, result[i].y - robot.y);
        EXPECT_LT(shifted_dist, original_dist); // Each point should move inward
    }
}

TEST_F(OffsetPerspectiveTest, DegenerateCaseSameAsRobot)
{
    std::vector<geometry_msgs::Point> segment = {
        makePoint(0, 0), makePoint(0, 0)
    };
    geometry_msgs::Point robot = makePoint(0, 0);

    auto result = bug.offsetSegmentTowardRobotPerspective(segment, robot, 20);

    ASSERT_EQ(result.size(), segment.size());
    for (size_t i = 0; i < result.size(); ++i)
    {
        EXPECT_TRUE(almostEqual(result[i].x, 0.0));
        EXPECT_TRUE(almostEqual(result[i].y, 0.0));
    }
}

// === ENTRY POINT ===
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_extract_obstacle_segments");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}