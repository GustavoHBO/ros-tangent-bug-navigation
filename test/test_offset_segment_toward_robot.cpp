#include <gtest/gtest.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <cmath>
#include "../src/TangentBug.h" // Replace with actual header path

class OffsetSegmentTest : public ::testing::Test {
protected:
    ros::NodeHandle nh_;
    TangentBug bug;

    OffsetSegmentTest() : nh_("~"), bug(nh_) {}

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

TEST_F(OffsetSegmentTest, OffsetHorizontalSegmentTowardRobotBelow)
{
    std::vector<geometry_msgs::Point> segment = {
        makePoint(0, 1), makePoint(2, 1)
    };
    geometry_msgs::Point robot = makePoint(1, 0);

    auto result = bug.offsetSegmentTowardRobot(segment, robot, 50); // 50 cm

    ASSERT_EQ(result.size(), 2);
    EXPECT_TRUE(almostEqual(result[0].x, 0));
    EXPECT_TRUE(almostEqual(result[0].y, 0.5));
    EXPECT_TRUE(almostEqual(result[1].x, 2));
    EXPECT_TRUE(almostEqual(result[1].y, 0.5));
}

TEST_F(OffsetSegmentTest, OffsetVerticalSegmentTowardRobotRight)
{
    std::vector<geometry_msgs::Point> segment = {
        makePoint(2, 1), makePoint(2, 3)
    };
    geometry_msgs::Point robot = makePoint(3, 2);

    auto result = bug.offsetSegmentTowardRobot(segment, robot, 20); // 20 cm

    ASSERT_EQ(result.size(), 2);
    EXPECT_TRUE(almostEqual(result[0].x, 2.2));
    EXPECT_TRUE(almostEqual(result[0].y, 1));
    EXPECT_TRUE(almostEqual(result[1].x, 2.2));
    EXPECT_TRUE(almostEqual(result[1].y, 3));
}

TEST_F(OffsetSegmentTest, OffsetDiagonalSegmentTowardRobot)
{
    std::vector<geometry_msgs::Point> segment = {
        makePoint(0, 0), makePoint(2, 2)
    };
    geometry_msgs::Point robot = makePoint(1, 0);

    auto result = bug.offsetSegmentTowardRobot(segment, robot, 42.42); // 14.14 cm ~ sqrt(2)/2

    ASSERT_EQ(result.size(), 2);
    EXPECT_TRUE(almostEqual(result[0].x, 0.3));
    EXPECT_TRUE(almostEqual(result[0].y, -0.3));
    EXPECT_TRUE(almostEqual(result[1].x, 2.3));
    EXPECT_TRUE(almostEqual(result[1].y, 1.7));
}

TEST_F(OffsetSegmentTest, HandlesDegenerateSegment)
{
    std::vector<geometry_msgs::Point> segment = {
        makePoint(0, 0), makePoint(1, 0)
    };
    geometry_msgs::Point robot = makePoint(0.5, 0.5);

    auto result = bug.offsetSegmentTowardRobot(segment, robot, 100);

    std::cout << "result[0]: " << result[0].x << ", " << result[0].y << std::endl;
    std::cout << "result[1]: " << result[1].x << ", " << result[1].y << std::endl;

    ASSERT_EQ(result.size(), 2); // Still return original
    EXPECT_TRUE(almostEqual(result[0].x, 0));
    EXPECT_TRUE(almostEqual(result[0].y, 1));
    EXPECT_TRUE(almostEqual(result[1].x, 1));
    EXPECT_TRUE(almostEqual(result[1].y, 1));
}

TEST_F(OffsetSegmentTest, ReturnsEmptyForEmptyInput)
{
    std::vector<geometry_msgs::Point> segment;
    geometry_msgs::Point robot = makePoint(0, 0);

    auto result = bug.offsetSegmentTowardRobot(segment, robot, 30);
    EXPECT_TRUE(result.empty());
}

TEST_F(OffsetSegmentTest, OffsetTriangleShape)
{
    std::vector<geometry_msgs::Point> triangle = {
        makePoint(0, 0),
        makePoint(1, 2),
        makePoint(2, 0),
        makePoint(0, 0)
    };
    geometry_msgs::Point robot = makePoint(1, -2);

    auto result = bug.offsetSegmentTowardRobot(triangle, robot, 50);

    std::cout << "result[0]: " << result[0].x << ", " << result[0].y << std::endl;
    std::cout << "result[1]: " << result[1].x << ", " << result[1].y << std::endl;
    std::cout << "result[2]: " << result[2].x << ", " << result[2].y << std::endl;
    std::cout << "result[3]: " << result[3].x << ", " << result[3].y << std::endl;

    ASSERT_EQ(result.size(), triangle.size());
    EXPECT_LT(result[1].y, triangle[1].y);  // Check that the apex moved downward
}

TEST_F(OffsetSegmentTest, OffsetSemiTriangleTowardRobot)
{
    std::vector<geometry_msgs::Point> semi_triangle = {
        makePoint(0, 0),
        makePoint(1, 2),
        makePoint(2, 0)
    };
    geometry_msgs::Point robot = makePoint(1, -2);

    auto result = bug.offsetSegmentTowardRobot(semi_triangle, robot, 50);

    ASSERT_EQ(result.size(), semi_triangle.size());
    EXPECT_LT(result[1].y, semi_triangle[1].y);  // Apex should move downward
}

TEST_F(OffsetSegmentTest, OffsetSemiCircleTowardRobot)
{
    std::vector<geometry_msgs::Point> semi_circle;
    int points = 20;
    double radius = 1.0;
    for (int i = 0; i <= points; ++i)
    {
        double theta = M_PI * i / points; // only 0 to PI for semi-circle
        semi_circle.push_back(makePoint(radius * cos(theta), radius * sin(theta)));
    }

    geometry_msgs::Point robot = makePoint(0, -1); // below the flat base
    auto result = bug.offsetSegmentTowardRobot(semi_circle, robot, 10);

    ASSERT_EQ(result.size(), semi_circle.size());
    for (size_t i = 0; i < result.size(); ++i)
    {
        EXPECT_LT(result[i].y, semi_circle[i].y); // should shift downward
    }
}

// === ENTRY POINT ===
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_extract_obstacle_segments");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}