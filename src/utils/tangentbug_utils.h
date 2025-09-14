#pragma once

#include <geometry_msgs/Point.h>

namespace tangentbug_utils
{
    struct IntersectionResult
    {
        geometry_msgs::Point point;
        bool intersects = false;
    };

    // Declarations only
    void spawnSphereAt(double x, double y, double z, const std::string &color_hex, int max_spheres = 1);
    void spawnLineSegment(
        double x1, double y1, double z1,
        double x2, double y2, double z2,
        const std::string &color_hex,
        int max_segments = 1);
    void deleteAllSpheres();
    std::vector<geometry_msgs::Point> simplifySegment(
        const std::vector<geometry_msgs::Point> &segment,
        double min_distance = 0.01,
        double angle_tolerance_deg = 5.0);
    IntersectionResult intersectSegments(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2,
                                         const geometry_msgs::Point &q1, const geometry_msgs::Point &q2);
    void clearAllSpheresAndSegments(std::string object_name = "sphere_OR_segment_");
}
