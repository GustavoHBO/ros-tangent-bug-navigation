#pragma once

#include <geometry_msgs/Point.h>

namespace tangentbug_utils
{
    // Declarations only
    void spawnSphereAt(double x, double y, double z, const std::string &color_hex, int max_spheres);
    void deleteAllSpheres(); 
    std::vector<geometry_msgs::Point> simplifySegment(
        const std::vector<geometry_msgs::Point> &segment,
        double min_distance = 0.01,
        double angle_tolerance_deg = 5.0);
}
