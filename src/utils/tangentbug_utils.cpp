#include "tangentbug_utils.h"

#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <deque>
#include <sstream>
#include <iomanip>

namespace tangentbug_utils
{

    static std::deque<std::string> sphere_history;

    // Convert #RRGGBB hex string to "R G B 1" format (0-1 scale)
    std::string hexToRGBA(const std::string &hex)
    {
        if (hex.size() != 7 || hex[0] != '#')
            return "0 0 1 1"; // default blue

        int r = std::stoi(hex.substr(1, 2), nullptr, 16);
        int g = std::stoi(hex.substr(3, 2), nullptr, 16);
        int b = std::stoi(hex.substr(5, 2), nullptr, 16);

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3)
           << (r / 255.0) << " " << (g / 255.0) << " " << (b / 255.0) << " 1";
        return ss.str();
    }

    void spawnSphereAt(double x, double y, double z, const std::string &color_hex, int max_spheres)
    {
        ros::NodeHandle nh;
        ros::ServiceClient spawn_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
        ros::ServiceClient delete_client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

        spawn_client.waitForExistence();
        delete_client.waitForExistence();

        // Remove oldest if at limit
        if (sphere_history.size() >= static_cast<size_t>(max_spheres))
        {
            std::string oldest = sphere_history.front();
            sphere_history.pop_front();

            gazebo_msgs::DeleteModel delete_srv;
            delete_srv.request.model_name = oldest;
            if (delete_client.call(delete_srv) && delete_srv.response.success)
            {
                ROS_INFO_STREAM("Deleted oldest sphere: " << oldest);
            }
            else
            {
                ROS_WARN_STREAM("Failed to delete oldest sphere: " << oldest);
            }
        }

        // Generate a unique model name
        std::string model_name = "sphere_" + std::to_string(ros::Time::now().toNSec());
        sphere_history.push_back(model_name);

        // Convert color
        std::string color_rgba = hexToRGBA(color_hex);

        // Build SDF
        std::ostringstream sdf;
        sdf << "<?xml version='1.0'?>"
            << "<sdf version='1.6'>"
            << "  <model name='unit_sphere'>"
            << "    <static>true</static>"
            << "    <link name='link'>"
            << "      <visual name='visual'>"
            << "        <geometry><box><size>0.05 0.05 0.05</size></box></geometry>"
            << "        <material><ambient>" << color_rgba << "</ambient></material>"
            << "      </visual>"
            << "    </link>"
            << "  </model>"
            << "</sdf>";

        gazebo_msgs::SpawnModel srv;
        srv.request.model_name = model_name;
        srv.request.model_xml = sdf.str();
        srv.request.initial_pose.position.x = x;
        srv.request.initial_pose.position.y = y;
        srv.request.initial_pose.position.z = z;
        srv.request.initial_pose.orientation.w = 1.0;

        if (spawn_client.call(srv) && srv.response.success)
        {
            ROS_INFO_STREAM("Spawned sphere at (" << x << ", " << y << ", " << z << ") with color " << color_hex);
        }
        else
        {
            ROS_ERROR_STREAM("Failed to spawn sphere: " << srv.response.status_message);
        }
    }

    void deleteAllSpheres()
    {
        ros::NodeHandle nh;
        ros::ServiceClient get_models = nh.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
        ros::ServiceClient delete_model = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

        gazebo_msgs::GetWorldProperties world_srv;
        if (!get_models.call(world_srv) || !world_srv.response.success)
        {
            ROS_ERROR("Failed to get world properties.");
            return;
        }

        for (const auto &model_name : world_srv.response.model_names)
        {
            if (model_name.find("sphere_") == 0)
            {
                gazebo_msgs::DeleteModel delete_srv;
                delete_srv.request.model_name = model_name;

                if (delete_model.call(delete_srv) && delete_srv.response.success)
                {
                    ROS_INFO_STREAM("Deleted sphere model: " << model_name);
                }
                else
                {
                    ROS_WARN_STREAM("Failed to delete model: " << model_name);
                }
            }
        }

        sphere_history.clear();
    }

    /**
     * @brief Simplifies a segment of points by removing points that are too close
     *       or nearly collinear with their neighbors.
     */
    std::vector<geometry_msgs::Point> simplifySegment(
        const std::vector<geometry_msgs::Point> &segment,
        double min_distance,
        double angle_tolerance_deg)
    {
        if (segment.size() < 3)
            return segment;

        std::vector<geometry_msgs::Point> simplified;
        simplified.push_back(segment.front());

        for (size_t i = 1; i < segment.size() - 1; ++i)
        {
            const auto &prev = segment[i - 1];
            const auto &curr = segment[i];
            const auto &next = segment[i + 1];

            // Remove if too close to previous point
            double dist = std::hypot(curr.x - prev.x, curr.y - prev.y);
            if (dist < min_distance)
                continue;

            // Check if nearly collinear
            double angle1 = atan2(curr.y - prev.y, curr.x - prev.x);
            double angle2 = atan2(next.y - curr.y, next.x - curr.x);
            double angle_diff = std::abs(angle1 - angle2);

            // Normalize to [0, pi]
            angle_diff = std::fmod(angle_diff + M_PI, 2 * M_PI) - M_PI;
            angle_diff = std::abs(angle_diff);

            if (angle_diff * 180.0 / M_PI < angle_tolerance_deg)
                continue; // skip this point

            simplified.push_back(curr);
        }

        simplified.push_back(segment.back());
        return simplified;
    }
}
