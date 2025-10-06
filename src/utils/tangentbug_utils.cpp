#include "tangentbug_utils.h"

#include <tf2/LinearMath/Transform.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/DeleteModel.h>
#include <string>
#include <deque>
#include <sstream>
#include <iomanip>

namespace tangentbug_utils
{

    static std::deque<std::string> sphere_history;
    static std::map<std::string, std::string> sphere_colors;  // Maps sphere name -> color
    static std::deque<std::string> segment_history;           // Keeps insertion order
    static std::map<std::string, std::string> segment_colors; // Maps segment name -> color

    inline bool isFinite(const geometry_msgs::Point &p)
    {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    }

    inline geometry_msgs::Point sub(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
    {
        geometry_msgs::Point r;
        r.x = a.x - b.x;
        r.y = a.y - b.y;
        r.z = a.z - b.z;
        return r;
    }
    inline double dot(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
    inline double dist2(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
    {
        const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
        return sqrt(dx * dx + dy * dy + dz * dz);
    }

    // Convert #RRGGBB hex string to "R G B 1" format (0-1 scale)
    std::string hexToRGBA(const std::string &hex)
    {
        if (hex.size() != 7 || hex[0] != '#')
        {
            std::cerr << "Invalid hex color format: " << hex << std::endl;
            return "1 1 0 1"; // default blue
        }

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

        // Count spheres of this color
        int color_count = 0;
        for (const auto &name : sphere_history)
            if (sphere_colors[name] == color_hex)
                color_count++;

        // Remove oldest sphere with this color ONLY if the count for that color reached the limit
        if (color_count >= max_spheres)
        {
            std::string sphere_to_delete;
            for (const auto &name : sphere_history)
            {
                if (sphere_colors[name] == color_hex) // Oldest with same color
                {
                    sphere_to_delete = name;
                    break;
                }
            }

            if (!sphere_to_delete.empty())
            {
                // Remove from history
                sphere_history.erase(
                    std::remove(sphere_history.begin(), sphere_history.end(), sphere_to_delete),
                    sphere_history.end());

                // Delete in Gazebo
                gazebo_msgs::DeleteModel delete_srv;
                delete_srv.request.model_name = sphere_to_delete;
                if (delete_client.call(delete_srv) && delete_srv.response.success)
                {
                    ROS_INFO_STREAM("Deleted sphere of color " << color_hex << ": " << sphere_to_delete);
                    sphere_colors.erase(sphere_to_delete);
                }
                else
                {
                    ROS_WARN_STREAM("Failed to delete sphere: " << sphere_to_delete);
                }
            }
        }

        // Generate a unique model name
        std::string model_name = "sphere_" + std::to_string(ros::Time::now().toNSec());
        sphere_history.push_back(model_name);
        sphere_colors[model_name] = color_hex; // Track the color

        // Convert color
        std::string color_rgba = hexToRGBA(color_hex);

        // Build SDF
        std::ostringstream sdf;
        sdf << "<?xml version='1.0'?>"
            << "<sdf version='1.6'>"
            << "  <model name='" << model_name << "'>"
            << "    <static>true</static>"
            << "    <link name='link'>"
            << "      <visual name='visual'>"
            << "        <geometry><box><size>0.05 0.05 2.5</size></box></geometry>"
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

    void spawnLineSegment(
        double x1, double y1, double z1,
        double x2, double y2, double z2,
        const std::string &color_hex,
        int max_segments)
    {
        return;
        ros::NodeHandle nh;
        ros::ServiceClient spawn_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
        ros::ServiceClient delete_client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

        spawn_client.waitForExistence();
        delete_client.waitForExistence();

        // Count segments of this color
        int color_count = 0;
        for (const auto &name : segment_history)
            if (segment_colors[name] == color_hex)
                color_count++;

        // Remove oldest segment with this color ONLY if the count for that color reached the limit
        if (color_count >= max_segments)
        {
            std::string segment_to_delete;
            for (const auto &name : segment_history)
            {
                if (segment_colors[name] == color_hex) // Oldest with same color
                {
                    segment_to_delete = name;
                    break;
                }
            }

            if (!segment_to_delete.empty())
            {
                // Remove from history
                segment_history.erase(
                    std::remove(segment_history.begin(), segment_history.end(), segment_to_delete),
                    segment_history.end());

                // Delete in Gazebo
                gazebo_msgs::DeleteModel delete_srv;
                delete_srv.request.model_name = segment_to_delete;
                if (delete_client.call(delete_srv) && delete_srv.response.success)
                {
                    ROS_INFO_STREAM("Deleted segment of color " << color_hex << ": " << segment_to_delete);
                    segment_colors.erase(segment_to_delete);
                }
                else
                {
                    ROS_WARN_STREAM("Failed to delete segment: " << segment_to_delete);
                }
            }
        }

        // Unique name
        std::string model_name = "segment_" + std::to_string(ros::Time::now().toNSec());
        segment_history.push_back(model_name);
        segment_colors[model_name] = color_hex; // Track the color

        // Calculate length & orientation
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dz = z2 - z1;
        double length = std::sqrt(dx * dx + dy * dy + dz * dz);

        // Skip zero-length segments (prevents Ogre crash)
        if (length < 0.01)
            length = 0.01;

        // Midpoint
        double mx = (x1 + x2) / 2.0;
        double my = (y1 + y2) / 2.0;
        double mz = (z1 + z2) / 2.0;

        // Direction -> quaternion
        tf2::Vector3 dir = tf2::Vector3(dx, dy, dz);
        dir.normalize();
        tf2::Vector3 up(1, 0, 0); // default axis for box
        tf2::Quaternion q;
        if (dir != up)
        {
            tf2::Vector3 rot_axis = up.cross(dir);
            rot_axis.normalize();
            double angle = std::acos(up.dot(dir));
            q.setRotation(rot_axis, angle);
        }
        else
        {
            q.setValue(0, 0, 0, 1);
        }

        // Convert color
        std::string color_rgba = hexToRGBA(color_hex);

        // SDF for the thin box (line segment)
        std::ostringstream sdf;
        sdf << "<?xml version='1.0'?>"
            << "<sdf version='1.6'>"
            << "  <model name='" << model_name << "'>"
            << "    <static>true</static>"
            << "    <link name='link'>"
            << "      <visual name='visual'>"
            << "        <geometry><box><size>" << length << " 0.02 2.5</size></box></geometry>"
            << "        <material><ambient>" << color_rgba << "</ambient></material>"
            << "      </visual>"
            << "    </link>"
            << "  </model>"
            << "</sdf>";

        gazebo_msgs::SpawnModel srv;
        srv.request.model_name = model_name;
        srv.request.model_xml = sdf.str();
        srv.request.initial_pose.position.x = mx;
        srv.request.initial_pose.position.y = my;
        srv.request.initial_pose.position.z = mz;
        srv.request.initial_pose.orientation.x = q.x();
        srv.request.initial_pose.orientation.y = q.y();
        srv.request.initial_pose.orientation.z = q.z();
        srv.request.initial_pose.orientation.w = q.w();

        if (spawn_client.call(srv) && srv.response.success)
        {
            ROS_INFO_STREAM("Spawned segment from (" << x1 << "," << y1 << "," << z1 << ") to ("
                                                     << x2 << "," << y2 << "," << z2 << ") with color " << color_hex);
        }
        else
        {
            ROS_ERROR_STREAM("Failed to spawn segment: " << srv.response.status_message);
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

    /**
     * Build a graph according to your spec:
     * - Iterate pts in order.
     * - Start a run at A0. While dist(An, An+1) <= max_gap, extend the run.
     * - When a break (> max_gap) or end is found:
     *     * If run has one point: emit edge [A0, A0].
     *     * Else: emit edge [A0, An].
     * - Graph nodes contain ONLY endpoints that appear in emitted edges.
     *
     * Notes:
     * - Non-finite points (NaN/Inf) break runs and are skipped.
     * - Uses squared distances to avoid sqrt.
     */
    Graph makeGraphRunsByGap(const std::vector<geometry_msgs::Point> &pts, double max_gap)
    {
        Graph g;
        const size_t n = pts.size();
        if (n == 0)
            return g;

        // Map from original index -> node index in g.nodes (or -1 if not yet added)
        std::vector<int> node_idx(n, -1);
        g.nodes.reserve(n);
        g.edges.reserve(n / 2);

        const double gap2 = max_gap * max_gap;

        auto ensure_node = [&](size_t i) -> int
        {
            int &m = node_idx[i];
            if (m == -1)
            {
                m = static_cast<int>(g.nodes.size());
                g.nodes.push_back(pts[i]);
            }
            return m;
        };

        size_t i = 0;
        // Skip leading non-finite points
        while (i < n && !isFinite(pts[i]))
            ++i;

        while (i < n)
        {
            // Start of run: A0
            size_t start = i;
            size_t last = i;

            // Extend while next is finite and within gap from current last
            size_t j = i + 1;
            while (j < n)
            {
                if (!isFinite(pts[j]))
                    break;
                if (dist2(pts[last], pts[j]) > gap2)
                    break;
                last = j;
                ++j;
            }

            // Emit edge: singleton if start==last, else [start,last]
            if (start == last)
            {
                int u = ensure_node(start);
                std::vector<Edge> edges;
                edges.push_back(Edge{u, u, 0.0});
                g.edges.push_back(edges); // single-point edge
            }
            else
            {
                int u = ensure_node(start);
                int v = ensure_node(last);
                std::vector<Edge> edges;
                edges.push_back(Edge{u, v, dist2(pts[start], pts[last])});
                g.edges.push_back(edges);
            }

            // Advance i:
            // If we broke due to a non-finite or large gap at j, the next run starts at j (skipping non-finites).
            i = j;
            while (i < n && !isFinite(pts[i]))
                ++i;
        }

        return g;
    }

    /**
     * @brief Intersects two line segments defined by points p1, p2 and q1, q2.
     *        Returns the intersection point if they intersect, otherwise returns
     *        a default Point with intersects set to false.
     */
    IntersectionResult intersectSegments(
        const geometry_msgs::Point &p1, const geometry_msgs::Point &p2,
        const geometry_msgs::Point &q1, const geometry_msgs::Point &q2)
    {
        IntersectionResult intersectionResult;

        double s1_x = p2.x - p1.x;
        double s1_y = p2.y - p1.y;
        double s2_x = q2.x - q1.x;
        double s2_y = q2.y - q1.y;

        double denom = (-s2_x * s1_y + s1_x * s2_y);
        if (fabs(denom) < 1e-6)
            return intersectionResult; // paralelo ou colinear

        double s = (-s1_y * (p1.x - q1.x) + s1_x * (p1.y - q1.y)) / denom;
        double t = (s2_x * (p1.y - q1.y) - s2_y * (p1.x - q1.x)) / denom;

        if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
        {
            intersectionResult.point.x = p1.x + (t * s1_x);
            intersectionResult.point.y = p1.y + (t * s1_y);
            intersectionResult.point.z = 0.0; // opcional
            intersectionResult.intersects = true;
        }

        return intersectionResult;
    }

    /**
     * @brief Gera dois segmentos paralelos ao segmento definido por p1 e p2,
     *        ambos a uma distância 'dist' em lados opostos (esquerda/direita).
     *        O segmento original fica centralizado entre eles.
     *
     * @param p1   Ponto inicial do segmento original.
     * @param p2   Ponto final do segmento original.
     * @param dist Distância lateral para os segmentos paralelos.
     * @return     Vetor contendo dois segmentos: {{p1_left, p2_left}, {p1_right, p2_right}}
     */
    std::vector<std::vector<geometry_msgs::Point>> parallelSegmentsAtDistance(
        const geometry_msgs::Point &p1,
        const geometry_msgs::Point &p2,
        double dist)
    {
        std::vector<std::vector<geometry_msgs::Point>> segments(2, std::vector<geometry_msgs::Point>(2));

        // Vetor diretor do segmento original
        const double dx = p2.x - p1.x;
        const double dy = p2.y - p1.y;
        const double len = std::hypot(dx, dy);

        if (len < 1e-9 || dist == 0.0)
        {
            segments[0][0] = p1;
            segments[0][1] = p2;
            segments[1][0] = p1;
            segments[1][1] = p2;
            return segments;
        }

        // Normal unitária "para a esquerda" (rotação de +90°)
        const double nx = -dy / len;
        const double ny = dx / len;

        // Deslocamentos
        const double ox = dist * nx;
        const double oy = dist * ny;

        // Segmento paralelo à esquerda
        geometry_msgs::Point p1_left, p2_left;
        p1_left.x = p1.x + ox;
        p1_left.y = p1.y + oy;
        p1_left.z = p1.z;
        p2_left.x = p2.x + ox;
        p2_left.y = p2.y + oy;
        p2_left.z = p2.z;

        // Segmento paralelo à direita
        geometry_msgs::Point p1_right, p2_right;
        p1_right.x = p1.x - ox;
        p1_right.y = p1.y - oy;
        p1_right.z = p1.z;
        p2_right.x = p2.x - ox;
        p2_right.y = p2.y - oy;
        p2_right.z = p2.z;

        segments[0][0] = p1_left;
        segments[0][1] = p2_left;
        segments[1][0] = p1_right;
        segments[1][1] = p2_right;

        return segments;
    }

    void clearAllSpheresAndSegments(std::string object_name)
    {
        ros::NodeHandle nh;
        ros::ServiceClient world_client = nh.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
        ros::ServiceClient delete_client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

        world_client.waitForExistence();
        delete_client.waitForExistence();

        gazebo_msgs::GetWorldProperties world_srv;
        if (!world_client.call(world_srv) || !world_srv.response.success)
        {
            ROS_ERROR("Failed to call /gazebo/get_world_properties");
            return;
        }

        int deleted_count = 0;
        for (const auto &name : world_srv.response.model_names)
        {
            if ((object_name == "sphere_OR_segment_" && name.find("sphere_") == 0 || name.find("segment_") == 0) || (name.find(object_name) == 0))
            {
                gazebo_msgs::DeleteModel delete_srv;
                delete_srv.request.model_name = name;
                if (delete_client.call(delete_srv) && delete_srv.response.success)
                {
                    ROS_INFO_STREAM("Deleted model: " << name);
                    deleted_count++;
                }
                else
                {
                    ROS_WARN_STREAM("Failed to delete model: " << name);
                }
            }
        }

        ROS_INFO_STREAM("Deleted " << deleted_count << " spheres/segments from Gazebo.");
    }

    // Constrói G1 (nós + arestas admissíveis x->V) a partir de LTG, usando a definição (V - x)·(T - x) > 0 e d(V,T) < d(x,T)
    void buildG1(const geometry_msgs::Point &x,
                 const geometry_msgs::Point &T,
                 Graph &LTG, // Grafo LTG (nós + arestas)
                 Graph &G1   // Grafo resultante (nós + arestas)
    )
    {
        auto &ltg_nodes = LTG.nodes;

        auto &V1_nodes = G1.nodes;
        auto &G1_edges = G1.edges;
        V1_nodes.clear();
        G1_edges.clear();

        V1_nodes.push_back(x); // garante {x} ∈ V1
        const int idx_x = 0;
        const geometry_msgs::Point Tx = sub(T, x);
        const double dxT2 = dist2(x, T);

        for (const auto &V : ltg_nodes)
        {
            geometry_msgs::Point Vx = sub(V, x);
            const double dVT2 = dist2(V, T);
            if (dot(Vx, Tx) > 0.0)
            {
                int idx_V = static_cast<int>(V1_nodes.size());
                V1_nodes.push_back(V);
                if (dVT2 < dxT2)
                {
                    std::vector<Edge> edges;
                    edges.push_back(Edge{idx_x, idx_V, dist2(x, V)}); // aresta admissível x->V
                    G1_edges.push_back(edges);                        // aresta admissível x->V
                }
            }
        }

        bool has_T = false;
        for (const auto &E : LTG.edges)
        {
            for(const auto &e : E){
                IntersectionResult result = intersectSegments(x, T, LTG.nodes[e.from], LTG.nodes[e.to]);
                if(result.intersects){
                    std::cout << "Interseção com LTG em: (" << result.point.x << ", " << result.point.y << ")" << std::endl;
                    has_T = true;
                }
            }
        }

        
        if (!has_T)
        {
            std::cout << "Adicionando aresta x->T pois não há interseção com LTG" << std::endl;
            // Se não houver interseção com T, adicionar aresta x->T
            int idx_T = static_cast<int>(V1_nodes.size());
            V1_nodes.push_back(T);
            G1_edges.push_back({Edge{idx_x, idx_T, dist2(x, T)}});
        }
    }

    Edge findClosestEdgeToTarget(
        const geometry_msgs::Point &T,
        Graph &subgraphG1)
    {
        Edge best_edge;
        double best_d2 = std::numeric_limits<double>::max();

        for (std::vector<Edge> &edges : subgraphG1.edges)
        {
            Edge eVT;
            for (Edge &e : edges)
            {
                const geometry_msgs::Point &V = subgraphG1.nodes[e.to];
                e.cost = e.cost ? e.cost : dist2(subgraphG1.nodes[e.from], V);
                double dVT2 = dist2(V, T);
                eVT = {e.to, -1, dVT2, true};
                if (e.cost + dVT2 < best_d2)
                {
                    best_d2 = e.cost + dVT2;
                    best_edge = e;
                }
                std::cout << "From (" << subgraphG1.nodes[e.from].x << ", " << subgraphG1.nodes[e.from].y << ") to ("
                          << V.x << ", " << V.y << "): cost=" << e.cost << ", dVT2=" << dVT2
                          << ", total=" << (e.cost + dVT2) << std::endl;
            }
            if (!edges.empty())
            {
                edges.push_back(eVT);
            }
        }
        return best_edge;
    }

    std::vector<geometry_msgs::Point> laserScanToPoints(
        const sensor_msgs::LaserScan &scan,
        const tf2::Transform &tf_laser_to_odom)
    {
        std::vector<geometry_msgs::Point> points;
        points.reserve(scan.ranges.size());

        double angle = scan.angle_min;
        for (size_t i = 0; i < scan.ranges.size(); i++, angle += scan.angle_increment)
        {
            float r = scan.ranges[i];
            if (std::isfinite(r))
            {
                // point in laser frame
                tf2::Vector3 v_laser(r * cos(angle), r * sin(angle), 0.0);

                // transform to odom frame
                tf2::Vector3 v_odom = tf_laser_to_odom * v_laser;

                geometry_msgs::Point p;
                p.x = v_odom.x();
                p.y = v_odom.y();
                p.z = v_odom.z();
                p.z = 0.0; // keep points in 2D plane
                points.push_back(p);
            }
        }
        return points;
    }

}
