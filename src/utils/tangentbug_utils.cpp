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

    static inline void normalize(double &x, double &y)
    {
        const double n = std::hypot(x, y);
        if (n > 1e-12)
        {
            x /= n;
            y /= n;
        }
        else
        {
            x = 1.0;
            y = 0.0;
        }
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
        return;
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
            << "        <geometry><box><size>0.07 0.07 2.5</size></box></geometry>"
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

        if (!(spawn_client.call(srv) && srv.response.success))
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
            << "        <geometry><box><size>" << length << " 0.025 2.2</size></box></geometry>"
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

    double angleBetweenPoints(const geometry_msgs::Point &p1,
                              const geometry_msgs::Point &p2,
                              const geometry_msgs::Point &p3)
    {
        // Vetor p1 -> p2
        double u_x = p2.x - p1.x;
        double u_y = p2.y - p1.y;

        // Vetor p1 -> p3
        double v_x = p3.x - p1.x;
        double v_y = p3.y - p1.y;

        // Produto escalar
        double dot = u_x * v_x + u_y * v_y;

        // Normas
        double mag_u = std::sqrt(u_x * u_x + u_y * u_y);
        double mag_v = std::sqrt(v_x * v_x + v_y * v_y);

        // Evita divisão por zero
        if (mag_u < 1e-9 || mag_v < 1e-9)
        {
            return 0.0; // ou lançar uma exceção, se preferir
        }

        // Cálculo do cosseno do ângulo
        double cos_angle = dot / (mag_u * mag_v);

        // Correções numéricas para manter dentro de [-1, 1]
        if (cos_angle > 1.0)
            cos_angle = 1.0;
        if (cos_angle < -1.0)
            cos_angle = -1.0;

        // Ângulo final (radianos)
        return std::acos(cos_angle);
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
    // Graph makeGraphRunsByGap(const std::vector<geometry_msgs::Point> &pts, double max_gap, double angle_threshold_deg)
    // {
    //     Graph g;
    //     const size_t n = pts.size();
    //     if (n == 0)
    //         return g;

    //     // Map from original index -> node index in g.nodes (or -1 if not yet added)
    //     std::vector<int> node_idx(n, -1);
    //     g.nodes.reserve(n);
    //     g.edges.reserve(n / 2);

    //     const double gap2 = max_gap * max_gap;

    //     auto ensure_node = [&](size_t i) -> int
    //     {
    //         int &m = node_idx[i];
    //         if (m == -1)
    //         {
    //             m = static_cast<int>(g.nodes.size());
    //             g.nodes.push_back(pts[i]);
    //         }
    //         return m;
    //     };

    //     size_t i = 0;
    //     // Skip leading non-finite points
    //     while (i < n && !isFinite(pts[i]))
    //         ++i;

    //     while (i < n)
    //     {
    //         // Start of run: A0
    //         size_t start = i;
    //         size_t last = i;

    //         // Extend while next is finite and within gap from current last
    //         size_t j = i + 1;
    //         while (j < n)
    //         {
    //             if (!isFinite(pts[j]))
    //                 break;
    //             if (dist2(pts[last], pts[j]) > gap2)
    //                 break;
    //             last = j;
    //             ++j;
    //         }

    //         // Emit edge: singleton if start==last, else [start,last]
    //         if (start == last)
    //         {
    //             int u = ensure_node(start);
    //             std::vector<Edge> edges;
    //             edges.push_back(Edge{u, u, 0.0});
    //             g.edges.push_back(edges); // single-point edge
    //         }
    //         else
    //         {
    //             int u = ensure_node(start);
    //             int v = ensure_node(last);
    //             std::vector<Edge> edges;
    //             edges.push_back(Edge{u, v, dist2(pts[start], pts[last])});
    //             g.edges.push_back(edges);
    //         }

    //         // Advance i:
    //         // If we broke due to a non-finite or large gap at j, the next run starts at j (skipping non-finites).
    //         i = j;
    //         while (i < n && !isFinite(pts[i]))
    //             ++i;
    //     }

    //     return g;
    // }

    static inline double angle_abs_diff(double a, double b)
    {
        double d = std::fmod(a - b, 2.0 * M_PI);
        if (d < -M_PI)
            d += 2.0 * M_PI;
        else if (d > M_PI)
            d -= 2.0 * M_PI;
        return std::fabs(d);
    }

    // direção do segmento (ângulo em rad) usando atan2
    static inline double segment_angle_rad(const geometry_msgs::Point &a,
                                           const geometry_msgs::Point &b)
    {
        return std::atan2(b.y - a.y, b.x - a.x);
    }

    static inline bool lineIntersection(double x1, double y1, double dx1, double dy1,
                                        double x2, double y2, double dx2, double dy2,
                                        double &xi, double &yi)
    {
        // (x1,y1) + t*(dx1,dy1)  intersects  (x2,y2) + s*(dx2,dy2)
        const double denom = dx1 * dy2 - dy1 * dx2; // cross((dx1,dy1),(dx2,dy2))
        if (std::fabs(denom) < 1e-12)
            return false; // paralelas/quase
        const double rx = x2 - x1, ry = y2 - y1;
        const double t = (rx * dy2 - ry * dx2) / denom;
        xi = x1 + t * dx1;
        yi = y1 + t * dy1;
        return true;
    }

    Graph makeGraphRunsByGap(const std::vector<geometry_msgs::Point> &pts,
                             double max_gap,
                             double angle_threshold_deg)
    {
        Graph G;

        const int n = static_cast<int>(pts.size());
        if (n <= 0)
        {
            G.edges.clear();
            return G;
        }
        if (n < 2)
        {
            G.nodes.reserve(pts.size());
            geometry_msgs::Point p = pts[0];
            G.nodes.push_back(p);
            G.edges.resize(1);
            G.edges[0].push_back(Edge{0, 0, 0.0, false});
            return G;
        }

        const double angle_thr_rad = angle_threshold_deg * M_PI / 180.0;

        // Guardar pares (start_idx, end_idx) dos runs válidos em relação ao vetor original
        std::vector<std::pair<int, int>> runs;

        int run_start = 0;       // índice do primeiro ponto do run (em pts)
        int last_kept = 0;       // último ponto aceito no run (em pts)
        bool in_run = false;     // já temos pelo menos um segmento no run?
        double last_seg_ang = 0; // ângulo do último segmento aceito

        run_start = 0;
        last_kept = 0;
        in_run = false;

        for (int i = 1; i < n; ++i)
        {
            const double gap = dist2(pts[i - 1], pts[i]);

            if (gap > max_gap)
            {
                // quebra de run por distância
                if (last_kept > run_start)
                {
                    runs.emplace_back(run_start, last_kept);
                }
                run_start = i;
                last_kept = i;
                in_run = false;
                continue;
            }

            // gap OK => checar direção
            const double ang = segment_angle_rad(pts[last_kept], pts[i]);

            if (!in_run)
            {
                // primeiro segmento do run
                last_seg_ang = ang;
                last_kept = i;
                in_run = true;
            }
            else
            {
                const double dang = angle_abs_diff(ang, last_seg_ang);
                if (dang <= angle_thr_rad)
                {
                    // mantém o run
                    last_seg_ang = ang;
                    last_kept = i;
                }
                else
                {
                    // quebra de run por ângulo
                    if (last_kept > run_start)
                    {
                        runs.emplace_back(run_start, last_kept);
                    }
                    // recomeça novo run em (i-1)->i
                    run_start = i - 1;
                    last_kept = i;
                    last_seg_ang = segment_angle_rad(pts[run_start], pts[i]);
                    in_run = true;
                }
            }
        }

        // fecha o último run, se válido
        if (last_kept > run_start)
        {
            runs.emplace_back(run_start, last_kept);
        }

        // ---- Construção do grafo: só extremos como nós; cada run vira uma aresta ----

        // Mapeia índice no vetor original -> índice no vetor G.nodes
        std::vector<int> map_orig_to_node(n, -1);

        auto add_node_if_needed = [&](int idx) -> int
        {
            if (map_orig_to_node[idx] != -1)
                return map_orig_to_node[idx];
            const int new_id = static_cast<int>(G.nodes.size());
            G.nodes.push_back(pts[idx]);
            map_orig_to_node[idx] = new_id;
            // garante que edges tem tamanho suficiente
            if (static_cast<int>(G.edges.size()) <= new_id)
                G.edges.resize(new_id + 1);
            return new_id;
        };

        for (const auto &run : runs)
        {
            const int i0 = run.first;
            const int i1 = run.second;
            if (i1 == i0)
                continue;

            const int u = add_node_if_needed(i0);
            const int v = add_node_if_needed(i1);
            const double w = dist2(pts[i0], pts[i1]);

            // grafo não-direcionado: adiciona as duas direções
            G.edges[u].push_back(Edge{u, v, w, false});
            G.edges[v].push_back(Edge{v, u, w, false});
        }

        // garante consistência (pode não ser necessário, mas é seguro)
        if (G.edges.size() < G.nodes.size())
            G.edges.resize(G.nodes.size());

        return G;
    }

    /**
     * Offset (inflar) uma polilinha no SENTIDO DO SENSOR.
     * - Cada aresta é deslocada paralelamente pela normal que aponta para o sensor.
     * - Nos vértices, unimos por interseção das duas arestas deslocadas (bevel/miter simples).
     * - Se não houver interseção (quase paralelas), fazemos bevel: usamos o fim de uma e início da outra.
     *
     * @param seg            Polilinha em frame global (pontos crus do obstáculo).
     * @param sensor_global  Posição do sensor no mesmo frame.
     * @param d              Distância de inflação (ex.: raio do robô + margem).
     * @param closed         true para polígono fechado; false para aberto (LIDAR típico).
     */
    std::vector<geometry_msgs::Point> offsetPolylineTowardSensor(const std::vector<geometry_msgs::Point> &seg,
                                                                 const geometry_msgs::Point &sensor_global,
                                                                 double d,
                                                                 bool closed)
    {
        std::vector<geometry_msgs::Point> out;
        const size_t N = seg.size();
        if (N == 0)
            return out;
        if (N == 1)
        {
            std::cout << "offsetPolylineTowardSensor: single point, creating radial offset." << std::endl;
            // ponto isolado: desloca radialmente PARA o sensor (ponto navegável seguro)
            geometry_msgs::Point p = seg[0];
            double vx = sensor_global.x - p.x;
            double vy = sensor_global.y - p.y;
            normalize(vx, vy); // vetor do ponto para o sensor
            geometry_msgs::Point q;
            q.x = p.x + d * vx;
            q.y = p.y + d * vy;
            q.z = 0.0;
            out.push_back(q);
            return out;
        }

        // Para cada aresta i: [Pi -> P(i+1)]
        const size_t M = closed ? N : (N - 1);

        // Pré-calcula tangentes e normais (para o sensor) e endpoints deslocados
        struct SegOff
        {
            double ax, ay, bx, by;
            double tx, ty;
        }; // offset endpoints + direção
        std::vector<SegOff> off;
        off.reserve(M);

        for (size_t i = 0; i < M; ++i)
        {
            const size_t i0 = i;
            const size_t i1 = (i + 1) % N;
            const auto &A = seg[i0];
            const auto &B = seg[i1];

            // tangente do segmento
            double tx = B.x - A.x;
            double ty = B.y - A.y;
            normalize(tx, ty);

            // normal candidata (rot90 da tangente)
            double nx = -ty, ny = tx;

            // decidir sentido PARA O SENSOR:
            // use o ponto médio para definir v = (sensor - mid); dot(n, v) > 0 => n aponta para o sensor
            const double mx = 0.5 * (A.x + B.x);
            const double my = 0.5 * (A.y + B.y);
            const double vx = sensor_global.x - mx;
            const double vy = sensor_global.y - my;
            if (nx * vx + ny * vy < 0.0)
            {
                nx = -nx;
                ny = -ny;
            } // garante n -> sensor

            // desloca endpoints da aresta
            SegOff s;
            s.tx = tx;
            s.ty = ty;
            s.ax = A.x + d * nx;
            s.ay = A.y + d * ny;
            s.by = B.y + d * ny;
            s.bx = B.x + d * nx;
            off.push_back(s);
        }

        // Construir vértices de saída
        out.reserve(N + (closed ? 0 : 1));

        if (!closed)
        {
            // endpoint inicial: apenas A0'
            geometry_msgs::Point q0;
            q0.x = off[0].ax;
            q0.y = off[0].ay;
            q0.z = 0.0;
            out.push_back(q0);
        }

        // vértices internos: interseção das retas deslocadas adjacentes
        for (size_t i = 0; i < (closed ? N : (N - 2)); ++i)
        {
            // no caso aberto, o vértice i+1 da polilinha original é “interno”
            const size_t k = closed ? i : (i + 1);

            const size_t segL = (k + M - 1) % M; // aresta à esquerda (anterior)
            const size_t segR = k % M;           // aresta à direita (próxima)

            // reta L: passa por (aL -> bL), direção tL
            const SegOff &L = off[segL];
            // reta R: passa por (aR -> bR), direção tR
            const SegOff &R = off[segR];

            double xi, yi;
            bool ok = lineIntersection(L.ax, L.ay, L.tx, L.ty,
                                       R.ax, R.ay, R.tx, R.ty,
                                       xi, yi);

            geometry_msgs::Point q;
            if (ok)
            {
                q.x = xi;
                q.y = yi;
                q.z = 0.0;
            }
            else
            {
                // quase paralelas: bevel simples usando “fim de L” e “início de R”
                // aqui escolhemos o ponto médio entre L.b e R.a para evitar gaps/overlaps
                q.x = 0.5 * (L.bx + R.ax);
                q.y = 0.5 * (L.by + R.ay);
                q.z = 0.0;
            }
            out.push_back(q);
        }

        if (!closed)
        {
            // endpoint final: apenas B_last'
            const SegOff &last = off.back();
            geometry_msgs::Point qN;
            qN.x = last.bx;
            qN.y = last.by;
            qN.z = 0.0;
            out.push_back(qN);
        }
        else
        {
            // fechado: também fecha o polígono; já adicionamos N vértices internos,
            // então não precisa repetir o primeiro.
        }

        if (!closed && out.size() >= 2)
        {
            // primeira ponta: move no sentido oposto à aresta (out[0] -> out[1])
            {
                double dx = out[1].x - out[0].x;
                double dy = out[1].y - out[0].y;
                const double n = std::hypot(dx, dy);
                if (n > 1e-8)
                {
                    dx /= n;
                    dy /= n;
                    out[0].x -= d * dx;
                    out[0].y -= d * dy;
                }
            }
            // última ponta: move no mesmo sentido da aresta (out[n-2] -> out[n-1])
            {
                const size_t npts = out.size();
                double dx = out[npts - 1].x - out[npts - 2].x;
                double dy = out[npts - 1].y - out[npts - 2].y;
                const double n = std::hypot(dx, dy);
                if (n > 1e-8)
                {
                    dx /= n;
                    dy /= n;
                    out[npts - 1].x += d * dx;
                    out[npts - 1].y += d * dy;
                }
            }
        }

        return out;
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
            for (const auto &e : E)
            {
                IntersectionResult result = intersectSegments(x, T, LTG.nodes[e.from], LTG.nodes[e.to]);
                if (result.intersects)
                {
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

    /**
     * Constrói o subgrafo G2 = { V ∈ LTG | d(V,T) < dMin } preservando arestas internas.
     *
     * @param ltg   Grafo LTG original (nós e arestas).
     * @param dMin  d_min(T): menor distância observada ao alvo durante o BF.
     * @param T     Alvo (mesmo frame dos nós).
     * @param eps   Tolerância numérica para comparação estrita (<).
     * @return      Subgrafo G2 na mesma estrutura (nodes + edges remapeados).
     */
    void buildG2(const Graph &ltg,
                 double dMin,
                 const geometry_msgs::Point &T,
                 Graph &g2,
                 double eps)
    {

        const std::size_t N = ltg.nodes.size();
        if (N == 0)
        {
            g2.edges.clear();
            return;
        }

        // 1) Quais nós ficam? (d(V,T) < dMin - eps)
        std::vector<int> old2new(N, -1);
        g2.nodes.reserve(N); // upper bound

        for (std::size_t i = 0; i < N; ++i)
        {
            const auto &v = ltg.nodes[i];
            const double d = dist2(v, T);

            if (d < dMin - eps)
            {
                old2new[i] = static_cast<int>(g2.nodes.size());
                g2.nodes.push_back(v);
            }
        }

        // Se não sobrou nó, retorna vazio
        const std::size_t N2 = g2.nodes.size();
        if (N2 == 0)
        {
            g2.edges.clear();
            return;
        }

        // 2) Prepara lista de adjacência do G2
        g2.edges.assign(N2, std::vector<Edge>());

        // 3) Cria as arestas virtuais do augumented graph
        for (std::size_t u = 0; u < N2; ++u)
        {
            const double duT2 = dist2(g2.nodes[u], T);
            const Edge e = {static_cast<int>(u), -1, duT2, true};
            g2.edges[u].push_back(e);
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
            double cost = 0.0;
            bool has_edge_virtual = false;
            for (const Edge &e : edges)
            {
                geometry_msgs::Point V = subgraphG1.nodes[e.to];
                auto point = subgraphG1.nodes[e.from];
                if(e.isVirtual){
                    V = T;
                }
                cost += dist2(point, V);
                spawnLineSegment(point.x, point.y, 0.0,
                                 V.x, V.y, 0.0, "#ff0000", 100);
                if (e.isVirtual)
                {
                    has_edge_virtual = true;
                }
            }

            if (!has_edge_virtual)
            {
                double dVT2 = dist2(subgraphG1.nodes[edges.back().to], T);
                eVT = {edges.back().to, -1, dVT2, true};
                cost += dVT2;
                edges.push_back(eVT);
            }

            if (cost < best_d2)
            {
                best_d2 = cost;
                best_edge = edges.front();
            }
        }
        if (best_edge.from == -1)
        {
            throw std::runtime_error("No edge found in findClosestEdgeToTarget");
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
