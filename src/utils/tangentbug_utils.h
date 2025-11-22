#pragma once

#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Transform.h>

namespace tangentbug_utils
{
    struct IntersectionResult
    {
        geometry_msgs::Point point;
        bool intersects = false;
    };

    struct Edge
    {
        int from, to;
        double cost;
        bool isVirtual = false; // aresta virtual (não representa um obstáculo real)
    };

    struct Segment
    {
        geometry_msgs::Point a;
        geometry_msgs::Point b;
    };

    struct Graph
    {
        std::vector<geometry_msgs::Point> nodes; // remaining points (filtered)
        std::vector<std::vector<Edge>> edges;    // edges between nodes within gap
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
                                                                 bool closed = false);

    std::vector<std::vector<geometry_msgs::Point>> parallelSegmentsAtDistance(
        const geometry_msgs::Point &p1,
        const geometry_msgs::Point &p2,
        double dist);

    void clearAllSpheresAndSegments(std::string object_name = "sphere_OR_segment_");
    void buildG1(const geometry_msgs::Point &x,
                 const geometry_msgs::Point &T,
                 Graph &LTG, // Grafo LTG (nós + arestas)
                 Graph &G1   // Grafo resultante (nós + arestas)
    );

    void buildG2(const Graph &ltg,
                 double dMin,
                 const geometry_msgs::Point &T,
                 Graph &g2,
                 double eps = 1e-6);

    std::vector<geometry_msgs::Point> laserScanToPoints(
        const sensor_msgs::LaserScan &scan,
        const tf2::Transform &tf_laser_to_odom);

    Graph makeGraphRunsByGap(const std::vector<geometry_msgs::Point> &pts,
                             double max_gap,
                             double angle_threshold_deg);

    Edge findClosestEdgeToTarget(
        const geometry_msgs::Point &T,
        Graph &subgraphG1);

    double angleBetweenPoints(const geometry_msgs::Point &p1,
                              const geometry_msgs::Point &p2,
                              const geometry_msgs::Point &p3);
}