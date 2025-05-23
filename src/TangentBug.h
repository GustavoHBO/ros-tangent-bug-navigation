#ifndef TANGENT_BUG_H
#define TANGENT_BUG_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <tf/tf.h>
#include <limits>
#include <vector>
#include <ros/package.h>
#include <fstream>

enum State
{
    MOVE_TO_GOAL,
    FOLLOW_CONTOUR,
    LEAVE_CONTOUR,
    EMERGENCY_STOP
};

class TangentBug
{

#ifdef UNIT_TEST
public:
    State getCurrentState() const { return current_state_; }
#endif
public:
    TangentBug(ros::NodeHandle &nh);

    // Callbacks
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // Navegação principal
    void navigate();

    // Computa o controle baseado no estado atual
    void computeControl();

    // private:

    ros::NodeHandle nh_;

    // Subscribers e Publishers
    ros::Subscriber scan_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher marker_pub_; // Publisher para markers no RViz

    // Dados de sensores e posição
    sensor_msgs::LaserScan current_scan_;
    geometry_msgs::Pose current_pose_;
    std::vector<float> laser_ranges_;

    // Variáveis do objetivo
    double goal_x_;
    double goal_y_;
    double best_boundary_x_;
    double best_boundary_y_;
    double best_boundary_cost_;
    bool boundary_loop_started_;
    bool boundary_loop_completed_;
    double goal_tolerance_;
    double min_distance_to_goal_;
    double best_distance_to_goal_;
    double safety_radius_;
    double contour_follow_distance_;
    double goal_angle_threshold_;
    double max_obstacle_distance_;
    double rotation_speed_;
    double contour_speed_;
    double robot_width_;
    double safety_margin_;
    double obstacle_threshold_;
    double gap_threshold_;
    geometry_msgs::Point best_leave_point_;

    // Estado do controlador
    State current_state_;

    // Velocidades máximas
    double max_linear_speed_;
    double max_angular_speed_;

    // Ponto inicial do contorno
    double contour_start_x_;
    double contour_start_y_;

    // Funções auxiliares
    void stopRobot(int repeat_times = 1);
    void rotateInPlace(double angular_speed = 0.5, double duration_seconds = 1.0);
    void moveToGoal(double distance_to_goal);
    void followContour();
    void leaveContour();
    double calculateDistance(double x1, double y1, double x2, double y2);
    double calculateHeadingToGoal();
    int isPathClear(double heading, double obstacle_threshold = 2.0);
    int selectBestBoundaryPoint();
    bool shouldExitBoundaryFollowing();
    void publishEventMarker(const std::string &event_name, double x, double y, float r, float g, float b);
    void moveTowardHeading(double target_heading);
    void moveTo(double x, double y);
    void resetBoundaryTracking();

    // Função para publicar o marcador da trajetória
    void publishTrajectoryMarker();

    // Função para spawnar o marcador do objetivo no Gazebo
    void spawnGoalMarker(double x, double y); // Nova função para spawnar o modelo no Gazebo

    // Helper methods for tangent bug logic
    std::pair<double, double> calculateTangentDirection();
    bool findContourDiscontinuity(std::pair<double, double> &discontinuity);
    void updateMinGoalDistance();

    void publishDebugMarkers(double cone_angle, double obstacle_threshold);
    void moveToPoint(double target_x, double target_y);
    void followObstacle();

    std::vector<int> detectObstacleBoundaries2D(
        const sensor_msgs::LaserScan &scan, 
        double threshold
    );
    std::vector<std::vector<geometry_msgs::Point>> extractObstacleSegments2D(
        const sensor_msgs::LaserScan &scan, 
        double threshold
    );
    geometry_msgs::Point findClosestEndpointToPoint(
        const std::vector<geometry_msgs::Point> &segment, const geometry_msgs::Point &target);
    std::vector<geometry_msgs::Point> findBestSegmentTowardGoal(const std::vector<std::vector<geometry_msgs::Point>> &segments, const geometry_msgs::Point &goal);
    std::vector<geometry_msgs::Point> offsetSegmentTowardRobot(const std::vector<geometry_msgs::Point> &segment, const geometry_msgs::Point &robot, double offset_distance_cm);
    std::vector<geometry_msgs::Point> offsetSegmentTowardRobotPerspective(
        const std::vector<geometry_msgs::Point> &segment,
        const geometry_msgs::Point &robot,
        double offset_distance_cm);
};

#endif
