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

enum State {
    MOVE_TO_GOAL,
    FOLLOW_CONTOUR,
    LEAVE_CONTOUR
};

class TangentBug {
public:
    TangentBug(ros::NodeHandle& nh);

    // Callbacks
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    // Navegação principal
    void navigate();
    
    // Computa o controle baseado no estado atual
    void computeControl();

private:
    ros::NodeHandle nh_;

    // Subscribers e Publishers
    ros::Subscriber scan_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher marker_pub_;  // Publisher para markers no RViz

    // Dados de sensores e posição
    sensor_msgs::LaserScan current_scan_;
    geometry_msgs::Pose current_pose_;
    std::vector<float> laser_ranges_;

    // Variáveis do objetivo
    double goal_x_;
    double goal_y_;
    double goal_tolerance_;
    double min_distance_to_goal_;
    double best_distance_to_goal_;

    // Estado do controlador
    State current_state_;

    // Velocidades máximas
    double max_linear_speed_;
    double max_angular_speed_;

    // Ponto inicial do contorno
    double contour_start_x_;
    double contour_start_y_;

    // Funções auxiliares
    void moveToGoal(double distance_to_goal);
    void followContour();
    void leaveContour();
    double calculateDistance(double x1, double y1, double x2, double y2);
    double calculateHeadingToGoal();
    int isPathClear(double heading, double obstacle_threshold = 1.0);

    // Função para publicar o marcador da trajetória
    void publishTrajectoryMarker();

    // Função para spawnar o marcador do objetivo no Gazebo
    void spawnGoalMarker(double x, double y);  // Nova função para spawnar o modelo no Gazebo
};

#endif
