#ifndef TANGENT_BUG_H
#define TANGENT_BUG_H

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <tf/tf.h>
#include <limits>
#include <vector>
#include <ros/package.h>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <mutex>
#include <numeric>

#include "utils/tangentbug_utils.h"

// Usando o namespace para funções utilitárias, se aplicável
using namespace tangentbug_utils;

enum State
{
    MOVE_TO_GOAL,
    FOLLOW_CONTOUR,
    LEAVE_CONTOUR,
    EMERGENCY_STOP
};

// Estados internos para a função de girar-e-avançar
enum TurnThenMoveState
{
    ALIGNING_TO_POINT, // Girando para alinhar com o ponto
    MOVING_TO_POINT    // Movendo-se em linha reta para o ponto
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
    ros::Timer control_timer_;

    std::mutex scan_mutex_;
    std::mutex odom_mutex_;

    // Dados de sensores e posição
    sensor_msgs::LaserScan current_scan_;
    geometry_msgs::Pose current_pose_;
    std::vector<float> laser_ranges_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    TurnThenMoveState current_turn_then_move_state_;
    geometry_msgs::Point target_point_for_turn_then_move_; // O ponto para onde estamos indo
    double alignment_tolerance_rad_;                       // Tolerância angular para considerar o alinhamento completo

    // Variáveis do objetivo
    geometry_msgs::Point point_goal_; // Ponto objetivo atual
    double goal_x_;                   // Ponto X do objetivo
    double goal_y_;                   // Ponto Y do objetivo

    // Distância em centímetros. Atualizado a cada leitura do LIDAR.
    // Representa a menor distância já observada entre qualquer ponto do contorno visível
    // do obstáculo sendo seguido (durante o modo boundary-following) e o objetivo.
    // Esse valor é atualizado continuamente enquanto o robô contorna o obstáculo,
    // sempre guardando o menor valor encontrado.
    double d_followed_;
    geometry_msgs::Point d_followed_point_; // Ponto do contorno mais próximo do objetivo

    // Distância em centímetros. Atualizado a cada leitura do LIDAR.
    // Representa a distância entre o "ponto bloqueador" (primeira interseção da linha reta
    // entre o robô e o objetivo com um obstáculo) e o objetivo.
    // Se não houver bloqueio nessa linha reta, d_reach_ é simplesmente a distância
    // direta do robô ao objetivo.
    double d_reach_;
    geometry_msgs::Point d_reach_point_; // Ponto do obstáculo mais próximo do objetivo

    // Distância em centímetros. Atualizado a cada leitura do LIDAR.
    // Representa a distância heurística usada para decidir a estratégia de movimento.
    // Definida como a soma da distância do robô (ou do ponto bloqueador) até um ponto da borda
    // mais próximo do objetivo, mais a distância desse ponto da borda até o objetivo.
    // Essa heurística permite comparar caminhos diretos versus seguir contorno,
    // orientando a troca entre os modos "motion-to-goal" e "boundary-following".
    double heuristic_distance_;

    // Variáveis de controle
    double goal_tolerance_;                       // Distância em CM de tolerância para o objetivo
    double rotation_speed_;                       // Velocidade de rotação do robô em rad/s
    double contour_speed_;                        // Velocidade de contorno do robô em m/s
    double robot_width_;                          // Largura do robô em centímetros
    double safety_margin_;                        // Margem de segurança em centímetros
    double obstacle_threshold_;                   // Limite de distância do obstáculo em metros
    double gap_threshold_;                        // Distância em centímetros. Utilizado para detectar lacunas entre os pontos dos obstáculos.
    double gap_segment_distance_threshold_ = 0.4; // Distância em centímetros. Utilizado para detectar segmentos de obstáculos.
    double gap_segment_angle_threshold_ = 10;     // Ângulo em graus. Utilizado para detectar segmentos de obstáculos.

    std::vector<geometry_msgs::Point> robot_trajectory_history_;      // Histórico da trajetória do robô
    std::vector<std::vector<geometry_msgs::Point>> obstacle_history_; // Histórico dos obstáculos detectados

    // Estado do controlador
    State current_state_;

    // Velocidades máximas
    double max_linear_speed_;  // Velocidade linear máxima do robô em m/s
    double max_angular_speed_; // Velocidade angular máxima do robô em rad/s

    // Funções auxiliares
    void stopRobot();
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

    geometry_msgs::Twist calculateMoveToGoalCommand(double distance_to_goal);
    geometry_msgs::Twist calculateFollowContourCommand();
    geometry_msgs::Twist calculateMoveToPointCommand(double target_x, double target_y);
    geometry_msgs::Twist calculateRotateInPlaceCommand(double angular_speed);

    std::vector<geometry_msgs::Point> current_stable_best_segment_;
    double current_stable_best_segment_distance_ = std::numeric_limits<double>::max();
    double segment_switch_threshold_percent_;

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

    std::vector<int> detectObstacleBoundaries2D(
        const sensor_msgs::LaserScan &scan,
        double threshold);
    std::vector<std::vector<geometry_msgs::Point>> extractObstacleSegments2D(
        const sensor_msgs::LaserScan &scan,
        double threshold);
    geometry_msgs::Point findClosestEndpointToPoint(
        const std::vector<geometry_msgs::Point> &segment, const geometry_msgs::Point &target);
    std::vector<geometry_msgs::Point> findBestSegmentTowardGoal(const std::vector<std::vector<geometry_msgs::Point>> &segments, const geometry_msgs::Point &goal);
    std::vector<geometry_msgs::Point> offsetSegmentTowardRobot(const std::vector<geometry_msgs::Point> &segment, const geometry_msgs::Point &robot, double offset_distance_cm);
    std::vector<geometry_msgs::Point> offsetSegmentTowardRobotPerspective(
        const std::vector<geometry_msgs::Point> &segment,
        const geometry_msgs::Point &robot,
        double offset_distance_cm);

    geometry_msgs::Twist calculateTwistToPointAndAlign(const geometry_msgs::Point &target_point);
};

inline double normalizeAngle(double angle)
{
    return atan2(sin(angle), cos(angle));
}

inline geometry_msgs::Point transformPointToGlobal(
    const geometry_msgs::Point &point_local,
    const geometry_msgs::Pose &robot_global_pose)
{
    geometry_msgs::Point point_global;

    // Posição global do robô
    double robot_x = robot_global_pose.position.x;
    double robot_y = robot_global_pose.position.y;

    // Orientação (yaw) global do robô
    double robot_yaw = tf::getYaw(robot_global_pose.orientation);

    // Rotação: Gira o ponto local de acordo com a orientação do robô
    // px' = px * cos(theta) - py * sin(theta)
    // py' = px * sin(theta) + py * cos(theta)
    point_global.x = robot_x + (point_local.x * std::cos(robot_yaw) - point_local.y * std::sin(robot_yaw));
    point_global.y = robot_y + (point_local.x * std::sin(robot_yaw) + point_local.y * std::cos(robot_yaw));
    point_global.z = point_local.z; // Mantém a coordenada Z, se houver

    return point_global;
}

#endif
