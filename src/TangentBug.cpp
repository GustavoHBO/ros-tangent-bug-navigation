#include "TangentBug.h"
#include <tf/transform_datatypes.h> // Para converter orientações
#include <tf/transform_listener.h>  // Para ouvir as transformações
#include <cmath>

inline double normalizeAngle(double angle)
{
    return atan2(sin(angle), cos(angle));
}

TangentBug::TangentBug(ros::NodeHandle &nh) : nh_(nh), current_state_(MOVE_TO_GOAL)
{

    // Subscribers and publishers
    scan_sub_ = nh_.subscribe("/scan", 50, &TangentBug::scanCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 50, &TangentBug::odomCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal/bug", 50, &TangentBug::goalCallback, this);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Initialize parameters with default values and log them
    nh_.param("goal_tolerance", goal_tolerance_, 0.2);
    ROS_INFO("Tolerância ao objetivo: %f", goal_tolerance_);

    nh_.param("max_linear_speed", max_linear_speed_, 0.5);
    ROS_INFO("Velocidade linear máxima: %f", max_linear_speed_);

    nh_.param("max_angular_speed", max_angular_speed_, 1.0);
    ROS_INFO("Velocidade angular máxima: %f", max_angular_speed_);

    nh_.param("rotation_speed", rotation_speed_, 0.5);
    ROS_INFO("Velocidade de rotação: %f", rotation_speed_);

    nh_.param("contour_speed", contour_speed_, 0.2);
    ROS_INFO("Velocidade de contorno: %f", contour_speed_);

    nh_.param("robot_width", robot_width_, 0.5);
    ROS_INFO("Largura do robô: %f", robot_width_);

    nh_.param("safety_margin", safety_margin_, 0.3);
    ROS_INFO("Margem de segurança: %f", safety_margin_);

    nh_.param("obstacle_threshold", obstacle_threshold_, 0.05);
    ROS_INFO("Limite de obstáculo: %f", obstacle_threshold_);

    nh_.param("gap_threshold", gap_threshold_, 0.2);
    ROS_INFO("Limite de lacuna: %f", gap_threshold_);

    // Initialize variables
    min_distance_to_goal_ = std::numeric_limits<double>::max();
    best_distance_to_goal_ = std::numeric_limits<double>::max();

    // Initialize goal to NaN
    goal_x_ = std::numeric_limits<double>::quiet_NaN();
    goal_y_ = std::numeric_limits<double>::quiet_NaN();
}

void TangentBug::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    if (msg->ranges.empty())
    {
        ROS_WARN("Varredura LIDAR recebida está vazia.");
        return;
    }

    // Verificar se todas as leituras são válidas
    for (const auto &range : msg->ranges)
    {
        if (std::isnan(range) || range < 0.0)
        {
            ROS_WARN("Valor inválido encontrado na varredura LIDAR.");
            return;
        }
    }

    laser_ranges_ = msg->ranges;
    current_scan_ = *msg; // Atualiza a varredura atual para cálculos futuros
    // ROS_INFO("Varredura LIDAR recebida: %zu leituras", msg->ranges.size());
}

void TangentBug::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (std::isnan(msg->pose.pose.position.x) || std::isnan(msg->pose.pose.position.y))
    {
        ROS_WARN("Odometria contém valores inválidos.");
        return;
    }

    current_pose_ = msg->pose.pose;
    // ROS_INFO("Odometria atualizada: Posição [%f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void TangentBug::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (std::isnan(msg->pose.position.x) || std::isnan(msg->pose.position.y))
    {
        ROS_WARN("Objetivo inválido recebido.");
        return;
    }

    // Verificar se o objetivo está muito próximo da posição atual
    double distance_to_goal = calculateDistance(current_pose_.position.x, current_pose_.position.y, msg->pose.position.x, msg->pose.position.y);
    if (distance_to_goal < goal_tolerance_)
    {
        ROS_INFO("Objetivo muito próximo, não é necessário movimento.");
        return;
    }

    goal_x_ = msg->pose.position.x;                              // Novo objetivo X
    goal_y_ = msg->pose.position.y;                              // Novo objetivo Y
    best_distance_to_goal_ = std::numeric_limits<double>::max(); // Reseta a melhor distância

    ROS_INFO("Novo objetivo recebido: Posição [%f, %f], Distância para o objetivo: %f", goal_x_, goal_y_, distance_to_goal);

    // Inserir o marcador de destino no Gazebo
    spawnGoalMarker(goal_x_, goal_y_);

    // Reseta para estado MOVE_TO_GOAL
    current_state_ = MOVE_TO_GOAL;
}

void TangentBug::navigate()
{
    ros::Rate rate(50);
    while (ros::ok())
    {
        computeControl();
        ros::spinOnce();
        rate.sleep();
    }
}

void TangentBug::computeControl()
{
    if (std::isnan(goal_x_) || std::isnan(goal_y_))
    {
        ROS_WARN_THROTTLE(5, "Aguardando um objetivo válido...");
        return;
    }
    const std::vector<std::vector<geometry_msgs::Point>> &obstacle_segments = extractObstacleSegments2D(current_scan_, 0.8); // Atualiza os segmentos de obstáculos
    std::cout << "obstacle_segments.size(): " << obstacle_segments.size() << std::endl;
    return;
    const double distance_to_goal = calculateDistance(current_pose_.position.x, current_pose_.position.y, goal_x_, goal_y_);
    ROS_INFO("Distância ao objetivo: %f", distance_to_goal);

    // Verifica se o robô já está no objetivo
    if (distance_to_goal < goal_tolerance_)
    {
        ROS_INFO("Objetivo alcançado!");
        stopRobot();
        return;
    }

    // Decisão baseada no estado atual
    switch (current_state_)
    {
    case MOVE_TO_GOAL:
        ROS_INFO("State: MOVE_TO_GOAL");
        if (isPathClear(calculateHeadingToGoal(), obstacle_threshold_) == 1)
        {
            moveToGoal(distance_to_goal);
        }
        else
        {
            ROS_INFO("Obstacle detected, changing state to FOLLOW_CONTOUR.");
            ROS_INFO("State transition: MOVE_TO_GOAL → FOLLOW_CONTOUR");
            current_state_ = FOLLOW_CONTOUR;
        }
        break;
    case FOLLOW_CONTOUR:
        ROS_INFO("State: FOLLOW_CONTOUR");
        followContour();
        if (isPathClear(calculateHeadingToGoal(), obstacle_threshold_) == 1)
        {
            ROS_INFO("Clear line of sight, changing state to LEAVE_CONTOUR.");
            stopRobot();
            // current_state_ = LEAVE_CONTOUR;
            ROS_INFO("State transition: FOLLOW_CONTOUR → MOVE_TO_GOAL");
            current_state_ = MOVE_TO_GOAL; // Mudar para MOVE_TO_GOAL diretamente
        }
        break;
    default:
        ROS_ERROR("Unknown state encountered!");
        stopRobot();
        break;
    }
    ROS_INFO("Estado atual: %d", current_state_);
}

void TangentBug::moveToGoal(double distance_to_goal)
{
    if (current_scan_.ranges.empty())
    {
        ROS_WARN("Cannot move to goal, LIDAR data is empty.");
        stopRobot();
        return;
    }

    geometry_msgs::Twist cmd_vel;

    // Calculate the heading to the goal and current robot orientation
    const double goal_heading = calculateHeadingToGoal();
    const double robot_yaw = tf::getYaw(current_pose_.orientation);

    // Compute heading error
    double heading_error = goal_heading - robot_yaw;
    heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error)); // Normalize to [-pi, pi]

    // Check for obstacles along the direct path
    if (isPathClear(goal_heading, obstacle_threshold_) != 1)
    {
        ROS_INFO("Obstacle detected on direct path, switching to FOLLOW_CONTOUR.");
        stopRobot();
        current_state_ = FOLLOW_CONTOUR;
        contour_start_x_ = current_pose_.position.x;
        contour_start_y_ = current_pose_.position.y;
        best_distance_to_goal_ = distance_to_goal;
        return;
    }

    // Proportional control for angular speed to face the goal
    cmd_vel.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, heading_error * 1.5));

    // Move forward only if orientation is close enough to goal direction
    if (std::abs(heading_error) < (M_PI / 6)) // Allow up to 30 degrees tolerance
    {
        cmd_vel.linear.x = std::min(max_linear_speed_, distance_to_goal * 0.7);
    }
    else
    {
        cmd_vel.linear.x = 0.0; // Stop forward motion while turning
    }

    // Check if goal is reached
    if (distance_to_goal < goal_tolerance_)
    {
        ROS_INFO("Goal reached successfully!");
        stopRobot();
        current_state_ = MOVE_TO_GOAL;
        return;
    }

    // Publish velocity commands
    cmd_vel_pub_.publish(cmd_vel);

    // Debugging output
    ROS_DEBUG("Moving to goal: distance=%.2f m, heading_error=%.2f rad, linear_vel=%.2f m/s, angular_vel=%.2f rad/s",
              distance_to_goal, heading_error, cmd_vel.linear.x, cmd_vel.angular.z);
}

void TangentBug::followContour()
{
    if (current_scan_.ranges.empty())
    {
        ROS_WARN("Cannot follow contour, LIDAR data is empty.");
        stopRobot();
        return;
    }

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = contour_speed_; // Safe, predefined contour speed

    const double robot_yaw = tf::getYaw(current_pose_.orientation);
    const size_t num_ranges = current_scan_.ranges.size();
    const double angle_increment = current_scan_.angle_increment;

    // Identify the closest obstacle to the robot within a reasonable range
    float min_distance = std::numeric_limits<float>::max();
    int min_index = -1;

    for (size_t i = 0; i < num_ranges; ++i)
    {
        float range = current_scan_.ranges[i];
        if (std::isfinite(range) && range < min_distance)
        {
            min_distance = range;
            min_index = i;
        }
    }

    // If no valid obstacle detected, rotate to find one
    if (min_index == -1)
    {
        // ROS_WARN("No obstacle detected, rotating to find contour.");
        // rotateInPlace(rotation_speed_, 0.5);
        current_state_ = MOVE_TO_GOAL;
        return;
    }

    // Calculate angle to closest obstacle
    double obstacle_angle = current_scan_.angle_min + min_index * angle_increment;

    // Desired contour-following distance from obstacle
    const double desired_contour_distance = safety_margin_;

    // Control logic: proportional controller to maintain contour distance
    const double distance_error = min_distance - desired_contour_distance;

    // If obstacle is on the left (positive angle), turn slightly right, and vice versa
    const double angular_direction = (obstacle_angle > 0) ? -1.0 : 1.0;

    // Adjust angular velocity proportionally to the distance error and angle to the obstacle
    cmd_vel.angular.z = angular_direction * (0.5 * std::abs(obstacle_angle) + 1.0 * distance_error);

    // Limit angular speed
    cmd_vel.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, cmd_vel.angular.z));

    // Check periodically if path to goal has become clear
    if (isPathClear(calculateHeadingToGoal(), obstacle_threshold_) == 1)
    {
        ROS_INFO("Clear path detected during contour following, preparing to leave contour.");
        stopRobot();
        current_state_ = MOVE_TO_GOAL;
        return;
    }

    // Publish the velocity command to follow the contour
    cmd_vel_pub_.publish(cmd_vel);

    // Debugging output
    ROS_DEBUG("Following contour: min_dist=%.2f, obstacle_angle=%.2f rad, linear_vel=%.2f m/s, angular_vel=%.2f rad/s",
              min_distance, obstacle_angle, cmd_vel.linear.x, cmd_vel.angular.z);
}

void TangentBug::leaveContour()
{
    double distance_from_contour_start = calculateDistance(current_pose_.position.x, current_pose_.position.y, contour_start_x_, contour_start_y_);

    // Verifica se já estamos suficientemente longe do ponto de início do contorno e se o caminho está livre
    int path_status = isPathClear(calculateHeadingToGoal(), obstacle_threshold_);
    std::cout << "path_status: " << path_status << std::endl;
    std::cout << "distance_from_contour_start: " << distance_from_contour_start << std::endl;
    if (distance_from_contour_start > 1.0 && path_status == 1)
    { // Caminho livre
        ROS_INFO("Saindo do contorno para o objetivo, caminho livre.");

        // Suavizar a transição publicando algumas mensagens de parada
        stopRobot();

        // Mudar para o estado de seguir o objetivo diretamente
        current_state_ = MOVE_TO_GOAL;
    }
    else if (path_status == -1)
    { // Ângulo fora do campo de visão do LIDAR
        ROS_WARN("Tentativa de sair do contorno, mas o ângulo ainda está fora do campo de visão do LIDAR, girando no próprio eixo...");
        rotateInPlace();
    }
    else
    {
        ROS_WARN("Tentativa de sair do contorno, mas o caminho ainda não está livre ou o robô não se afastou o suficiente.");

        // Manter a distância de segurança do obstáculo durante a tentativa de sair do contorno
        rotateInPlace(rotation_speed_, 0.5);
    }

    // Log de depuração para acompanhamento
    ROS_INFO("Distância do início do contorno: %f", distance_from_contour_start);
}

void TangentBug::stopRobot(int repeat_times)
{
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    for (int i = 0; i < repeat_times; i++)
    {
        cmd_vel_pub_.publish(stop_msg);
        ros::Duration(0.1).sleep();
    }
}

void TangentBug::rotateInPlace(double angular_speed, double duration_seconds)
{
    std::cout << "rotateInPlace" << std::endl;
    geometry_msgs::Twist rotate_msg;
    rotate_msg.linear.x = 0.0;
    rotate_msg.angular.z = angular_speed;

    ros::Rate rate(50);
    int ticks = duration_seconds * 50; // duration in seconds * rate
    for (int i = 0; i < ticks; ++i)
    {
        cmd_vel_pub_.publish(rotate_msg);
        rate.sleep();
    }
    std::cout << "rotateInPlace: stopRobot" << std::endl;
    stopRobot();
}

double TangentBug::calculateDistance(double x1, double y1, double x2, double y2)
{
    // Cálculo da distância euclidiana
    double distance = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

    // Log mais detalhado para depuração
    ROS_INFO("Posicao atual: [%f, %f], Objetivo: [%f, %f]", x1, y1, x2, y2);

    // Retorna a distância calculada
    return distance;
}

/**
 * Calcula o ângulo para o objetivo com base na posição atual do robô.
 */
double TangentBug::calculateHeadingToGoal()
{
    // Verificar se já estamos no objetivo ou muito perto dele
    if (fabs(goal_x_ - current_pose_.position.x) < 0.01 && fabs(goal_y_ - current_pose_.position.y) < 0.01)
    {
        ROS_INFO("Já estamos no objetivo!");
        return 0.0; // Retornar 0 ou algum valor apropriado
    }

    // Calcular o ângulo para o objetivo usando atan2
    double goal_heading = atan2(goal_y_ - current_pose_.position.y, goal_x_ - current_pose_.position.x);

    // Log para depuração
    ROS_INFO("Ângulo para o objetivo (radianos): %f", goal_heading);

    // Normalizar o ângulo para o intervalo [0, 2*pi] se necessário
    goal_heading = fmod(goal_heading + 2 * M_PI, 2 * M_PI);

    // Retornar o ângulo calculado
    return goal_heading;
}

int TangentBug::isPathClear(double heading, double obstacle_threshold)
{
    double box_width = (robot_width_ / 2) + obstacle_threshold;
    double cone_angle = std::atan2(safety_margin_, box_width);

    if (current_scan_.ranges.empty())
    {
        ROS_WARN("LIDAR data is empty.");
        return 0;
    }

    const auto &ranges = current_scan_.ranges;
    double angle_min = current_scan_.angle_min;
    double angle_increment = current_scan_.angle_increment;

    std::cout << "angle_min: " << angle_min << std::endl;
    std::cout << "angle_increment: " << angle_increment << std::endl;
    std::cout << "cone_angle: " << cone_angle << std::endl;

    // Compute relative heading
    double robot_yaw = tf::getYaw(current_pose_.orientation);

    for (size_t i = 0; i < ranges.size(); ++i)
    {
        const float range = ranges[i];
        if (std::isnan(range) || range <= 0.0)
            continue;

        // Compute relative angle, adjusted by desired heading
        double angle = angle_min + i * angle_increment - heading;

        // Decide dynamic required distance
        double required_distance;
        if (std::abs(angle) <= cone_angle)
        {
            required_distance = safety_margin_;
            if (range < required_distance)
            {
                ROS_WARN("[Cone] Obstacle at %.2f rad (%.1f deg), range %.2f m < %.2f m",
                         angle, angle * 180.0 / M_PI, range, required_distance);
                stopRobot();
                return 0;
            }
        }
        else
        {
            double sin_angle = std::sin(angle);

            required_distance = (std::abs(sin_angle) > 1e-3) ? (box_width / std::abs(sin_angle)) : current_scan_.range_max;

            if (range < required_distance)
            {
                ROS_WARN("[Box] Obstacle at %.2f rad (%.1f deg), range %.2f m < %.2f m",
                         angle, angle * 180.0 / M_PI, range, required_distance);
                stopRobot();
                return 0;
            }
        }
    }

    ROS_INFO("Cone and safety box are clear.");
    return 1;
}

void TangentBug::spawnGoalMarker(double x, double y)
{
    ros::ServiceClient spawn_model_client = nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    ros::ServiceClient delete_client = nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

    gazebo_msgs::SpawnModel spawn_model_srv;

    // Caminho para o modelo SDF
    std::string model_path = ros::package::getPath("tangent_bug_2dnav") + "/urdf/models/goal_marker/goal.sdf";

    // Ler o arquivo SDF do disco
    std::ifstream model_file(model_path);
    std::stringstream buffer;
    buffer << model_file.rdbuf();
    spawn_model_srv.request.model_xml = buffer.str();

    // Nome do modelo
    spawn_model_srv.request.model_name = "goal_marker";
    gazebo_msgs::DeleteModel delete_srv;
    delete_srv.request.model_name = spawn_model_srv.request.model_name;

    // Definir a posição onde o marcador será spawnado
    spawn_model_srv.request.initial_pose.position.x = x;
    spawn_model_srv.request.initial_pose.position.y = y;
    spawn_model_srv.request.initial_pose.position.z = 0.1; // Um pouco acima do solo
    spawn_model_srv.request.initial_pose.orientation.w = 1.0;

    // Namespace vazio
    spawn_model_srv.request.robot_namespace = "";

    // Verifica se o modelo existe e tenta deletá-lo
    if (delete_client.call(delete_srv))
    {
        if (delete_srv.response.success)
        {
            ROS_INFO("Modelo '%s' deletado com sucesso.", spawn_model_srv.request.model_name.c_str());
        }
        else
        {
            ROS_WARN("Falha ao deletar o modelo '%s': %s", spawn_model_srv.request.model_name.c_str(), delete_srv.response.status_message.c_str());
        }
    }
    else
    {
        ROS_ERROR("Erro ao chamar o serviço de deleção do modelo.");
    }

    // Esperar até o serviço estar disponível
    if (spawn_model_client.call(spawn_model_srv))
    {
        ROS_INFO("Marcador de destino inserido no Gazebo no ponto [%f, %f]", x, y);
    }
    else
    {
        ROS_ERROR("Falha ao spawnar o marcador de destino no Gazebo.");
    }
}

void TangentBug::moveToPoint(double target_x, double target_y)
{
    geometry_msgs::Twist cmd_vel;

    double target_heading = atan2(target_y - current_pose_.position.y, target_x - current_pose_.position.x);
    double robot_yaw = tf::getYaw(current_pose_.orientation);

    double heading_error = target_heading - robot_yaw;
    heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));

    double distance_to_target = calculateDistance(current_pose_.position.x, current_pose_.position.y, target_x, target_y);

    // Rotate robot first to face target point
    if (std::abs(heading_error) > (M_PI / 18)) // ~10 degrees tolerance
    {
        cmd_vel.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, heading_error * 1.2));
        cmd_vel.linear.x = 0.0;
    }
    else
    {
        cmd_vel.linear.x = std::min(max_linear_speed_, distance_to_target * 0.7);
        cmd_vel.angular.z = heading_error * 0.5;
    }

    cmd_vel_pub_.publish(cmd_vel);

    ROS_DEBUG("moveToPoint: target=(%.2f, %.2f), distance=%.2f, heading_error=%.2f, linear_vel=%.2f, angular_vel=%.2f",
              target_x, target_y, distance_to_target, heading_error, cmd_vel.linear.x, cmd_vel.angular.z);
}

void TangentBug::followObstacle()
{
    if (current_scan_.ranges.empty())
    {
        ROS_WARN("LIDAR data is empty.");
        stopRobot();
        return;
    }

    // Find min and max index of obstacle range
    size_t min_index = 0;
    size_t max_index = 0;
    float min_distance = std::numeric_limits<float>::max();

    for (size_t i = 0; i < current_scan_.ranges.size(); ++i)
    {
        float range = current_scan_.ranges[i];
        if (std::isfinite(range) && range < min_distance)
        {
            min_distance = range;
            min_index = i;
        }
    }

    // Define max_index to create a segment (e.g., ±15 degrees from min_index)
    int segment_width = 15 * M_PI / 180 / current_scan_.angle_increment;
    min_index = std::max(0, static_cast<int>(min_index) - segment_width);
    max_index = std::min(static_cast<int>(current_scan_.ranges.size() - 1), static_cast<int>(min_index) + 2 * segment_width);

    // Calculate midpoint of the segment
    int mid_index = (min_index + max_index) / 2;
    float mid_angle = current_scan_.angle_min + mid_index * current_scan_.angle_increment;
    float mid_distance = current_scan_.ranges[mid_index];

    // Transform midpoint from polar to Cartesian coordinates relative to robot
    double obstacle_x = current_pose_.position.x + mid_distance * cos(mid_angle + tf::getYaw(current_pose_.orientation));
    double obstacle_y = current_pose_.position.y + mid_distance * sin(mid_angle + tf::getYaw(current_pose_.orientation));

    // Move towards the midpoint of the detected obstacle segment
    moveToPoint(obstacle_x, obstacle_y);

    ROS_DEBUG("followObstacle: obstacle midpoint=(%.2f, %.2f), mid_distance=%.2f, mid_angle=%.2f",
              obstacle_x, obstacle_y, mid_distance, mid_angle);
}

std::vector<int> TangentBug::detectObstacleBoundaries2D(const sensor_msgs::LaserScan &scan, double threshold)
{
    std::vector<int> obstacle_boundaries;

    if (scan.ranges.empty())
    {
        ROS_WARN("LIDAR data is empty.");
        return obstacle_boundaries;
    }

    const auto &ranges = scan.ranges;
    const double angle_min = scan.angle_min;
    const double angle_increment = scan.angle_increment;

    for (size_t i = 1; i < ranges.size(); ++i)
    {
        float r1 = ranges[i - 1];
        float r2 = ranges[i];

        if (!std::isfinite(r1) || !std::isfinite(r2))
            continue;

        double angle1 = angle_min + (i - 1) * angle_increment;
        double angle2 = angle_min + i * angle_increment;

        double x1 = r1 * std::cos(angle1);
        double y1 = r1 * std::sin(angle1);
        double x2 = r2 * std::cos(angle2);
        double y2 = r2 * std::sin(angle2);

        double euclidean_dist = std::hypot(x2 - x1, y2 - y1);

        if (euclidean_dist > threshold)
        {
            obstacle_boundaries.push_back(i);
        }
    }

    ROS_DEBUG("Detected %zu 2D obstacle boundaries.", obstacle_boundaries.size());
    return obstacle_boundaries;
}

std::vector<std::vector<geometry_msgs::Point>> TangentBug::extractObstacleSegments2D(
    const sensor_msgs::LaserScan &scan, double threshold)
{
    std::cout << "extractObstacleSegments2D" << std::endl;
    std::vector<std::vector<geometry_msgs::Point>> segments;
    std::vector<geometry_msgs::Point> current_segment;

    if (scan.ranges.empty())
    {
        ROS_WARN("extractObstacleSegments2D: Empty scan.");
        return segments;
    }

    double angle = scan.angle_min;
    for (size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment)
    {
        float range = scan.ranges[i];
        if (!std::isfinite(range))
            continue;

        geometry_msgs::Point pt;
        pt.x = range * std::cos(angle);
        pt.y = range * std::sin(angle);
        pt.z = 0.0;

        if (!current_segment.empty())
        {
            const geometry_msgs::Point &last = current_segment.back();
            double dist = std::hypot(pt.x - last.x, pt.y - last.y);

            if (dist >= threshold)
            {
                segments.push_back(current_segment);
                current_segment.clear();
            }
        }

        current_segment.push_back(pt);
    }

    // Add the final segment
    if (!current_segment.empty())
    {
        segments.push_back(current_segment);
    }

    return segments;
}

geometry_msgs::Point TangentBug::findClosestEndpointToPoint(
    const std::vector<std::vector<geometry_msgs::Point>> &segments,
    const geometry_msgs::Point &target)
{
    geometry_msgs::Point closest_point;
    double min_distance = std::numeric_limits<double>::max();

    for (const auto &segment : segments)
    {
        if (segment.empty())
            continue;

        const auto &first = segment.front();
        const auto &last = segment.back();

        double dist_first = std::hypot(target.x - first.x, target.y - first.y);
        if (dist_first < min_distance)
        {
            min_distance = dist_first;
            closest_point = first;
        }

        double dist_last = std::hypot(target.x - last.x, target.y - last.y);
        if (dist_last < min_distance)
        {
            min_distance = dist_last;
            closest_point = last;
        }
    }

    return closest_point;
}
