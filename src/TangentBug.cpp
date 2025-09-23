#include "TangentBug.h"
#include <tf/transform_datatypes.h>
#include <cmath>

TangentBug::TangentBug(ros::NodeHandle &nh) : nh_(nh),
                                              current_state_(MOVE_TO_GOAL),
                                              tf_buffer_(ros::Duration(1.0)), // cache de 10s
                                              tf_listener_(tf_buffer_)         // inicia listener
{

    // Subscribers and publishers
    scan_sub_ = nh_.subscribe("/scan", 1, &TangentBug::scanCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &TangentBug::odomCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal/bug", 1, &TangentBug::goalCallback, this);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Initialize parameters with default values and log them
    ros::NodeHandle pnh("~");
    nh_.param("goal_tolerance", goal_tolerance_, 0.10);
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

    nh_.param("safety_margin", safety_margin_, 0.5);
    ROS_INFO("Margem de segurança: %f", safety_margin_);

    nh_.param("obstacle_threshold", obstacle_threshold_, 0.6);
    ROS_INFO("Limite de obstáculo: %f", obstacle_threshold_);

    nh_.param("gap_threshold", gap_threshold_, 0.8);
    ROS_INFO("Limite de lacuna: %f", gap_threshold_);

    nh_.param("segment_switch_threshold_percent", segment_switch_threshold_percent_, 0.05); // 5% como padrão
    ROS_INFO("Limiar de troca de segmento (percentual): %f", segment_switch_threshold_percent_);

    // Inicializa o estado interno de girar - e - avançar current_turn_then_move_state_ = ALIGNING_TO_POINT;
    // Marca o ponto alvo como inválido para garantir que a primeira chamada o defina.
    target_point_for_turn_then_move_.x = std::numeric_limits<double>::quiet_NaN();
    target_point_for_turn_then_move_.y = std::numeric_limits<double>::quiet_NaN();
    target_point_for_turn_then_move_.z = 0.0;

    // Define a tolerância para considerar o alinhamento completo (ex: 0.05 rad ~ 2.8 graus)
    nh_.param("alignment_tolerance_rad", alignment_tolerance_rad_, 0.05);
    ROS_INFO("Tolerância de alinhamento (radianos): %f", alignment_tolerance_rad_);

    // Initialize variables
    current_stable_best_segment_distance_ = std::numeric_limits<double>::max();
    d_reach_ = std::numeric_limits<double>::max();

    // Initialize goal to NaN
    goal_x_ = std::numeric_limits<double>::quiet_NaN();
    goal_y_ = std::numeric_limits<double>::quiet_NaN();

    // Inicializa o timer de controle para chamar computeControl periodicamente
    // É crucial para o comportamento não-bloqueante do nó.
    // control_timer_ = nh_.createTimer(ros::Duration(1.0 / 60.0), &TangentBug::computeControl, this); // 60 Hz
    ROS_INFO("TangentBug node initialized. Waiting for goal...");
    clearAllSpheresAndSegments();
    stopRobot(); // Garante que o robô comece parado
}

void TangentBug::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lk(scan_mutex_);
    current_scan_ = *msg;
    // ROS_INFO("Varredura LIDAR recebida: %zu leituras", msg->ranges.size());

    const int num_ranges_valid = std::count_if(current_scan_.ranges.begin(), current_scan_.ranges.end(),
                                               [](float range)
                                               { return !std::isnan(range) && range > 0.0 && std::isfinite(range); });
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

    point_goal_ = msg->pose.position; // Atualiza o ponto objetivo
    goal_x_ = msg->pose.position.x;   // Novo objetivo X
    goal_y_ = msg->pose.position.y;   // Novo objetivo Y

    ROS_INFO("Novo objetivo recebido: Posição [%f, %f], Distância para o objetivo: %f", goal_x_, goal_y_, distance_to_goal);

    // Inserir o marcador de destino no Gazebo
    spawnGoalMarker(goal_x_, goal_y_);
}

void TangentBug::navigate()
{
    stopRobot();
    ros::Rate rate(100);
    while (ros::ok())
    {
        std::cout << "TangentBug::navigate()" << std::endl;
        computeControl();
        ros::spinOnce();
    }
}

void TangentBug::computeControl()
{
    std::cout << "------------------------------------------------------------------------" << std::endl;
    std::cout << "TangentBug::computeControl()" << std::endl;
    if (std::isnan(goal_x_) || std::isnan(goal_y_))
    {
        stopRobot(); // Publica 0 para evitar movimento inesperado
        ROS_WARN_THROTTLE(5, "Aguardando um objetivo válido...");
        return;
    }

    if (current_scan_.ranges.empty() || std::isnan(current_pose_.position.x) || std::isnan(goal_x_))
    {
        ROS_WARN_THROTTLE(5, "Aguardando dados LIDAR, odometria ou objetivo válido para iniciar o controle.");
        stopRobot();
        return;
    }

    if (robot_trajectory_history_.empty())
    {
        // Marca a posição inicial do robô
        geometry_msgs::Point initial_position = current_pose_.position;
        robot_trajectory_history_.push_back(initial_position);
        spawnSphereAt(initial_position.x, initial_position.y, 0.0, "#FFFFFF"); // Marca a posição inicial do robô no Gazebo
    }

    // const geometry_msgs::Point last_robot_position = robot_trajectory_history_.back();
    // if (calculateDistance(last_robot_position.x, last_robot_position.y, current_pose_.position.x, current_pose_.position.y) > 0.1 || robot_trajectory_history_.size() == 1)
    // {
    //     // Adiciona a nova posição do robô ao histórico
    //     robot_trajectory_history_.push_back(current_pose_.position);
    //     spawnSphereAt(current_pose_.position.x, current_pose_.position.y, 0.0, "#FFFFFE"); // Marca a nova posição do robô no Gazebo
    //     clearAllSpheresAndSegments("segment_");
    // }
    // else
    // {
    //     ROS_DEBUG("A posição do robô não mudou significativamente desde a última atualização. Ignorando.");
    //     return; // Ignora se a posição do robô não mudou significativamente
    // }

    const auto &robot_position = current_pose_.position;
    geometry_msgs::Twist cmd_vel;

    double distance_to_goal = calculateDistance(robot_position.x, robot_position.y, goal_x_, goal_y_);
    ROS_INFO("Distância ao objetivo: %f", distance_to_goal);
    spawnSphereAt(goal_x_, goal_y_, 0.0, "#FF0000"); // Atualiza o marcador do objetivo no Gazebo

    // Verifica se o robô já está no objetivo
    if (distance_to_goal < goal_tolerance_)
    {
        ROS_INFO("Objetivo alcançado!");
        stopRobot();
        current_state_ = MOVE_TO_GOAL;                      // Reseta o estado
        goal_x_ = std::numeric_limits<double>::quiet_NaN(); // Reseta o objetivo
        goal_y_ = std::numeric_limits<double>::quiet_NaN(); // Reseta o objetivo
        return;
    }

    sensor_msgs::LaserScan scan;
    {
        std::lock_guard<std::mutex> lk(scan_mutex_);
        if (current_scan_.ranges.empty())
            return;
        scan = current_scan_; // cópia local estável
    }

    // Obtém os segmentos de obstáculos do LIDAR
    std::cout << "Extraindo segmentos de obstáculos do LIDAR..." << std::endl;
    std::cout << "Número de leituras do LIDAR: " << scan.ranges.size() << std::endl;

    // for (size_t i = 0; i < scan.ranges.size(); ++i)
    // {
    //     std::cout << "Leitura " << i << ": " << scan.ranges[i] << std::endl;
    // }

    const int num_ranges_valid = std::count_if(scan.ranges.begin(), scan.ranges.end(),
                                               [](float range)
                                               { return !std::isnan(range) && range > 0.0 && std::isfinite(range); });
    std::cout << "Número de leituras válidas do LIDAR: " << num_ranges_valid << std::endl;

    const std::vector<std::vector<geometry_msgs::Point>> global_segments = extractObstacleSegments2D(scan, gap_threshold_);
    std::cout << "Número de segmentos globais: " << global_segments.size() << std::endl;
    // Segmentos simplificados
    std::vector<std::vector<geometry_msgs::Point>> simplified_segments = std::vector<std::vector<geometry_msgs::Point>>();

    // Percorre todos os segmentos globais e simplifica cada um deles
    for (const auto &segment : global_segments)
    {
        // Simplifica cada segmento
        std::vector<geometry_msgs::Point> simplified_segment = simplifySegment(segment, robot_width_, 30.0);
        //  simplified_segment = offsetSegmentTowardRobotPerspective(simplified_segment, robot_position, 30.0);
        if (!simplified_segment.empty())
        {
            simplified_segments.push_back(simplified_segment);
        }
    }

    for (const auto &segment : simplified_segments)
    {
        std::cout << "Segmento simplificado: " << std::endl;
        for (const auto &point : segment)
        {
            std::cout << "(" << point.x << ", " << point.y << ") " << std::endl;
            spawnSphereAt(point.x, point.y, 0.0, "#00FF00", 1); // Marca os pontos do segmento simplificado no Gazebo
        }
    }

    std::cout << "Número de segmentos simplificados: " << simplified_segments.size() << std::endl;
    const int total_points_global_segments = std::accumulate(global_segments.begin(), global_segments.end(), 0,
                                                             [](int sum, const std::vector<geometry_msgs::Point> &segment)
                                                             { return sum + segment.size(); });

    const int total_points_simplified_segments = std::accumulate(simplified_segments.begin(), simplified_segments.end(), 0,
                                                                 [](int sum, const std::vector<geometry_msgs::Point> &segment)
                                                                 { return sum + segment.size(); });

    ROS_DEBUG("Total de pontos nos segmentos globais: %d, Total de pontos nos segmentos simplificados: %d", total_points_global_segments, total_points_simplified_segments);
    std::cout << "Total de pontos nos segmentos globais: " << total_points_global_segments
              << ", Total de pontos nos segmentos simplificados: " << total_points_simplified_segments << std::endl;

    // Procura o segmento de reta que intercepta o segmento do robô ao objetivo
    std::vector<geometry_msgs::Point> intersection_segment;

    // Encontrar a interseção mais próxima
    double min_dist_intersect = std::numeric_limits<double>::max();
    double current_min_dist_to_goal = std::numeric_limits<double>::max();
    geometry_msgs::Point closest_intersection;
    geometry_msgs::Point closest_point_to_goal = point_goal_;
    bool found_intersection = false;

    // spawnLineSegment(robot_position.x, robot_position.y, 0.0, goal_x_, goal_y_, 0.0, "#430073"); // Linha do robô ao objetivo
    std::vector<std::vector<geometry_msgs::Point>> parallel_segments = parallelSegmentsAtDistance(robot_position, point_goal_, (robot_width_ + safety_margin_) / 2.0);
    spawnLineSegment(parallel_segments[0][0].x, parallel_segments[0][0].y, 0.0,
                     parallel_segments[0][1].x, parallel_segments[0][1].y, 0.0, "#000FFF", 2); // Linha paralela 1
    spawnLineSegment(parallel_segments[1][0].x, parallel_segments[1][0].y, 0.0,
                     parallel_segments[1][1].x, parallel_segments[1][1].y, 0.0, "#000FFF", 2); // Linha paralela 2
    for (const auto &segment : simplified_segments)
    {
        for (size_t i = 0; i + 1 < segment.size(); ++i)
        {
            bool intersects;
            // std::cout << "Verificando o ponto " << i << " do segmento: (" << segment[i].x << ", " << segment[i].y << ")"
            //           << " até o ponto " << i + 1 << ": (" << segment[i + 1].x << ", " << segment[i + 1].y << ")" << std::endl;
            // std::cout << "Segmento do robô ao objetivo: (" << segment_robot_to_goal[0].x << ", " << segment_robot_to_goal[0].y << ")"
            //           << " até (" << segment_robot_to_goal[1].x << ", " << segment_robot_to_goal[1].y << ")" << std::endl;

            IntersectionResult intersection = intersectSegments(segment[i], segment[i + 1],
                                                                parallel_segments[0][0], parallel_segments[0][1]);

            geometry_msgs::Point inter = intersection.point;
            intersects = intersection.intersects;
            if (!intersects)
            {
                intersection = intersectSegments(
                    segment[i], segment[i + 1],
                    parallel_segments[1][0], parallel_segments[1][1]);
                inter = intersection.point;
                intersects = intersection.intersects;
            }

            // spawnLineSegment(segment[i].x, segment[i].y, 0.0, segment[i + 1].x, segment[i + 1].y, 0.0, "#787878", segment.size()); // Linha do segmento do obstáculo

            double dist;
            if (intersects)
            {
                std::cout << "Interseção encontrada no ponto: (" << inter.x << ", " << inter.y << ")" << std::endl;
                dist = calculateDistance(robot_position.x, robot_position.y, inter.x, inter.y);
                std::cout << "Distância do robô até a interseção: " << dist << std::endl;

                intersection_segment = {segment[i], segment[i + 1]};
                if (dist < min_dist_intersect)
                {
                    min_dist_intersect = dist;
                    closest_intersection = inter;
                    found_intersection = true;
                }
                // if (dist < current_min_dist_to_goal)
                // {
                //     current_min_dist_to_goal = dist;
                //     closest_point_to_goal = inter;
                // }
            }

            dist = calculateDistance(segment[i].x, segment[i].y, goal_x_, goal_y_);
            std::cout << "Ponto do segmento: (" << segment[i].x << ", " << segment[i].y << ")" << std::endl;
            std::cout << "Distância do segmento até o objetivo: " << dist << std::endl;
            if (dist < current_min_dist_to_goal)
            {
                current_min_dist_to_goal = dist;
                closest_point_to_goal = segment[i];
            }
            if (i + 2 == segment.size())
            {
                dist = calculateDistance(segment[i + 1].x, segment[i + 1].y, goal_x_, goal_y_);
                std::cout << "Ponto do segmento: (" << segment[i + 1].x << ", " << segment[i + 1].y << ")" << std::endl;
                std::cout << "Distância do segmento até o objetivo: " << dist << std::endl;
                if (dist < current_min_dist_to_goal)
                {
                    current_min_dist_to_goal = dist;
                    closest_point_to_goal = segment[i + 1];
                }
            }
        }
    }

    // if (current_min_dist_to_goal > distance_to_goal || current_state_ == MOVE_TO_GOAL)
    // {
    //     std::cout << "Podemos ir direto ao objetivo" << std::endl;
    //     d_reach_point_ = point_goal_;
    // }

    std::cout << "Estado atual: " << (current_state_ == MOVE_TO_GOAL ? "MOVE_TO_GOAL" : current_state_ == FOLLOW_CONTOUR ? "FOLLOW_CONTOUR"
                                                                                                                         : "LEAVE_CONTOUR")
              << std::endl;
    std::cout << "Tem interseção: " << (found_intersection ? "Sim" : "Não") << std::endl;
    std::cout << "d_reach_:                     " << d_reach_ << std::endl;
    std::cout << "distance_to_goal:             " << distance_to_goal << std::endl;
    std::cout << "current_min_dist_to_goal:     " << current_min_dist_to_goal << std::endl;
    std::cout << "Indo para o ponto de alcance: (" << d_reach_point_.x << ", " << d_reach_point_.y << ")" << std::endl;
    std::cout << "Ponto atual do robô:          (" << robot_position.x << ", " << robot_position.y << ")" << std::endl;

    // Indo para o objetivo
    if (current_state_ == MOVE_TO_GOAL)
    {
        current_min_dist_to_goal = current_min_dist_to_goal == std::numeric_limits<double>::max() ? distance_to_goal : current_min_dist_to_goal;
        const float heuristic_to_goal_via_closest_obstacle_point = calculateDistance(robot_position.x, robot_position.y,
                                                                           closest_point_to_goal.x, closest_point_to_goal.y) +
                                                            calculateDistance(closest_point_to_goal.x, closest_point_to_goal.y,
                                                                              goal_x_, goal_y_);
        d_reach_ = distance_to_goal < d_reach_ ? distance_to_goal : d_reach_;
        // if (current_min_dist_to_goal > distance_to_goal || found_intersection)
        // if (found_intersection || current_min_dist_to_goal > d_reach_)
        if (!(!found_intersection || heuristic_to_goal_via_closest_obstacle_point < distance_to_goal))
        // if (found_intersection)
        {
            current_state_ = FOLLOW_CONTOUR;
            std::cout << "Vamos ter que seguir a borda" << std::endl;
            // d_reach_point_ = intersection_segment[0]; // Inicializa com o primeiro ponto do primeiro segmento
            // ultimo ponto do segmento
            d_reach_point_ = simplified_segments[0][simplified_segments[0].size() - 1];
        }
        else
        {
            std::cout << "Podemos ir direto ao objetivo" << std::endl;
            d_reach_point_ = point_goal_;
            // d_reach_ = distance_to_goal; // Eu acho que tem que ser assim
        }
    }
    else if (current_state_ == FOLLOW_CONTOUR)
    {

        if (current_min_dist_to_goal < d_reach_ && !found_intersection)
        {
            std::cout << "G2 não é vazio" << std::endl;
            d_reach_point_ = closest_point_to_goal;
            current_state_ = LEAVE_CONTOUR;
        }
        else
        {
            std::cout << "Continuando a seguir o contorno" << std::endl;
            d_reach_point_ = simplified_segments.size() > 0 ? simplified_segments[0][simplified_segments[0].size() - 1] : point_goal_; // Continua indo para o primeiro ponto do primeiro segmento
            // d_reach_point_ = closest_point_to_goal;
        }
        d_reach_ = distance_to_goal < d_reach_ ? distance_to_goal : d_reach_;

        // else
        // {
        //     d_reach_point_ = closest_point_to_goal;
        //     d_reach_ = current_min_dist_to_goal;
        //     std::cout << "G2 é vazio" << std::endl;
        // }
    }
    else if (current_state_ == LEAVE_CONTOUR)
    {
        if (distance_to_goal < d_reach_)
        {
            current_state_ = MOVE_TO_GOAL;
            d_reach_point_ = point_goal_;
            // d_reach_ = distance_to_goal;
            std::cout << "Voltando para motion-to-goal" << std::endl;
        }
    }

    std::cout << "Indo para o estado: " << (current_state_ == MOVE_TO_GOAL ? "MOVE_TO_GOAL" : current_state_ == FOLLOW_CONTOUR ? "FOLLOW_CONTOUR"
                                                                                                                               : "LEAVE_CONTOUR")
              << std::endl;
    std::cout << "d_reach_: " << d_reach_ << std::endl;
    std::cout << "Distância ao objetivo: " << distance_to_goal << std::endl;
    std::cout << "Distância mínima atual ao objetivo: " << current_min_dist_to_goal << std::endl;
    std::cout << "Indo para o ponto de alcance: (" << d_reach_point_.x << ", " << d_reach_point_.y << ")" << std::endl;
    std::cout << "Ponto atual do robô: (" << robot_position.x << ", " << robot_position.y << ")" << std::endl;

    spawnSphereAt(d_reach_point_.x, d_reach_point_.y, 0.0, "#FFFF00", 1); // Atualiza o marcador do ponto de alcance no Gazebo

    std::cout << "Indo para o ponto de alcance: (" << d_reach_point_.x << ", " << d_reach_point_.y << ")" << std::endl;
    moveToPoint(d_reach_point_.x, d_reach_point_.y);
    // Espera 1 segundo
    ros::Duration(1.0).sleep();
    stopRobot(); // Garante que o robô pare antes de mudar de direção
}

// Refatorada para calcular e retornar Twist
geometry_msgs::Twist TangentBug::calculateMoveToGoalCommand(double distance_to_goal)
{
    geometry_msgs::Twist cmd_vel;

    const double goal_heading = calculateHeadingToGoal();
    const double robot_yaw = tf::getYaw(current_pose_.orientation);
    double heading_error = goal_heading - robot_yaw;
    heading_error = normalizeAngle(heading_error);

    cmd_vel.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, heading_error * 1.5));

    if (std::abs(heading_error) < (M_PI / 6.0)) // Allow up to 30 degrees tolerance (M_PI / 6)
    {
        cmd_vel.linear.x = std::min(max_linear_speed_, distance_to_goal * 0.7);
    }
    else
    {
        cmd_vel.linear.x = 0.0; // Stop forward motion while turning
    }
    ROS_DEBUG("MoveToGoal: dist=%.2f, error=%.2f, lin=%.2f, ang=%.2f",
              distance_to_goal, heading_error, cmd_vel.linear.x, cmd_vel.angular.z);
    return cmd_vel;
}

// Refatorada para calcular e retornar Twist
// geometry_msgs::Twist TangentBug::calculateFollowContourCommand()
// {
//     geometry_msgs::Twist cmd_vel;

//     if (current_scan_.ranges.empty())
//     {
//         ROS_WARN_THROTTLE(1.0, "Cannot follow contour, LIDAR data is empty. Stopping.");
//         return geometry_msgs::Twist();
//     }

//     const auto &robot_position = current_pose_.position;
//     geometry_msgs::Point current_goal_position;
//     current_goal_position.x = goal_x_;
//     current_goal_position.y = goal_y_;
//     current_goal_position.z = 0.0;

//     double current_distance_to_goal = calculateDistance(robot_position.x, robot_position.y, current_goal_position.x, current_goal_position.y);

//     if (current_distance_to_goal < best_distance_to_goal_)
//     {
//         best_distance_to_goal_ = current_distance_to_goal;
//         best_leave_point_ = robot_position;
//         ROS_DEBUG_THROTTLE(1.0, "Updated best_leave_point_: (%.2f, %.2f) with distance %.2f", best_leave_point_.x, best_leave_point_.y, best_distance_to_goal_);
//         std::cout << " Updated: Melhor distância ao objetivo: " << best_distance_to_goal_ << std::endl;
//     }

//     // EXTRAÇÃO DE SEGMENTOS COM TODOS OS PONTOS DENTRO DO CONE OU DO QUADRADO DE COLISÃO
//     const auto global_segments = extractObstacleSegments2D(current_scan_, gap_threshold_);

//     if (global_segments.empty())
//     {
//         ROS_WARN_THROTTLE(1.0, "No obstacle segments to follow. Rotating in place to find one.");
//         cmd_vel.linear.x = 0.0;
//         cmd_vel.angular.z = rotation_speed_; // Gira para tentar encontrar um obstáculo
//         // cmd_vel.angular.z = 0.0; // Gira para tentar encontrar um obstáculo
//         return cmd_vel;
//     }

//     // Seleciona o melhor segmento (o mais próximo do objetivo), olha apenas o ponto inicial e o final do segmento
//     std::vector<geometry_msgs::Point> candidate_segment = findBestSegmentTowardGoal(global_segments, current_goal_position);
//     double candidate_distance_to_goal = std::numeric_limits<double>::max();

//     if (!candidate_segment.empty())
//     {
//         // Encontra o ponto mais próximo do objetivo no segmento candidato
//         geometry_msgs::Point endpoint_candidate = findClosestEndpointToPoint(candidate_segment, current_goal_position);
//         candidate_distance_to_goal = calculateDistance(endpoint_candidate.x, endpoint_candidate.y, current_goal_position.x, current_goal_position.y);
//     }

//     bool segment_is_significantly_better =
//         (candidate_distance_to_goal < current_stable_best_segment_distance_ * (1.0 - segment_switch_threshold_percent_));
//     if (current_stable_best_segment_.empty() || segment_is_significantly_better || candidate_segment.empty())
//     {
//         if (!candidate_segment.empty())
//         {
//             current_stable_best_segment_ = candidate_segment;
//             current_stable_best_segment_distance_ = candidate_distance_to_goal;
//             ROS_INFO_THROTTLE(2.0, "Switched to new stable best segment (%.2f%% better). New best dist: %.2f",
//                               segment_switch_threshold_percent_ * 100.0, current_stable_best_segment_distance_);
//         }
//         else
//         {
//             ROS_WARN_THROTTLE(1.0, "Candidate segment is empty, holding previous best segment if available.");
//             if (current_stable_best_segment_.empty()) // TODO talvez verificar se devo ir direto para o destino
//             {
//                 cmd_vel.linear.x = 0.0;
//                 cmd_vel.angular.z = rotation_speed_;
//                 return cmd_vel;
//             }
//         }
//     }

//     if (current_stable_best_segment_.empty())
//     {
//         ROS_WARN_THROTTLE(1.0, "No stable segment to follow after hysteresis. Rotating.");
//         cmd_vel.linear.x = 0.0;
//         cmd_vel.angular.z = rotation_speed_;
//         return cmd_vel;
//     }

//     // Simplifica o segmento para reduzir ruído e redundância, deve conter apenas 2 pontos(por conta da etapa de extractObstacleSegments2D)
//     // Parâmetros: min_distance (0.5), angle_tolerance_deg (5.0)
//     std::vector<geometry_msgs::Point> path_to_follow = simplifySegment(current_stable_best_segment_, 0.5, 5.0);

//     if (path_to_follow.empty())
//     {
//         ROS_WARN_THROTTLE(1.0, "Simplified stable segment is empty. Cannot follow contour. Rotating.");
//         cmd_vel.linear.x = 0.0;
//         cmd_vel.angular.z = rotation_speed_;
//         return cmd_vel;
//     }

//     for (size_t i = 0; i < path_to_follow.size(); ++i)
//     {
//         std::cout << "Segmento ponto " << i << ": (" << path_to_follow[i].x << ", " << path_to_follow[i].y << ")" << std::endl;
//         spawnSphereAt(path_to_follow[i].x, path_to_follow[i].y, 0.1, "#FF0000", 10);
//     }

//     // Aplica offset ao segmento (afastando-o do obstáculo) para garantir segurança
//     auto offset_path = offsetSegmentTowardRobot(path_to_follow, robot_position, safety_margin_ * 100.0);

//     if (offset_path.empty())
//     {
//         ROS_WARN_THROTTLE(1.0, "Offset path for stable segment is empty. Cannot follow contour. Rotating.");
//         cmd_vel.linear.x = 0.0;
//         cmd_vel.angular.z = rotation_speed_;
//         return cmd_vel;
//     }

//     std::cout << "Segmento offsetado: " << offset_path.size() << " pontos." << std::endl;

//     for (size_t i = 0; i < offset_path.size(); ++i)
//     {
//         std::cout << "Ponto " << i << ": (" << offset_path[i].x << ", " << offset_path[i].y << ")" << std::endl;
//         spawnSphereAt(offset_path[i].x, offset_path[i].y, 0.1, "#0000FF", 10);
//     }

//     // O ponto que o robô deve seguir é o mais próximo do robô no caminho offset.
//     const auto closest_point_on_offset = findClosestEndpointToPoint(offset_path, robot_position);
//     ROS_DEBUG("Closest point on offset path: (%.2f, %.2f)", closest_point_on_offset.x, closest_point_on_offset.y);
//     std::cout << "Ponto mais próximo no caminho offset: (" << closest_point_on_offset.x << ", " << closest_point_on_offset.y << ")" << std::endl;

//     // Visualizar o ponto para onde o robô está se direcionando
//     spawnSphereAt(closest_point_on_offset.x, closest_point_on_offset.y, 0.2, "#00FF00", 5);

//     // Agora, calcule o comando de velocidade para ir até `closest_point_on_offset`
//     // Use a função calculateMoveToPointCommand para isso.
//     cmd_vel = calculateMoveToPointCommand(closest_point_on_offset.x, closest_point_on_offset.y);

//     // No modo de contorno, você geralmente quer uma velocidade linear constante
//     // e ajustar a angular.
//     cmd_vel.linear.x = contour_speed_; // Mantém a velocidade linear desejada para o contorno

//     // Se o calculateMoveToPointCommand já calcula linear.x, você pode sobrescrevê-lo aqui
//     // para garantir que a velocidade linear seja sempre a contour_speed_
//     // ou integrar melhor a lógica de "ir para o ponto" com a "velocidade de contorno".

//     // Exemplo: se calculateMoveToPointCommand retorna uma velocidade linear muito baixa porque está quase alinhado,
//     // ainda queremos mover na velocidade de contorno.
//     if (std::abs(cmd_vel.angular.z) < 0.2)
//     { // Se estiver bem alinhado
//         cmd_vel.linear.x = contour_speed_;
//     }
//     else
//     {
//         cmd_vel.linear.x = contour_speed_ * 0.5; // Reduz a velocidade linear enquanto gira muito
//     }

//     ROS_DEBUG("FollowContour: linear=%.2f, angular=%.2f", cmd_vel.linear.x, cmd_vel.angular.z);
//     return cmd_vel;
// }

/**
 * Calcula o comando de velocidade para mover o robô em direção a um ponto específico.
 */
// geometry_msgs::Twist TangentBug::calculateMoveToPointCommand(double target_x, double target_y)
// {
//     geometry_msgs::Twist cmd;
//     geometry_msgs::Point p = current_pose_.position;

//     // Verificação de validade da pose atual
//     if (std::isnan(p.x) || std::isnan(p.y))
//     {
//         ROS_WARN_THROTTLE(1.0, "Current robot pose is invalid (NaN) in calculateMoveToPointCommand.");
//         return geometry_msgs::Twist(); // Retorna comando zero
//     }

//     double dx = target_x - p.x;
//     double dy = target_y - p.y;
//     double distance = std::hypot(dx, dy);
//     double angle_to_target = atan2(dy, dx);
//     double yaw = tf::getYaw(current_pose_.orientation);
//     double angle_error = normalizeAngle(angle_to_target - yaw);

//     // Se o robô já está muito perto do alvo, retorne velocidade zero
//     if (distance < goal_tolerance_) // Usar 'goal_tolerance_' ou uma nova tolerância para `moveToPoint`
//     {
//         ROS_DEBUG("Target point reached: distance=%.2f, angle_error=%.2f", distance, angle_error);
//         return geometry_msgs::Twist(); // Retorna comando zero
//     }

//     // Controle angular proporcional
//     cmd.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, angle_error * 1.5));

//     // Move forward if facing roughly the right direction
//     if (std::abs(angle_error) < 0.2) // Tolerância angular para avançar (0.2 rad ~ 11.5 graus)
//     {
//         cmd.linear.x = std::min(distance * 0.7, max_linear_speed_); // Velocidade linear proporcional à distância
//     }
//     else
//     {
//         cmd.linear.x = 0.0; // Apenas gira se o erro angular for grande
//     }

//     ROS_DEBUG("calculateMoveToPointCommand: Target=(%.2f, %.2f), Dist=%.2f, AngleError=%.2f, Lin=%.2f, Ang=%.2f",
//               target_x, target_y, distance, angle_error, cmd.linear.x, cmd.angular.z);

//     return cmd; // Retorna o comando de velocidade
// }

geometry_msgs::Twist TangentBug::calculateMoveToPointCommand(double target_x, double target_y)
{
    geometry_msgs::Twist cmd;

    const auto &pos = current_pose_.position;

    // Validação básica da pose atual (posição + orientação)
    if (std::isnan(pos.x) || std::isnan(pos.y))
    {
        ROS_WARN_THROTTLE(1.0, "Pose inválida (NaN) em calculateMoveToPointCommand.");
        return geometry_msgs::Twist(); // zero
    }
    // (Opcional) cheque quaternion ~ unitário
    // const auto& q = current_pose_.orientation;
    // const double qn2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    // if (std::fabs(qn2 - 1.0) > 1e-2) { ... }

    std::cout << "Objetivo atual: (" << target_x << ", " << target_y << ")" << std::endl;
    std::cout << "Pose atual: (" << pos.x << ", " << pos.y << ")" << std::endl;

    const double dx = target_x - pos.x;
    const double dy = target_y - pos.y;
    const double distance = std::hypot(dx, dy);

    const double angle_to_target = std::atan2(dy, dx);
    const double yaw = tf2::getYaw(current_pose_.orientation);

    // Normalização robusta sem depender de normalizeAngle
    const double angle_error = std::atan2(std::sin(angle_to_target - yaw),
                                          std::cos(angle_to_target - yaw));

    // Chegou?
    if (distance < goal_tolerance_)
    {
        ROS_DEBUG("Target reached: dist=%.3f, ang_err=%.3f", distance, angle_error);
        return geometry_msgs::Twist(); // zero
    }

    // Ganhos (torne parâmetros ROS se possível)
    const double k_ang = 1.5;
    const double k_lin = 0.7;
    const double align_threshold = 0.2; // rad (~11.5°)

    // Saturação angular (C++14-friendly)
    double w = k_ang * angle_error;
    if (w > max_angular_speed_)
        w = max_angular_speed_;
    if (w < -max_angular_speed_)
        w = -max_angular_speed_;
    cmd.angular.z = w;

    // Só avança se estiver razoavelmente alinhado
    if (std::fabs(angle_error) < align_threshold)
    {
        double v = k_lin * distance;
        if (v > max_linear_speed_)
            v = max_linear_speed_;
        cmd.linear.x = v;
    }
    else
    {
        cmd.linear.x = 0.0;
    }

    ROS_DEBUG("moveToPointCmd: tgt=(%.2f,%.2f) dist=%.2f ang_err=%.2f v=%.2f w=%.2f",
              target_x, target_y, distance, angle_error, cmd.linear.x, cmd.angular.z);

    return cmd;
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
    const auto &robot_position = current_pose_.position;
    geometry_msgs::Point goal_position;
    goal_position.x = 5;
    goal_position.y = 5;
    goal_position.z = 0.0;

    const auto segments = extractObstacleSegments2D(current_scan_, 0.8); // Atualiza os segmentos de obstáculos
    std::cout << "obstacle_segments.size(): " << segments.size() << std::endl;

    if (segments.empty())
    {
        ROS_WARN("No obstacle segments to follow.");
        return;
    }

    // Track best distance to goal during boundary following
    double current_distance_to_goal = calculateDistance(robot_position.x, robot_position.y, goal_position.x, goal_position.y);

    // Select the best segment (closest to goal by endpoint)
    auto best_segment = findBestSegmentTowardGoal(segments, goal_position);
    std::cout << "best_segment.size(): " << best_segment.size() << std::endl;

    // Simplify the segment to reduce noise
    best_segment = simplifySegment(best_segment, 0.5, 5.0);
    std::cout << "best_segment.size(): " << best_segment.size() << std::endl;

    if (best_segment.empty())
    {
        ROS_WARN("No valid segment for following.");
        return;
    }

    // Offset segment toward robot using perspective
    auto offset_path = offsetSegmentTowardRobotPerspective(best_segment, robot_position, 30.0);
    if (offset_path.empty())
    {
        ROS_WARN("Offset path is empty.");
        return;
    }

    // Spawn spheres at offset points for visualization
    for (const auto &pt : offset_path)
    {
        spawnSphereAt(pt.x, pt.y, 0.1, "#FF0000", 10); // Spawn sphere at offset point
    }

    const auto closest_point = findClosestEndpointToPoint(offset_path, robot_position);
    std::cout << "closest_point: " << closest_point.x << ", " << closest_point.y << std::endl;

    spawnSphereAt(closest_point.x, closest_point.y, 0.6, "#00FF00", 10); // Spawn sphere at offset point
    std::cout << "closest_point: " << closest_point.x << ", " << closest_point.y << std::endl;
    moveToPoint(closest_point.x, closest_point.y);
    std::cout << "Stopping robot..." << std::endl;
    stopRobot();
    std::cout << "Robot stopped." << std::endl;
    // deleteAllSpheres();
}

/**
 * Para parar o robô, publicamos uma mensagem de velocidade linear e angular zeradas.
 * Isso é útil para garantir que o robô pare imediatamente.
 */
void TangentBug::stopRobot()
{
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    cmd_vel_pub_.publish(stop_msg);
}

void TangentBug::rotateInPlace(double angular_speed, double duration_seconds)
{
    std::cout << "rotateInPlace" << std::endl;
    geometry_msgs::Twist rotate_msg;
    rotate_msg.linear.x = 0.0;
    rotate_msg.angular.z = angular_speed;

    int ticks = duration_seconds * 50; // duration in seconds * rate
    for (int i = 0; i < ticks; ++i)
    {
        cmd_vel_pub_.publish(rotate_msg);
    }
    std::cout << "rotateInPlace: stopRobot" << std::endl;
    stopRobot();
}

/**
 * Calcula a distância entre dois pontos (x1, y1) e (x2, y2).
 * Utiliza a fórmula da distância euclidiana.
 */
double TangentBug::calculateDistance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

/**
 * Calcula o ângulo para o objetivo com base na posição atual do robô.
 */
double TangentBug::calculateHeadingToGoal()
{
    // Verificar se já estamos no objetivo ou muito perto dele
    double distance_to_goal = calculateDistance(current_pose_.position.x, current_pose_.position.y, goal_x_, goal_y_);
    if (distance_to_goal < goal_tolerance_)
    {
        ROS_INFO("Já estamos no objetivo!");
        return 0.0;
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

// void TangentBug::moveToPoint(double x, double y)
// {
//     ros::Rate rate(20.0); // controla frequência de publicação (20 Hz)

//     while (ros::ok())
//     {
//         geometry_msgs::Twist cmd;

//         const auto &pos = current_pose_.position;
//         const double dx = x - pos.x;
//         const double dy = y - pos.y;

//         const double distance = std::hypot(dx, dy);
//         const double angle_to_target = std::atan2(dy, dx);
//         const double yaw = tf2::getYaw(current_pose_.orientation);

//         // erro angular normalizado [-pi, pi]
//         const double angle_error = std::atan2(std::sin(angle_to_target - yaw),
//                                               std::cos(angle_to_target - yaw));

//         // chegou?
//         if (distance < goal_tolerance_)
//         {
//             stopRobot();  // função auxiliar para publicar Twist zero
//             ROS_INFO_STREAM("Alvo (" << x << "," << y << ") alcançado.");
//             break;
//         }

//         // ganhos
//         const double k_ang = 1.5;
//         const double k_lin = 0.7;

//         // saturação angular
//         double ang_cmd = k_ang * angle_error;
//         if (ang_cmd > max_angular_speed_)
//             ang_cmd = max_angular_speed_;
//         if (ang_cmd < -max_angular_speed_)
//             ang_cmd = -max_angular_speed_;
//         cmd.angular.z = ang_cmd;

//         // só avança se alinhado
//         if (std::fabs(angle_error) < 0.2)
//         {
//             cmd.linear.x = std::min(k_lin * distance, max_linear_speed_);
//         }
//         else
//         {
//             cmd.linear.x = 0.0;
//         }

//         cmd_vel_pub_.publish(cmd);

//         ROS_DEBUG("moveToPoint: dist=%.2f, ang_err=%.2f, cmd(v=%.2f,w=%.2f)",
//                   distance, angle_error, cmd.linear.x, cmd.angular.z);

//         ros::spinOnce();
//         rate.sleep();
//     }
// }

void TangentBug::moveToPoint(double x, double y)
{
    geometry_msgs::Twist cmd;

    const auto &pos = current_pose_.position;
    const double dx = x - pos.x;
    const double dy = y - pos.y;

    const double distance = std::hypot(dx, dy);
    const double angle_to_target = std::atan2(dy, dx);
    const double yaw = tf2::getYaw(current_pose_.orientation);

    // erro angular normalizado para [-pi, pi]
    const double raw_ang_err = angle_to_target - yaw;
    const double angle_error =
        std::atan2(std::sin(raw_ang_err), std::cos(raw_ang_err));

    // chegou?
    std::cout << "Distância ao alvo: " << distance << ", Tolerância: " << goal_tolerance_ << std::endl;
    if (distance < goal_tolerance_)
    {
        stopRobot();
        return;
    }

    // ====== PARÂMETROS (torne-os parametrizáveis via rosparam/dyn_reconf) ======
    const double k_ang = 1.5;                // ganho P angular
    const double max_w = max_angular_speed_; // limite de |w|

    const double vmax = max_linear_speed_; // limite de v
    const double v_min_ratio = 0.08;       // fração do vmax para nunca zerar v
    const double v_min = v_min_ratio * vmax;

    const double slow_dist = 1.0;                          // raio a partir do qual v chega perto de vmax
    const double k_dist = 2.0 / std::max(1e-6, slow_dist); // inclinação da tanh
    const double k_ang_gauss = 1.5;                        // quanto o erro angular "pesa" para reduzir v (gaussiana)
    const double w_turn_knee = 0.8;                        // joelho para reduzir v quando |w| alto

    // ====== CONTROLE ANGULAR (sempre comanda w) ======
    double w_cmd = k_ang * angle_error;
    if (w_cmd > max_w)
        w_cmd = max_w;
    if (w_cmd < -max_w)
        w_cmd = -max_w;

    // ====== CONTROLE LINEAR (sempre comanda v > 0 fora da tolerância) ======
    // Componente por distância (0→longe, 1→perto do vmax):
    // usa tanh para suavizar aproximação
    const double dist_scale = std::tanh(k_dist * distance); // em (0, ~1)

    // Atenuação suave por erro angular (nunca zera):
    // gaussiana em torno de 0: exp(-k*err^2) ∈ (0,1]
    const double ang_scale = std::exp(-k_ang_gauss * angle_error * angle_error);

    // Redução quando curva é muito fechada (|w| alto)
    const double turn_scale = 1.0 / (1.0 + std::abs(w_cmd) / std::max(1e-6, w_turn_knee));

    // Velocidade alvo combinando os fatores; garante piso v_min
    double v_cmd = vmax * dist_scale * ang_scale * turn_scale;
    if (v_cmd < v_min)
        v_cmd = v_min;
    if (v_cmd > vmax)
        v_cmd = vmax;

    cmd.linear.x = v_cmd;
    cmd.angular.z = w_cmd;

    std::cout << "moveToPoint(simul): target=(" << x << "," << y << "), pos=("
              << pos.x << "," << pos.y << "), dist=" << distance
              << ", ang_err=" << angle_error
              << ", cmd(v=" << cmd.linear.x << ", w=" << cmd.angular.z << ")"
              << std::endl;

    cmd_vel_pub_.publish(cmd);

    ROS_DEBUG("moveToPoint(simul): dist=%.2f, ang_err=%.2f, cmd(v=%.2f,w=%.2f)",
              distance, angle_error, cmd.linear.x, cmd.angular.z);
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
static inline std::vector<geometry_msgs::Point>
offsetPolylineTowardSensor(const std::vector<geometry_msgs::Point> &seg,
                           const geometry_msgs::Point &sensor_global,
                           double d,
                           bool closed = false)
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

// -----------------------------------------------------------------------------
// Helper: infla um segmento deslocando cada ponto ao longo da normal do contorno
// para o lado "de fora" (mais distante do sensor).
// static inline std::vector<geometry_msgs::Point>
// inflateSegmentObstaclePerspective(const std::vector<geometry_msgs::Point>& seg,
//                                   const geometry_msgs::Point& sensor_global,
//                                   double d)
// {
//     std::vector<geometry_msgs::Point> out;
//     out.reserve(seg.size());
//     auto rot90 = [](double x, double y) { return std::pair<double,double>{-y, x}; };

//     const size_t N = seg.size();
//     if (N == 0) return out;

//     // Caso N == 1: usa vetor radial em relação ao sensor
//     if (N == 1) {
//         const auto& p = seg[0];
//         double vx = p.x - sensor_global.x;
//         double vy = p.y - sensor_global.y;
//         double nrm = std::hypot(vx, vy);
//         if (nrm < 1e-8) nrm = 1.0;
//         vx /= nrm; vy /= nrm;

//         // >>> escolha a direção conforme sua intenção:
//         // Para INFLAR PARA LONGE do sensor (engordar obstáculo):
//         //   normal = +radial (afastando do sensor)
//         // Para INFLAR PARA O LADO DO SENSOR:
//         //   normal = -radial
//         // double nx = vx, ny = vy;        // longe do sensor
//         double nx = -vx, ny = -vy;    // para o lado do sensor

//         geometry_msgs::Point q;
//         q.x = p.x + d * nx;
//         q.y = p.y + d * ny;
//         q.z = 0.0;
//         out.push_back(q);
//         return out;
//     }

//     for (size_t i = 0; i < N; ++i) {
//         const auto& p = seg[i];

//         // Tangente (diferenças centrais / unilaterais nas pontas)
//         double tx, ty;
//         if (i == 0) {
//             tx = seg[1].x - p.x;  ty = seg[1].y - p.y;
//         } else if (i + 1 == N) {
//             tx = p.x - seg[i-1].x; ty = p.y - seg[i-1].y;
//         } else {
//             tx = seg[i+1].x - seg[i-1].x; ty = seg[i+1].y - seg[i-1].y;
//         }

//         double nrm = std::hypot(tx, ty);
//         if (nrm < 1e-8) { tx = 1.0; ty = 0.0; nrm = 1.0; }
//         tx /= nrm; ty /= nrm;

//         auto n = rot90(tx, ty);
//         double nx = n.first, ny = n.second;

//         // Vetor do sensor ao ponto
//         const double vx = p.x - sensor_global.x;
//         const double vy = p.y - sensor_global.y;

//         // >>> Escolha a direção da normal:
//         // (A) Inflar PARA O LADO DO SENSOR (contrair em direção ao robô)
//         if (nx*vx + ny*vy > 0.0) { nx = -nx; ny = -ny; }

//         // (B) Inflar PARA LONGE DO SENSOR (engordar obstáculo)  <<< comum em navegação
//         // if (nx*vx + ny*vy < 0.0) { nx = -nx; ny = -ny; }

//         // Limite para não ultrapassar o sensor (sanidade)
//         const double max_to_sensor = std::max(d, std::hypot(vx, vy) - 1e-3);

//         // Miter-limit simples em quinas
//         double miter = max_to_sensor;
//         if (i > 0) {
//             const double ex = p.x - seg[i-1].x;
//             const double ey = p.y - seg[i-1].y;
//             miter = std::max(miter, 0.5 * std::hypot(ex, ey));
//         }
//         if (i + 1 < N) {
//             const double ex = seg[i+1].x - p.x;
//             const double ey = seg[i+1].y - p.y;
//             miter = std::max(miter, 0.5 * std::hypot(ex, ey));
//         }

//         const double d_use = std::min(d, miter);

//         geometry_msgs::Point q;
//         q.x = p.x + d_use * nx;
//         q.y = p.y + d_use * ny;
//         q.z = 0.0;
//         out.push_back(q);
//     }
//     return out;
// }

/**
 * Extrai segmentos de obstáculos 2D a partir de dados de varredura LIDAR.
 * Cada segmento é uma sequência de pontos que representam um obstáculo detectado.
 *
 * @param scan Dados da varredura LIDAR.
 * @param threshold Distância mínima entre pontos para considerar como parte do mesmo segmento.
 * @return Vetor de segmentos, onde cada segmento é um vetor de pontos (geometry_msgs::Point).
 */
std::vector<std::vector<geometry_msgs::Point>> TangentBug::extractObstacleSegments2D(
    const sensor_msgs::LaserScan &scan, double threshold)
{
    std::vector<std::vector<geometry_msgs::Point>> segments;
    std::vector<geometry_msgs::Point> current_segment;
    std::vector<geometry_msgs::Point> current_segment_raw;
    current_segment_raw.reserve(64);

    if (scan.ranges.empty())
    {
        ROS_ERROR("extractObstacleSegments2D: Empty scan.");
        return segments;
    }

    std::string laser_frame_id = scan.header.frame_id; // "sick_laser_link"
    std::string global_frame_id = "odom";              // Ou "map"

    geometry_msgs::TransformStamped transform_laser_to_global;

    try
    {
        // Primeiro: verifique se o TF está disponível até um timeout (equivale ao waitForTransform do tf1)
        const ros::Duration timeout(0.5); // meio segundo
        const bool ok = tf_buffer_.canTransform(
            /*target*/ global_frame_id,
            /*source*/ laser_frame_id,
            /*time*/ scan.header.stamp,
            /*timeout*/ timeout);

        if (!ok)
        {
            ROS_ERROR("TF2 unavailable from %s to %s at time %.3f (timeout %.2fs).",
                      laser_frame_id.c_str(), global_frame_id.c_str(),
                      scan.header.stamp.toSec(), timeout.toSec());
            return segments;
        }

        // Depois: obtenha a transformação (opcional: um timeout pequeno aqui também)
        transform_laser_to_global = tf_buffer_.lookupTransform(
            /*target*/ global_frame_id,
            /*source*/ laser_frame_id,
            /*time*/ scan.header.stamp,
            /*timeout*/ ros::Duration(0.05));
    }
    catch (const tf2::TransformException &ex)
    {
        ROS_ERROR("TF2 error getting transform from %s to %s at %.3f: %s",
                  laser_frame_id.c_str(), global_frame_id.c_str(),
                  scan.header.stamp.toSec(), ex.what());
        throw std::runtime_error("TF2 error");
        // return segments; // lide com o erro conforme sua lógica
    }

    geometry_msgs::PointStamped origin_laser, origin_global;
    origin_laser.header = scan.header;
    origin_laser.point.x = origin_laser.point.y = origin_laser.point.z = 0.0;
    try
    {
        tf2::doTransform(origin_laser, origin_global, transform_laser_to_global);
    }
    catch (const tf2::TransformException &ex)
    {
        ROS_ERROR("TF2 doTransform origin failed: %s", ex.what());
        return segments;
    }

    // 2) Varre o scan, projeta cada ponto para o frame global e segmenta por distância
    //    (otimizado com avanço incremental de cos/sin para evitar trig no loop)
    double angle = scan.angle_min;
    double c_th = std::cos(angle);
    double s_th = std::sin(angle);
    const double c_inc = std::cos(scan.angle_increment);
    const double s_inc = std::sin(scan.angle_increment);
    const double rmin = scan.range_min;
    const double rmax = scan.range_max;
    double prev_r = std::numeric_limits<double>::quiet_NaN();
    bool have_prev = false;

    // Util para fechar segmento atual
    auto close_and_inflate = [&]()
    {
        if (!current_segment_raw.empty())
        {
            const double inflate_m = safety_margin_; // ajuste conforme seus membros/params
            auto inflated = offsetPolylineTowardSensor(current_segment_raw,
                                                       origin_global.point,
                                                       inflate_m);
            if (!inflated.empty())
                segments.push_back(std::move(inflated));
            current_segment_raw.clear();
            current_segment_raw.reserve(64);
        }
    };

    bool have_last = false;
    geometry_msgs::Point last_raw;

    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        const float r = scan.ranges[i];

        // Avanço trig para próximo passo já no final; aqui usamos c_th/s_th atuais
        const auto advance_angle = [&]()
        {
            const double c_next = c_inc * c_th - s_inc * s_th;
            const double s_next = s_inc * c_th + c_inc * s_th;
            c_th = c_next;
            s_th = s_next;
        };

        if (!std::isfinite(r) || r < rmin || r > rmax)
        {
            close_and_inflate();
            have_last = false;
            advance_angle();
            continue;
        }

        // Ponto CRU em laser
        geometry_msgs::PointStamped p_laser;
        p_laser.header = scan.header;
        p_laser.point.x = static_cast<double>(r) * c_th;
        p_laser.point.y = static_cast<double>(r) * s_th;
        p_laser.point.z = 0.0;

        // Transforma para GLOBAL (ponto CRU)
        geometry_msgs::PointStamped p_global_st;
        try
        {
            tf2::doTransform(p_laser, p_global_st, transform_laser_to_global);
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(1.0, "TF2 doTransform beam failed: %s", ex.what());
            close_and_inflate();
            have_last = false;
            advance_angle();
            continue;
        }

        const geometry_msgs::Point &pt = p_global_st.point;

        // Decide quebra por distância entre pontos CRUS em global
        if (have_last)
        {
            const double dx = pt.x - last_raw.x;
            const double dy = pt.y - last_raw.y;
            const double dist = std::hypot(dx, dy);
            if (!std::isfinite(dist) || dist >= threshold)
            {
                close_and_inflate();
                have_last = false; // o atual iniciará novo segmento abaixo
            }
        }

        // Acumula ponto CRU no segmento atual
        current_segment_raw.push_back(pt);
        last_raw = pt;
        have_last = true;

        // Próximo feixe
        advance_angle();
    }
    close_and_inflate();

    return segments;
}

// std::vector<std::vector<geometry_msgs::Point>> TangentBug::extractObstacleSegments2D(
//     const sensor_msgs::LaserScan &scan, double threshold)
// {
//     std::vector<std::vector<geometry_msgs::Point>> segments;
//     std::vector<geometry_msgs::Point> current_segment;

//     // Parâmetros/checagens básicas
//     if (scan.ranges.empty())
//     {
//         ROS_ERROR("extractObstacleSegments2D: Empty scan.");
//         return segments;
//     }
//     if (!(scan.angle_increment > 0.0))
//     {
//         ROS_ERROR("extractObstacleSegments2D: Invalid angle_increment (%.6f).", scan.angle_increment);
//         return segments;
//     }
//     if (!(scan.range_max > scan.range_min))
//     {
//         ROS_ERROR("extractObstacleSegments2D: Invalid range limits [%.3f, %.3f].", scan.range_min, scan.range_max);
//         return segments;
//     }
//     if (!(threshold > 0.0))
//     {
//         ROS_WARN("extractObstacleSegments2D: Non-positive threshold (%.3f). Clamping to 0.05 m.", threshold);
//         threshold = 0.05;
//     }

//     const std::string laser_frame_id = scan.header.frame_id; // ex.: "sick_laser_link"
//     const std::string global_frame_id = "odom";              // ajuste conforme sua árvore TF
//     const double dtheta = scan.angle_increment;
//     const bool per_beam_time = (scan.time_increment > 0.0);
//     const ros::Time t0 = scan.header.stamp;

//     // Utilitário para fechar e guardar um segmento atual (com filtro de tamanho mínimo)
//     auto close_segment = [&]()
//     {
//         const size_t kMinPoints = 2; // evite ruído de 1 ponto
//         if (!current_segment.empty())
//         {
//             if (current_segment.size() >= kMinPoints)
//             {
//                 segments.push_back(current_segment);
//             }
//             current_segment.clear();
//         }
//     };

//     // Tente obter um transform único caso não haja time_increment (melhor que nada)
//     geometry_msgs::TransformStamped tf_at_scan_time;
//     bool have_single_tf = false;

//     if (!per_beam_time)
//     {
//         try
//         {
//             // Melhor esforço no timestamp do scan; adiciona um pequeno timeout
//             tf_at_scan_time = tf_buffer_.lookupTransform(
//                 /*target_frame*/ global_frame_id, // ex: "base_link" ou "odom"
//                 /*source_frame*/ laser_frame_id,  // ex: "sick_laser_link"
//                 /*time*/ t0,                      // timestamp do LaserScan
//                 /*timeout*/ ros::Duration(0.05));

//             have_single_tf = true;
//         }
//         catch (const tf2::TransformException &ex)
//         {
//             ROS_WARN("TF2 at scan time unavailable (%s->%s @ %.3f): %s",
//                      laser_frame_id.c_str(), global_frame_id.c_str(), t0.toSec(), ex.what());
//         }
//     }

//     // Estado para a métrica de "salto" entre feixes
//     double prev_range = std::numeric_limits<double>::quiet_NaN();
//     bool have_prev = false;

//     // Opcional: pequeno cache do último TF por-feixe (reduz lookups se timestamps repetem)
//     geometry_msgs::TransformStamped last_tf_beam;
//     ros::Time last_tf_time = ros::Time(0);
//     const double tf_cache_dt = 0.005; // 5 ms

//     for (size_t i = 0; i < scan.ranges.size(); ++i)
//     {
//         const double r = static_cast<double>(scan.ranges[i]);
//         // invalidações: NaN/Inf e fora de faixa
//         const bool valid = std::isfinite(r) && (r >= scan.range_min) && (r <= scan.range_max);
//         if (!valid)
//         {
//             close_segment();
//             have_prev = false;
//             continue;
//         }

//         // Tempo do feixe (deskew quando possível)
//         const ros::Time ti = per_beam_time ? (t0 + ros::Duration(i * scan.time_increment)) : t0;

//         // Transform do laser -> global
//         geometry_msgs::TransformStamped T_laser_to_global;
//         try
//         {
//             if (per_beam_time)
//             {
//                 // Reusa TF se a diferença de tempo for muito pequena (economiza chamadas)
//                 if (!last_tf_time.isZero() && std::abs((ti - last_tf_time).toSec()) < tf_cache_dt)
//                 {
//                     T_laser_to_global = last_tf_beam;
//                 }
//                 else
//                 {
//                     T_laser_to_global = tf_buffer_.lookupTransform(
//                         global_frame_id, laser_frame_id, ti, ros::Duration(0.02));
//                     last_tf_beam = T_laser_to_global;
//                     last_tf_time = ti;
//                 }
//             }
//             else if (have_single_tf)
//             {
//                 T_laser_to_global = tf_at_scan_time;
//             }
//             else
//             {
//                 // Fallback: último transform disponível (pior, porém funcional)
//                 T_laser_to_global = tf_buffer_.lookupTransform(
//                     global_frame_id, laser_frame_id, ros::Time(0), ros::Duration(0.02));
//             }
//         }
//         catch (const tf2::TransformException &ex)
//         {
//             ROS_WARN_THROTTLE(1.0, "TF2 lookup failed (%s->%s @ %.3f): %s",
//                               laser_frame_id.c_str(), global_frame_id.c_str(), ti.toSec(), ex.what());
//             close_segment();
//             have_prev = false;
//             continue;
//         }

//         // Ângulo do feixe
//         const double angle = scan.angle_min + static_cast<double>(i) * dtheta;

//         // Verificação de "quebra" entre feixes adjacentes (Lei dos Cossenos)
//         if (have_prev)
//         {
//             const double d_edge = std::sqrt(
//                 prev_range * prev_range + r * r - 2.0 * prev_range * r * std::cos(dtheta));
//             if (!std::isfinite(d_edge) || d_edge > threshold)
//             {
//                 close_segment();
//             }
//         }

//         // Ponto no frame do laser
//         geometry_msgs::PointStamped pt_laser, pt_global;
//         pt_laser.header.frame_id = laser_frame_id;
//         pt_laser.header.stamp = ti;
//         pt_laser.point.x = r * std::cos(angle);
//         pt_laser.point.y = r * std::sin(angle);
//         pt_laser.point.z = 0.0;

//         try
//         {
//             tf2::doTransform(pt_laser, pt_global, T_laser_to_global);
//         }
//         catch (const tf2::TransformException &ex)
//         {
//             ROS_WARN_THROTTLE(1.0, "TF2 doTransform failed: %s", ex.what());
//             close_segment();
//             have_prev = false;
//             continue;
//         }

//         current_segment.push_back(pt_global.point);

//         // Atualiza estado
//         prev_range = r;
//         have_prev = true;
//     }

//     // Último segmento
//     close_segment();

//     return segments;
// }
// std::vector<std::vector<geometry_msgs::Point>>
// TangentBug::extractObstacleSegments2D(const sensor_msgs::LaserScan& scan, double threshold)
// {
//     std::vector<std::vector<geometry_msgs::Point>> segments;
//     std::vector<geometry_msgs::Point> current_segment;

//     // Checagens básicas
//     if (scan.ranges.empty()) {
//         ROS_ERROR("extractObstacleSegments2D: Empty scan.");
//         return segments;
//     }
//     if (!(scan.angle_increment > 0.0)) {
//         ROS_ERROR("extractObstacleSegments2D: Invalid angle_increment (%.6f).", scan.angle_increment);
//         return segments;
//     }
//     if (!(scan.range_max > scan.range_min)) {
//         ROS_ERROR("extractObstacleSegments2D: Invalid range limits [%.3f, %.3f].", scan.range_min, scan.range_max);
//         return segments;
//     }
//     if (!(threshold > 0.0)) {
//         ROS_WARN("extractObstacleSegments2D: Non-positive threshold (%.3f). Clamping to 0.05 m.", threshold);
//         threshold = 0.05;
//     }

//     // Parâmetros internos simples (ajuste se necessário)
//     const size_t  kMinPoints          = 3;     // descarta segmentos muito curtos
//     const size_t  kMaxInvalidGap      = 1;     // até 1 leitura inválida não quebra
//     const bool    kIgnoreAtMaxRange   = true;  // ignora pontos perto do range_max
//     const double  kRelJumpFrac        = 0.30;  // quebra tb por salto relativo (~30%)

//     const double dtheta = scan.angle_increment;
//     const double rmin   = scan.range_min;
//     const double rmax   = scan.range_max;
//     const double near_max_eps = 0.98 * rmax;   // “perto do máximo” (simuladores adoram isso)

//     auto close_segment = [&](size_t /*end_idx*/) {
//         if (!current_segment.empty()) {
//             if (current_segment.size() >= kMinPoints) {
//                 segments.push_back(current_segment);
//             }
//             current_segment.clear();
//         }
//     };

//     auto push_point = [&](double r, double ang) {
//         geometry_msgs::Point p;
//         p.x = r * std::cos(ang);
//         p.y = r * std::sin(ang);
//         p.z = 0.0;
//         current_segment.emplace_back(std::move(p));
//     };

//     double prev_r   = std::numeric_limits<double>::quiet_NaN();
//     bool   have_prev = false;
//     size_t invalid_streak = 0;

//     for (size_t i = 0; i < scan.ranges.size(); ++i) {
//         const double r = static_cast<double>(scan.ranges[i]);
//         const bool valid = std::isfinite(r) && (r >= rmin) && (r <= rmax);
//         const double ang = scan.angle_min + static_cast<double>(i) * dtheta;

//         if (!valid || (kIgnoreAtMaxRange && r >= near_max_eps)) {
//             // permite um “buraco” curto antes de quebrar
//             invalid_streak++;
//             if (invalid_streak > kMaxInvalidGap) {
//                 if (!current_segment.empty()) close_segment(i ? i - 1 : 0);
//                 have_prev = false;
//                 prev_r = std::numeric_limits<double>::quiet_NaN();
//             }
//             continue;
//         }

//         // reset do buraco
//         invalid_streak = 0;

//         // Quebra por salto entre feixes adjacentes
//         if (have_prev) {
//             const double d_edge = std::sqrt(prev_r*prev_r + r*r - 2.0*prev_r*r*std::cos(dtheta));
//             const double rel_jump = std::abs(r - prev_r) / std::max(1e-6, std::min(r, prev_r));
//             if (!std::isfinite(d_edge) || d_edge > threshold || rel_jump > kRelJumpFrac) {
//                 close_segment(i ? i - 1 : 0);
//             }
//         }

//         // Empilha ponto (no frame do laser)
//         push_point(r, ang);
//         prev_r = r;
//         have_prev = true;
//     }

//     // Fecha último segmento, se houver
//     if (!current_segment.empty()) close_segment(scan.ranges.size() - 1);

//     return segments;
// }

/**
 * @brief Finds the closest endpoint (first or last) of a given segment to a target point.
 *
 * @param segment The segment of points to evaluate.
 * @param target The target point to which the distance is calculated.
 * @return The endpoint (first or last point of the segment) that is closest to the target point.
 * If the segment is empty, a default-constructed geometry_msgs::Point is returned.
 */
geometry_msgs::Point TangentBug::findClosestEndpointToPoint(
    const std::vector<geometry_msgs::Point> &segment,
    const geometry_msgs::Point &target)
{
    // Se o segmento estiver vazio, não há pontos para comparar, retorna um ponto padrão.
    // É importante considerar como você quer lidar com esse caso.
    if (segment.empty())
    {
        return geometry_msgs::Point(); // Retorna um ponto com coordenadas (0,0,0) ou o padrão.
    }

    const auto &first = segment.front();
    const auto &last = segment.back();

    // Calcula a distância do primeiro ponto ao alvo
    double dist_first = std::hypot(target.x - first.x, target.y - first.y);

    // Calcula a distância do último ponto ao alvo
    double dist_last = std::hypot(target.x - last.x, target.y - last.y);

    // Compara as distâncias e retorna o ponto mais próximo
    if (dist_first <= dist_last) // Usamos <= para garantir que o 'first' seja escolhido em caso de empate
    {
        return first;
    }
    else
    {
        return last;
    }
}

std::vector<geometry_msgs::Point> TangentBug::findBestSegmentTowardGoal(
    const std::vector<std::vector<geometry_msgs::Point>> &segments,
    const geometry_msgs::Point &goal)
{
    std::vector<geometry_msgs::Point> best_segment;
    double min_goal_distance = std::numeric_limits<double>::max();

    for (const auto &segment : segments)
    {
        if (segment.empty())
            continue;

        const auto &start = segment.front();
        const auto &end = segment.back();

        double dist_start = std::hypot(goal.x - start.x, goal.y - start.y);
        double dist_end = std::hypot(goal.x - end.x, goal.y - end.y);

        double min_segment_dist = std::min(dist_start, dist_end);

        if (min_segment_dist < min_goal_distance)
        {
            min_goal_distance = min_segment_dist;
            best_segment = segment;
        }
    }
    return best_segment;
}

std::vector<geometry_msgs::Point> TangentBug::offsetSegmentTowardRobot(
    const std::vector<geometry_msgs::Point> &segment,
    const geometry_msgs::Point &robot,
    double offset_distance_cm)
{
    std::vector<geometry_msgs::Point> offset_segment;

    if (segment.size() < 1)
        return offset_segment;

    geometry_msgs::Point p1 = segment.front();
    geometry_msgs::Point p2 = segment.back();

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double length = std::hypot(dx, dy);

    double nx, ny;

    if (length < 1e-6)
    {
        // Use vector from point to robot if p1 and p2 are nearly the same
        double dir_x = robot.x - p1.x;
        double dir_y = robot.y - p1.y;
        double norm = std::hypot(dir_x, dir_y);
        if (norm < 1e-6)
            return segment; // robot and point are nearly coincident
        nx = dir_x / norm;
        ny = dir_y / norm;
    }
    else
    {
        // Normal to the segment direction
        nx = -dy / length;
        ny = dx / length;

        // Check direction toward robot
        double cx = (p1.x + p2.x) / 2.0;
        double cy = (p1.y + p2.y) / 2.0;
        double rx = robot.x - cx;
        double ry = robot.y - cy;

        if ((nx * rx + ny * ry) < 0)
        {
            nx = -nx;
            ny = -ny;
        }
    }

    double offset_m = offset_distance_cm / 100.0;

    for (const auto &pt : segment)
    {
        geometry_msgs::Point shifted;
        shifted.x = pt.x + offset_m * nx;
        shifted.y = pt.y + offset_m * ny;
        shifted.z = 0.0;
        offset_segment.push_back(shifted);
    }

    return offset_segment;
}

// std::vector<geometry_msgs::Point> TangentBug::offsetSegmentTowardRobotPerspective(
//     const std::vector<geometry_msgs::Point> &segment,
//     const geometry_msgs::Point &robot,
//     double offset_distance_cm)
// {
//     std::vector<geometry_msgs::Point> offset_segment;

//     if (segment.empty())
//         return offset_segment;

//     double offset_m = offset_distance_cm / 100.0;

//     for (const auto &pt : segment)
//     {
//         double dx = robot.x - pt.x;
//         double dy = robot.y - pt.y;
//         double norm = std::hypot(dx, dy);

//         geometry_msgs::Point shifted = pt;

//         if (norm > 1e-6)
//         {
//             // Reverse direction if offset is negative (move away from robot)
//             shifted.x += offset_m * (dx / norm);
//             shifted.y += offset_m * (dy / norm);
//         }

//         offset_segment.push_back(shifted);
//     }

//     return offset_segment;
// }

std::vector<geometry_msgs::Point> TangentBug::offsetSegmentTowardRobotPerspective(
    const std::vector<geometry_msgs::Point> &segment,
    const geometry_msgs::Point &robot,
    double offset_distance_cm) // Novo parâmetro para a distância de expansão
{
    std::vector<geometry_msgs::Point> offset_and_expanded_segment;

    if (segment.empty())
        return offset_and_expanded_segment;

    double offset_m = offset_distance_cm / 100.0;
    double expansion_m = offset_distance_cm / 100.0; // Converte a expansão para metros

    // --- Passo 1: Aplicar o offset original ---
    std::vector<geometry_msgs::Point> temp_offset_segment; // Segmento temporário após o offset

    for (const auto &pt : segment)
    {
        double dx = robot.x - pt.x;
        double dy = robot.y - pt.y;
        double norm = std::hypot(dx, dy);

        geometry_msgs::Point shifted = pt;

        if (norm > 1e-6) // Evita divisão por zero
        {
            // O sinal de offset_m determina se o ponto é movido para perto ou longe do robô
            shifted.x += offset_m * (dx / norm);
            shifted.y += offset_m * (dy / norm);
        }

        temp_offset_segment.push_back(shifted);
    }

    // Se após o offset o segmento ficou vazio (não deveria ocorrer se não estava vazio antes)
    if (temp_offset_segment.empty())
    {
        return offset_and_expanded_segment;
    }

    // --- Passo 2: Expandir o segmento deslocado ---

    // Se o segmento tem apenas um ponto após o offset, não podemos definir uma direção para expansão.
    // Retornamos o segmento com o offset aplicado.
    if (temp_offset_segment.size() == 1)
    {
        return temp_offset_segment;
    }

    geometry_msgs::Point first_offset_point = temp_offset_segment.front();
    geometry_msgs::Point last_offset_point = temp_offset_segment.back();

    // Calcular o vetor direcional do segmento deslocado
    double segment_dx = last_offset_point.x - first_offset_point.x;
    double segment_dy = last_offset_point.y - first_offset_point.y;
    double segment_norm = std::hypot(segment_dx, segment_dy);

    if (segment_norm < 1e-6)
    {                               // Segmento muito curto ou pontos sobrepostos após o offset
        return temp_offset_segment; // Não podemos definir uma direção clara para expansão
    }

    // Vetor unitário na direção do segmento
    double unit_dx = segment_dx / segment_norm;
    double unit_dy = segment_dy / segment_norm;

    // Calcular o novo ponto no início (projetando para trás)
    geometry_msgs::Point new_first_point;
    new_first_point.x = first_offset_point.x - (unit_dx * expansion_m);
    new_first_point.y = first_offset_point.y - (unit_dy * expansion_m);
    new_first_point.z = first_offset_point.z; // Mantém o Z

    // Calcular o novo ponto no final (projetando para frente)
    geometry_msgs::Point new_last_point;
    new_last_point.x = last_offset_point.x + (unit_dx * expansion_m);
    new_last_point.y = last_offset_point.y + (unit_dy * expansion_m);
    new_last_point.z = last_offset_point.z; // Mantém o Z

    // Adicionar o novo primeiro ponto
    offset_and_expanded_segment.push_back(new_first_point);

    // Adicionar todos os pontos do segmento deslocado (original)
    for (const auto &pt : temp_offset_segment)
    {
        offset_and_expanded_segment.push_back(pt);
    }

    // Adicionar o novo último ponto
    offset_and_expanded_segment.push_back(new_last_point);

    return offset_and_expanded_segment;
}

geometry_msgs::Twist TangentBug::calculateTwistToPointAndAlign(const geometry_msgs::Point &target_point)
{
    geometry_msgs::Twist cmd_vel; // Comando de velocidade a ser construído e retornado

    // 1. Verificação e Definição do Ponto Alvo Interno
    // Esta lógica verifica se o 'target_point' fornecido é diferente do que a função está rastreando.
    // Se for um novo ponto, ou se a função acabou de ser reiniciada (NaN), ela reinicia o processo.
    // Usamos 'std::hypot' para comparar a distância, o que é mais robusto para doubles.
    if (std::isnan(target_point_for_turn_then_move_.x) ||
        std::hypot(target_point_for_turn_then_move_.x - target_point.x,
                   target_point_for_turn_then_move_.y - target_point.y) > 0.001) // Tolerância de 1mm para mudança de ponto
    {
        target_point_for_turn_then_move_ = target_point;   // Armazena o novo ponto alvo
        current_turn_then_move_state_ = ALIGNING_TO_POINT; // Reinicia para o estado de alinhamento
        ROS_INFO("New target point for turn-then-move: (%.2f, %.2f). Starting ALIGNING_TO_POINT.",
                 target_point.x, target_point.y);
    }

    // Posição e Orientação atuais do robô
    const auto &robot_position = current_pose_.position;
    double robot_yaw = tf::getYaw(current_pose_.orientation);

    // Vetor do robô para o alvo
    double dx = target_point.x - robot_position.x;
    double dy = target_point.y - robot_position.y;
    double distance_to_target = std::hypot(dx, dy);
    double angle_to_target = std::atan2(dy, dx);
    double angle_error = normalizeAngle(angle_to_target - robot_yaw); // Erro angular normalizado para [-PI, PI]

    // 2. Lógica da Máquina de Estados Interna (ALIGNING_TO_POINT ou MOVING_TO_POINT)
    switch (current_turn_then_move_state_)
    {
    case ALIGNING_TO_POINT:
        ROS_DEBUG("TurnThenMoveState: ALIGNING_TO_POINT. Current Angle Error: %.2f rad (%.1f deg)",
                  angle_error, angle_error * 180.0 / M_PI);

        cmd_vel.linear.x = 0.0; // O robô não se move linearmente enquanto gira

        // Controle Proporcional para a velocidade angular (gira para alinhar)
        cmd_vel.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, angle_error * 1.5));

        // Se o robô estiver bem alinhado (erro angular dentro da tolerância), transiciona
        if (std::abs(angle_error) < alignment_tolerance_rad_)
        {
            ROS_INFO("Aligned to target point (%.2f, %.2f). Transitioning to MOVING_TO_POINT.",
                     target_point.x, target_point.y);
            current_turn_then_move_state_ = MOVING_TO_POINT;
            // Opcional: define a velocidade angular para zero imediatamente para evitar overshooting.
            cmd_vel.angular.z = 0.0;
        }
        break;

    case MOVING_TO_POINT:
        ROS_DEBUG("TurnThenMoveState: MOVING_TO_POINT. Distance to target: %.2f m", distance_to_target);

        // Verifica se o ponto alvo já foi alcançado
        if (distance_to_target < goal_tolerance_)
        {
            ROS_INFO("Target point (%.2f, %.2f) reached. Stopping.", target_point.x, target_point.y);
            // Reinicia o alvo interno para NaN. Isso força um reinício (ALIGNING_TO_POINT)
            // na próxima vez que essa função for chamada com *qualquer* ponto.
            target_point_for_turn_then_move_.x = std::numeric_limits<double>::quiet_NaN();
            return geometry_msgs::Twist(); // Retorna comando zero para parar o robô
        }

        // Move para frente com velocidade proporcional à distância, limitada pela velocidade máxima.
        cmd_vel.linear.x = std::min(max_linear_speed_, distance_to_target * 0.7);

        // Pequeno ajuste angular para manter o alinhamento enquanto avança.
        // Se o robô desviar um pouco, ele faz uma correção suave.
        if (std::abs(angle_error) > alignment_tolerance_rad_ / 2.0)
        {
            cmd_vel.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, angle_error * 0.5));
        }
        else
        {
            cmd_vel.angular.z = 0.0; // Se bem alinhado, não há rotação
        }
        break;
    }
    return cmd_vel; // Retorna o comando de velocidade calculado para o estágio atual
}