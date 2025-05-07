#include "TangentBug.h"
#include <tf/transform_datatypes.h> // Para converter orientações
#include <tf/transform_listener.h>  // Para ouvir as transformações

TangentBug::TangentBug(ros::NodeHandle &nh) : nh_(nh), current_state_(MOVE_TO_GOAL)
{

    // Subscribers and publishers
    scan_sub_ = nh_.subscribe("/scan", 50, &TangentBug::scanCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 50, &TangentBug::odomCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 50, &TangentBug::goalCallback, this);

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

    nh_.param("robot_width", robot_width_, 0.8);
    ROS_INFO("Largura do robô: %f", robot_width_);

    nh_.param("safety_margin", safety_margin_, 1.0);
    ROS_INFO("Margem de segurança: %f", safety_margin_);

    nh_.param("obstacle_threshold", obstacle_threshold_, 1.0);
    ROS_INFO("Limite de obstáculo: %f", obstacle_threshold_);

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
}

void TangentBug::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (std::isnan(msg->pose.pose.position.x) || std::isnan(msg->pose.pose.position.y))
    {
        ROS_WARN("Odometria contém valores inválidos.");
        return;
    }

    current_pose_ = msg->pose.pose;
    ROS_INFO("Odometria atualizada: Posição [%f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y);
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

    const double distance_to_goal = calculateDistance(current_pose_.position.x, current_pose_.position.y, goal_x_, goal_y_);
    ROS_INFO("Distância ao objetivo: %f", distance_to_goal);

    publishTrajectoryMarker(); // Publica a linha indicando a trajetória

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
        ROS_INFO_ONCE("State: MOVE_TO_GOAL");
        if (isPathClear(calculateHeadingToGoal(), obstacle_threshold_) == 1)
        {
            moveToGoal(distance_to_goal);
        }
        else
        {
            ROS_INFO("Obstacle detected, changing state to FOLLOW_CONTOUR.");
            contour_start_x_ = current_pose_.position.x;
            contour_start_y_ = current_pose_.position.y;
            best_distance_to_goal_ = distance_to_goal;
            current_state_ = FOLLOW_CONTOUR;
        }
        break;
    case FOLLOW_CONTOUR:
        ROS_INFO_ONCE("State: FOLLOW_CONTOUR");
        followContour();
        if (isPathClear(calculateHeadingToGoal(), obstacle_threshold_) == 1)
        {
            ROS_INFO("Clear line of sight, changing state to LEAVE_CONTOUR.");
            stopRobot();
            // current_state_ = LEAVE_CONTOUR;
            current_state_ = MOVE_TO_GOAL; // Mudar para MOVE_TO_GOAL diretamente
        }
        break;
    case LEAVE_CONTOUR:
        ROS_INFO_ONCE("State: LEAVE_CONTOUR");
        leaveContour();
        if (calculateDistance(current_pose_.position.x, current_pose_.position.y, contour_start_x_, contour_start_y_) > 1.0 &&
            isPathClear(calculateHeadingToGoal(), obstacle_threshold_) == 1)
        {
            ROS_INFO("Successfully left contour, changing state to MOVE_TO_GOAL.");
            stopRobot();
            current_state_ = MOVE_TO_GOAL;
        }
        break;
    default:
        ROS_ERROR("Unknown state encountered!");
        stopRobot();
        break;
    }
}

void TangentBug::moveToGoal(double distance_to_goal)
{
    double heading_to_goal = calculateHeadingToGoal();

    // Verificar se o caminho está livre na direção do objetivo
    int path_status = isPathClear(heading_to_goal, obstacle_threshold_);
    if (path_status == 1)
    { // Caminho livre
        geometry_msgs::Twist cmd_vel;

        // Suavizar a velocidade linear conforme a distância ao objetivo
        cmd_vel.linear.x = std::min(max_linear_speed_, distance_to_goal * 0.5);

        // Ajustar a rotação do robô para se alinhar ao objetivo
        double heading_error = heading_to_goal - tf::getYaw(current_pose_.orientation);

        // Normalizar o erro de heading para o intervalo [-pi, pi]
        while (heading_error > M_PI)
            heading_error -= 2 * M_PI;
        while (heading_error < -M_PI)
            heading_error += 2 * M_PI;

        // Aplicar um fator proporcional ao erro de rotação para suavizar o movimento angular
        cmd_vel.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, heading_error * 0.5));

        // Publicar o comando de velocidade
        cmd_vel_pub_.publish(cmd_vel);
    }
    else if (path_status == 0)
    { // Caminho bloqueado
        ROS_INFO("Obstáculo detectado, seguindo contorno...");

        // Armazenar a posição atual como ponto inicial do contorno
        contour_start_x_ = current_pose_.position.x;
        contour_start_y_ = current_pose_.position.y;

        // Atualizar a melhor distância ao objetivo
        best_distance_to_goal_ = calculateDistance(current_pose_.position.x, current_pose_.position.y, goal_x_, goal_y_);

        // Mudar para o estado de seguir o contorno
        current_state_ = FOLLOW_CONTOUR;
    }
    else if (path_status == -1)
    { // Ângulo fora do campo de visão do LIDAR
        ROS_INFO("Ângulo fora do campo de visão do LIDAR, girando no próprio eixo...");
        rotateInPlace();
    }
}

void TangentBug::followContour()
{
    const sensor_msgs::LaserScan current_scan = current_scan_;
    const std::vector<float> &ranges = current_scan.ranges;
    const geometry_msgs::Pose current_pose = current_pose_;
    double distance_to_goal = calculateDistance(current_pose.position.x, current_pose.position.y, goal_x_, goal_y_);

    // Atualiza a melhor distância se a nova distância for menor
    if (distance_to_goal < best_distance_to_goal_)
    {
        best_distance_to_goal_ = distance_to_goal;
    }

    // Lógica de seguir contorno: use sensores para ajustar o movimento do robô
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = contour_speed_; // Velocidade padrão de seguimento de contorno
    cmd_vel.angular.z = 0.0;

    // Calcular o ângulo relativo com base na orientação atual do robô
    const double heading = calculateHeadingToGoal();
    double angle = heading - tf::getYaw(current_pose_.orientation);
    ROS_INFO("Ângulo relativo para o objetivo: %f", angle);

    // Normalizar o ângulo para o intervalo [-pi, pi]
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;

    ROS_INFO("Ângulo relativo normalizado: %f", angle);

    // Verificar se o ângulo está dentro do campo de visão do LIDAR
    ROS_INFO("Ângulo mínimo: %f, Ângulo máximo: %f", current_scan.angle_min, current_scan.angle_max);
    if (angle < current_scan.angle_min || angle > current_scan.angle_max)
    {
        ROS_WARN("Angulo alvo fora do campo de visão do LIDAR: %f", angle);
        current_state_ = MOVE_TO_GOAL; // Indica que o ângulo está fora da visão do LIDAR
    }

    // Converter o ângulo em um índice para acessar o LIDAR
    const int index = std::lround((angle - current_scan.angle_min) / current_scan.angle_increment);

    const double robot_width = robot_width_;   // Largura do robô em metros
    const double safety_margin = safety_margin_; // Margem de segurança em metros
    const double fov = M_PI;          // Campo de visão do LIDAR em radianos (180 graus)
    double angle_diff = current_scan.angle_max - current_scan.angle_min;
    const double tr = angle_diff / current_scan.angle_increment;
    const int total_rays = std::lround(tr);
    const int base_range_to_check = std::lround((((robot_width + safety_margin) / 2.0) * (total_rays / fov)) / 2.0);
    const double obstacle_threshold = obstacle_threshold_;

    // Calcula o ângulo de desvio verificando múltiplos raios à frente
    bool obstacle_detected = false;
    for (int i = -base_range_to_check; i <= base_range_to_check; ++i)
    {
        const int check_index = index + i;
        if (check_index >= 0 && check_index < ranges.size() && ranges[index] < 1.0)
        { // Valor de desvio
            // Ajusta a velocidade angular proporcionalmente à proximidade do obstáculo
            // Verificar se a leitura é NaN ou inválida
            if (std::isnan(ranges[check_index]) || ranges[check_index] <= 0.0)
            {
                cmd_vel.angular.z = 0;
                obstacle_detected = true;
            }
            // Verificar se há um obstáculo mais próximo que o limite
            if (ranges[check_index] < obstacle_threshold)
            {
                cmd_vel.angular.z = std::max(0.5, 1.0 - ranges[check_index]);
                obstacle_detected = true;
            }
            break;
        }
    }

    // Se nenhum obstáculo foi detectado, aumentar a velocidade linear
    // if (!obstacle_detected)
    // {
    //     cmd_vel.linear.x = 0.5; // Avança mais rápido quando não há obstáculos
    // }

    // Publica o comando de velocidade
    cmd_vel_pub_.publish(cmd_vel);

    // Verifica se o caminho para o objetivo está livre
    int path_status = isPathClear(calculateHeadingToGoal(), obstacle_threshold_);
    if (path_status == 1)
    { // Caminho livre
        ROS_INFO("Linha de visão limpa, saindo do contorno...");
        stopRobot();
        current_state_ = LEAVE_CONTOUR;
    }
    else if (path_status == -1)
    { // Ângulo fora do campo de visão do LIDAR
        current_state_ = FOLLOW_CONTOUR;
        ROS_INFO("Ângulo fora do campo de visão do LIDAR, girando no próprio eixo...");
        rotateInPlace();
    }
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

void TangentBug::rotateInPlace(double angular_speed, double duration_seconds) {
    std::cout << "rotateInPlace" << std::endl;
    geometry_msgs::Twist rotate_msg;
    rotate_msg.linear.x = 0.0;
    rotate_msg.angular.z = angular_speed;

    ros::Rate rate(50);
    int ticks = duration_seconds * 50; // duration in seconds * rate
    for (int i = 0; i < ticks; ++i) {
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
    std::cout << "isPathClear" << std::endl;
    std::cout << "current_state_: " << current_state_ << std::endl;
    std::cout << "Heading: " << heading << std::endl;
    const double robot_width = robot_width_;   // Largura do robô em metros
    const double safety_margin = safety_margin_; // Margem de segurança em metros
    const double fov = M_PI;          // Campo de visão do LIDAR em radianos (180 graus)
    const sensor_msgs::LaserScan current_scan = current_scan_;
    // std::cout << "current_scan.angle_max: " << current_scan_.angle_max << std::endl;

    const std::vector<float> &ranges = current_scan.ranges;
    // std::cout << "ranges size: " << ranges.size() << std::endl;
    // std::cout << "current_scan.angle_max: " << current_scan.angle_max << std::endl;
    // std::cout << "current_scan.angle_min: " << current_scan.angle_min << std::endl;
    double angle_diff = current_scan.angle_max - current_scan.angle_min;
    const double tr = angle_diff / current_scan.angle_increment;
    // std::cout << "tr: " << tr << std::endl;
    const int total_rays = std::lround(tr);
    // ROS_INFO("tr: %f", tr);
    // ROS_INFO("Ângulo de visão do LIDAR em raios: [fov] %f", fov);
    // ROS_INFO("Ângulo de visão do LIDAR em graus: [fov] %f", fov * 180.0 / M_PI);
    // ROS_INFO("current_scan.angle_max: %f", current_scan.angle_max);
    // ROS_INFO("current_scan.angle_min: %f", current_scan.angle_min);
    // ROS_INFO("current_scan.angle_increment: %f", current_scan.angle_increment);
    // ROS_INFO("current_scan.angle_max - current_scan.angle_min: %f", angle_diff);
    // ROS_INFO("Total de raios no LIDAR: [total_rays] %d", total_rays);
    const int base_range_to_check = std::lround(((robot_width + safety_margin) / 2.0) * (total_rays / fov));

    // Calcular o ângulo relativo com base na orientação atual do robô
    double angle = heading - tf::getYaw(current_pose_.orientation);
    ROS_INFO("Ângulo relativo para o objetivo: %f", angle);

    // Normalizar o ângulo para o intervalo [-pi, pi]
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;

    ROS_INFO("Ângulo relativo normalizado: %f", angle);

    // Verificar se o ângulo está dentro do campo de visão do LIDAR
    ROS_INFO("Ângulo mínimo: %f, Ângulo máximo: %f", current_scan.angle_min, current_scan.angle_max);
    if (angle < current_scan.angle_min || angle > current_scan.angle_max)
    {
        ROS_WARN("Angulo alvo fora do campo de visão do LIDAR: %f", angle);
        return -1; // Indica que o ângulo está fora da visão do LIDAR
    }

    // Converter o ângulo em um índice para acessar o LIDAR
    const int index = std::lround((angle - current_scan.angle_min) / current_scan.angle_increment);

    // Verificar múltiplos raios ao redor do ângulo para garantir uma leitura mais robusta
    // ROS_INFO("Índice do LIDAR: %d", index);
    // ROS_INFO("Verificando %d raios ao redor do índice...", base_range_to_check);
    // ROS_INFO("Current scan size: %ld", ranges.size());
    // ROS_INFO("Current scan angle increment: %f", current_scan.angle_increment);
    // ROS_INFO("Laser Rages size: %ld", ranges.size());

    for (int i = -base_range_to_check; i <= base_range_to_check; ++i)
    {
        const int check_index = index + i;
        if (check_index >= 0 && check_index < ranges.size())
        {
            // Verificar se a leitura é NaN ou inválida
            if (std::isnan(ranges[check_index]) || ranges[check_index] <= 0.0)
            {
                ROS_WARN("Leitura inválida no LIDAR");
                ROS_WARN("Caminho bloqueado por leitura inválida.");
                return 0; // Indica que o caminho está bloqueado por leitura inválida
            }
            // Verificar se há um obstáculo mais próximo que o limite
            if (ranges[check_index] < obstacle_threshold)
            {
                ROS_WARN("Obstáculo detectado a %f metros.", ranges[check_index]);
                return 0; // Indica que o caminho está bloqueado por um obstáculo
            }
        }
    }
    ROS_INFO("Caminho livre para o objetivo.");
    return 1; // Indica que o caminho está livre
}

void TangentBug::publishTrajectoryMarker()
{
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "odom"; // Frame de referência, use o frame que está sendo usado para odometria
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "trajectory_line";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    // Tipo LINE_STRIP conecta os pontos com uma linha
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // Definir a escala da linha
    line_strip.scale.x = 0.05; // Espessura da linha

    // Cor da linha (azul neste exemplo)
    line_strip.color.r = 0.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0; // Transparência

    // Adicionar o ponto atual do robô à linha
    geometry_msgs::Point p;
    p.x = current_pose_.position.x;
    p.y = current_pose_.position.y;
    p.z = 0.0; // Para desenhar no plano XY

    line_strip.points.push_back(p);

    // Adicionar o ponto do destino
    p.x = goal_x_;
    p.y = goal_y_;
    p.z = 0.0;
    line_strip.points.push_back(p);

    // Publicar o marcador
    marker_pub_.publish(line_strip);
}

void TangentBug::spawnGoalMarker(double x, double y)
{
    ros::NodeHandle nh;
    ros::ServiceClient spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    ros::ServiceClient delete_client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

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