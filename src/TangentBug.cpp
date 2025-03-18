#include "TangentBug.h"
#include <tf/transform_datatypes.h> // Para converter orientações
#include <tf/transform_listener.h>  // Para ouvir as transformações

TangentBug::TangentBug(ros::NodeHandle& nh) : nh_(nh), current_state_(MOVE_TO_GOAL) {

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

    // Initialize variables
    min_distance_to_goal_ = std::numeric_limits<double>::max();
    best_distance_to_goal_ = std::numeric_limits<double>::max();

    // Initialize goal to NaN
    goal_x_ = std::numeric_limits<double>::quiet_NaN();
    goal_y_ = std::numeric_limits<double>::quiet_NaN();
}


void TangentBug::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (msg->ranges.empty()) {
        ROS_WARN("Varredura LIDAR recebida está vazia.");
        return;
    }

    // Verificar se todas as leituras são válidas
    for (const auto& range : msg->ranges) {
        if (std::isnan(range) || range < 0.0) {
            ROS_WARN("Valor inválido encontrado na varredura LIDAR.");
            return;
        }
    }

    laser_ranges_ = msg->ranges;
    current_scan_ = *msg; // Atualiza a varredura atual para cálculos futuros
}

void TangentBug::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (std::isnan(msg->pose.pose.position.x) || std::isnan(msg->pose.pose.position.y)) {
        ROS_WARN("Odometria contém valores inválidos.");
        return;
    }

    current_pose_ = msg->pose.pose;
    ROS_INFO("Odometria atualizada: Posição [%f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void TangentBug::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (std::isnan(msg->pose.position.x) || std::isnan(msg->pose.position.y)) {
        ROS_WARN("Objetivo inválido recebido.");
        return;
    }

    // Verificar se o objetivo está muito próximo da posição atual
    double distance_to_goal = calculateDistance(current_pose_.position.x, current_pose_.position.y, msg->pose.position.x, msg->pose.position.y);
    if (distance_to_goal < goal_tolerance_) {
        ROS_INFO("Objetivo muito próximo, não é necessário movimento.");
        return;
    }

    goal_x_ = msg->pose.position.x; // Novo objetivo X
    goal_y_ = msg->pose.position.y; // Novo objetivo Y
    best_distance_to_goal_ = std::numeric_limits<double>::max(); // Reseta a melhor distância

    ROS_INFO("Novo objetivo recebido: Posição [%f, %f], Distância para o objetivo: %f", goal_x_, goal_y_, distance_to_goal);

    // Inserir o marcador de destino no Gazebo
    spawnGoalMarker(goal_x_, goal_y_);

    // Reseta para estado MOVE_TO_GOAL
    current_state_ = MOVE_TO_GOAL;
}

void TangentBug::navigate() {
    ros::Rate rate(50);
    while (ros::ok()) {
        computeControl();
        ros::spinOnce();
        rate.sleep();
    }
}

void TangentBug::computeControl() {

    // Verifica se temos um objetivo válido
    if (std::isnan(goal_x_) || std::isnan(goal_y_)) {
        ROS_WARN_THROTTLE(5, "Aguardando um objetivo válido...");
        return; // Não faz nada até receber um objetivo
    }

    // Calcula a distância até o objetivo
    double distance_to_goal = calculateDistance(current_pose_.position.x, current_pose_.position.y, goal_x_, goal_y_);
    
    publishTrajectoryMarker();  // Publica a linha indicando a trajetória

    // Verifica se o robô já está no objetivo
    if (distance_to_goal < goal_tolerance_) {
        ROS_INFO("Objetivo alcançado!");
        geometry_msgs::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;

        // Suaviza a parada enviando várias mensagens de parada
        for (int i = 0; i < 5; i++) {
            cmd_vel_pub_.publish(stop_msg);
            ros::Duration(0.1).sleep();
        }
        return;
    }
 
    // Decisão baseada no estado atual
    switch (current_state_) {
        case MOVE_TO_GOAL:
            ROS_INFO("Movendo-se para o objetivo...");
            moveToGoal(distance_to_goal);
            break;
        case FOLLOW_CONTOUR:
            ROS_INFO("Seguindo contorno...");
            followContour();
            break;
        case LEAVE_CONTOUR:
            ROS_INFO("Saindo do contorno...");
            leaveContour();
            break;
        default:
            ROS_ERROR("Estado desconhecido: %d", current_state_);
            break;
    }
}

void TangentBug::moveToGoal(double distance_to_goal) {
    double heading_to_goal = calculateHeadingToGoal();
    
    // Verificar se o caminho está livre na direção do objetivo
    int path_status = isPathClear(heading_to_goal);
    if (path_status == 1) { // Caminho livre
        geometry_msgs::Twist cmd_vel;
        
        // Suavizar a velocidade linear conforme a distância ao objetivo
        cmd_vel.linear.x = std::min(max_linear_speed_, distance_to_goal * 0.5);
        
        // Ajustar a rotação do robô para se alinhar ao objetivo
        double heading_error = heading_to_goal - tf::getYaw(current_pose_.orientation);
        
        // Normalizar o erro de heading para o intervalo [-pi, pi]
        while (heading_error > M_PI) heading_error -= 2 * M_PI;
        while (heading_error < -M_PI) heading_error += 2 * M_PI;
        
        // Aplicar um fator proporcional ao erro de rotação para suavizar o movimento angular
        cmd_vel.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, heading_error * 0.5));
        
        // Publicar o comando de velocidade
        cmd_vel_pub_.publish(cmd_vel);
    } else if (path_status == 0) { // Caminho bloqueado
        ROS_INFO("Obstáculo detectado, seguindo contorno...");
        
        // Armazenar a posição atual como ponto inicial do contorno
        contour_start_x_ = current_pose_.position.x;
        contour_start_y_ = current_pose_.position.y;
        
        // Atualizar a melhor distância ao objetivo
        best_distance_to_goal_ = calculateDistance(current_pose_.position.x, current_pose_.position.y, goal_x_, goal_y_);
        
        // Mudar para o estado de seguir o contorno
        current_state_ = FOLLOW_CONTOUR;
    } else if (path_status == -1) { // Ângulo fora do campo de visão do LIDAR
        ROS_INFO("Ângulo fora do campo de visão do LIDAR, girando no próprio eixo...");
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.5; // Girar até que o ângulo esteja dentro do campo de visão
        cmd_vel_pub_.publish(cmd_vel);
    }
}

void TangentBug::followContour() {
    double distance_to_goal = calculateDistance(current_pose_.position.x, current_pose_.position.y, goal_x_, goal_y_);

    // Atualiza a melhor distância se a nova distância for menor
    if (distance_to_goal < best_distance_to_goal_) {
        best_distance_to_goal_ = distance_to_goal;
    }

    // Lógica de seguir contorno: use sensores para ajustar o movimento do robô
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.2; // Velocidade padrão de seguimento de contorno
    cmd_vel.angular.z = 0.0;

    // Calcula o ângulo de desvio verificando múltiplos raios à frente
    bool obstacle_detected = false;
    int range_to_check = 5; // Verificar 5 ângulos à esquerda e à direita do robô
    for (int i = -range_to_check; i <= range_to_check; ++i) {
        int index = (laser_ranges_.size() / 2) + i;
        if (index >= 0 && index < laser_ranges_.size() && laser_ranges_[index] < 1.0) { // Valor de desvio
            // Ajusta a velocidade angular proporcionalmente à proximidade do obstáculo
            cmd_vel.angular.z = std::max(0.5, 1.0 - laser_ranges_[index]); 
            obstacle_detected = true;
            break;
        }
    }

    // Se nenhum obstáculo foi detectado, aumentar a velocidade linear
    if (!obstacle_detected) {
        cmd_vel.linear.x = 0.5; // Avança mais rápido quando não há obstáculos
    }

    // Publica o comando de velocidade
    cmd_vel_pub_.publish(cmd_vel);

    // Verifica se o caminho para o objetivo está livre
    int path_status = isPathClear(calculateHeadingToGoal());
    if (path_status == 1) { // Caminho livre
        ROS_INFO("Linha de visão limpa, saindo do contorno...");
        geometry_msgs::Twist slow_down_msg;
        slow_down_msg.linear.x = 0.0;
        slow_down_msg.angular.z = 0.0;
        for (int i = 0; i < 5; ++i) {
            cmd_vel_pub_.publish(slow_down_msg);  // Envia várias mensagens para suavizar a parada
            ros::Duration(0.1).sleep();
        }
        current_state_ = LEAVE_CONTOUR;
    } else if (path_status == -1) { // Ângulo fora do campo de visão do LIDAR
        ROS_INFO("Ângulo fora do campo de visão do LIDAR, girando no próprio eixo...");
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.5; // Girar até que o ângulo esteja dentro do campo de visão
        cmd_vel_pub_.publish(cmd_vel);
    }
}

void TangentBug::leaveContour() {
    double distance_from_contour_start = calculateDistance(current_pose_.position.x, current_pose_.position.y, contour_start_x_, contour_start_y_);

    // Verifica se já estamos suficientemente longe do ponto de início do contorno e se o caminho está livre
    int path_status = isPathClear(calculateHeadingToGoal());
    if (distance_from_contour_start > 1.0 && path_status == 1) { // Caminho livre
        ROS_INFO("Saindo do contorno para o objetivo, caminho livre.");
        
        // Suavizar a transição publicando algumas mensagens de parada
        geometry_msgs::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        for (int i = 0; i < 5; ++i) {
            cmd_vel_pub_.publish(stop_msg);
            ros::Duration(0.1).sleep();
        }

        // Mudar para o estado de seguir o objetivo diretamente
        current_state_ = MOVE_TO_GOAL;
    } else if (path_status == -1) { // Ângulo fora do campo de visão do LIDAR
        ROS_WARN("Tentativa de sair do contorno, mas o ângulo ainda está fora do campo de visão do LIDAR, girando no próprio eixo...");
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.5;
        cmd_vel_pub_.publish(cmd_vel);
    } else {
        ROS_WARN("Tentativa de sair do contorno, mas o caminho ainda não está livre ou o robô não se afastou o suficiente.");
        
        // Manter a distância de segurança do obstáculo durante a tentativa de sair do contorno
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.2; // Velocidade linear moderada para afastar-se do contorno
        cmd_vel.angular.z = 0.0; // Manter-se estável em direção ao ponto livre

        cmd_vel_pub_.publish(cmd_vel);
    }

    // Log de depuração para acompanhamento
    ROS_INFO("Distância do início do contorno: %f", distance_from_contour_start);
}

double TangentBug::calculateDistance(double x1, double y1, double x2, double y2) {
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
double TangentBug::calculateHeadingToGoal() {
    // Verificar se já estamos no objetivo ou muito perto dele
    if (fabs(goal_x_ - current_pose_.position.x) < 0.01 && fabs(goal_y_ - current_pose_.position.y) < 0.01) {
        ROS_INFO("Já estamos no objetivo!");
        return 0.0;  // Retornar 0 ou algum valor apropriado
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

int TangentBug::isPathClear(double heading, double obstacle_threshold) {
    static double robot_width = 0.5; // Largura do robô em metros
    static double safety_margin = 0.5; // Margem de segurança em metros
    static double fov = M_PI; // Campo de visão do LIDAR em radianos (180 graus)
    static int total_rays = static_cast<int>((current_scan_.angle_max - current_scan_.angle_min) / current_scan_.angle_increment);
    static int base_range_to_check = static_cast<int>(((robot_width + safety_margin) / 2.0) * (total_rays / fov));

    // Calcular o ângulo relativo com base na orientação atual do robô
    double angle = heading - tf::getYaw(current_pose_.orientation);
    ROS_INFO("Ângulo relativo para o objetivo: %f", angle);
    
    // Normalizar o ângulo para o intervalo [-pi, pi]
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;

    ROS_INFO("Ângulo relativo normalizado: %f", angle);

    // Verificar se o ângulo está dentro do campo de visão do LIDAR
    ROS_INFO("Ângulo mínimo: %f, Ângulo máximo: %f", current_scan_.angle_min, current_scan_.angle_max);
    if (angle < current_scan_.angle_min || angle > current_scan_.angle_max) {
        ROS_WARN("Angulo alvo fora do campo de visão do LIDAR: %f", angle);
        return -1; // Indica que o ângulo está fora da visão do LIDAR
    }

    // Converter o ângulo em um índice para acessar o LIDAR
    int index = static_cast<int>((angle - current_scan_.angle_min) / current_scan_.angle_increment);
    
    // Verificar múltiplos raios ao redor do ângulo para garantir uma leitura mais robusta
    for (int i = -base_range_to_check; i <= base_range_to_check; ++i) {
        int check_index = index + i;
        if (check_index >= 0 && check_index < static_cast<int>(laser_ranges_.size())) {
            // Verificar se a leitura é NaN ou inválida
            if (std::isnan(laser_ranges_[check_index]) || laser_ranges_[check_index] <= 0.0) {
                ROS_WARN("Leitura inválida no LIDAR");
                return 0; // Indica que o caminho está bloqueado por leitura inválida
            }
            // Verificar se há um obstáculo mais próximo que o limite
            if (laser_ranges_[check_index] < obstacle_threshold) {
                ROS_WARN("Obstáculo detectado a %f metros.", laser_ranges_[check_index]);
                return 0;  // Indica que o caminho está bloqueado por um obstáculo
            }
        }
    }
    ROS_INFO("Caminho livre para o objetivo.");
    return 1; // Indica que o caminho está livre
}

void TangentBug::publishTrajectoryMarker() {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "odom";  // Frame de referência, use o frame que está sendo usado para odometria
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "trajectory_line";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    // Tipo LINE_STRIP conecta os pontos com uma linha
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // Definir a escala da linha
    line_strip.scale.x = 0.05;  // Espessura da linha

    // Cor da linha (azul neste exemplo)
    line_strip.color.r = 0.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;  // Transparência

    // Adicionar o ponto atual do robô à linha
    geometry_msgs::Point p;
    p.x = current_pose_.position.x;
    p.y = current_pose_.position.y;
    p.z = 0.0;  // Para desenhar no plano XY

    line_strip.points.push_back(p);

    // Adicionar o ponto do destino
    p.x = goal_x_;
    p.y = goal_y_;
    p.z = 0.0;
    line_strip.points.push_back(p);

    // Publicar o marcador
    marker_pub_.publish(line_strip);
}

void TangentBug::spawnGoalMarker(double x, double y) {
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
    spawn_model_srv.request.initial_pose.position.z = 0.1;  // Um pouco acima do solo
    spawn_model_srv.request.initial_pose.orientation.w = 1.0;

    // Namespace vazio
    spawn_model_srv.request.robot_namespace = "";

    // Verifica se o modelo existe e tenta deletá-lo
    if (delete_client.call(delete_srv)) {
        if (delete_srv.response.success) {
            ROS_INFO("Modelo '%s' deletado com sucesso.", spawn_model_srv.request.model_name.c_str());
        } else {
            ROS_WARN("Falha ao deletar o modelo '%s': %s", spawn_model_srv.request.model_name.c_str(), delete_srv.response.status_message.c_str());
        }
    } else {
        ROS_ERROR("Erro ao chamar o serviço de deleção do modelo.");
    }

    // Esperar até o serviço estar disponível
    if (spawn_model_client.call(spawn_model_srv)) {
        ROS_INFO("Marcador de destino inserido no Gazebo no ponto [%f, %f]", x, y);
    } else {
        ROS_ERROR("Falha ao spawnar o marcador de destino no Gazebo.");
    }
}