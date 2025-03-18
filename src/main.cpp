#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

class TangentBug {
public:
    TangentBug() {
        // Inicializa o publisher para comandos de movimento
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Inicializa o subscriber para os dados do laser
        laser_sub = nh.subscribe("/scan", 10, &TangentBug::laserCallback, this);

        // Estado inicial do algoritmo
        state = GOAL_SEEK;

        // Definindo o objetivo (modifique conforme necessário)
        goal_x = 5.0;
        goal_y = 5.0;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
        float front_distance = *std::min_element(msg->ranges.begin(), msg->ranges.begin() + 10);
        float left_distance = *std::min_element(msg->ranges.begin() + 80, msg->ranges.begin() + 100);
        float right_distance = *std::min_element(msg->ranges.begin() + 260, msg->ranges.begin() + 280);

        switch (state) {
            case GOAL_SEEK:
                goalSeek(front_distance);
                break;
            case OBSTACLE_FOLLOWING:
                obstacleFollowing(left_distance, right_distance);
                break;
        }
    }

    void goalSeek(float front_distance) {
        // Calcula o ângulo para o objetivo
        float angle_to_goal = atan2(goal_y - robot_y, goal_x - robot_x);

        if (front_distance < 1.0) {  // Obstáculo detectado na frente
            state = OBSTACLE_FOLLOWING;
            cmd.linear.x = 0;
            cmd.angular.z = 0.5;  // Começa a seguir o obstáculo
        } else {
            cmd.linear.x = 0.5;  // Avança em direção ao objetivo
            cmd.angular.z = angle_to_goal;
        }

        cmd_vel_pub.publish(cmd);
    }

    void obstacleFollowing(float left_distance, float right_distance) {
        // Contorna o obstáculo
        if (left_distance > 1.5 && right_distance > 1.5) {
            state = GOAL_SEEK;
        } else {
            cmd.linear.x = 0.3;
            cmd.angular.z = (left_distance < right_distance) ? 0.5 : -0.5;
        }

        cmd_vel_pub.publish(cmd);
    }

private:
    enum State {
        GOAL_SEEK,
        OBSTACLE_FOLLOWING
    };

    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber laser_sub;

    geometry_msgs::Twist cmd;
    State state;

    float goal_x, goal_y;
    float robot_x = 0.0, robot_y = 0.0;  // Posição inicial do robô (simulada)
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "tangent_bug");

    TangentBug tangentBug;

    ros::spin();

    return 0;
}
