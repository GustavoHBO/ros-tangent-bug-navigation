#include "ros/ros.h"
#include "TangentBug.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "tangent_bug");
    ros::NodeHandle nh;
    TangentBug tb(nh);
    tb.navigate();
    return 0;
}
