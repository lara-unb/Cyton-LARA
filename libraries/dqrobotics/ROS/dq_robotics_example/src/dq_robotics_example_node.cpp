//ROS
#include <ros/ros.h>

//Eigen
#include <Eigen/Dense>

//DQ Robotics
#include <dq_robotics/DQ.h>
#include <dq_robotics/DQ_kinematics.h>


using namespace DQ_robotics;
using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dq_robotics_example_node");
    ros::NodeHandle node_handle;

    DQ dq(1,2,3,4,5,6,7,8);
    ROS_INFO_STREAM(" " << dq);

}
