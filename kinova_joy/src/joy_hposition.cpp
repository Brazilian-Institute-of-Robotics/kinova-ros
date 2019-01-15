#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"


float joints[6] = {0.0, 2.9, 1.3, 4.2, 1.4, 0.0};

int main(int argc, char** argv){
    ros::init(argc, argv, "joy_interface");
    ros::NodeHandle nh;

    //Here I'm getting joy output and making it a joint_controller input 
    ros::Publisher joint_1 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_1_position_controller/command", 1000);
    ros::Publisher joint_2 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_2_position_controller/command", 1000);
    ros::Publisher joint_3 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_3_position_controller/command", 1000);
    ros::Publisher joint_4 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_4_position_controller/command", 1000);
    ros::Publisher joint_5 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_5_position_controller/command", 1000);
    ros::Publisher joint_6 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_6_position_controller/command", 1000);

    //Publishing messages at /joy rate
    ros::Rate loop_rate(10);

    //The message objects to send command controller
    std_msgs::Float64 msg1;
    std_msgs::Float64 msg2;
    std_msgs::Float64 msg3;
    std_msgs::Float64 msg4;
    std_msgs::Float64 msg5;
    std_msgs::Float64 msg6;

    while(ros::ok()){
        joint_1.publish(msg1);
        joint_2.publish(msg2);
        joint_3.publish(msg3);
        joint_4.publish(msg4);
        joint_5.publish(msg5);
        joint_6.publish(msg6);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();

    return 0;
}