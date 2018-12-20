#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64.h"

double h1_state;
double v1_state;
double v2_state;

void callBackFunction(const sensor_msgs::Joy& msg){
    h1_state = msg.axes[0]*3.141592; 
    v1_state = msg.axes[1]*(3.141592);

    //h2_state = msg.axes[2]*3.141592;
    v2_state = msg.axes[3]*(3.141592);
    //msg.buttons
}

int main(int argc, char** argv){
    ros::init(argc, argv, "joy_interface");
    ros::NodeHandle nh;

    //Here i'm getting joy output and making it a joint_controller input 
    ros::Publisher joint_1 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_1_position_controller/command", 1000);
    ros::Publisher joint_2 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_2_position_controller/command", 1000);
    ros::Publisher joint_3 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_3_position_controller/command", 1000);
    ros::Subscriber sub = nh.subscribe("joy", 1000, callBackFunction);

    //Publishing messages at /joy rate
    ros::Rate loop_rate(10);

    //The message objetc to receive joy output
    sensor_msgs::Joy msg;

    while(ros::ok()){
        //Creating the message object for joint_controller input
        std_msgs::Float64 msg1;
        std_msgs::Float64 msg2;
        std_msgs::Float64 msg3;

        msg1.data = h1_state;
        msg2.data = v1_state;
        msg3.data = v2_state;

        joint_1.publish(msg1);
        joint_2.publish(msg2);
        joint_3.publish(msg3);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();

    return 0;
}