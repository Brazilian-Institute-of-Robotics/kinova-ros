#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64.h"

double analogic_x;
double analogic_y;
double channel_;

int actualize;

void callBackFunction(const sensor_msgs::Joy& msg){
    analogic_x = msg.axes[0]*3.141592; 
    analogic_y = msg.axes[1]*(3.141592);

    actualize = msg.buttons[0];

    channel_ = msg.axes[6]*(-1);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "joy_interface");
    ros::NodeHandle nh;
    ROS_INFO("Hello");
    //It specifies if command interface is on/off
    bool status = false;

    // channel will specify wich of the 6 joints is receiving command
    int channel = 0; 

    //Here I'm getting joy output and making it a joint_controller input 
    ros::Publisher joint_1 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_1_position_controller/command", 1000);
    ros::Publisher joint_2 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_2_position_controller/command", 1000);
    ros::Publisher joint_3 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_3_position_controller/command", 1000);
    ros::Publisher joint_4 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_4_position_controller/command", 1000);
    ros::Publisher joint_5 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_5_position_controller/command", 1000);
    ros::Publisher joint_6 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_6_position_controller/command", 1000);

    ros::Subscriber sub = nh.subscribe("joy", 1000, callBackFunction);

    //Publishing messages at /joy rate
    ros::Rate loop_rate(10);

    //The message object to receive joy output
    sensor_msgs::Joy msg;

    //The message objects to send command controller
    std_msgs::Float64 msg1;
    std_msgs::Float64 msg2;
    std_msgs::Float64 msg3;

    while(ros::ok()){
        if(channel_ != 0.){
            channel = channel + (int)channel_;
            channel = (channel % 6);
            if(channel <= 0){
                channel = channel + 6;
            }
        }
        ROS_INFO("Channel %d", channel);
        //Creating the message object for joint_controller input
        msg1.data = analogic_x;
        msg2.data = analogic_y;

        if(actualize == 1){
            ROS_INFO("Status %d", status);
            status = !status;
        }

        if(status == true){
            switch(channel){
                case 1:
                    ROS_INFO("Joint %d", channel);
                    joint_1.publish(msg1);
                break;

                case 2:
                    ROS_INFO("Joint %d", channel);
                    joint_2.publish(msg2);
                break;

                case 3:
                    ROS_INFO("Joint %d", channel);
                    joint_3.publish(msg2);
                break;

                case 4:
                    ROS_INFO("Joint %d", channel);
                    joint_4.publish(msg2);
                break;

                case 5:
                    ROS_INFO("Joint %d", channel);
                    joint_5.publish(msg2);
                break;

                case 6:
                    ROS_INFO("Joint %d", channel);
                    joint_6.publish(msg2);
                break;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();

    return 0;
}