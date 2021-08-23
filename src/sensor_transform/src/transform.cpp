// minimal_simulator node:
// wsn example node that both subscribes and publishes
// does trivial system simulation, F=ma, to update velocity given F specified on topic "force_cmd"
// publishes velocity on topic "velocity"
#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>  //for absolute function

std_msgs::Float64 g_velocity;
std_msgs::Float64 g_force;
geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to robot
float lc_1; // global var for load cell 1
float lc_2; // global var for load cell 2

void lc_1_Callback(const std_msgs::Float64& message_holder) {
    // checks for messages on topic "lc_1"
    ROS_INFO("Received load cell 1 value is: %f", message_holder.data);
    lc_1 =  message_holder.data;
}

void lc_2_Callback(const std_msgs::Float64& message_holder) {
    // checks for messages on topic "lc_1"
    ROS_INFO("Received load cell 2 value is: %f", message_holder.data);
    lc_2 =  message_holder.data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "transform"); //name this node
    // when this compiled code is run, ROS will recognize it as a node called "minimal_simulator"
    ros::NodeHandle nh; // node handle
    //create a Subscriber object and have it subscribe to the topic "force_cmd"
    ros::Subscriber lc_1_subscriber_object = nh.subscribe("lc_1", 1, lc_1_Callback);
    ros::Subscriber lc_2_subscriber_object = nh.subscribe("lc_2", 1, lc_2_Callback);
    //simulate accelerations and publish the resulting velocity;
    ros::Publisher my_publisher_object = nh.advertise<std_msgs::Float64>("velocity", 1);
    //publish velocity commads in geometry_msgs ty
    ros::Publisher twist_commander = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);


    // start with all zeros in the command message; should be the case by default, but just to be safe..
    twist_cmd.linear.x=0.0;
    twist_cmd.linear.y=0.0;
    twist_cmd.linear.z=0.0;
    twist_cmd.angular.x=0.0;
    twist_cmd.angular.y=0.0;
    twist_cmd.angular.z=0.0;

    double mass = 100.0;
    double dt = 0.5; //10ms integration time step
    double sample_rate = 1.0 / dt; // compute the corresponding update frequency
    ros::Rate naptime(sample_rate);
    g_velocity.data = 0.0; //initialize velocity to zero
    g_force.data = 0.0; // initialize force to 0; will get updated by callback
    while (ros::ok()) {
        g_velocity.data = g_velocity.data + (g_force.data / mass) * dt; // Euler integration of
        //acceleration

        if (lc_1 >150 && lc_2 >150){
          ROS_INFO("Load cell 1&2 activated");
          g_force.data = 1.0;
          twist_cmd.linear.x=1.0;

          float diff = abs(lc_1-lc_2);
          if(diff>50){

            if(lc_1>lc_2){
              twist_cmd.angular.z=1;
              ROS_INFO("Going right");
            }
            else{
              twist_cmd.angular.z=-1;
                ROS_INFO("Going left");
            }
          }
         else{
              twist_cmd.angular.z=0;
            }

          }

        // else if (lc_2 >200){
        //   g_force.data = 1.0;
        //   twist_cmd.linear.x=1.0;
        //   ROS_INFO("Load cell 2 activated");
        // }
        else{
            g_force.data = 0;
            twist_cmd.linear.x=0.0;
            twist_cmd.angular.z=0;
        }

        ROS_INFO("Force value is: %f", g_force.data);
        my_publisher_object.publish(g_velocity); // publish the system state (trivial--1-D)
        twist_commander.publish(twist_cmd);

        ROS_INFO("velocity is = %f", g_velocity.data);
        ROS_INFO("cmd_vel for x linear direction is = /%f", twist_cmd.linear.x);

        ros::spinOnce(); //allow data update from callback
        naptime.sleep(); // wait for remainder of specified period; this loop rate is faster than
        // the update rate of the 10Hz controller that specifies force_cmd
        // however, simulator must advance each 10ms regardless
    }
    return 0; // should never get here, unless roscore dies
}
