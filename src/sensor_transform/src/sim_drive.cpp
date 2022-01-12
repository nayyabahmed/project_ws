// minimal_simulator node:
// wsn example node that both subscribes and publishes
// does trivial system simulation, F=ma, to update velocity given F specified on topic "force_cmd"
// publishes velocity on topic "velocity"
#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>  //for absolute function

// std_msgs::Float64 g_velocity;
// std_msgs::Float64 g_force;
// ros::Publisher my_publisher_object;

geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to robot
ros::Publisher twist_commander;  // global publisher object for velocity commands

float lc_1; // global var for load cell 1
float lc_2; // global var for load cell 2
float lc_3; // global var for load cell 3
float lc_4; // global var for load cell 4
float fsr_1; // global var for FSR 1
float fsr_2; // global var for FSR 4
float front_range = 10; // global value for average front range distance from lidar scan subscriber


void lc_1_Callback(const std_msgs::Float64& message_holder) {
    // checks for messages on topic "lc_1"
    ROS_INFO("Received load cell 1 value is: %f", message_holder.data);
    lc_1 =  message_holder.data;
}

void lc_2_Callback(const std_msgs::Float64& message_holder) {
    // checks for messages on topic "lc_2"
    ROS_INFO("Received load cell 2 value is: %f", message_holder.data);
    lc_2 =  message_holder.data;
}
void lc_3_Callback(const std_msgs::Float64& message_holder) {
    // checks for messages on topic "lc_3"
    ROS_INFO("Received load cell 3 value is: %f", message_holder.data);
    lc_3 =  message_holder.data;
}
void lc_4_Callback(const std_msgs::Float64& message_holder) {
    // checks for messages on topic "lc_4"
    ROS_INFO("Received load cell 4 value is: %f", message_holder.data);
    lc_4 =  message_holder.data;
}

void fsr_1_Callback(const std_msgs::Float64& message_holder) {
    // checks for messages on topic "fsr_1"
    ROS_INFO("Received FSR 1 value is: %f", message_holder.data);
    fsr_1 =  message_holder.data;
}
void fsr_2_Callback(const std_msgs::Float64& message_holder) {
    // checks for messages on topic "fsr_2"
    ROS_INFO("Received FSR 2 value is: %f", message_holder.data);
    fsr_2 =  message_holder.data;
}

void front_range_Callback(const std_msgs::Float64& message_holder) {
    // checks for messages on topic "fsr_2"
    ROS_INFO("Received front range received: %f", message_holder.data);
    front_range =  message_holder.data;
}

void move(); //function for moving robot in gazebo

int main(int argc, char **argv) {
    ros::init(argc, argv, "transform"); //name this node
    // when this compiled code is run, ROS will recognize it as a node called "minimal_simulator"
    ros::NodeHandle nh; // node handle
    //create a Subscriber object and have it subscribe to the topic "force_cmd"
    ros::Subscriber lc_1_subscriber_object = nh.subscribe("lc_1", 1, lc_1_Callback);
    ros::Subscriber lc_2_subscriber_object = nh.subscribe("lc_2", 1, lc_2_Callback);
    ros::Subscriber lc_3_subscriber_object = nh.subscribe("lc_3", 1, lc_3_Callback);
    ros::Subscriber lc_4_subscriber_object = nh.subscribe("lc_4", 1, lc_4_Callback);
    ros::Subscriber fsr_1_subscriber_object = nh.subscribe("fsr_1", 1, fsr_1_Callback);
    ros::Subscriber fsr_2_subscriber_object = nh.subscribe("fsr_2", 1, fsr_2_Callback);
    ros::Subscriber frontRange_object = nh.subscribe("front_range", 1, front_range_Callback);
    // //simulate accelerations and publish the resulting velocity;
    // my_publisher_object = nh.advertise<std_msgs::Float64>("velocity", 1);

    //publish velocity commads in geometry_msgs
    twist_commander = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    move();

        // the update rate of the 10Hz controller that specifies force_cmd
        // however, simulator must advance each 10ms regardless

    return 0; // should never get here, unless roscore dies
}

void moveforward();
void moveback();

void move(){
  // start with all zeros in the command message; should be the case by default, but just to be safe..
  twist_cmd.linear.x=0.0;
  twist_cmd.linear.y=0.0;
  twist_cmd.linear.z=0.0;
  twist_cmd.angular.x=0.0;
  twist_cmd.angular.y=0.0;
  twist_cmd.angular.z=0.0;
  bool isObstacle= false;

  double dt = 0.1; //10ms integration time step
  double sample_rate = 1.0 / dt; // compute the corresponding update frequency
  ros::Rate naptime(sample_rate);
  // double mass = 100.0;
  // g_velocity.data = 0.0; //initialize velocity to zero
  // g_force.data = 0.0; // initialize force to 0; will get updated by callback
  while (ros::ok()) {
      // g_velocity.data = g_velocity.data + (g_force.data / mass) * dt; // Euler integration of
      //acceleration

      // check of the evera
      if(front_range<3.0){
        isObstacle=false;
        ROS_INFO("Obstacle reached move backwards");
      }
      else{
        isObstacle=true;
      }

     if (lc_1 >500 && lc_4 >500 && isObstacle){
        moveforward();
      }
     else if(lc_2 >500 && lc_3 >500){
          moveback();
        }

     else{
          // g_force.data = 0;
          twist_cmd.linear.x=0.0;
          twist_cmd.angular.z=0;
        }

      twist_commander.publish(twist_cmd);  //publish velocity commands

      // ROS_INFO("Force value is: %f", g_force.data);
      // my_publisher_object.publish(g_velocity); // publish the system state (trivial--1-D)
      //
      //
      // ROS_INFO("velocity is = %f", g_velocity.data);
      // ROS_INFO("cmd_vel for x linear direction is = /%f", twist_cmd.linear.x);

      ros::spinOnce(); //allow data update from callback
      naptime.sleep(); // wait for remainder of specified period; this loop rate is faster than
    }
}

void moveforward(){
  ROS_INFO("Load cell 1&4 activated. Moving forward");
  // g_force.data = 1.0;
  twist_cmd.linear.x=0.5;

  float diff = abs(lc_1-lc_4);
  if(diff>300){

    if(lc_1>lc_4){
      twist_cmd.angular.z=-0.5;
      ROS_INFO("Going right");
    }
    else{
      twist_cmd.angular.z=0.5;
        ROS_INFO("Going left");
    }
  }
 else{
      twist_cmd.angular.z=0;
    }

}
void moveback(){
  ROS_INFO("Load cell 1&4 activated. Moving forward");
  // g_force.data = 1.0;
  twist_cmd.linear.x=-0.5;

  float diff = abs(lc_2-lc_3);
  if(diff>300){

    if(lc_2>lc_3){
      twist_cmd.angular.z=-0.5;
      ROS_INFO("Going right");
    }
    else{
      twist_cmd.angular.z=0.5;
        ROS_INFO("Going left");
    }
  }
 else{
      twist_cmd.angular.z=0;
    }


}
