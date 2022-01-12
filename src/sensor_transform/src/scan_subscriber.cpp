#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "laserscan/LaserScanner.h"
#include <std_msgs/Float64.h>

using namespace std;

sensor_msgs::LaserScan _scanMsg;
ros::Subscriber scanSubscriber;

std_msgs::Float64 front_range;
ros::Publisher pubRange;


void scanCallback (sensor_msgs::LaserScan scanMessage);

int main(int argc, char **argv){

	//initialize the ROS node
	ros::init(argc, argv, "scan_subscriber_cpp");
	ros::NodeHandle n;

	//subscribe to the laser scanner topic
	scanSubscriber = n.subscribe("/scan", 10, scanCallback);
	pubRange = n.advertise<std_msgs::Float64>("front_range",1);

	ros::spin();
}

void scanCallback (sensor_msgs::LaserScan scanMessage){
	//_scanMsg = scanMessage;
	cout<<"minimum range: " <<LaserScanner::getMinimumRange(scanMessage)<<endl;
    cout<<"maximum range: " <<LaserScanner::getMaximumRange(scanMessage)<<endl;
    cout<<"average range: " <<LaserScanner::getAverageRange(scanMessage,0,180)<<endl;

		front_range.data = LaserScanner::getAverageRange(scanMessage,45,135);
		ROS_INFO("The average front range is: %f", front_range.data);
		pubRange.publish(front_range);

    if (LaserScanner::isObstacleTooClose(scanMessage,0,180,0.69)==true){
        cout<<"obstacle too close"<<endl;
    }
    cout<<endl;

}
