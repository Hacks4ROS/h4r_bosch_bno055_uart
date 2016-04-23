#include <ros/ros.h>
#include "BoschBno055Uart.h"

int main(int argc, char** argv)
{
	ros::init(argc,argv,"bosch_bno055_node");
	ros::Time::init();

	h4r_bosch_bno055_uart::BoschBno055Uart node;
	return node.run();
}
