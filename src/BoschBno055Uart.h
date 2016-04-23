/*
 * BoschBno055Uart.h
 *
 *  Created on: 22.04.2016
 *      Author: Christian Holl
 *      
 * @todo insert LICENSE!
 *      All rights reserved! (©2016)
 */

#ifndef BOSCHBNO055UART_H_
#define BOSCHBNO055UART_H_

#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>

namespace h4r_bosch_bno055_uart
{
/**
 * @todo add license and move to header file
 * h4r_bosch_bno055_uart
 * BoschBno055Uart
 */
class BoschBno055Uart
{
private:
	ros::NodeHandle n_;
	ros::NodeHandle nh_;
	double rate_;

	sensor_msgs::Imu msg_imu_;
	sensor_msgs::Imu msg_imu_raw_;
	sensor_msgs::MagneticField msg_mag_;
	sensor_msgs::Temperature Temperature_;
	serial::Serial serial_;

public:
	BoschBno055Uart();
	virtual ~BoschBno055Uart();
	int run();
};/* class BoschBno055Uart */



} /* namespace h4r_bosch_bno055_uart */




#endif /* BOSCHBNO055UART_H_ */
