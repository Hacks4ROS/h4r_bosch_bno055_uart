
#include "BoschBno055Uart.h"

/**
 * @todo add license and move to source file
 * Class source
 * h4r_bosch_bno055_uart
 * BoschBno055Uart
 */
namespace h4r_bosch_bno055_uart
{


BoschBno055Uart::BoschBno055Uart()
:n_()
,nh_("~")
{
  nh_.param("rate", rate_, 10.0);
  std::string frame_id="/bosch_imu";
  nh_.param("frame_id", frame_id);

  std::string serial_device;
  nh_.param("serial_device", serial_device);
  msg_imu_.header.frame_id=frame_id;

  serial_.setBaudrate(115200);
  serial_.setBytesize(serial::eightbits);
  serial_.setFlowcontrol(serial::flowcontrol_none);
  serial_.setStopbits(serial::stopbits_one);
  serial_.setPort(serial_device);
}

BoschBno055Uart::~BoschBno055Uart()
{

}


int BoschBno055Uart::run()
{
	ros::Rate loop_rate(rate_);

	serial_.open();
	while(ros::ok() && serial_.isOpen())
	{




		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

}/* namespace h4r_bosch_bno055_uart */
