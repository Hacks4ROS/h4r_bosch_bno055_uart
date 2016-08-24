
#include "BoschBno055Uart.h"
#include <netinet/in.h>
#include <boost/detail/endian.hpp>


#define    ACCEL_FCT 1000.0
#define    MAG_FCT 16.0
#define    GYRO_FCT 900.0


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
,serial_()
,serial_timeout_(serial::Timeout::simpleTimeout(50))
,pub_imu_(nh_.advertise<sensor_msgs::Imu>("data",1,false))
,pub_imu_raw_(nh_.advertise<sensor_msgs::Imu>("data_raw",1,false))
,pub_mag_(nh_.advertise<sensor_msgs::MagneticField>("mag",1,false))
,pub_temp_(nh_.advertise<sensor_msgs::Temperature>("temp",1,false))
,pub_calibstate_(nh_.advertise<std_msgs::ByteMultiArray>("calib",1,false))
{
  nh_.param("rate", rate_, 10.0);
  std::string frame_id="/bosch_imu";
  nh_.param("frame_id", frame_id);
  msg_imu_.header.frame_id=frame_id;
  msg_imu_raw_.header.frame_id=frame_id;
  msg_mag_.header.frame_id=frame_id;
  msg_temperature_.header.frame_id=frame_id;

  //Init covariances
  for (int n = 0; n < 9; ++n)
  {
	  msg_imu_.orientation_covariance[n]=-1;
	  msg_imu_.angular_velocity_covariance[n]=-1;
	  msg_imu_.linear_acceleration_covariance[n]=-1;
	  msg_imu_raw_.orientation_covariance[n]=-1;
	  msg_imu_raw_.angular_velocity_covariance[n]=-1;
	  msg_imu_raw_.linear_acceleration_covariance[n]=-1;
  }


  std::string serial_device="/dev/ttyUSB0";
  nh_.param("serial_device", serial_device);


  serial_.setBaudrate(115200);
  serial_.setBytesize(serial::eightbits);
  serial_.setFlowcontrol(serial::flowcontrol_none);
  serial_.setStopbits(serial::stopbits_one);
  serial_.setPort(serial_device);
  serial_.setTimeout(serial_timeout_);
}

BoschBno055Uart::~BoschBno055Uart()
{

}


int BoschBno055Uart::run()
{
	ros::Rate loop_rate(rate_);

	serial_.open();
	serial_.flush(); //flush io buffers

	if(writeIMUByte(PAGE0_OPR_MODE,OP_MODE_CONFIG))
	{
		ROS_ERROR("Could not write config mode");
		exit(1);
	}

	if(writeIMUByte(PAGE0_PWR_MODE,PWR_MODE_NORMAL))
	{
		ROS_ERROR("Could not write normal power mode");
		exit(1);
	}

	if(writeIMUByte(PAGE_ID,0))
	{
		ROS_ERROR("Could not write register page 0");
		exit(1);
	}

	if(writeIMUByte(PAGE0_SYS_TRIGGER,0x00))
	{
		ROS_ERROR("Could not write sys trigger");
		exit(1);
	}

	if(writeIMUByte(PAGE0_UNIT_SEL,0x83))
	{
		ROS_ERROR("Could not write units");
		exit(1);
	}

	if(writeIMUByte(PAGE0_AXIS_MAP_CONFIG,0x24))
	{
		ROS_ERROR("Could not write axis mapping");
		exit(1);
	}

	if(writeIMUByte(PAGE0_AXIS_MAP_SIGN,0x06))
	{
		ROS_ERROR("Could not write axis sign");
		exit(1);
	}

	if(writeIMUByte(PAGE0_OPR_MODE,OP_MODE_NDOF))
	{
		ROS_ERROR("Could not write operation mode NDOF");
		exit(1);
	}



	ImuData imu_data;
	while(ros::ok())
	{
		if(!serial_.isOpen())
		{
			ROS_ERROR("Serial not opened!");
			exit(1);
		}



		if(!readIMU(PAGE0_ACC_DATA_X_LSB,(uint8_t*)&imu_data,46))
		{

			#ifndef BOOST_LITTLE_ENDIAN
			//Swap bytes if necessary
			for (int n = 0; n < 22; ++n)
			{
				imu_data.nthos_array[n]=le16toh(imu_data.nthos_array[n]);
			}
			#endif

			//IMU raw
			msg_imu_raw_.header.stamp=ros::Time::now();
			msg_imu_raw_.linear_acceleration.x=((double)(imu_data.imu.AccelerationDataX))/ACCEL_FCT;
			msg_imu_raw_.linear_acceleration.y=((double)(imu_data.imu.AccelerationDataY))/ACCEL_FCT;
			msg_imu_raw_.linear_acceleration.z=((double)(imu_data.imu.AccelerationDataZ))/ACCEL_FCT;
			msg_imu_raw_.angular_velocity.x=((double)(imu_data.imu.GyroscopeDataX))/GYRO_FCT;
			msg_imu_raw_.angular_velocity.y=((double)(imu_data.imu.GyroscopeDataY))/GYRO_FCT;
			msg_imu_raw_.angular_velocity.z=((double)(imu_data.imu.GyroscopeDataZ))/GYRO_FCT;

			//IMU filtered
			msg_imu_.header.stamp=ros::Time::now();
			msg_imu_.linear_acceleration.x=((double)(imu_data.imu.LinearAccelerationDataX))/ACCEL_FCT;
			msg_imu_.linear_acceleration.y=((double)(imu_data.imu.LinearAccelerationDataY))/ACCEL_FCT;
			msg_imu_.linear_acceleration.z=((double)(imu_data.imu.LinearAccelerationDataZ))/ACCEL_FCT;
			msg_imu_.angular_velocity.x=((double)(imu_data.imu.GyroscopeDataX))/GYRO_FCT;
			msg_imu_.angular_velocity.y=((double)(imu_data.imu.GyroscopeDataY))/GYRO_FCT;
			msg_imu_.angular_velocity.z=((double)(imu_data.imu.GyroscopeDataZ))/GYRO_FCT;
			msg_imu_.orientation.x=((double)(imu_data.imu.QuaternionxData));
			msg_imu_.orientation.y=((double)(imu_data.imu.QuaternionyData));
			msg_imu_.orientation.z=((double)(imu_data.imu.QuaternionzData));
			msg_imu_.orientation.w=((double)(imu_data.imu.QuaternionwData));

			//Magnetometer
			msg_mag_.header.stamp=ros::Time::now();
			msg_mag_.magnetic_field.x=((double)(imu_data.imu.MagnetometerDataX))/MAG_FCT;
			msg_mag_.magnetic_field.y=((double)(imu_data.imu.MagnetometerDataY))/MAG_FCT;
			msg_mag_.magnetic_field.z=((double)(imu_data.imu.MagnetometerDataZ))/MAG_FCT;

			//Temperature
			msg_temperature_.header.stamp=ros::Time::now();
			msg_temperature_.temperature=imu_data.temperature;

			msg_calib_.data.push_back(imu_data.calibration_status & 0x3);//MAG
			msg_calib_.data.push_back((imu_data.calibration_status & 0x3<<2)>>2);//ACC
			msg_calib_.data.push_back((imu_data.calibration_status & 0x3<<4)>>4);//GYR
			msg_calib_.data.push_back((imu_data.calibration_status & 0x3<<6)>>6);//SYS

			pub_imu_.publish(msg_imu_);
			pub_imu_raw_.publish(msg_imu_raw_);
			pub_mag_.publish(msg_mag_);
			pub_temp_.publish(msg_temperature_);
			pub_calibstate_.publish(msg_calib_);
		}
		else
		{
			ROS_ERROR("Could not read imu data!");
		}



		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

bool BoschBno055Uart::writeIMU(BNO055Register reg,const void *data, unsigned length)
{
	std::vector<uint8_t>out_data;
	out_data.push_back(0xAA);
	out_data.push_back(0); //WRITE
	out_data.push_back((uint8_t)reg);
	out_data.push_back(length);
	for (int b = 0; b < length; ++b)
	{
		out_data.push_back(*((uint8_t*)data+b));
	}

	serial_.flushInput();
	serial_.write(out_data);
	serial_.flushOutput();

	uint8_t in_data[2]={0,0};

	if(serial_.read(in_data,2)==2)
	{
		return !(in_data[0]==0xEE && in_data[1]==0x01);
	}

	return true;
}

bool BoschBno055Uart::readIMU(BNO055Register reg, uint8_t *data, unsigned length)
{
	std::vector<uint8_t>out_data;
		out_data.push_back(0xAA);
		out_data.push_back(1); //WRITE
		out_data.push_back((uint8_t)reg);
		out_data.push_back(length);
		serial_.flushInput();
		serial_.write(out_data);
		serial_.flushOutput();

		std::vector<uint8_t> data_in;
		serial_.read(data_in,2+length);

	if(data_in[0]==0xBB && data_in[1]==length)
	{
		memcpy(data,data_in.data()+2,length);
		return false;
	}
	else
	{
		ROS_ERROR_STREAM("Got "<<std::hex<<data_in[0]<<" Len/Error:"<<std::dec<<data_in[1]);
		return true;
	}
}

}/* namespace h4r_bosch_bno055_uart */
