#include <init.h>
#include "RPi_Sensor.h"
#include "RPi_BNO055.h"
#include <utility/imumaths.h>

Whacko::Whacko()
{
	std::cout << "Whacko being created...\n";

	if (gpioInitialise() < 0)
	{
		std::cout <<"Initialisation error of the GPIO \n Closing program..."<< std::endl;
		exit(0);
	}

	//Create the BNO055 IMU object
	Adafruit_BNO055 myimu = Adafruit_BNO055();
	myimu._HandleBNO=i2cOpen(myimu._i2cChannel,BNO055_ADDRESS_A,0);
	// Initialise the sensor 
	if(!myimu.begin())
	{
 		std::cout << "Ooops, no BNO055 detected, check your wiring!" << std::endl;
		exit(0);
	}
	usleep(1000*100);
	//Calibrating IMU
	bool calibrated = false;
	int cal_count = 0;
	while(!calibrated)
	{
		/* Display calibration status for each sensor. */
	 	uint8_t system, gyro, accel, mag = 0;
	 	myimu.getCalibration(&system, &gyro, &accel, &mag);
	 	std::cout<< "CALIBRATIO: Sys=" << (int)system << " Gyro=" << (int) gyro
		   << " Accel=" << (int) accel << " Mag=" << (int)mag << std::endl;

		usleep(1000 * 100);
		if( (int)system == 3 and (int) accel == 3)
		{
			cal_count++;
			if(cal_count > 10)
			{
				calibrated = true;
			}
		}
	}

	std::cout << "Whacko has become.\n";
}


arma::colvec Whacko::get9dof()
{
	// Returns 9dof reading in form eulerxyz, gyroxyz, linear_accelxyz
	imu::Vector<3> euler = myimu.getVector(Adafruit_BNO055::VECTOR_EULER);
//	imu::Vector<3> gyro = myimu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//	imu::Vector<3> linear_accel = myimu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

	arma::colvec ndof_reading(9);
//	ndof_reading = {euler.x(), euler.y(), euler.z(), gyro.x(), gyro.y(), gyro.z(),
//	       linear_accel.x(), linear_accel.y(), linear_accel.z()};	
	ndof_reading = {0,1,2,3,4,5,6,7,8};
	return ndof_reading;
}
