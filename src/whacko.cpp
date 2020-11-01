#include <init.h>


using namespace std;

Whacko::Whacko()
{
	cout << "Whacko being created...\n";

	if (gpioInitialise() < 0)
	{
		cout <<"Initialisation error of the GPIO \n Closing program..."<< endl;
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
	//Calibrating IMU
	bool calibrated = true;
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

	cout << "Whacko has become.\n";
}


array<float, 9> Whacko::get9dof()
{
	imu::Vector<3> euler = myimu.getVector(Adafruit_BNO055::VECTOR_EULER);
	imu::Vector<3> gyro = myimu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	imu::Vector<3> linear_accel = myimu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

	array<float,3> a_euler = {(float)euler.x(), (float)euler.y(), (float)euler.z()};
	array<float,3> a_gyro = {(float)gyro.x(), (float)gyro.y(), (float)gyro.z()};
	array<float,3> a_linear_accel = {(float)linear_accel.x(), (float)linear_accel.y(), (float)linear_accel.z()};
	array<float, 9> ndof_reading = {euler[0], euler[1], euler[2], gyro[0], gyro[1], gyro[2],
	       linear_accel[0], linear_accel[1], linear_accel[2]};	
	return ndof_reading;
}