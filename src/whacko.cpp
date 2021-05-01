#include <init.h>


Whacko::Whacko()
{
	std::cout << "Whacko being created...\n";

	if (gpioInitialise() < 0)
	{
		std::cout <<"Initialisation error of the GPIO \n Closing program..."<< std::endl;
		exit(0);
	}

	//Create the BNO055 IMU object
	myimu = Adafruit_BNO055();
	myimu._HandleBNO=i2cOpen(myimu._i2cChannel,BNO055_ADDRESS_A,0);
	// Initialise the sensor 
	if(!myimu.begin())
	{
 		std::cout << "Ooops, no BNO055 detected, check your wiring!" << std::endl;
		exit(0);
	}
	
	myimu.setExtCrystalUse(false);
	
	int8_t temp = myimu.getTemp();
	std::cout << "Current Temperature: " << (int)temp << " C" << std::endl;
	
	//Calibrating IMU
	bool calibrated = true;
	int cal_count = 0;
	while(!calibrated)
	{
		/* Display calibration status for each sensor. */
		imu::Vector<3> euler = myimu.getVector(Adafruit_BNO055::VECTOR_EULER);
		/* Display some euler data */
		std::cout << "X: " << euler.x() <<  " Y: " << euler.y() << " Z: "
		  << euler.z() << "\t\t";
		
	 	uint8_t system, gyro, accel, mag = 0;
	 	myimu.getCalibration(&system, &gyro, &accel, &mag);
	 	std::cout<< "CALIBRATION: Sys=" << (int)system << " Gyro=" << (int) gyro
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
	
	//Create the ADS1115 ADC object 
	myads.setGain(GAIN_ONE);
	myads.begin();
	//
	
	// Initialize the servo board
	pcaboard.init(1, 0x40);
	std::cout << "Initializing pcaboard.\n";
	usleep(1000 * 100);
	pcaboard.setPWMFreq(PWMFREQ);
	std::cout << "Setting pwmfreq to " << PWMFREQ << ". \n";
	usleep(1000 * 100);
	
	std::cout << "Whacko has become.\n";
}

int Whacko::shutdown()
{
	std::cout << "Shutting down..." << std::endl;
	for (int i = 0; i < 16; i++)
	{
		pcaboard.setPWM(i, 0, 0);
		usleep(1000 * 10);
	}
	pcaboard.reset();
	usleep(1000 * 10);
	return 0;
}

Eigen::VectorXd Whacko::get9dof()
{
	// - VECTOR_GYROSCOPE     - rad/s
	// - VECTOR_EULER         - degrees
	// - VECTOR_LINEARACCEL   - m/s^2
	// Gyro reading is raw, needs to be filtered. Maybe KF it all?
	// Returns 9dof reading in form eulerxyz, gyroxyz, linear_accelxyz
	imu::Vector<3> euler = myimu.getVector(Adafruit_BNO055::VECTOR_EULER);
	imu::Vector<3> gyro = myimu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	imu::Vector<3> linear_accel = myimu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

	Eigen::VectorXd ndof_reading(9);
	ndof_reading << euler.x(), euler.y(), euler.z(), gyro.x(), gyro.y(), gyro.z(),
					linear_accel.x(), linear_accel.y(), linear_accel.z();
	//std::cout << ndof_reading << "\n";
	/*
	if (ndof_reading.maxCoeff() > 1000 || ndof_reading.minCoeff() < -1000)
	{
		return Eigen::VectorXd::Zero(9);
	}
	*/
	return ndof_reading;
}

int Whacko::move_servo(float degrees, int channel)
{
	if (degrees > 180 || degrees < 0)
	{
		std::cout << "Command out of range!" << std::endl;
		return 1;
	}
	
	int width = (int) SERVO_MIN_PULSE_WIDTH + (float) ( SERVO_MAX_PULSE_WIDTH - SERVO_MIN_PULSE_WIDTH) / 180. * degrees;
	int analog_val = int(float(width) /  1000000 * PWMFREQ * 4096);
	pcaboard.setPWM(channel, 0, analog_val);
	return 0;
}

/*
Eigen::VectorXd Whacko::getservopos()
{
	uint16_t adc_left_knee;
	
	adc_left_knee  = myads.readADC_SingleEnded(LEFT_KNEE_ADS_CHANNEL);

	if (adc_left_knee >  SERVO_MAX_READING + 10000)
	{
		adc_left_knee = SERVO_MIN_READING;
	}
	double adc_left_leg_deg = (double)(adc_left_knee - SERVO_MIN_READING) / (double)(SERVO_MAX_READING - SERVO_MIN_READING) * 180.;
	Eigen::VectorXd reading(1);
	reading << adc_left_knee;
	return reading;
}
*/

Eigen::VectorXd Whacko::getservopos_no_deg()
{
	uint16_t adc_right_hip;
	uint16_t adc_left_hip;
	uint16_t adc_right_knee;
	uint16_t adc_left_knee;
	
	adc_right_hip  = myads.readADC_SingleEnded(RIGHT_HIP_ADS_CHANNEL);
	adc_left_hip  = myads.readADC_SingleEnded(LEFT_HIP_ADS_CHANNEL);
	adc_right_knee  = myads.readADC_SingleEnded(RIGHT_KNEE_ADS_CHANNEL);
	adc_left_knee  = myads.readADC_SingleEnded(LEFT_KNEE_ADS_CHANNEL);
	
	Eigen::VectorXd reading(4);
	reading << (double) adc_right_hip, (double) adc_left_hip, (double) adc_right_knee, (double) adc_left_knee;
	return reading;
}
