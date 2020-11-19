#ifndef INIT_DONE
#define INIT_DONE

#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include "RPi_BNO055.h"
#include "Adafruit_ADS1015.h"
#include "RPi_Sensor.h"
#include <utility/imumaths.h>
#include <eigen3/Eigen/Dense>

const int LEFT_LEG_PIN {18};
const int RIGHT_LEG_PIN {19};

const int SERVO_MAX_READING {26416};
const int SERVO_MIN_READING {272};



class Whacko
{
	public:
		Whacko();
		
		Eigen::VectorXd get9dof();
	
		Eigen::VectorXd getservopos();
	
		Eigen::VectorXd getservopos_no_deg();
		
		int move_servo(float degrees, int pin);
	
	private:
		Adafruit_BNO055 myimu;
		Adafruit_ADS1115 myads;
};

Eigen::VectorXd P_theta_balance(const Eigen::VectorXd& state);

Eigen::VectorXd P_theta_dd_balance(const Eigen::VectorXd& state, const Eigen::VectorXd& est_leg_location);
bool tune_offset();

void update_initial_state(const Eigen::VectorXd& new_init_state);
void init_constants();
Eigen::VectorXd get_initial_state();

#endif
