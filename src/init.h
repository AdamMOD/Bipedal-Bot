#ifndef INIT_DONE
#define INIT_DONE

#include <cmath>
#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <cmath>  
#include "RPi_BNO055.h"
#include "Adafruit_ADS1015.h"
#include "RPi_Sensor.h"
#include "PCA9685.h"
#include <utility/imumaths.h>
#include <eigen3/Eigen/Dense>

const int LEFT_HIP_PCA_CHANNEL {1};
const int LEFT_HIP_ADS_CHANNEL {1};

// Flips sign of commanded angle. 
// Array in order of pca channels, ie right_hip,left_hip, 0, 0, right_knee,left_knee
const int SERVO_FLIP[6] {-1, 1, 0, 0, 1, -1};
const float SERVO_ZERO[6] {60., 50., 0, 0, 60., 53.};

const int RIGHT_HIP_PCA_CHANNEL {0};

const int RIGHT_HIP_ADS_CHANNEL {0};

const int LEFT_KNEE_PCA_CHANNEL {5};
const int LEFT_KNEE_ADS_CHANNEL {3};

const int RIGHT_KNEE_PCA_CHANNEL {4};
const int RIGHT_KNEE_ADS_CHANNEL {2};

const int SERVO_MAX_READING {26416};
const int SERVO_MIN_READING {272};

const int SERVO_MIN_PULSE_WIDTH {1000};
const int SERVO_MAX_PULSE_WIDTH {2000};

const int PWMFREQ {330};
const float MAX_SERVO_ANGLE {120.};


class Whacko
{
	public:
		Whacko();
	
		int shutdown();
		
		Eigen::VectorXd get9dof();
	
		Eigen::VectorXd getservopos();
	
		Eigen::VectorXd getservopos_no_deg();
		
		int move_servo(float degrees, int pin);
	
		int zero_servos();
	
	private:
		Adafruit_BNO055 myimu;
		Adafruit_ADS1115 myads;
		PCA9685 pcaboard;
};

Eigen::VectorXd P_theta_balance(const Eigen::VectorXd& state);

Eigen::VectorXd P_theta_and_leg_balance(const Eigen::VectorXd& state, const Eigen::VectorXd& leg_location);
Eigen::VectorXd P_theta_dd_balance(const Eigen::VectorXd& state, const Eigen::VectorXd& est_leg_location);
bool tune_offset();

void update_initial_state(const Eigen::VectorXd& new_init_state);
void init_constants();

float map(float x, float in_min, float in_max, float out_min, float out_max);

Eigen::VectorXd get_initial_state();

#endif
