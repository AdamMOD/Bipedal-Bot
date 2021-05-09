#include <init.h>


Eigen::MatrixXd K(1, 2);
Eigen::MatrixXd Kleg(2, 2);

Eigen::VectorXd reshape(2);
Eigen::VectorXd initial_state(6);

void init_constants()
{
	reshape << 1, 1;
	K << 2,
		 0;
	Kleg << 2, .1,
			0, .1;
	//righthip, lefthip, rightknee, leftknee, pitch, pitchrate
	initial_state <<  60, 50, 60, 53, -5, 0;
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void update_initial_state(const Eigen::VectorXd& new_init_state)
{
	initial_state = new_init_state;
}

Eigen::VectorXd get_initial_state()
{
	return initial_state;
}

Eigen::VectorXd P_theta_balance(const Eigen::VectorXd& state)
{
	Eigen::VectorXd control(2);
	control = reshape * (K * (state - initial_state.tail(2)));
	return control;
}	

Eigen::VectorXd P_theta_and_leg_balance(const Eigen::VectorXd& state, const Eigen::VectorXd& leg_location)
{
	Eigen::VectorXd control(2);
	control = reshape * (Kleg * (state - initial_state.tail(2)));
	return control;
}	

Eigen::VectorXd P_theta_dd_balance(const Eigen::VectorXd& state, const Eigen::VectorXd& est_leg_location)
{
	Eigen::VectorXd control(2);
	float balance_control_right = est_leg_location(1) - (initial_state(2) - state(1)) * .1;
	float balance_control_left = est_leg_location(0) - (initial_state(2) + state(1)) * .1;
	control << balance_control_left, balance_control_right;
	// control += initial_state.head(2);
	return control;
}

bool tune_offset()
{
	
}