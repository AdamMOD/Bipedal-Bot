#include <init.h>


Eigen::MatrixXd K(1, 2);

Eigen::VectorXd flip_servo_left(2);
Eigen::VectorXd initial_state(6);

void init_constants()
{
	flip_servo_left << -1, 1;
	K << 2,
		 0;
	//righthip, lefthip, rightknee, leftknee, pitch, pitchrate
	initial_state <<  90, 75, 90, 80, -5, 0;
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
	//std::cout << "SUB: " << initial_state.tail(2) - state.tail(2) << " " << state.tail(2) << " " << initial_state.tail(2) << "\n";
	control = initial_state.head(2) + flip_servo_left * (K * (initial_state.tail(2) - state));
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