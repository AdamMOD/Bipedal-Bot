#include <init.h>
#include <string>
#include <fstream>
#include <chrono>

Eigen::VectorXd balance(Whacko mywhacko);
void test_servo_data(Whacko mywhacko);

void self_balance_loop(Whacko mywhacko);

void self_balance_setup(Whacko mywhacko);

int main(int argc, char* argv[])
{
	Whacko mywhacko;
	Eigen::VectorXd imu_reading(9);
	Eigen::VectorXd command(2);
	Eigen::VectorXd leg_location(2);
	Eigen::VectorXd state(3);
	Eigen::VectorXd zero_state(6);
	
	init_constants();
	zero_state = get_initial_state() * 1;
	int count;
	
	//Eigen::VectorXd new_init_state = balance(mywhacko);
	//update_initial_state(new_init_state);
	
	
	std::cout << "Press enter to continue..." << "\n";
	std::cin.get();
	
	mywhacko.zero_servos();
	usleep(1000 * 1000);
	
	// balance(mywhacko);
	
	//self_balance_setup(mywhacko);
	//test_servo_data(mywhacko);
	
	//usleep(1000 * 1000);
	self_balance_loop(mywhacko);
	
	usleep(1000 * 1000);
	mywhacko.zero_servos();
	mywhacko.shutdown();
	
	return 0;
}


Eigen::VectorXd balance(Whacko mywhacko)
{
	Eigen::VectorXd new_init_state(1);
	Eigen::VectorXd imu_reading(9);
	std::cout << "Balancing. State at count 59 (in 60s) is new zero." << std::endl;
	int count = 0;
	while (count < 600)
	{
		imu_reading = mywhacko.get9dof() * 1;
		new_init_state << (double) imu_reading(1);
		std::cout << "Balanced state reading " << new_init_state << std::endl;
		usleep(1000 * 100);
		count++;
		std::cout << count << std::endl;
	}
	return new_init_state;
}

void test_servo_data(Whacko mywhacko)
{
	std::cout << "Starting servo measurement test..." << "\n";
	std::ofstream myFile("/home/pi/srvotst.csv");
	myFile << "Time,Right_Hip,Left_Hip,U\n";

	Eigen::VectorXd leg_location(4);
	__uint64_t now;
	__uint64_t time_zero;
	float cmd;
	float angle_zero;
	Eigen::VectorXd zero_state(6);
	
	zero_state = get_initial_state() * 1;
	
	mywhacko.zero_servos();
	
	usleep(1000 * 1500);
	time_zero = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	std::cout << "T = 0" << "\n";
	while (now - time_zero < 5000)
	{
		now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		//cmd = 0.0;
		//square
		//cmd = 40. * (2 * (int ) (std::sin((float)(now - time_zero) / 1000. * 2.) > 0) - 1);
		//sin
		cmd = 20 +  10. * std::sin((float)(now - time_zero) / 1000. * 2);
		mywhacko.move_servo(cmd, RIGHT_HIP_PCA_CHANNEL);
		//mywhacko.move_servo(cmd, LEFT_HIP_PCA_CHANNEL);
		leg_location = mywhacko.getservopos() * 1;
		myFile << now - time_zero  << "," << leg_location(0) << "," << leg_location(1) << "," << cmd << "\n";
		//std::cout << now - time_zero << "\n";
		//myFile << now - time_zero  << "," << cmd << "\n";
		//usleep(1000 * 10);
	}
	
	myFile.close();
	std::cout << "End of servo measurement test." << "\n";
}

void self_balance_loop(Whacko mywhacko)
{
	Eigen::VectorXd imu_reading(9);
	Eigen::VectorXd command(2);
	Eigen::VectorXd state(2);
	Eigen::VectorXd leg_location(4);
	
	Eigen::VectorXd prev_state(2);
	__uint64_t now;
	__uint64_t time_zero;
	int count;
	
	std::ofstream myFile("/home/pi/run.csv");
	myFile << "Time,Command_0,Command 1,Pitch,Pitch_Rate,Right_Hip,Left_Hip\n";
	
	imu_reading = mywhacko.get9dof() * 1;
	state <<  imu_reading(1), imu_reading(4);
	
	time_zero = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	
	while (now-time_zero < 20000)
	{
		count++;
		imu_reading = mywhacko.get9dof() * 1;
		if (std::abs((state(0) - imu_reading(1))) <= 5 && std::abs((state(1) - imu_reading(4))) <= 5)
		{
			state <<  imu_reading(1), imu_reading(4);
		}
		command = P_theta_and_leg_balance(state, leg_location) * 1;

		mywhacko.move_servo(command(0), LEFT_HIP_PCA_CHANNEL);
		mywhacko.move_servo(command(1), RIGHT_HIP_PCA_CHANNEL);
		leg_location = mywhacko.getservopos() * 1;
		now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		myFile << now - time_zero << "," << command(0) << "," << command(1) << "," << state(0) << "," << state(1) << "," << leg_location(0) << "," << leg_location(1) << "\n";
		
		if (count % 50 == 0)
		{
			std::cout << "STATE: " << state << '\n';
			std::cout << "COMMAND: " <<  command << '\n';
		}
	}
	myFile.close();
	
}

void self_balance_setup(Whacko mywhacko)
{
	Eigen::VectorXd imu_reading(9);
	Eigen::VectorXd leg_location(4);
	Eigen::VectorXd state(2);
	Eigen::VectorXd zero_state(4);
	zero_state = get_initial_state() * 1;
	
	std::cout << "Orient the thing till it's balanced" << "\n";
	bool balanced = false;
	while (!balanced)
	{
		usleep(1000 * 10);
		imu_reading = mywhacko.get9dof() * 1;
		state <<  imu_reading(1), imu_reading(4);
		std::cout << "STATE " << state << '\n';
		balanced = (state(0) > zero_state(4) -.2 && state(0) < zero_state(4) +.2);
	}
}
