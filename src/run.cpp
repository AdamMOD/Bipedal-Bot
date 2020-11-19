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
	Eigen::VectorXd zero_state(4);
	zero_state = get_initial_state() * 1;
	init_constants();
	int count;
	
	//Eigen::VectorXd new_init_state = balance(mywhacko);
	//update_initial_state(new_init_state);
	
	
	std::cout << "Press enter to continue..." << "\n";
	std::cin.get();
	
	//test_servo_data(mywhacko);
	
	self_balance_setup(mywhacko);
	usleep(1000 * 1000);
	self_balance_loop(mywhacko);
	
	std::cout << "Shutting down..." << std::endl;
	gpioTerminate();
	return 0;
}

Eigen::VectorXd balance(Whacko mywhacko)
{
	Eigen::VectorXd new_init_state(4);
	Eigen::VectorXd leg_location(2);
	Eigen::VectorXd imu_reading(9);
	std::cout << "Balancing. State at count 59 (in 60s) is new zero." << std::endl;
	int count = 0;
	while (count < 60)
	{
		leg_location = mywhacko.getservopos() * 1;
		imu_reading = mywhacko.get9dof() * 1;
		new_init_state << leg_location(0), leg_location(1), imu_reading(1), 0;
		std::cout << "Balanced state reading " << new_init_state << std::endl;
		usleep(1000 * 1000);
		count++;
		std::cout << count << std::endl;
	}
	return new_init_state;
}

void test_servo_data(Whacko mywhacko)
{
	std::cout << "Starting servo measurement test..." << "\n";
	std::ofstream myFile("/home/pi/servo_dat_slow_right_2-newwiring.csv");
	myFile << "Time,Left,Right\n";
	
	Eigen::VectorXd leg_location(2);
	int count = 0;
	int tcount = 0;
	__uint64_t now;

	mywhacko.move_servo(1, LEFT_LEG_PIN);
	mywhacko.move_servo(120, RIGHT_LEG_PIN);
	usleep(1000 * 1500);
	
	
	while (count < 4)
	{
		int deg_counter = 120;
		//mywhacko.move_servo(1, LEFT_LEG_PIN);
		//mywhacko.move_servo(179, RIGHT_LEG_PIN);
		leg_location = mywhacko.getservopos_no_deg() * 1;
		while (tcount < 80)
		{
			tcount++;
			deg_counter--;
			mywhacko.move_servo(deg_counter, RIGHT_LEG_PIN);
			leg_location = mywhacko.getservopos_no_deg() * 1;
			now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
			myFile << now  << "," << leg_location(0) << "," << leg_location(1) << "\n";
			//usleep(1000 * 10);
		}
		tcount = 0;
		deg_counter = 40;
		//usleep(1000 * 500);
		//mywhacko.move_servo(179, LEFT_LEG_PIN);
		//mywhacko.move_servo(1, RIGHT_LEG_PIN);
		leg_location = mywhacko.getservopos_no_deg() * 1;
		while (tcount < 80)
		{
			tcount++;
			deg_counter++;
			mywhacko.move_servo(deg_counter, RIGHT_LEG_PIN);
			leg_location = mywhacko.getservopos_no_deg() * 1;
			now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
			myFile << now  << "," << leg_location(0) << "," << leg_location(1) << "\n";
			//usleep(1000 * 10);
		}
		tcount = 0;
		count++;
	}
	
	myFile.close();
	std::cout << "End of servo measurement test." << "\n";
}

void self_balance_loop(Whacko mywhacko)
{
	Eigen::VectorXd imu_reading(9);
	Eigen::VectorXd command(2);
	Eigen::VectorXd leg_location(2);
	Eigen::VectorXd state(4);
	int count;
	
	imu_reading = mywhacko.get9dof() * 1;
	
	while (imu_reading(1) < 80 && imu_reading(1) > -80)
	{
		count++;
		imu_reading = mywhacko.get9dof() * 1;
		leg_location = mywhacko.getservopos() * 1;
		//command = P_theta_dd_balance(imu_reading, leg_location);
		state <<  leg_location(0), leg_location(1), imu_reading(1), imu_reading(3);
		command = P_theta_balance(state) * 1;
		mywhacko.move_servo(command(0), LEFT_LEG_PIN);
		mywhacko.move_servo(command(1), RIGHT_LEG_PIN);
		
		if (count % 10 == 0)
		{
			std::cout << "STATE: " << state << '\n';
			std::cout << "COMMAND: " <<  command << '\n';
		}
		
		usleep(1000 * 10);
	}
}

void self_balance_setup(Whacko mywhacko)
{
	Eigen::VectorXd imu_reading(9);
	Eigen::VectorXd leg_location(2);
	Eigen::VectorXd state(3);
	Eigen::VectorXd zero_state(4);
	zero_state = get_initial_state() * 1;
	
	std::cout << "Orient the thing till it's balanced" << "\n";
	bool balanced = false;
	while (!balanced)
	{
		usleep(1000 * 10);
		imu_reading = mywhacko.get9dof() * 1;
		//std::cout << "IMU READ: " << imu_reading << '\n';
		leg_location = mywhacko.getservopos() * 1;
		state <<  leg_location(0), leg_location(1), imu_reading(1) ;
		balanced = (state(2) > zero_state(2) -.2 && state(2) < zero_state(2) +.2);
	}
	
	mywhacko.move_servo(zero_state(0), LEFT_LEG_PIN);
	mywhacko.move_servo(zero_state(1), RIGHT_LEG_PIN);
}
