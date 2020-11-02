#include <init.h>


int main(int argc, char* argv[])
{
	Whacko mywhacko;
	arma::colvec reading(9);
	reading = mywhacko.get9dof();
	std::cout << reading << std::endl;
	return 0;
}
