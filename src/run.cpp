#include <init.h>


using namespace std;


int main(int argc, char* argv[])
{
	Whacko mywhacko;
	array<float, 9> reading = mywhacko.get9dof();
	cout << reading[8] << endl;
	return 0;
}
