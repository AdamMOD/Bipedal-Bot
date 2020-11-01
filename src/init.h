#ifndef INIT_DONE
#define INIT_DONE

#include <array>
#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include "RPi_BNO055.h"
#include "Adafruit_ADS1015.h"


using namespace std;


class Whacko
{
	public:
		Whacko();
		
		array<float, 9> get9dof();
	
	private:
		Adafruit_BNO055 myimu;
};


#endif
