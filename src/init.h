#ifndef INIT_DONE
#define INIT_DONE

#include <armadillo>
//#include <mlpack/core.hpp>
#include <array>
#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include "RPi_BNO055.h"
#include "Adafruit_ADS1015.h"


//using namespace mlpack;


class Whacko
{
	public:
		Whacko();
		
		arma::colvec get9dof();
	
	private:
		Adafruit_BNO055 myimu;
};


#endif
