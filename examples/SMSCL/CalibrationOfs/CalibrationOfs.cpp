#include <iostream>
#include "SCServo.h"

SMSCL sm;

int main(int argc, char **argv)
{
	if(argc<2){
        std::cout<< "argc error!"<<std::endl;
        return 0;
	}
	std::cout<< "serial:"<<argv[1]<<std::endl;
    if(!sm.begin(115200, argv[1])){
        std::cout<< "Failed to init smscl motor!"<<std::endl;
        return 0;
    }

	sm.CalibrationOfs(1);
	std::cout<< "Calibration Ofs"<<std::endl;
	sm.end();
	return 1;
}

