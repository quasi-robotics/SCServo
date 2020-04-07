#include <iostream>
#include "SCServo.h"

SCSCL sc;

int main(int argc, char **argv)
{
	if(argc<2){
        std::cout<< "argc error!"<<std::endl;
        return 0;
	}
	std::cout<< "serial:"<<argv[1]<<std::endl;
    if(!sc.begin(115200, argv[1])){
        std::cout<< "Failed to init scscl motor!"<<std::endl;
        return 0;
    }
	while(1){
		int Pos;
		int Speed;
		int Load;
		int Voltage;
		int Temper;
		int Move;
		int Current;
		if(sc.FeedBack(1)!=-1){
			Pos = sc.ReadPos(-1);
			Speed = sc.ReadSpeed(-1);
			Load = sc.ReadLoad(-1);
			Voltage = sc.ReadVoltage(-1);
			Temper = sc.ReadTemper(-1);
			Move = sc.ReadMove(-1);
			Current = sc.ReadCurrent(-1);
			std::cout<< "pos ="<<Pos<<std::endl;
			std::cout<< "Speed ="<<Speed<<std::endl;
			std::cout<< "Load ="<<Load<<std::endl;
			std::cout<< "Voltage ="<<Voltage<<std::endl;
			std::cout<< "Temper ="<<Temper<<std::endl;
			std::cout<< "Move ="<<Move<<std::endl;
			std::cout<< "Current ="<<Current<<std::endl;
			usleep(10*1000);
		}else{
			std::cout<< "read err ="<<std::endl;
			sleep(2);
		}
	}
	sc.end();
	return 1;
}

