/*
以下例子在SCS15中测试通过，如果测试其它型号SCS系列舵机请更改合适的位置、速度与延时参数。
*/

#include <iostream>
#include "SCServo.h"

SCSCL sc;

u8 ID[2] = {1, 2};
u16 Position[2];
u16 Speed[2];

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
		Position[0] = 1000;
		Position[1] = 1000;
		Speed[0] = 1500;
		Speed[1] = 1500;
		sc.SyncWritePos(ID, 2, Position, 0, Speed);//舵机((ID1/ID2))以最高速度V=1500步/秒,运行至P1=1000
		std::cout<< "pos ="<<1000<<std::endl;
		usleep(754*1000);//[(P1-P0)/V]*1000+100
  
		Position[0] = 20;
		Position[1] = 20;
		Speed[0] = 1500;
		Speed[1] = 1500;
		sc.SyncWritePos(ID, 2, Position, 0, Speed);//舵机((ID1/ID2))以最高速度V=1500步/秒,运行至P1=20
		std::cout<< "pos ="<<0<<std::endl;
		usleep(754*1000);//[(P1-P0)/V]*1000+100
	}
	sc.end();
	return 1;
}

