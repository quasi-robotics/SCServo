/*
以下例子在SMS60中测试通过，如果测试其它型号SMS系列舵机请更改合适的位置、速度与延时参数。
*/

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
	while(1){
		sm.WritePosEx(1, 4095, 2250, 50);//舵机(ID1)以最高速度V=2250步/秒，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
		std::cout<< "pos ="<<1000<<std::endl;
		usleep(2270*1000);//[(P1-P0)/V]*1000+100
  
		sm.WritePosEx(1, 0, 2250, 50);//舵机(ID1)以最高速度V=2250步/秒，加速度A=50(50*100步/秒^2)，运行至P0=0位置
		std::cout<< "pos ="<<0<<std::endl;
		usleep(2270*1000);//[(P1-P0)/V]*1000+100
	}
	sm.end();
	return 1;
}

