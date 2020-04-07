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

	sm.unLockEprom(1);//打开EPROM保存功能
	std::cout<< "unLock Eprom"<<std::endl;
	sm.writeByte(1, SMSBL_ID, 2);//ID
	std::cout<< "write ID:"<<2<<std::endl;
	sm.LockEprom(2);////关闭EPROM保存功能
	std::cout<< "Lock Eprom"<<std::endl;
	sm.end();
	return 1;
}

