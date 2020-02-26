/*
 * SCServo.cpp
 * 飞特串行舵机硬件接口层程序
 * 日期: 2017.8.22
 * 作者: 谭雄乐
 */


#include "SCSerial.h"

SCSerial::SCSerial()
{
	IOTimeOut = 100;
	pSerial = NULL;
}

SCSerial::SCSerial(u8 End): SCS(End)
{
	IOTimeOut = 100;
	pSerial = NULL;
}

SCSerial::SCSerial(u8 End, u8 Level): SCS(End, Level)
{
	IOTimeOut = 100;
	pSerial = NULL;
}

int SCSerial::readSCS(unsigned char *nDat, int nLen)
{
	int Size = 0;
	int ComData;
	#if defined(SCS_USE_MICROS)
		unsigned long t_begin = micros();
	#else
		unsigned long t_begin = millis();
	#endif
	unsigned long t_user;
	while(1){
		ComData = pSerial->read();
		if(ComData!=-1){
			if(nDat){
				nDat[Size] = ComData;
			}
			Size++;
			#if defined(SCS_USE_MICROS)
				t_begin = micros();
			#else
				t_begin = millis();
			#endif
		}
		if(Size>=nLen){
			break;
		}
		#if defined(SCS_USE_MICROS)
			t_user = micros() - t_begin;
		#else
			t_user = millis() - t_begin;
		#endif
		if(t_user>IOTimeOut){
			break;
		}
	}
	return Size;
}

int SCSerial::writeSCS(unsigned char *nDat, int nLen)
{
	if(nDat==NULL){
		return 0;
	}
	return pSerial->write(nDat, nLen);
}

int SCSerial::writeSCS(unsigned char bDat)
{
	return pSerial->write(&bDat, 1);
}

void SCSerial::flushSCS()
{
	while(pSerial->read()!=-1);
}
