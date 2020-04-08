/*
 * SMSBL.cpp
 * 飞特SMSBL系列串行舵机应用层程序
 * 日期: 2018.8.2
 * 作者: 谭雄乐
 */

#if defined(_MSC_VER)
#include <malloc.h>
#else
#include <alloca.h>
#endif
#include "SMSBL.h"

SMSBL::SMSBL(SerialIO* pSerial) : SCSerial(pSerial)
{
}

SMSBL::SMSBL(SerialIO* pSerial, u8 End): SCSerial(pSerial, End)
{
}

SMSBL::SMSBL(SerialIO* pSerial, u8 End, u8 Level): SCSerial(pSerial, End, Level)
{
}

int SMSBL::EnableTorque(u8 ID, u8 Enable)
{
	return writeByte(ID, SMSBL_TORQUE_ENABLE, Enable);
}

int SMSBL::ReadTorqueEnable(u8 ID)
{
	return readByte(ID, SMSBL_TORQUE_ENABLE);
}


int SMSBL::writePos(u8 ID, s16 Position, u16 Speed, u16 Time, u8 ACC, u8 Fun)
{
	if(Position<0){
		Position = -Position;
		Position |= (1<<15);
	}
	flushSCS();
	u8 buf[7];
	buf[0] = ACC;
	Host2SCS(buf+1, buf+2, Position);
	Host2SCS(buf+3, buf+4, Time);
	Host2SCS(buf+5, buf+6, Speed);
	writeBuf(ID, SMSBL_ACC, buf, 7, Fun);
	return Ack(ID);
}

//写位置指令
//舵机ID，Position位置，加速度ACC，速度Speed
int SMSBL::WritePos(u8 ID, s16 Position, u16 Speed, u16 Time, u8 ACC)
{
	return writePos(ID, Position, Speed, Time, ACC, INST_WRITE);
}

//异步写位置指令
//舵机ID，Position位置，加速度ACC，速度Speed
int SMSBL::RegWritePos(u8 ID, s16 Position, u16 Speed, u16 Time, u8 ACC)
{
	return writePos(ID, Position, Speed, Time, ACC, INST_REG_WRITE);
}

void SMSBL::RegWriteAction()
{
	writeBuf(0xfe, 0, NULL, 0, INST_ACTION);
}

//写位置指令
//舵机ID[]数组，IDN数组长度，Position位置，ACC加速度，速度Speed
void SMSBL::SyncWritePos(u8 ID[], u8 IDN, s16 Position[], u16 Speed, u16 Time, u8 ACC)
{
  u8* offbuf = static_cast<u8 *>(alloca(7 * IDN));
  for(u8 i = 0; i<IDN; i++) {
    if(Position[i]<0){
      Position[i] = -Position[i];
      Position[i] |= (1<<15);
    }
    offbuf[i*7] = ACC;
    Host2SCS(offbuf+i*7+1, offbuf+i*7+2, Position[i]);
    Host2SCS(offbuf+i*7+3, offbuf+i*7+4, Time);
    Host2SCS(offbuf+i*7+5, offbuf+i*7+6, Speed);
  }
  syncWrite(ID, IDN, SMSBL_ACC, offbuf, 7);
}

//写位置指令
//舵机ID[]数组，IDN数组长度，Position位置，ACC加速度，速度Speed
void SMSBL::SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u16 Time[], u8 ACC[])
{
  u8* offbuf = static_cast<u8 *>(alloca(7 * IDN));
  for(u8 i = 0; i<IDN; i++){
    if(Position[i]<0){
      Position[i] = -Position[i];
      Position[i] |= (1<<15);
    }

    offbuf[i*7] = ACC ? ACC[i] : 0;
    Host2SCS(offbuf+i*7+1, offbuf+i*7+2, Position[i]);
    Host2SCS(offbuf+i*7+3, offbuf+i*7+4, Time ? Time[i] : 0);
    Host2SCS(offbuf+i*7+5, offbuf+i*7+6, Speed ? Speed[i] : 0);
  }
  syncWrite(ID, IDN, SMSBL_ACC, offbuf, 7);
}

//读位置，超时Err=1
s16 SMSBL::ReadPos(u8 ID)
{
    Err = 0;
	s16 curPos = readWord(ID, SMSBL_PRESENT_POSITION_L);
	if(curPos==-1){
        Err = 1;
		return -1;
	}
    if(curPos&(1<<15)){
        curPos = -(curPos&~(1<<15));
    }

	return curPos;
}

//速度控制模式
int SMSBL::WriteSpe(u8 ID, s16 Speed, u8 ACC)
{
	if(Speed<0){
		Speed = -Speed;
		Speed |= (1<<15);
	}
	int res = writeByte(ID, SMSBL_ACC, ACC);
	if(res==-1){
		return -1;
	}
	return writeWord(ID, SMSBL_GOAL_SPEED_L, Speed);
}

//PWM输出模式
 int SMSBL::WritePWM(u8 ID, s16 pwmOut)
{
	if(pwmOut<0){
		pwmOut = -pwmOut;
		pwmOut |= (1<<15);
	}
	return writeWord(ID, SMSBL_GOAL_SPEED_L, pwmOut);
}

//读输出扭力，超时返回-1
int SMSBL::ReadLoad(u8 ID)
{
    Err = 0;
	int Load = readWord(ID, SMSBL_PRESENT_LOAD_L);
	if(Load==-1){
        Err = 1;
        return -1;
	}
    if(Load&(1<<10)){
        Load = -(Load&~(1<<10));
    }
	return Load;
}

//读电压，超时返回-1
 int SMSBL::ReadVoltage(u8 ID)
{
    Err = 0;
    int Voltage = readByte(ID, SMSBL_PRESENT_VOLTAGE);
    if(Voltage==-1){
        Err = 1;
        return -1;
    }
    return Voltage;
}

//读温度，超时返回-1
int SMSBL::ReadTemper(u8 ID)
{
    Err = 0;
	int Temper = readByte(ID, SMSBL_PRESENT_TEMPERATURE);
    if(Temper==-1){
        Err = 1;
        return -1;
    }
    return Temper;
}

int SMSBL::wheelMode(u8 ID)
{
	return writeByte(ID, SMSBL_MODE, 1);		
}

int SMSBL::pwmMode(u8 ID)
{
	return writeByte(ID, SMSBL_MODE, 2);		
}

int SMSBL::jointMode(u8 ID, u16 minAngle, u16 maxAngle)
{
	return writeByte(ID, SMSBL_MODE, 0);	
}

//恢复舵机参数为默认值
int SMSBL::Recovery(u8 ID)
{
	flushSCS();
	writeBuf(ID, 0, NULL, 0, INST_RECOVER);
	return Ack(ID);
}

//复位舵机
int SMSBL::Reset(u8 ID)
{
	flushSCS();
	writeBuf(ID, 0, NULL, 0, INST_RESET);
	return Ack(ID);
}

int SMSBL::WriteOfs(u8 ID, s16 Ofs)
{
	if(Ofs<0){
		Ofs = -Ofs;
		Ofs |= (1<<11);
	}
	return writeWord(ID, SMSBL_OFS_L, Ofs);
}

int SMSBL::UnLockEprom(u8 ID)
{
	return writeByte(ID, SMSBL_LOCK, 0);//打开EPROM保存功能
}

int SMSBL::LockEprom(u8 ID)
{
	return writeByte(ID, SMSBL_LOCK, 1);//关闭EPROM保存功能
}

int SMSBL::WritePunch(u8 ID, u16 new_punch)
{
	return writeWord(ID, SMSBL_PUNCH_L, new_punch);	
}

int SMSBL::WriteP(u8 ID, u8 new_P)
{
    return writeByte(ID, SMSBL_COMPLIANCE_P, new_P);
}               

int SMSBL::WriteI(u8 ID, u8 new_I)
{
    return writeByte(ID, SMSBL_COMPLIANCE_I, new_I);
}               

int SMSBL::WriteD(u8 ID, u8 new_D)    
{
    return writeByte(ID, SMSBL_COMPLIANCE_D, new_D);
}

int SMSBL::WriteMaxTorque(u8 ID, u16 new_torque)
{
	return writeWord(ID, SMSBL_MAX_TORQUE_L, new_torque);	
}

int SMSBL::ReadOfs(u8 ID)
{
    Err = 0;
	s16 Ofs = readWord(ID, SMSBL_OFS_L);
	if(Ofs==-1){
        Err = 1;
		return -1;
	}
    if(Ofs&(1<<15)){
        Ofs = -(Ofs&~(1<<15));
    }
	return Ofs;
}

int SMSBL::ReadSpeed(u8 ID)
{
    Err = 0;
	s16 Speed = readWord(ID, SMSBL_PRESENT_SPEED_L);
	if(Speed==-1){
        Err = 1;
		return -1;
	}
    if(Speed&(1<<15)){
        Speed = -(Speed&~(1<<15));
    }
	return Speed;
}

int SMSBL::ReadCurrent(u8 ID)
{
    Err = 0;
	s16 Current = readWord(ID, SMSBL_PRESENT_CURRENT_L);
	if(Current==-1){
	    Err = 1;
		return -1;
	}
    if(Current&(1<<15)){
        Current = -(Current&~(1<<15));
    }
	return Current;
}

int SMSBL::ReadMove(u8 ID)
{
    Err = 0;
	int Move = readByte(ID, SMSBL_MOVING);
    if(Move==-1){
        Err = 1;
        return -1;
    }
	return Move;
}

int SMSBL::ReadPunch(u8 ID)
{
    Err = 0;
	return readWord(ID, SMSBL_PUNCH_L);
}

int SMSBL::ReadP(u8 ID)
{
    Err = 0;
	return readByte(ID, SMSBL_COMPLIANCE_P);
}

int SMSBL::ReadI(u8 ID)
{
    Err = 0;
	return readByte(ID, SMSBL_COMPLIANCE_I);
}

int SMSBL::ReadD(u8 ID)
{
    Err = 0;
	return readByte(ID, SMSBL_COMPLIANCE_D);
}

int SMSBL::ReadMaxTorque(u8 ID)
{
    Err = 0;
	return readWord(ID, SMSBL_MAX_TORQUE_L);
}
