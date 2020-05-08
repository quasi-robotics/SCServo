#ifndef _SCSERIAL_H
#define _SCSERIAL_H

#include "SCS.h"

class SCSerial : public SCS
{
public:
    SCSerial(SerialIO* pSerial) : SCS(pSerial) {};
    SCSerial(SerialIO* pSerial, u8 End) : SCS(pSerial, End) {};
    SCSerial(SerialIO* pSerial, u8 End, u8 Level) : SCS(pSerial, End, Level) {};

public:
      // functions
    virtual int WritePos(u8 ID, s16 Position, u16 Speed=0, u16 Time = 0, u8 ACC = 0) = 0;
    virtual int RegWritePos(u8 ID, s16 Position, u16 Speed = 0, u16 Time = 0, u8 ACC = 0) = 0;

    virtual void SyncWritePos(u8 ID[], u8 IDN, s16 Position[], u16 Speed = 0, u16 Time = 0, u8 ACC = 0) = 0;
    virtual void SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[] = nullptr, u16 Time[] = nullptr, u8 ACC[] = nullptr) = 0;

    virtual int WriteOfs(u8 ID, s16 Ofs)                                                  {  return  0  ; }
    virtual int WriteSpe(u8 ID, s16 Speed, u8 ACC = 0)                                    {  return  0  ; }
    virtual int wheelMode(u8 ID)                                                          {  return  0  ; }
    virtual int pwmMode(u8 ID)                                                            {  return  0  ; }
    virtual int jointMode(u8 ID, u16 minAngle = 0, u16 maxAngle = 1023)                    {  return  0  ; }
    virtual s16 ReadPos(u8 ID, u8 *Err = NULL)                                            {  return  0  ; }
    virtual int UnLockEprom(u8 ID)                                                        {  return  0  ; }
    virtual int LockEprom(u8 ID)                                                          {  return  0  ; }
    virtual int WritePWM(u8 ID, s16 pwmOut)                                               {  return  0  ; }
    virtual int EnableTorque(u8 ID, u8 Enable)                                            {  return  0  ; }
    virtual int ReadLoad(u8 ID, u8 *Err = NULL)                                           {  return  0  ; }
    virtual int ReadVoltage(u8 ID)                                                        {  return  0  ; }
    virtual int ReadTemper(u8 ID)                                                         {  return  0  ; }
    virtual int ReadSpeed(u8 ID, u8 *Err = NULL)                                          {  return  0  ; }
    virtual int ReadCurrent(u8 ID, u8 *Err = NULL)                                        {  return  0  ; }
    virtual int ReadMove(u8 ID)															  {  return  0  ; }

    virtual int WritePunch(u8 ID,u16 new_punch)                                        {  return  0  ; }
    virtual int WriteP(u8 ID, u8 new_P)                                                {  return  0  ; }
    virtual int WriteI(u8 ID, u8 new_I)                                                {  return  0  ; }
    virtual int WriteD(u8 ID, u8 new_D)                                                {  return  0  ; }
    virtual int WriteMaxTorque(u8 ID, u16 new_torque)                                  {  return  0  ; }
    virtual int ReadPunch(u8 ID)                                                       {  return  0  ; }
    virtual int ReadP(u8 ID)                                                           {  return  0  ; }
    virtual int ReadI(u8 ID)                                                           {  return  0  ; }
    virtual int ReadD(u8 ID)                                                           {  return  0  ; }
    virtual int ReadMaxTorque(u8 ID)                                                   {  return  0  ; }
    virtual int ReadTorqueEnable(u8 ID)                                                {  return  0  ; }
    virtual int ReadOfs(u8 ID, u8 *Err = NULL)                                         {  return  0  ; }

    virtual int getErr(){  return Err;  }

protected:
    int Err;
};

#endif