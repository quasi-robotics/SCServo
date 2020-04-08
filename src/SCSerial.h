/*
 * SCServo.h
 * 飞特串行舵机硬件接口层程序
 * 日期: 2018.8.2
 * 作者: 谭雄乐
 */

#ifndef _SCSERIAL_H
#define _SCSERIAL_H

#if defined(ARDUINO)
  #include "Arduino.h"
#else
  #include <stdio.h>
  #include <fcntl.h>
  #include <string.h>

  #if defined(_MSC_VER)
    #include <windows.h>
  #else
    #include <termios.h>
    #include <unistd.h>
  #endif
#endif

#include "SCS.h"

class SerialIO {
public:
    explicit SerialIO(unsigned long int IOTimeOut = 100) : IOTimeOut(IOTimeOut) {}
    virtual ~SerialIO() = default;

    static SerialIO* getSerialIO(unsigned long int IOTimeOut = 100);

    void setIOTimeOut(unsigned long int IOTimeOut) {
        this->IOTimeOut = IOTimeOut;
    }

    virtual int write(unsigned char *nDat, int nLen) = 0;//输出nLen字节
    virtual int read(unsigned char *nDat, int nLen) = 0;//输入nLen字节
    virtual int write(unsigned char bDat) = 0;//输出1字节
    virtual void flush() = 0;//刷新接口缓冲区

    virtual bool setBaudRate(int baudRate) = 0;
    virtual bool begin(int baudRate, const char* serialPort) = 0;

protected:
    unsigned long int IOTimeOut;//输入输出超时
};

class SCSerial : public SCS
{
public:
    SCSerial(SerialIO* pSerial);
    SCSerial(SerialIO* pSerial, u8 End);
    SCSerial(SerialIO* pSerial, u8 End, u8 Level);

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
    virtual int Recovery(u8 ID)															  {  return  0  ; }
    virtual int Reset(u8 ID)                                                              {  return  0  ; }
    virtual int UnLockEprom(u8 ID)                                                        {  return  0  ; }
    virtual int LockEprom(u8 ID)                                                          {  return  0  ; }
    virtual int WritePWM(u8 ID, s16 pwmOut)                                               {  return  0  ; }
    virtual int EnableTorque(u8 ID, u8 Enable)                                            {  return  0  ; }
    virtual void RegWriteAction()                                                         {  return     ; }
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
    virtual int readSCS(unsigned char *nDat, int nLen) { return pSerial->read(nDat, nLen); }
    virtual int writeSCS(unsigned char *nDat, int nLen) { return pSerial->write(nDat, nLen); }
    virtual int writeSCS(unsigned char bDat) {return pSerial->write(bDat); }
    virtual void flushSCS() { return pSerial->flush(); }

    SerialIO* pSerial;//串口指针
    int Err;
};


#if defined(ARDUINO)

class ArduinoSerial : public SerialIO {
public:
    ArduinoSerial(HardwareSerial *pSerial, unsigned long int IOTimeOut = 100) : pSerial(pSerial), SerialIO( IOTimeOut) {}

    virtual int write(unsigned char *nDat, int nLen);//输出nLen字节
    virtual int read(unsigned char *nDat, int nLen);//输入nLen字节
    virtual int write(unsigned char bDat);//输出1字节
    virtual void flush();//刷新接口缓冲区

    virtual bool setBaudRate(int baudRate);
    virtual bool begin(int baudRate, const char* serialPort);

protected:
    HardwareSerial *pSerial;//串口指针
};

#elif defined(_MSC_VER)

class WindowsSerial : public SerialIO {
public:
    WindowsSerial(unsigned long int IOTimeOut = 100) : SerialIO(IOTimeOut), serial_handle_(INVALID_HANDLE_VALUE) {}
    virtual ~WindowsSerial() { end(); }

    virtual int write(unsigned char *nDat, int nLen);//输出nLen字节
    virtual int read(unsigned char *nDat, int nLen);//输入nLen字节
    virtual int write(unsigned char bDat);//输出1字节
    virtual void flush();//刷新接口缓冲区

    virtual bool setBaudRate(int baudRate);
    virtual bool begin(int baudRate, const char* serialPort);
    virtual void end();

protected:
    HANDLE  serial_handle_;
};

#else

class LinuxSerial : public SerialIO {
public:
    LinuxSerial(unsigned long int IOTimeOut = 100) : SerialIO(IOTimeOut), fd(-1) {}
    virtual ~LinuxSerial() { end(); }

    virtual int write(unsigned char *nDat, int nLen);//输出nLen字节
    virtual int read(unsigned char *nDat, int nLen);//输入nLen字节
    virtual int write(unsigned char bDat);//输出1字节
    virtual void flush();//刷新接口缓冲区

    virtual bool setBaudRate(int baudRate);
    virtual bool begin(int baudRate, const char* serialPort);
    virtual void end();

protected:
    int fd;//serial port handle
    struct termios orgopt;//fd ort opt
    struct termios curopt;//fd cur opt
};

#endif

#endif