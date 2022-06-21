#ifndef _SCS_H
#define _SCS_H

#if defined(ARDUINO)
    #include "Arduino.h"

    struct iovec {
        void  *iov_base;    /* Starting address */
        size_t iov_len;     /* Number of bytes to transfer */
    };
#else
    #include <stdio.h>
    #include <fcntl.h>
    #include <string.h>

    #if defined(_MSC_VER)
        #include <windows.h>

        struct iovec {
            void  *iov_base;    /* Starting address */
            size_t iov_len;     /* Number of bytes to transfer */
        };

    #else
        #include <termios.h>
        #include <unistd.h>

        #if defined(__linux__)
            #include <sys/uio.h>
        #endif
    #endif
#endif

#include <stddef.h>
#include "INST.h"


class SerialIO {
public:
    explicit SerialIO(unsigned long int IOTimeOut = 100) : IOTimeOut(IOTimeOut) {}
    virtual ~SerialIO() = default;

    static SerialIO* getSerialIO(unsigned long int IOTimeOut = 100);

    void setIOTimeOut(unsigned long int IOTimeOut_) {
        IOTimeOut = IOTimeOut_;
    }

    virtual int read(unsigned char *nDat, int nLen) = 0;//输入nLen字节
    virtual int write(const unsigned char *nDat, int nLen) = 0;//输出nLen字节
    virtual int write(unsigned char bDat) = 0;//输出1字节
    virtual int writev(const iovec* iov, size_t n) = 0;
    virtual void flush() = 0;//刷新接口缓冲区

    virtual bool setBaudRate(int baudRate) = 0;
    virtual bool begin(int baudRate, const char* serialPort) = 0;

protected:
    unsigned long int IOTimeOut;//输入输出超时
};

class SCS{
public:
	SCS(SerialIO* pSerial);
	SCS(SerialIO* pSerial, u8 End);
	SCS(SerialIO* pSerial, u8 End, u8 Level);
	
	int genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);//普通写指令
	int regWrite(u8 ID, u8 MemAddr,u8 *nDat, u8 nLen);//异步写指令
	void syncWrite(const u8 *ID, u8 IDN, u8 MemAddr, const u8 *nDat, u8 nLen);//同步写指令
	int writeByte(u8 ID, u8 MemAddr, u8 bDat);//写1个字节
	int writeWord(u8 ID, u8 MemAddr, u16 wDat);//写2个字节
	int Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen);//读指令
	int readByte(u8 ID, u8 MemAddr);//读1个字节
	int readWord(u8 ID, u8 MemAddr);//读2个字节
	int Ping(u8 ID);//Ping指令
    int Recovery(u8 ID);//复位舵机参数为默认值
    int Reset(u8 ID);//复位舵机
    void RegWriteAction();//执行异步写指令

public:
	u8	Level;//舵机返回等级
	u8	End;//处理器大小端结构
	s8	Error;

public:
    int readSCS(u8 *nDat, int nLen) { return pSerial->read(nDat, nLen); }
    int writeSCS(const u8 *nDat, int nLen) { return pSerial->write(nDat, nLen); }
    int writeSCS(u8 bDat) {return pSerial->write(bDat); }
    int writeSCS(const iovec* iov, size_t n) {return pSerial->writev(iov, n);}
    void flushSCS() { return pSerial->flush(); }

    SerialIO* pSerial;//串口指针

	void writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun);
	void Host2SCS(u8 *DataL, u8* DataH, u16 Data);//1个16位数拆分为2个8位数
	u16	SCS2Host(u8 DataL, u8 DataH);//2个8位数组合为1个16位数
	int	Ack(u8 ID);//返回应答
};

#if defined(ARDUINO)

class ArduinoSerial : public SerialIO {
public:
    ArduinoSerial(HardwareSerial *pSerial, unsigned long int IOTimeOut = 100, int dir_pin = -1) : 
        pSerial(pSerial), SerialIO( IOTimeOut), dir_pin_(dir_pin) {}

    virtual int write(const unsigned char *nDat, int nLen);//输出nLen字节
    virtual int read(unsigned char *nDat, int nLen);//输入nLen字节
    virtual int write(unsigned char bDat);//输出1字节
    int writev(const iovec* iov, size_t n) override;
    virtual void flush();//刷新接口缓冲区

    virtual bool setBaudRate(int baudRate);
    virtual bool begin(int baudRate, const char* serialPort = nullptr);

protected:
    HardwareSerial *pSerial;//串口指针
    int dir_pin_;

    friend class DirectionControl;
};

#elif defined(_MSC_VER)

class WindowsSerial : public SerialIO {
public:
    WindowsSerial(unsigned long int IOTimeOut = 100) : SerialIO(IOTimeOut), serial_handle_(INVALID_HANDLE_VALUE) {}
    ~WindowsSerial() override { end(); }

    int read(unsigned char *nDat, int nLen) override;
    int write(const unsigned char *nDat, int nLen) override;
    int write(unsigned char bDat) override;
    int writev(const iovec* iov, size_t n) override;
    void flush() override;

    bool setBaudRate(int baudRate) override;
    bool begin(int baudRate, const char* serialPort) override;
    void end();

protected:
    HANDLE  serial_handle_;
};

#else

class LinuxSerial : public SerialIO {
public:
    LinuxSerial(unsigned long int IOTimeOut = 100) : SerialIO(IOTimeOut), fd(-1) {}
    ~LinuxSerial() override { end(); }

    int read(unsigned char *nDat, int nLen) override;

    int write(const unsigned char *nDat, int nLen);
    int write(unsigned char bDat) override;
    int writev(const iovec* iov, size_t n) override;

    void flush() override;

    bool setBaudRate(int baudRate) override;
    bool begin(int baudRate, const char* serialPort) override;
    void end();

protected:
    int fd;//serial port handle
    struct termios curopt;//fd cur opt
};

#endif


#endif
