#include <stddef.h>
#include "SCS.h"

#if !defined(ARDUINO) && !defined(_MSC_VER)
  #include <fcntl.h>
  #include <sys/select.h>
#include <alloca.h>

#endif

SCS::SCS(SerialIO* pSerial)
{
    this->pSerial = pSerial;
	Level = 1;//除广播指令所有指令返回应答
	End = 0;//舵机处理器与控制板处理器端结构不一致
	Error = -1;
}

SCS::SCS(SerialIO* pSerial, u8 End)
{
    this->pSerial = pSerial;
	Level = 1;//除广播指令所有指令返回应答
	this->End = End;
	Error = -1;
}

SCS::SCS(SerialIO* pSerial, u8 End, u8 Level)
{
    this->pSerial = pSerial;
	this->Level = Level;
	this->End = End;
	Error = -1;
}

//1个16位数拆分为2个8位数
//DataL为低位，DataH为高位
void SCS::Host2SCS(u8 *DataL, u8* DataH, u16 Data)
{
	if(End){
		*DataL = (Data>>8);
		*DataH = (Data&0xff);
	}else{
		*DataH = (Data>>8);
		*DataL = (Data&0xff);
	}
}

//2个8位数组合为1个16位数
//DataL为低位，DataH为高位
u16 SCS::SCS2Host(u8 DataL, u8 DataH)
{
	u16 Data;
	if(End){
		Data = DataL;
		Data<<=8;
		Data |= DataH;
	}else{
		Data = DataH;
		Data<<=8;
		Data |= DataL;
	}
	return Data;
}

void SCS::writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun)
{
    std::vector<iovec> iov;
    iov.reserve(3);

	u8 msgLen = 2;
	u8 bBuf[6];
	u8 CheckSum = 0;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
	bBuf[4] = Fun;

	if(nDat){
		msgLen += nLen + 1;
		bBuf[3] = msgLen;
		bBuf[5] = MemAddr;
		//writeSCS(bBuf, 6);
        iov.push_back({bBuf, 6});
	}else{
		bBuf[3] = msgLen;
		//writeSCS(bBuf, 5);
        iov.push_back({bBuf, 5});
	}
	CheckSum = ID + msgLen + Fun + MemAddr;
	u8 i = 0;
	if(nDat){
		for(i=0; i<nLen; i++)
			CheckSum += nDat[i];

		//writeSCS(nDat, nLen);
        iov.push_back({nDat, nLen});
	}
    CheckSum = ~CheckSum;
	//writeSCS(CheckSum);
    iov.push_back({&CheckSum, 1});

    writeSCS(iov);
}

//普通写指令
//舵机ID，MemAddr内存表地址，写入数据，写入长度
int SCS::genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen)
{
	flushSCS();
	writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE);
	return Ack(ID);
}

//异步写指令
//舵机ID，MemAddr内存表地址，写入数据，写入长度
int SCS::regWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen)
{
	flushSCS();
	writeBuf(ID, MemAddr, nDat, nLen, INST_REG_WRITE);
	return Ack(ID);
}

//同步写指令
//舵机ID[]数组，IDN数组长度，MemAddr内存表地址，写入数据，写入长度
void SCS::syncWrite(const u8 *ID, u8 IDN, u8 MemAddr, const u8 *nDat, u8 nLen)
{
	u8 mesLen = ((nLen+1)*IDN+4);
	u8 Sum = 0;
	u8 bBuf[7];
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;
	bBuf[3] = mesLen;
	bBuf[4] = INST_SYNC_WRITE;
	bBuf[5] = MemAddr;
	bBuf[6] = nLen;
	writeSCS(bBuf, 7);

	u8* mesbuf = static_cast<u8*>(alloca(mesLen - 3));

	Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;
	u8 i, j;
	for(i=0; i<IDN; i++){
	    memcpy(mesbuf + (nLen+1)*i, &ID[i], 1);
		//writeSCS(ID[i]);
        memcpy(mesbuf + (nLen+1)*i + 1, nDat+i*nLen, nLen);
		//writeSCS(nDat+i*nLen, nLen);
		Sum += ID[i];
		for(j=0; j<nLen; j++){
			Sum += nDat[i*nLen+j];
		}
	}
	Sum = ~Sum;
    memcpy(mesbuf + (nLen+1)*IDN, &Sum, 1);
	//writeSCS(Sum);

	writeSCS(mesbuf, mesLen - 3);
}

int SCS::writeByte(u8 ID, u8 MemAddr, u8 bDat)
{
	flushSCS();
	writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
	return Ack(ID);
}

int SCS::writeWord(u8 ID, u8 MemAddr, u16 wDat)
{
	flushSCS();
	u8 buf[2];
	Host2SCS(buf+0, buf+1, wDat);
	writeBuf(ID, MemAddr, buf, 2, INST_WRITE);
	return Ack(ID);
}

//读指令
//舵机ID，MemAddr内存表地址，返回数据nData，数据长度nLen
int SCS::Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen)
{
	flushSCS();
	writeBuf(ID, MemAddr, &nLen, 1, INST_READ);
	u8 bBuf[6];
	if(readSCS(bBuf, 5)!=5){
		Error = -1;
		return 0;
	}
	if(bBuf[0]!=0xff || bBuf[1]!=0xff){
		Error = -1;
		return -1;		
	}
	int Size = readSCS(nData, nLen);

	if(readSCS(bBuf+5, 1)!=1){
		Error = -1;
		return 0;
	}
	u8 calSum = bBuf[2]+bBuf[3]+bBuf[4];
	u8 i;
	for(i=0; i<Size; i++){
		calSum += nData[i];
	}
	calSum = ~calSum;
	if(calSum!=bBuf[5]){
		Error = -1;
		return 0;
	}
	Error = bBuf[4];
	return Size;
}

//读1字节，超时返回-1
int SCS::readByte(u8 ID, u8 MemAddr)
{
	u8 bDat;
	int Size = Read(ID, MemAddr, &bDat, 1);
	if(Size!=1){
		return -1;
	}else{
		return bDat;
	}
}

//读2字节，超时返回-1
int SCS::readWord(u8 ID, u8 MemAddr)
{	
	u8 nDat[2];
	int Size;
	u16 wDat;
	Size = Read(ID, MemAddr, nDat, 2);
	if(Size!=2)
		return -1;
	wDat = SCS2Host(nDat[0], nDat[1]);
	return wDat;
}

int	SCS::Ack(u8 ID)
{
	Error = 0;
	if(ID != FEETECH_BROADCAST_ID && Level){
		u8 bBuf[6];
		u8 Size = readSCS(bBuf, 6);
		if(Size!=6){
			Error = -1;
			return 0;
		}
		if(bBuf[0]!=0xff || bBuf[1]!=0xff){
			Error = -1;
			return -1;		
		}
		u8 calSum = ~(bBuf[2]+bBuf[3]+bBuf[4]);
		if(calSum!=bBuf[5]){
			Error = -1;
			return -1;			
		}
		Error = bBuf[4];
	}
	return 1;
}

//Ping指令，返回舵机ID，超时返回-1
int	SCS::Ping(u8 ID)
{
	u8 bBuf[6];
	int Size;
	flushSCS();
	writeBuf(ID, 0, NULL, 0, INST_PING);
	Size = readSCS(bBuf, 6);
	if(Size!=6){
		Error = Size;
		return -1;
	}
	if(bBuf[0]!=0xff || bBuf[1]!=0xff){
		Error = -2;
		return -1;		
	}
	u8 calSum = ~(bBuf[2]+bBuf[3]+bBuf[4]);
	if(calSum!=bBuf[5]){
		Error = -3;
		return -1;			
	}
	Error = bBuf[4];
	return bBuf[2];
}

//复位舵机参数为默认值
int SCS::Recovery(u8 ID)
{
    flushSCS();
    writeBuf(ID, 0, NULL, 0, INST_RECOVER);
    return Ack(ID);
}

//复位舵机
int SCS::Reset(u8 ID)
{
    flushSCS();
    writeBuf(ID, 0, NULL, 0, INST_RESET);
    return Ack(ID);
}

void SCS::RegWriteAction()
{
    writeBuf(0xfe, 0, NULL, 0, INST_ACTION);
}





# if defined(ARDUINO)
// ArduinoSerial

bool WindowsSerial::begin(int baudrate, const char* port_name)
{
    pSerial->begin(baudrate);
}

bool WindowsSerial::setBaudRate(int baudrate)
{
    pSerial->end();
    pSerial->begin(baudrate);
}

int ArduinoSerial::read(unsigned char *nDat, int nLen)
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

int ArduinoSerial::write(const unsigned char *nDat, int nLen)
{
  if(nDat==NULL){
    return 0;
  }
  return pSerial->write(nDat, nLen);
}

int ArduinoSerial::write(unsigned char bDat)
{
  return pSerial->write(&bDat, 1);
}

void ArduinoSerial::flush()
{
  while(pSerial->read()!=-1);
}
#elif defined(_MSC_VER)

bool WindowsSerial::begin(int baudrate, const char* port_name)
{
    COMMTIMEOUTS timeouts;
    DWORD dwError;

    end();

    char port_name_[15];
    sprintf_s(port_name_, sizeof(port_name_), "\\\\.\\%s", port_name);

    serial_handle_ = CreateFileA(port_name_, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (serial_handle_ == INVALID_HANDLE_VALUE)
    {
        return false;
    }

    if( !setBaudRate(baudrate) )
        goto DXL_HAL_OPEN_ERROR;

    if (SetCommMask(serial_handle_, 0) == FALSE) // Not using Comm event
        goto DXL_HAL_OPEN_ERROR;
    if (SetupComm(serial_handle_, 4096, 4096) == FALSE) // Buffer size (Rx,Tx)
        goto DXL_HAL_OPEN_ERROR;
    if (PurgeComm(serial_handle_, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR) == FALSE) // Clear buffer
        goto DXL_HAL_OPEN_ERROR;
    if (ClearCommError(serial_handle_, &dwError, NULL) == FALSE)
        goto DXL_HAL_OPEN_ERROR;

    if (GetCommTimeouts(serial_handle_, &timeouts) == FALSE)
        goto DXL_HAL_OPEN_ERROR;
    // Timeout (Not using timeout)
    // Immediatly return
    timeouts.ReadIntervalTimeout = 0;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    timeouts.ReadTotalTimeoutConstant = IOTimeOut; // must not be zero.
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 0;
    if (SetCommTimeouts(serial_handle_, &timeouts) == FALSE)
        goto DXL_HAL_OPEN_ERROR;

    //tx_time_per_byte_ = (1000.0 / (double)baudrate_) * 10.0;
    return true;

  DXL_HAL_OPEN_ERROR:
    end();
    return false;
}

bool WindowsSerial::setBaudRate(int baudrate)
{
    DCB dcb;

    dcb.DCBlength = sizeof(DCB);
    if (GetCommState(serial_handle_, &dcb) == FALSE)
        return false;

    // Set baudrate
    dcb.BaudRate = (DWORD)baudrate;
    dcb.ByteSize = 8;                    // Data bit = 8bit
    dcb.Parity = NOPARITY;             // No parity
    dcb.StopBits = ONESTOPBIT;           // Stop bit = 1
    dcb.fParity = NOPARITY;             // No Parity check
    dcb.fBinary = 1;                    // Binary mode
    dcb.fNull = 0;                    // Get Null byte
    dcb.fAbortOnError = 0;
    dcb.fErrorChar = 0;
    // Not using XOn/XOff
    dcb.fOutX = 0;
    dcb.fInX = 0;
    // Not using H/W flow control
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fDsrSensitivity = 0;
    dcb.fOutxDsrFlow = 0;
    dcb.fOutxCtsFlow = 0;

    return SetCommState(serial_handle_, &dcb);
}

void WindowsSerial::end()
{
    if (serial_handle_ != INVALID_HANDLE_VALUE)
    {
        CloseHandle(serial_handle_);
        serial_handle_ = INVALID_HANDLE_VALUE;
    }
}

int WindowsSerial::read(unsigned char *nDat, int nLen) {
    DWORD dwRead = 0;

    if (ReadFile(serial_handle_, nDat, (DWORD)nLen, &dwRead, NULL) == FALSE)
        return -1;

    return (int)dwRead;
}

int WindowsSerial::write(const unsigned char *nDat, int nLen) {
    DWORD dwWrite = 0;

    if (WriteFile(serial_handle_, nDat, (DWORD)nLen, &dwWrite, NULL) == FALSE)
        return -1;

    return (int)dwWrite;
}

int WindowsSerial::write(unsigned char bDat) {
    if (WriteFile(serial_handle_, &bDat, (DWORD)1, NULL, NULL) == FALSE)
        return -1;

    return 1;
}

int WindowsSerial::writev(const std::vector<iovec>& iovs) {
    // Unfortunately Scatter/Gather IO in Windows does not work with serial ports
    for( const auto& iov : iovs ) {
        if( WriteFile(serial_handle_, iov.iov_base, (DWORD)iov.iov_len, NULL, NULL) == FALSE)
            return -1;
    }

    return 1;
}

void WindowsSerial::flush() {
    PurgeComm(serial_handle_, PURGE_RXABORT | PURGE_RXCLEAR);
}

#elif defined(__linux__)
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <sys/errno.h>

// LinuxSerial

bool LinuxSerial::begin(int baudRate, const char* serialPort)
{
  if(fd != -1){
    close(fd);
    fd = -1;
  }
  //printf("servo port:%s\n", serialPort);
  if(serialPort == NULL)
    return false;
  fd = open(serialPort, O_RDWR | O_NOCTTY /*| O_NONBLOCK*/);
  if(fd == -1){
    //perror("open:");
    return false;
  }

//  fcntl(fd, F_SETFL, O_NONBLOCK);

  return setBaudRate(baudRate);
}

bool LinuxSerial::setBaudRate(int baudRate)
{
  if(fd==-1){
    return false;
  }
  tcgetattr(fd, &curopt);
  speed_t CR_BAUDRATE;
  switch(baudRate){
    case 9600:
      CR_BAUDRATE = B9600;
      break;
    case 19200:
      CR_BAUDRATE = B19200;
      break;
    case 38400:
      CR_BAUDRATE = B38400;
      break;
    case 57600:
      CR_BAUDRATE = B57600;
      break;
    case 115200:
      CR_BAUDRATE = B115200;
      break;
    case 230400:
      CR_BAUDRATE = B230400;
      break;
    case 460800:
      CR_BAUDRATE = B460800;
      break;
#if defined(B500000)
    case 500000:
      CR_BAUDRATE = B500000;
      break;
#endif
#if defined(B1000000)
    case 1000000:
      CR_BAUDRATE = B1000000;
      break;
#endif
    default:
      return false;
  }
  cfsetispeed(&curopt, CR_BAUDRATE);
  cfsetospeed(&curopt, CR_BAUDRATE);

  //printf("serial speed %d\n", baudRate);
  //Mostly 8N1
  curopt.c_cflag &= ~PARENB;
  curopt.c_cflag &= ~CSTOPB;
  curopt.c_cflag &= ~CSIZE;
  curopt.c_cflag |= CS8;
  curopt.c_cflag |= CREAD;
  curopt.c_cflag |= CLOCAL;//disable modem status check
  cfmakeraw(&curopt);//make raw mode
  curopt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  curopt.c_cc[VTIME] = (IOTimeOut > 0 && IOTimeOut < 100) ? 1 : IOTimeOut / 100;
  curopt.c_cc[VMIN] = 0;

  if(tcsetattr(fd, TCSANOW, &curopt) != 0)
    return false;

  struct serial_struct serinfo;
  serinfo.reserved_char[0] = 0;
  if( ioctl(fd, TIOCGSERIAL, &serinfo) >= 0 ) {
      serinfo.flags |= ASYNC_LOW_LATENCY;
      if( ioctl(fd, TIOCSSERIAL, &serinfo) < 0 )
          return false;
  }
  else if( errno != ENOTTY )
      return false;

  return true;
}

void LinuxSerial::end()
{
  if(fd != -1)
    close(fd);
  fd = -1;
}

int LinuxSerial::read(unsigned char *nDat, int nLen) {
    int br = 0;

    while( br < nLen ) {
        int n = ::read(fd, nDat + br, nLen - br);
        if( n < 0 )         // error
            return n;
        else if( n == 0 )   // timeout, br is all we got
            return br;
        else
            br += n;
    }
    return br;
}

int LinuxSerial::write(const unsigned char *nDat, int nLen) {
    return ::write(fd, nDat, nLen);
}

int LinuxSerial::write(unsigned char bDat) {
    return ::write(fd, &bDat, 1);
}

int LinuxSerial::writev(const std::vector<iovec>& iov) {
    return ::writev(fd, iov.data(), iov.size());
}

void LinuxSerial::flush() {
  tcflush(fd, TCIFLUSH);
}
#endif

SerialIO *SerialIO::getSerialIO(unsigned long int IOTimeOut) {
#if defined(__linux__)
    return new LinuxSerial(IOTimeOut);
//#elif defined(__APPLE__)
//    return (PortHandler *)(new PortHandlerMac(port_name));
#elif defined(_WIN32) || defined(_WIN64)
    return new WindowsSerial(IOTimeOut);
#endif
}
