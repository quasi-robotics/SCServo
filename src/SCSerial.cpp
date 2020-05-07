/*
 * SCServo.cpp
 * 飞特串行舵机硬件接口层程序
 * 日期: 2017.8.22
 * 作者: 谭雄乐
 */

#include "SCSerial.h"

#if !defined(ARDUINO) && !defined(_MSC_VER)
  #include <fcntl.h>
  #include <sys/select.h>
#endif

SCSerial::SCSerial(SerialIO* pSerial)
{
	this->pSerial = pSerial;
}

SCSerial::SCSerial(SerialIO* pSerial, u8 End): SCS(End)
{
  this->pSerial = pSerial;
}

SCSerial::SCSerial(SerialIO* pSerial, u8 End, u8 Level): SCS(End, Level)
{
  this->pSerial = pSerial;
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
  fd = open(serialPort, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if(fd == -1){
    //perror("open:");
    return false;
  }

  fcntl(fd, F_SETFL, O_NONBLOCK);

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
  int fs_sel;
  fd_set fs_read;

  struct timeval time;

  FD_ZERO(&fs_read);
  FD_SET(fd,&fs_read);

  time.tv_sec = 0;
  time.tv_usec = IOTimeOut*1000;

  //使用select实现串口的多路通信
  fs_sel = select(fd+1, &fs_read, NULL, NULL, &time);
  if(fs_sel){
    fs_sel = ::read(fd, nDat, nLen);
    //printf("nLen = %d fs_sel = %d\n", nLen, fs_sel);
    return fs_sel;
  }else{
    //printf("serial read fd read return 0\n");
    return 0;
  }
}

int LinuxSerial::write(const unsigned char *nDat, int nLen) {
  return ::write(fd, nDat, nLen);
}

int LinuxSerial::write(unsigned char bDat) {
  return ::write(fd, &bDat, 1);
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
