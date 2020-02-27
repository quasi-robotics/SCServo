/*
 * SCServo.cpp
 * 飞特串行舵机硬件接口层程序
 * 日期: 2017.8.22
 * 作者: 谭雄乐
 */

#include "SCSerial.h"

#if !defined(ARDUINO)
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

int ArduinoSerial::write(unsigned char *nDat, int nLen)
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
#else
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
    perror("open:");
    return false;
  }
  fcntl(fd, F_SETFL, _FNDELAY);

  return setBaudRate(baudRate);
}

bool LinuxSerial::setBaudRate(int baudRate)
{
  if(fd==-1){
    return false;
  }
  tcgetattr(fd, &orgopt);
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
  if(tcsetattr(fd, TCSANOW, &curopt) == 0){
    return true;
  }else{
    perror("tcsetattr:");
    return false;
  }
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

int LinuxSerial::write(unsigned char *nDat, int nLen) {
  return ::write(fd, nDat, nLen);
}

int LinuxSerial::write(unsigned char bDat) {
  return ::write(fd, &bDat, 1);
}

void LinuxSerial::flush() {
  tcflush(fd, TCIFLUSH);
}
#endif