#if !defined(ARDUINO)
#include "SCServo.h"

int main(int argc, char* argv[]) {
#if !defined(_MSC_VER)
  LinuxSerial lxs;

  lxs.begin(115200, "/dev/ttyUSB0");

  SCSCL scscl(&lxs);
  SMSBL smbl(&lxs);
  SMSCL smcl(&lxs);
#endif
}

#endif