#if !defined(ARDUINO)
#include "SCServo.h"

int main(int argc, char* argv[]) {
  LinuxSerial lxs;

  lxs.begin(115200, "/dev/ttyUSB0");

  SCSCL scscl(&lxs);
  SMSBL smbl(&lxs);
  SMSCL smcl(&lxs);
}

#endif