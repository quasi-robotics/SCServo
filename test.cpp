#if !defined(ARDUINO)
#include "SMSBL.h"

int main(int argc, char* argv[]) {
  LinuxSerial lxs;

  lxs.begin(115200, "/dev/ttyUSB0");
}

#endif