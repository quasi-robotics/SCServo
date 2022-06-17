#include <SCServo.h>

ArduinoSerial scSerial(&Serial1, 100);
SMSBL sm(&scSerial);

void setup()
{
  Serial1.begin(115200);
  Serial.begin(115200);
  sm.WritePos(1, 0, 50);
  delay(2000);
}

void loop()
{
  s16 pos = sm.ReadPos(1);
  Serial.println(pos, DEC);
  sm.WritePos(1, 4095, 50, 0, 5);//SM舵机(ID0)以最高50速度(50*0.737=36rpm)，5(5*0.878=4.93度/秒^2)加速度，运行至4095位置
  delay(6000); 
  pos = sm.ReadPos(1);
  Serial.println(pos, DEC);
  sm.WritePos(1, 0, 50, 0, 5);//SM舵机(ID0)以最高50速度(50*0.737=36rpm)，5(5*0.878=4.93度/秒^2)加速度，运行至0位置
  delay(6000);
}