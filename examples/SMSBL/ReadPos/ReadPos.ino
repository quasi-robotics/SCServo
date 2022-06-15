#include <SCServo.h>

ArduinoSerial scSerial(&Serial1, 100);
SMSBL sms(&scSerial);
int LEDpin = 13;
void setup()
{
	pinMode(LEDpin,OUTPUT);
	digitalWrite(LEDpin, HIGH);
	Serial1.begin(115200);
	Serial.begin(115200);
}

void loop()
{
	s16 pos = sms.ReadPos(1);
	if(sms.getErr()==0)
	{
		digitalWrite(LEDpin, LOW);
		Serial.println(pos, DEC);
    delay(10);
	}
	else
	{
    digitalWrite(LEDpin, HIGH);
    delay(2000);
	}
}