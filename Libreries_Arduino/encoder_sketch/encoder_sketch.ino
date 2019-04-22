#include <Encoder.h>

Encoder leftEnc(18, 19);
Encoder rightEnc(2, 3);

long rightPulses, leftPulses =0;

void setup() 
{
   pinMode(2, INPUT);
   pinMode(3, INPUT);
   pinMode(18, INPUT);
   pinMode(19, INPUT);
}

void loop() 
  {
      rightPulses = rightEnc.read();
      leftPulses  = leftEnc.read();
  }
