#include <DCCServiceModePacket.h>
#include <DCCServiceModeQueue.h>
#include <DCCServiceModeScheduler.h>

/********************
* Creates a minimum DCC command station from a potentiometer connected to analog pin 0,
* and a button connected to ground on one end and digital pin 4 on the other end. See this link
* http://www.arduino.cc/en/Tutorial/AnalogInput
* The DCC waveform is output on Pin 9, and is suitable for connection to an LMD18200-based booster directly,
* or to a single-ended-to-differential driver, to connect with most other kinds of boosters.
********************/

DCCServiceModeScheduler dps;

void setup()
{
  Serial.begin(115200);
  Serial.println("Hello!");
  
  dps.setup();
  
  ////////
  
  dps.directModeWriteByte(29,6); //a safe value for CV 29
}

void loop()
{
  dps.update();
}
