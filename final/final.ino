#include <ModbusMaster.h>     // Arduino library for communicating with Modbus slaves over RS485

#define MAX485_DE      3
#define MAX485_RE_NEG  2
#define LED 13

ModbusMaster node;

const double Pi = 3.141592;
double radius { 6.3 };        // Radius of Wheels
double D { 40 };              // distance between two wheels

uint16_t result;

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}
void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}


void setup() {

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(115200);

  node.begin(1, Serial);

  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);


  node.writeSingleRegister(0x200e, 0x08);     // Enable

  node.writeSingleRegister(0x200e, 0x10);     // Start

}

void loop() {

  TurnInPlace(50, Pi / 2 , 1);

  MoveStraight(80, 150);

  OneWheelTurn(50, Pi / 2, 1);

  MoveStraight(50, 320);

  wait(8000);

  TurnInPlace(40, Pi / 2, 1);

  MoveStraight(50, 120);
  
  OneWheelTurn(50, Pi / 2, 1);

  wait(10000);

  MoveStraight(30, 320);

  node.writeSingleRegister(0x200e, 0x07);     // Stop
  
  while(1);
}


double RPMtoMPS (double velocity)
// convert RPM to Meter per second
{
  return ( (2.0 * Pi * radius * velocity ) / 60.0);
}

void MoveStraight(uint16_t velocity, uint16_t distance )   
/* velocity (rpm) , distance (cm)
This function Moves forward until that distance but does not end up with stop comand
*/
{
  double time = (double)distance / abs(RPMtoMPS((double)velocity));
  node.clearTransmitBuffer();
  node.setTransmitBuffer(0, -velocity);
  node.setTransmitBuffer(1, velocity);
  node.writeMultipleRegisters(0x2088, 2);
  delay(time * 1000);
}

void TurnInPlace(uint16_t velocity, double angle, bool direction)  
/* direction 0 turns right and 1 turns left
   angle should be radian
   This function turns at center of robot, wheels move opposite direction
*/
{ 
  double distance { angle * (D / 2) };
  double time = distance / abs(RPMtoMPS((double)velocity));
  node.clearTransmitBuffer();
  if ( direction == 0)
  {
    node.setTransmitBuffer(0, -velocity);
    node.setTransmitBuffer(1, -velocity);    
  }
  else
  {
    node.setTransmitBuffer(0, velocity);
    node.setTransmitBuffer(1, velocity);
  }
  node.writeMultipleRegisters(0x2088, 2);
  
  delay(time * 1000);

}


void TurnInPlace2(uint16_t velocity, double angle, bool direction)  //direction 0 turns right and 1 turns left
//  use this function when speed is high
{ 
  uint16_t LS, RS;
  double distance { angle * (D / 2) };
  double time = distance / abs(RPMtoMPS((double)velocity));
  node.clearResponseBuffer();
  result = node.readHoldingRegisters(0x20AB, 2);
  if(result == node.ku8MBSuccess)
  {
    LS = node.getResponseBuffer(0);
    RS = node.getResponseBuffer(1);
  }
  node.clearTransmitBuffer();
  if ( direction == 0)
  {
    node.setTransmitBuffer(0, -(velocity + 3 * LS / 100));
    node.setTransmitBuffer(1, -velocity);    
  }
  else
  {
    node.setTransmitBuffer(0, velocity);
    node.setTransmitBuffer(1, velocity + 3 * RS / 100);
  }
  node.writeMultipleRegisters(0x2088, 2);
  
  delay(time * 1000);

}

void OneWheelTurn(int16_t velocity, double angle, bool direction)  
/*  velocity is RPM , angle should be radian, direction 0 turns right and 1 turns left
  one wheel of robot is stopped, one is moveing so turns at outside center
*/
{
  double distance { D * angle };
  double time = distance / abs(RPMtoMPS((double)velocity));
  node.clearTransmitBuffer();
  if(direction == 0)
  {
    node.setTransmitBuffer(0, -velocity);              
    node.setTransmitBuffer(1, 0);
  }
  else
  {
    node.setTransmitBuffer(0, 0);          
    node.setTransmitBuffer(1, velocity);
  }
  node.writeMultipleRegisters(0x2088, 2);
  
  delay(time* 1.07 * 1000);

}

void wait(unsigned long time)
{
  node.clearTransmitBuffer();
  node.setTransmitBuffer(0, 0);   
  node.setTransmitBuffer(1, 0);
  node.writeMultipleRegisters(0x2088, 2);
  delay(time);
}

