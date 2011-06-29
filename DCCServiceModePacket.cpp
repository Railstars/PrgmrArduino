#include "DCCServiceModePacket.h"


DCCServiceModePacket::DCCServiceModePacket(): _size_repeat(0x40) //size(1), repeat(0)
{
  _data[0] = 0xFF; //default to idle packet
  _data[1] = 0x00;
}

uint8_t DCCServiceModePacket::getBitstream(uint8_t rawuint8_ts[]) //returns size of array.
{
  uint8_t i;
  uint8_t XOR = 0;
  for(i = 0; i < (_size_repeat>>6); ++i)
  {
    rawuint8_ts[i] = _data[i];
    XOR ^= _data[i];
  }

  rawuint8_ts[i] = XOR;
  return i;  
}

uint8_t DCCServiceModePacket::getSize(void)
{
  return (_size_repeat>>6);
}

void DCCServiceModePacket::setData(uint8_t value)
{
  _data[0] = value;
  _size_repeat = (_size_repeat & 0x3F) | (1<<6);
}

void DCCServiceModePacket::setData(uint8_t value1, uint8_t value2)
{
  _data[0] = value1;
  _data[1] = value2;
  _size_repeat = (_size_repeat & 0x3F) | (2<<6);
}

void DCCServiceModePacket::setData(uint8_t value1, uint8_t value2, uint8_t value3)
{
  _data[0] = value1;
  _data[1] = value2;
  _data[2] = value3;
  _size_repeat = (_size_repeat & 0x3F) | (3<<6);
}

void DCCServiceModePacket::makeReset(void)
{
  _data[0] = 0;
  _data[1] = 0;
  _size_repeat = (2<<6);
}