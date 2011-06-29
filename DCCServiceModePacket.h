#ifndef __DCCSERVICEMODEPACKET_H__
#define __DCCSERVICEMODEPACKET_H__

#include "WProgram.h"

typedef unsigned char byte;


class DCCServiceModePacket
{
  public:
    DCCServiceModePacket();
    
    uint8_t getBitstream(byte rawbytes[]); //returns size of array.
    uint8_t getSize(void);
    void setData(uint8_t value);
    void setData(uint8_t value1, uint8_t value2);
    void setData(uint8_t value1, uint8_t value2, uint8_t value3);

    inline void setRepeat(byte new_repeat) { _size_repeat = (_size_repeat&0xC0 | new_repeat&0x3F) ;}
    inline uint8_t getRepeat(void) { return _size_repeat & 0x3F; }//return repeat; }

    void makeReset(void);

  private:
   //A DCC service mode packet is at most 4 bytes: 3 of data, one of XOR
    byte _data[4];
    byte _size_repeat;  //a bit field! 0b11000000 = 0xC0 = size; 0x00111111 = 0x3F = repeat
};

#endif //__DCCSERVICEMODEPACKET_H__
