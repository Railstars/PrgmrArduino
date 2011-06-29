#ifndef __DCCSERVICEMODEQUEUE_H__
#define __DCCSERVICEMODEQUEUE_H__

#include "WProgram.h"

/**
 * A FIFO queue for holding DCC service mode packets.
 * Copyright 2011 D.E. Goodman-Wilson
**/

#include "DCCServiceModePacket.h"

#define QUEUE_SIZE 5 //TODO!!

class DCCServiceModeQueue
{
  private:
    DCCServiceModePacket _queue[QUEUE_SIZE]; //TODO!!
    byte _read_pos;
    byte _size;
  public:
    DCCServiceModeQueue(void);
    
    void setup(void);
    void clear(void) {_read_pos = _size = 0;}
    
    bool isEmpty(void) {return _read_pos == _size;}
        
    virtual bool insertPacket(DCCServiceModePacket &packet); //makes a local copy, does not take over memory management!
    virtual bool readPacket(DCCServiceModePacket &packet); //does not hand off memory management of packet. used immediately.
};

#endif //__DCCSERVICEMODEQUEUE_H__
