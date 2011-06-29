#include "DCCServiceModeQueue.h"

DCCServiceModeQueue::DCCServiceModeQueue(void) : _read_pos(0), _size(0)
{
  return;
}

void DCCServiceModeQueue::setup(void)
{
//  for(uint8_t i = 0; i<QUEUE_SIZE; ++i)
//  {
//    _queue[i].setup();
//  }
}

bool DCCServiceModeQueue::insertPacket(DCCServiceModePacket &packet)
{
  if(_size == QUEUE_SIZE) //queue is already full!
    return false;
    
  if(!packet.getRepeat()) //zero repeat set
    return false;
    
  memcpy(&_queue[_size],&packet,sizeof(DCCServiceModePacket));
  ++_size;
  return true;
}

// void DCCServiceModeQueue::printQueue(void)
// {
//   byte i, j;
//   for(i = 0; i < size; ++i)
//   {
//     for(j = 0; j < (queue[i].size_repeat>>4); ++j)
//     {
//       Serial.print(queue[i].data[j],BIN);
//       Serial.print(" ");
//     }
//     if(i == _read_pos) Serial.println("   r");
//     else if(i == write_pos) Serial.println("    w");
//     else Serial.println("");
//   }
// }

/* Goes through each packet in the queue, repeats it getRepeat() times, and discards it */
bool DCCServiceModeQueue::readPacket(DCCServiceModePacket &packet)
{
  if(isEmpty())
    return false;

  if(_queue[_read_pos].getRepeat()) //if the topmost packet needs repeating
  {
    _queue[_read_pos].setRepeat(_queue[_read_pos].getRepeat()-1); //decrement the current packet's repeat count
  }
  else //the topmost packet is ready to be discarded; use the DCCServiceModeQueue mechanism
  {
    ++_read_pos;
  }

  if(!isEmpty()) //anything remaining in the queue?
  {
    memcpy(&packet,&_queue[_read_pos],sizeof(DCCServiceModePacket));
    return true;
  }
  
  return false;
}