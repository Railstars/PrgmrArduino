#ifndef __DCCSERVICEMODESCHEDULER_H__
#define __DCCSERVICEMODESCHEDULER_H__
#include "DCCServiceModePacket.h"
#include "DCCServiceModeQueue.h"
#include "WProgram.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef CURRENT_SENSE_PIN
#define CURRENT_SENSE_PIN 3
#endif

#ifndef OVER_CURRENT_VALUE
#define OVER_CURRENT_VALUE 200
#endif

#ifndef ACK_THRESHOLD
#define ACK_THRESHOLD 100
#endif

enum physical_register_t
{
  address_register,
  start_voltage_register,
  acceleration_register,
  deceleration_register,
  basic_configuration_register,
  reserved_register,
  version_number_register,
  manufacturer_ID_register
};

enum programmer_mode_t
{
  idle_mode,
  power_on_initial_mode,
  power_on_mode,
  recovery_mode
};

class DCCServiceModeScheduler
{
  public:
  
    DCCServiceModeScheduler(void);
    
//    bool writeByte(uint16_t CV, uint8_t value);
//    uint8_t readByte(uint16_t CV);
//    bool writeAddress(uint_16t address);
//    uint_16t readAddress();
//    bool hardReset(void); //reset decoder to factory state.
    
    void setup(void); //for any post-constructor initialization

    //to be called periodically within loop()
    void update(void);

    //reset software
    void reset(void);
    
    
    bool checkACK(void) {return _wasACKd;}

    
//    bool checkSupportDirectMode(void);
    bool directModeWriteByte(uint16_t CV, uint8_t value);
//    bool directModeVerifyByte(uint16_t CV, uint8_t value);
//    bool directModeWriteBit(uint16_t CV, uint8_t bit, uint8_t value);
//    bool directModeVerifyBit(uint16_t CV, uint8_t bit, uint8_t value);

//    bool addressOnlyModeWriteAddress(uint8_t address);
//    bool addressOnlyModeVerifyAddress(uint8_t address);

//    bool physicalRegsiterModeWriteByte(physical_register_t register, uint8_t value);
//    bool physicalRegsiterModeVerifyByte(physical_register_t register, uint8_t value);
    
//    bool pagedRegisterModeWriteByte(uint16_t CV, uint8_t value);
//    bool pagedRegisterModeVerifyByte(uint16_t CV, uint8_t value);


  private:

    bool _accepts_direct_mode;
    bool _tested_direct_mode;
    bool _wasACKd;
    programmer_mode_t _mode;
    uint16_t _over_current_timer;
    uint16_t _ack_timer;
    DCCServiceModeQueue _power_on_queue;
    DCCServiceModeQueue _instruction_queue;
    DCCServiceModeQueue _conditional_queue;
};


#endif //__DCCSERVICEMODESCHEDULER_H__
