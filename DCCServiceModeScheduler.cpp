#include "DCCServiceModeScheduler.h"

/*
 * DCC Service Mode Waveform Generator
 *
 * 
 * Hardware requirements:
 *     *An h-bridge with inputs wired as indicated below, and outputs to a programming track.
 *     *A current sensor on the h-bridge, wired to the analog input as below.
 *     *A locomotive with a decoder installed, reset to factory defaults.
 *
 * Author: D.E. Goodman-Wilson dgoodman@railstars.com
 * Website: http://railstars.com/software/PrgmrArduino/
 *
 * Copyright 2011 Don Goodman-Wilson
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *  
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *  
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *  
 */

/*****************************************************************/


///An enumerated type for keeping track of the state machine used in the timer2 ISR
enum DCC_output_state_t {
  dos_idle,
  dos_send_preamble,
  dos_send_bstart,
  dos_send_byte,
  dos_end_bit
};

DCC_output_state_t DCC_state = dos_idle; //just to start out

#define PREAMBLE_LENGTH 20 //at least 20 '1's in a service mode preamble.

#define ACK_TIMEOUT 5 //6ms +- 1ms

//TODO

/// The currently queued packet to be put on the rails.
byte current_packet[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
/// How many data bytes in the queued packet?
volatile byte current_packet_size = 0;
/// How many bytes remain to be put on the rails?
volatile byte current_byte_counter = 0;
/// How many bits remain in the current data byte/preamble before changing states?
volatile byte current_bit_counter = PREAMBLE_LENGTH; //init to 28 1's for the preamble
/// A fixed-content packet to send when idle
//byte DCC_Idle_Packet[3] = {255,0,255};
/// A fixed-content packet to send to reset all decoders on layout
//byte DCC_Reset_Packet[3] = {0,0,0};

/** S 9.1 A specifies that '1's are represented by a square wave with a half-period of 58us (valid range: 55-61us)
    and '0's with a half-period of >100us (valid range: 95-9900us)
    Because '0's are stretched to provide DC power to non-DCC locos, we need two zero counters,
     one for the top half, and one for the bottom half.

   Here is how to calculate the timer1 counter values (from ATMega328 datasheet, chapter 17.7.2):
 f_{OC2A} = \frac{f_{clk_I/O}}{2*N*(1+OCR2A)})
 where N = prescalar, and OCR2A is the TOP we need to calculate.
 We know the desired half period for each case, 58us and >100us.
 So:
 for ones:
 58us = (8*(1+OCR2A)) / (16MHz)
 58us * 16MHz = 8*(1+OCR2A)
 58us * 2MHz = 1+OCR2A
 OCR2A = 115

 for zeros:
 100us * 2MHz = 1+OCR2A
 OCR2A = 199 
*/
unsigned int one_count=115; //58us
unsigned int zero_count=199; //100us


/**  On ATmega168 and ATmega328, we will use the 8-bit Timer2.
But because the AT90CAN128 does not offer an OC2B pin (!?), even though the ATmega2560 does,
we will use the 16-bit Timer3 on these chips. TODO
*/

/// Setup phase: configure and enable Timer2/ CTC interrupt, set OC2A and OC2B to toggle on CTC
void setup_DCC_service_mode_waveform_generator()
{
  
 //Set the OC2A and OC2B pins (Timer2 output pins A and B) to output mode
 //On Arduino UNO, etc, OC2A is Port B/Pin 3 = digital pin 11; and OC2B Port D/Pin 3 = digital pin 3
#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__)
  DDRB |= _BV(DDB3);
  DDRD |= _BV(DDD3);
#endif

  // Configure Timer2 in CTC mode, for waveform generation, set to toggle OC2A, OC2B, at /8 prescalar, interrupt at CTC
  TCCR2A = _BV(COM2A0) | _BV(COM2B0) | _BV(WGM21);
  TCCR2B = _BV(CS21);

  // start by outputting a '1'
  OCR2A = OCR2B = one_count; //Whenever we set OCR2A, we must also set OCR2B, or else pin OC2B will get out of sync with OC2A!
  TCNT2 = 0; //get the timer rolling (not really necessary? defaults to 0. Just in case.)
    
  //finally, force a toggle on OC2A so that pin OC2A will always complement pin OC2B
  TCCR2B |= _BV(FOC2A);

}

void DCC_service_mode_waveform_generation_hajime()
{
  //force a toggle on OC2A so that pin OC2A will always complement pin OC2B
  TCCR2B |= _BV(FOC2A);
  //enable the compare match interrupt
  TIMSK2 |= _BV(OCIE2A);
}

void DCC_service_mode_waveform_generation_yame()
{
  //disable the compare match interrupt
  TIMSK2 &= ~_BV(OCIE2A);
  //set both outputs to zero to disable H-Bridge
#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__)
  PINB &= ~_BV(PINB3);
  PIND &= ~_BV(PIND3);
#endif
}

/// This is the Interrupt Service Routine (ISR) for Timer1 compare match on output A. We don't worry about the interrupt on output B, since we want B to mirror A; we'll update them both here.
ISR(TIMER2_COMPA_vect)
{
  //in CTC mode, timer TCINT2 automatically resets to 0 when it matches OCR2A. Depending on the next bit to output,
  //we may have to alter the value in OCR2A, maybe.
  //to switch between "one" waveform and "zero" waveform, we assign a value to OCR2A.
  
  //remember, anything we set for OCR2A takes effect IMMEDIATELY, so we are working within the cycle we are setting.
  //first, check to see if we're in the second half of a byte; only act on the first half of a byte
  //On Arduino UNO, etc, OC2A is digital pin 11, or Port B/Pin 3
  if(!(PINB & _BV(PINB3))) //if the pin is high, time to do our work setting up whether this should be a zero or one.
  {
     //time to switch things up, maybe. send the current bit in the current packet.
     //if this is the last bit to send, queue up another packet (might be the idle packet).
    switch(DCC_state)
    {
      /// Idle: Check if a new packet is ready. If it is, fall through to dos_send_premable. Otherwise just stick a '1' out there.
      case dos_idle:
        if(!current_byte_counter) //if no new packet
        {
//          Serial.println("X");
          OCR2A = OCR2B = one_count; //just send ones if we don't know what else to do. safe bet.
          break;
        }
        //looks like there's a new packet for us to dump on the wire!
        //for debugging purposes, let's print it out
//        if(current_packet[1] != 0xFF)
//        {
//          Serial.print("Packet: ");
//          for(byte j = 0; j < current_packet_size; ++j)
//          {
//            Serial.print(current_packet[j],HEX);
//            Serial.print(" ");
//          }
//          Serial.println("");
//        }
        DCC_state = dos_send_preamble; //and fall through to dos_send_preamble
      /// Preamble: In the process of producing 14 '1's, counter by current_bit_counter; when complete, move to dos_send_bstart
      case dos_send_preamble:
        OCR2A = OCR2B = one_count;
//        Serial.print("P");
        if(!--current_bit_counter)
          DCC_state = dos_send_bstart;
        break;
      /// About to send a data byte, but have to peceed the data with a '0'. Send that '0', then move to dos_send_byte
      case dos_send_bstart:
        OCR2A = OCR2B = zero_count;
        DCC_state = dos_send_byte;
        current_bit_counter = 8;
//        Serial.print(" 0 ");
        break;
      /// Sending a data byte; current bit is tracked with current_bit_counter, and current byte with current_byte_counter
      case dos_send_byte:
        if(((current_packet[current_packet_size-current_byte_counter])>>(current_bit_counter-1)) & 1) //is current bit a '1'?
        {
          OCR2A = OCR2B = one_count;
//          Serial.print("1");
        }
        else //or is it a '0'
        {
          OCR2A = OCR2B = zero_count;
//          Serial.print("0");
        }
        if(!--current_bit_counter) //out of bits! time to either send a new byte, or end the packet
        {
          if(!--current_byte_counter) //if not more bytes, move to dos_end_bit
          {
            DCC_state = dos_end_bit;
          }
          else //there are more bytes…so, go back to dos_send_bstart
          {
            DCC_state = dos_send_bstart;
          }
        }
        break;
      /// Done with the packet. Send out a final '1', then head back to dos_idle to check for a new packet.
      case dos_end_bit:
        OCR2A = OCR2B = one_count;
        DCC_state = dos_idle;
        current_bit_counter = PREAMBLE_LENGTH; //in preparation for a premable...
//        Serial.println(" 1");
        break;
    }
  }
}

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////
  
DCCServiceModeScheduler::DCCServiceModeScheduler(void) : _accepts_direct_mode(false), _mode(idle_mode), _wasACKd(false), _over_current_timer(0), _ack_timer(0)
{
  _mode = idle_mode;
  _wasACKd = false;
  _power_on_queue.setup();
  _instruction_queue.setup();
  _conditional_queue.setup();
  _over_current_timer = 0;
  _ack_timer = 0;
}

void DCCServiceModeScheduler::reset(void)
{
  _mode = idle_mode;
  _wasACKd = false;
  _power_on_queue.clear();
  _instruction_queue.clear();
  _conditional_queue.clear();
  _over_current_timer = 0;
  _ack_timer = 0;
}

void DCCServiceModeScheduler::setup(void) //for any post-constructor initialization
{
  setup_DCC_service_mode_waveform_generator();
}

//to be called periodically within loop()
//TODO This function is pretty much a mess! clean this up once we have a better idea of what we're doing.

//modes:
//  power-on: 20+ packets to permit decoder to power on. A load of 250mA sustained after 100ms = short circuit.
//  instruction transmit:
//  wait for ack: watch for an increase in current draw (at least 60mA) for TODO ms to count as ACK
//      else timeout at end of packet queue and register as NAK
//  power-off:
//

void DCCServiceModeScheduler::update(void) //checks queues, puts whatever's pending on the rails via global current_packet. easy-peasy
{

  //first, check for an over-current; abort current operation if detected.
  if(_over_current_timer) //if the timer is started
  {
    uint16_t check = analogRead(CURRENT_SENSE_PIN);
    uint16_t time = millis();
    if( (check >= OVER_CURRENT_VALUE) && ( (time - _over_current_timer) >= 100) )
    {
      //over current detected! shut it down!
      DCC_service_mode_waveform_generation_yame();
      _mode = idle_mode;
      _power_on_queue.clear();
      _instruction_queue.clear();
      _conditional_queue.clear();
      _over_current_timer = 0;
      return;
    }
  }


  if(_mode == idle_mode)
    return;

  //Feed me! The ISR needs a packet!
  if(!current_byte_counter)
  {
    //now, queue up the next packet
    DCCServiceModePacket p = DCCServiceModePacket();

    switch(_mode)
    {
      case power_on_initial_mode:
        //turn on the h-bridge
        //start the over-current timer
        _wasACKd = false; //reset ack
        _over_current_timer = millis();
        DCC_service_mode_waveform_generation_hajime();
        _mode = power_on_mode;
        //no break
      case power_on_mode:
        if(!_power_on_queue.readPacket(p))
        {
          _instruction_queue.readPacket(p);
          _mode = recovery_mode;
        }
        break;
      
      case recovery_mode:
        //need to watch for an ack from here forward!
        uint16_t check = analogRead(CURRENT_SENSE_PIN);
        uint16_t time = millis();
        if(check >= ACK_THRESHOLD)
        {
          //check/start the timer
          if(_ack_timer)
          {
            if( (time - _ack_timer) >= ACK_TIMEOUT )
            {
              //got an ack!! What to do with it!?
              reset();
              _wasACKd = true;
              return;
            }
          }
        }
        if(!_instruction_queue.readPacket(p))
        {
          //done with sequence. power off and reset.
          DCC_service_mode_waveform_generation_yame();
          reset();
          return;
        }
        break;
    }
  
    //feed the ISR:
    current_packet_size = p.getBitstream(current_packet); //feed to the starving ISR.
    current_byte_counter = current_packet_size;
  }
}


/** Here's the general gist of how we're going to write a CV (or part thereof):
    set up the repeat queue with the necessary packets for the mode, operation, CV, and value
    set a flag to indicate how many packets should pass before we start watching for an ack
    allow repeat() to run...
    when the indicated number of packets begin to pass, watch for an ACK. Will need a pin change
    interrupt on one of the analog pins, and a timer? use millis().
    if ACK received, shut down programming track
    if no ack received by the time we run out of packets to send, count as NAK
    Will use a similar procedure for verifying a CV.
**/

bool DCCServiceModeScheduler::directModeWriteByte(uint16_t CV, uint8_t value)
{
  //Packet stream:
  //  power-on (see above)
  //  3 or more reset packets
  //  5 or more writes to a single CV
  //  6 or more identical write or reset packets (decoder recovery time)
  //  power off
  
  DCCServiceModePacket p;
  
  //first, check to see if something is currently being put on the rails.
  if(_mode != idle_mode)
    return false;
    
  // load up 20 idle packets for power-on
  p.setRepeat(20);
  _power_on_queue.insertPacket(p); //p defaults to idle packet, no need to set it up.
    
  // load up 3 or more reset packets
  p.makeReset();
  _power_on_queue.insertPacket(p);

  
  //Instructions packets using Direct CV Addressing are 4 byte packets of the format:
  //   long-preamble 0 011111AA 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1
  p.setData( (0b01111100 | ((CV>>8)&0x03)), CV&0xFF, value);
  p.setRepeat(5);
  _instruction_queue.insertPacket(p);
  
  //and send one or more reset packets if an ack is found. Ugh.
  p.makeReset();
  p.setRepeat(6);
  _instruction_queue.insertPacket(p);

  _mode = power_on_initial_mode;
  
  return true;
}