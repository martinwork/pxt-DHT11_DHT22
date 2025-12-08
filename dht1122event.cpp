/**
 * (c) 2025 Micro:bit Educational Foundation and contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "pxt.h"

namespace dht11_dht22 {

#if MICROBIT_CODAL

class CDHT1122Event
{
public:
  CDHT1122Event();
  ~CDHT1122Event()
  { 
    stop();
  }

  typedef enum eError  
  { 
    eErrorNone, 
    eErrorNotRead, 
    eErrorNoResponse, 
    eErrorTimeout,
    eErrorChecksum
  } eError;

  volatile uint8_t bytes[5];
  volatile bool  valid;
  volatile uint8_t lastError;

  void read( MicroBitPin *pin);

  void readAsync( MicroBitPin *pin);
  void stop();

  bool ready();       // inactive
  bool reading();     // active
  bool requesting();  // send low for 18ms 
  bool rxpulldown();  // pull up and expect pull down
  bool rxpulse();     // expect 80us high pulse
  bool receiving();   // receive bits - short/long pulse for 0/1
  bool complete();    // all bits received

protected:
  MicroBitPin *pin;

  static uint16_t timerValue0;

  typedef enum eTime  
  { 
    eTimeRequest = 18, // ms
    eTimeRXPulse = 80, 
    eTimeOut = 10 // ms 5200us = 40 + 80 + 80 + 40 * (50 + 75) 
  } eTime;

  typedef enum eState
  { 
    eStateReady = -99, 
    eStateRequest = -3, 
    eStateReceive = -2, 
    eStateReceivePulse = -1, 
    eStateBits = 40 
  } eState;

  volatile int state;

  volatile uint32_t lastEdge;

  volatile int timerTicks;

  void receive();

  void onFall( MicroBitEvent evt);
  void onRise( MicroBitEvent evt);

  void startTimer( int milliseconds);
  void stopTimer();
  void onTimer( MicroBitEvent e);
  uint16_t allocateTimerValue();

  void periodicCallback();

  FiberLock lock;
  void lockInit()   { lock.wait(); }
  void lockNotify() { lock.notifyAll(); lock.wait(); }
  void lockWait()   { lock.wait(); }
};

uint16_t CDHT1122Event::timerValue0 = 0;

CDHT1122Event::CDHT1122Event()
{
  pin = NULL;
  valid = false;
  lastError = eErrorNotRead;
  state = eStateReady;
  timerTicks = 0;
}


void CDHT1122Event::read( MicroBitPin *pin)
{
  readAsync( pin);
  //DMESG( "wait \r\n");
  lockWait();
}


inline bool CDHT1122Event::ready()      { return state == eStateReady; } // inactive
inline bool CDHT1122Event::reading()    { return state != eStateReady; } 
inline bool CDHT1122Event::requesting() { return state == eStateRequest; }  // send low for 18ms 
inline bool CDHT1122Event::rxpulldown() { return state == eStateReceive; }  // pull up and expect pull down
inline bool CDHT1122Event::rxpulse()    { return state == eStateReceivePulse; }  // expect 80us high pulse
inline bool CDHT1122Event::receiving()  { return state >= 0; }   // receive bits - short/long pulse for 0/1
inline bool CDHT1122Event::complete()   { return state == eStateBits; }   // receive bits - short/long pulse for 0/1


void CDHT1122Event::readAsync( MicroBitPin *_pin)
{
  //DMESG("readAsync %p %d \r\n", _pin, (int) _pin->id);
  stop();

  pin = _pin;

  state = eStateRequest;
  for ( int i = 0; i < 5; i++) bytes[i] = 0;

  uBit.messageBus.listen( pin->id, MICROBIT_PIN_EVT_RISE, this, &CDHT1122Event::onRise, MESSAGE_BUS_LISTENER_IMMEDIATE);
  uBit.messageBus.listen( pin->id, MICROBIT_PIN_EVT_FALL, this, &CDHT1122Event::onFall, MESSAGE_BUS_LISTENER_IMMEDIATE);

  pin->getDigitalValue();
  pin->setDigitalValue( 0);
  startTimer( eTimeRequest);       // at least 18ms
}


void CDHT1122Event::receive()
{
  state = eStateReceive;
  lastEdge = system_timer_current_time_us();
  // Assume pin has required external pull up and required PullMode already set
  pin->getDigitalValue();
  pin->eventOn(MICROBIT_PIN_EVENT_ON_EDGE);
  startTimer( eTimeOut);
}


void CDHT1122Event::stop()
{
  //DMESG("stop \r\n");

  stopTimer();
  if ( pin)
  {
    uBit.messageBus.ignore( pin->id, MICROBIT_PIN_EVT_RISE, this, &CDHT1122Event::onRise);
    uBit.messageBus.ignore( pin->id, MICROBIT_PIN_EVT_FALL, this, &CDHT1122Event::onFall);
    pin->eventOn( MICROBIT_PIN_EVENT_NONE);
  }
  state = eStateReady;
  pin = NULL;
  lockNotify();
}


void CDHT1122Event::onRise( MicroBitEvent evt)
{ 
  //DMESG("onRise %d %d \r\n", (int) state, (int) evt.timestamp);
  lastEdge = (uint32_t) evt.timestamp; 
}


void CDHT1122Event::onFall( MicroBitEvent evt)
{
  uint32_t now   = (uint32_t) evt.timestamp;
  uint32_t pulse = now - lastEdge;
  lastEdge = now;

  //DMESG("onFall %d %d %d \r\n", (int) state, (int) evt.timestamp, (int) pulse);

  if ( !receiving())
  {
    if ( rxpulldown())
    {
      // the initial pull down
    }
    else
    {
      // the hi pulse after the initial pull down
      if ( pulse < eTimeRXPulse)
      {
        lastError = eErrorNoResponse;
        stop();
        return;
      }
    }
  }
  else if ( state < eStateBits)
  {
    int bit = pulse > 49 ? 1 : 0;
    volatile uint8_t *byte = &bytes[ state / 8];
    *byte = ( (*byte) << 1) | bit;
  }

  state++;

  if ( state >= eStateBits)
  {
#ifdef CODAL_CONFIG_H
    //DMESG("e %x %x %x %x %x %x \r\n", (int) bytes[0], (int) bytes[1], (int) bytes[2], (int) bytes[3], (int) bytes[4], (int) ( (bytes[0] + bytes[1] + bytes[2] + bytes[3]) % 256));
#endif // CODAL_CONFIG_H

    uint32_t sum = 0;
    for ( int i = 0; i < 4; i++)
      sum += bytes[i];
    if ( sum % 256 != bytes[4])
    {
      lastError = eErrorChecksum;
      stop();
      return;
    }
    else
    {
      valid = true;
    }

    lastError = eErrorNone;
    stop();
  }
}


uint16_t CDHT1122Event::allocateTimerValue()
{
  if ( timerValue0 == 0)
  {
    timerValue0 = allocateNotifyEvent();
    //DMESG( "timerValue0 %d \r\n", (int) timerValue0);
    for ( int i = ID_PIN_P1; i <= ID_PIN_P20; i++)
    {
      uint16_t value = allocateNotifyEvent();
      //DMESG( "timerValue[%d] %d \r\n", i, (int) value);
    }
  }
  int id = pin->id;
  if ( id < ID_PIN_P0 || id > ID_PIN_P20) id = ID_PIN_P20;
  return timerValue0 + id;
}


void CDHT1122Event::startTimer( int milliseconds)
{
  //DMESG("startTimer %d \r\n", milliseconds);
  uBit.messageBus.listen( DEVICE_ID_NOTIFY, allocateTimerValue(), this, &CDHT1122Event::onTimer, MESSAGE_BUS_LISTENER_IMMEDIATE);
  system_timer_event_after( milliseconds, DEVICE_ID_NOTIFY, allocateTimerValue());
}


void CDHT1122Event::stopTimer()
{
  //DMESG("stopTimer \r\n");
  system_timer_cancel_event( DEVICE_ID_NOTIFY, allocateTimerValue());
  uBit.messageBus.ignore( DEVICE_ID_NOTIFY, allocateTimerValue(), this, &CDHT1122Event::onTimer);
}


void CDHT1122Event::onTimer( MicroBitEvent e)
{
  //DMESG("onTimer \r\n");
  stopTimer();

  if ( requesting())
  {
    receive();
  }
  else if ( reading())
  {
    lastError = eErrorTimeout;
    stop();
  }
}


CDHT1122Event dht1122Event;

#endif // MICROBIT_CODAL

    //%
    void v2Read( int dataPin) {
    #if MICROBIT_CODAL
        MicroBitPin *pin = getPin( dataPin);
        if ( pin) 
          dht1122Event.read( pin);
    #endif // MICROBIT_CODAL
    }

    //%
    void v2ReadAsync( int dataPin) {
    #if MICROBIT_CODAL
        MicroBitPin *pin = getPin( dataPin);
        if ( pin) 
          dht1122Event.readAsync( pin);
    #endif // MICROBIT_CODAL
    }

    //%
    int v2Byte( int index) {
    #if MICROBIT_CODAL
        return dht1122Event.bytes[index];
    #else
        return 0;
    #endif // MICROBIT_CODAL
    }

    //%
    bool v2Responding() {
    #if MICROBIT_CODAL
        return dht1122Event.lastError != CDHT1122Event::eErrorNoResponse;
    #else
        return true;
    #endif // MICROBIT_CODAL
    }

    //%
    bool v2Successful() {
    #if MICROBIT_CODAL
        return dht1122Event.lastError == CDHT1122Event::eErrorNone;
    #else
        return true;
    #endif // MICROBIT_CODAL
    }
  }