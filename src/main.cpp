// User Setting
//===================================================
#define FOR_DEBUG

#define DEVICE_TYPE_JOYSTICK  (0)
#define DEVICE_TYPE_MOUSE     (1)
//#define DEVICE_TYPE           (DEVICE_TYPE_JOYSTICK)
#define DEVICE_TYPE           (DEVICE_TYPE_MOUSE)
//===================================================


#if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega32U4__) && !defined(__AVR_ATtiny85__)
#error do not support
#endif


#if defined(FOR_DEBUG) && !defined(__AVR_ATtiny85__)
#define USAGE_DEBUG
#endif
#if defined(__AVR_ATmega32U4__)
#if DEVICE_TYPE == DEVICE_TYPE_JOYSTICK
#define USAGE_JOYSTICK
#elif DEVICE_TYPE == DEVICE_TYPE_MOUSE
#define USAGE_MOUSE
#endif
#endif
#if !defined(__AVR_ATtiny85__)
//#define USAGE_SPI
#endif


#define ENCODER_A_PIN (3)
#define ENCODER_B_PIN (4)


#include <Arduino.h>
#include <Wire.h>

#if defined(USAGE_SPI)
#include <SPI.h>
#endif

#if defined(USAGE_JOYSTICK)
#include "Joystick.h"
#endif
#if defined(USAGE_MOUSE)
#include "Mouse.h"
#endif

#if ARDUINO_AVR_LEONARDO
#endif

#if defined(USAGE_JOYSTICK)
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, 0, 0,
  false, false, false,
  false, false, false,
  false, false, false, false, true); // Steering
#endif

volatile int32_t encoderCount = 0;

inline int32_t getEncoderCount() 
{
  return encoderCount;
}

inline void setEncoderCount(int32_t value) 
{
  encoderCount = value;
}

int mode = 1;
int32_t sampleInterval = 100;
int32_t curValue = 0;
int32_t minValue = -500;
int32_t maxValue = 500;


int32_t readWireInt32()
{
  char vv[4];

  if (Wire.available() < 4) return 0;

  vv[0] = Wire.read();
  vv[1] = Wire.read();
  vv[2] = Wire.read();
  vv[3] = Wire.read();

  return *(int32_t *)vv;
}


void recvWireEvent(int param) 
{
  while (Wire.available() > 0) {
    int v = Wire.read();
    int v2;
    switch(v) {
      case 0: v2 = Wire.read();
              mode = v2;
              break;
      case 1: encoderCount = readWireInt32();
              break;
      case 2: sampleInterval = readWireInt32();
              break;
      case 3: minValue = readWireInt32();
              maxValue = readWireInt32();
              if (encoderCount < minValue) encoderCount = minValue;
              else if (encoderCount > maxValue) encoderCount = maxValue;
#if defined(USAGE_JOYSTICK)
              Joystick.setSteeringRange(minValue, maxValue);
#endif
              break;
    }
  }
}


void reqWireEvent() 
{
  if (mode == 0) {
    int32_t v = encoderCount;
    Wire.write((const uint8_t *)&v, sizeof(int32_t));
  } else if (mode == 1) {
    Wire.write((const uint8_t *)&curValue, sizeof(int32_t));
  }
}

void pciSetup(byte pin)
{
  noInterrupts();

#if defined(__AVR_ATmega328P__)
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
#elif defined(__AVR_ATmega32U4__)
  PCICR  |= bit(PCIE0);
  PCMSK0 |= bit(pin);  
#elif defined(__AVR_ATtiny85__)
  PCMSK |= bit(pin);
  GIMSK |= bit(PCIE);
#endif

  interrupts();
}


void setup() 
{
  Wire.begin(0x34);
  Wire.onReceive(recvWireEvent);
  Wire.onRequest(reqWireEvent);

#if defined(USAGE_SPI)
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(SS, INPUT);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPCR |= _BV(SPE);                        // SPI Enable
  SPCR &= ~_BV(MSTR);                      // select Slave Mode
  SPCR |= _BV(SPIE);                       // SPI Interrupt Enable
#endif

#if defined(USAGE_JOYSTICK)
  Joystick.setSteeringRange(minValue, maxValue);
  Joystick.begin();
#endif
#if defined(USAGE_MOUSE)
  Mouse.begin();
#endif

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pciSetup(ENCODER_A_PIN);
  pciSetup(ENCODER_B_PIN);

#if defined(USAGE_DEBUG)
  Serial.begin(9600);
#endif        
}


#if defined(USAGE_SPI)
// int32_t readSPIInt32()
// {
//   char vv[4];

//   vv[0] = SPDR;
//   vv[1] = SPDR;
//   vv[2] = SPDR;
//   vv[3] = SPDR;

//   return *(int32_t *)vv;
// }

// void writeSPI(const uint8_t *buf, int len)
// {
//   for (int i = 0; i < len; i++) {
//     SPDR = buf[i];
//   }
// }

// ISR (SPI_STC_vect)
// {
//   while ( !(SPSR & (1<<SPIF)) ) {
//     int v = SPDR;
//     int v2;
//     switch(v) {
//       case 0: v2 = SPDR;
//               mode = v2;
//               break;
//       case 1: encoderCount = readSPIInt32();
//               break;
//       case 2: sampleInterval = readSPIInt32();
//               break;
//       case 3: minValue = readSPIInt32();
//               maxValue = readSPIInt32();
//               if (encoderCount < minValue) encoderCount = minValue;
//               else if (encoderCount > maxValue) encoderCount = maxValue;
// #if defined(USAGE_JOYSTICK)
//               Joystick.setSteeringRange(minValue, maxValue);
// #endif
//               break;
//       case 4: if (mode == 0) {
//                 int32_t v = encoderCount;
//                 writeSPI((const uint8_t *)&v, sizeof(int32_t));
//               } else if (mode == 1) {
//                 writeSPI((const uint8_t *)&curValue, sizeof(int32_t));
//               }
//               break;
//     }
//   }  
// }
#endif

#if defined(__AVR_ATmega328P__)
ISR(PCINT2_vect)
#elif defined(__AVR_ATmega32U4__)
ISR(PCINT0_vect)
#elif defined(__AVR_ATtiny85__)
ISR(PCINT0_vect)
#endif
{
  static int lastStateA = 1;
  static int lastStateB = 1;

  noInterrupts();
  int a = digitalRead(ENCODER_A_PIN);
  int b = digitalRead(ENCODER_B_PIN);
  if (a != lastStateA){
    lastStateA = a;
    if (a == b) {
      if (encoderCount > minValue) encoderCount--;
    } else {
      if (encoderCount < maxValue) encoderCount++;
    }
  } else if (b != lastStateB){
    lastStateB = b;
    if (a != b) {
      if (encoderCount > minValue) encoderCount--;
    } else {
      if (encoderCount < maxValue) encoderCount++;
    }
  }
  interrupts();
}

void loop()
{
  static unsigned long oldTime = 0;

  if (mode == 0) {
#if defined(USAGE_JOYSTICK) || defined(USAGE_MOUSE) ||defined(USAGE_DEBUG)
    int ms = millis();

    if (ms - oldTime > (unsigned long)sampleInterval) {
      oldTime = ms;

      int32_t val = getEncoderCount();

      if(val != curValue) {
        curValue = val;

#if defined(USAGE_JOYSTICK)
        Joystick.setSteering(curValue);
#endif        
#if defined(USAGE_MOUSE)
        Mouse.move(curValue, 0, 0);
#endif
#if defined(USAGE_DEBUG)
        Serial.println(curValue);
#endif        
      }
    }
#endif
  } else if (mode == 1) {
    int ms = millis();

    if (ms - oldTime > (unsigned long)sampleInterval) {
      oldTime = ms;

      int32_t val = getEncoderCount();
      setEncoderCount(0);

      if (val != curValue) {
        curValue = val;

#if defined(USAGE_JOYSTICK)
        Joystick.setSteering(curValue);
#endif
#if defined(USAGE_MOUSE)
        Mouse.move(curValue, 0, 0);
#endif
#if defined(USAGE_DEBUG)
        Serial.println(curValue);
#endif        
      }
    }
  }
}
