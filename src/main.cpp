/********************************************************************************
 * Copyright (C) 2021 Ju, Gyeong-min
 ********************************************************************************/

// User Setting
//=========================================================================
//#define USE_DEBUG                       /// for Debugging

#define USE_I2C                         /// use I2C
#define USE_SPI                         /// use SPI
#define USE_SERIAL                      /// use Serial (Command Shell)

#define USE_EEPROM                      /// use EEPROM

#define DEFAULT_I2C_ADDR      (0x34)    /// default i2c address

#define DEFAULT_ROTARY_PPR    (100)     /// default rotary encoder P/R
#define DEFAULT_MOUSE_DPI     (1000)    /// default mouse resolution D/I
#define DEFAULT_SAMPLE_RATE   (10)      /// default sample rate (ms)

#define USE_A_B_BOTH_INTRP              /// use a, b pin interrupt
#define USE_CHANGE_INTRP                /// use pin change interrupt

#define DEVICE_TYPE_MOUSE     (0)
#define DEVICE_TYPE_JOYSTICK  (1)
#define DEVICE_TYPE           (DEVICE_TYPE_MOUSE)  /// DEVICE_TYPE_MOUSE or DEVICE_TYPE_JOYSTICK
//=========================================================================


#if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega32U4__) && !defined(__AVR_ATtiny85__)
#error do not support
#endif

#define VERSION ("0.1.0")

// interface feature
#if defined(__AVR_ATmega328P__)
#if defined(USE_I2C)
#define FEATURE_I2C
#endif
#if defined(USE_SPI)
#define FEATURE_SPI
#endif
#if defined(USE_SERIAL)
#define FEATURE_CMD_SHELL
#endif
#elif defined(__AVR_ATmega32U4__)
#if defined(USE_SERIAL)
#define FEATURE_CMD_SHELL
#endif
#if DEVICE_TYPE == DEVICE_TYPE_JOYSTICK
#define FEATURE_JOYSTICK
#elif DEVICE_TYPE == DEVICE_TYPE_MOUSE
#define FEATURE_MOUSE
#endif
#elif defined(__AVR_ATtiny85__)
#if defined(USE_I2C)
#define FEATURE_I2C
#endif
#endif

#if defined(USE_EEPROM)
#define FEATURE_EEPROM
#endif

#if defined(USE_DEBUG) && !defined(__AVR_ATtiny85__)
#define FEATURE_DEBUG
#endif

#if defined(FEATURE_MOUSE) || defined(FEATURE_JOYSTICK)
#define FEATURE_HID_DEVICE
#endif
#if defined(FEATURE_I2C) || defined(FEATURE_SPI)
#define FEATURE_COMM_DEVICE
#endif


#if defined(__AVR_ATtiny85__)
#define ENCODER_A_PIN    (3)
#define ENCODER_B_PIN    (4)
#define ENCODER_INTR_PIN (5)
#define FEATURE_PC_INTRP
#else
#define ENCODER_A_PIN    (2)
#define ENCODER_B_PIN    (3)
#define ENCODER_INTR_PIN (5)
#define FEATURE_EXT_INTRP
#endif

#include <Arduino.h>

#if defined(FEATURE_I2C)
#include <Wire.h>
#endif

#if defined(FEATURE_SPI)
#include <SPI.h>
#endif

#if defined(FEATURE_JOYSTICK)
#include "Joystick.h"
#endif
#if defined(FEATURE_MOUSE)
#include "Mouse.h"
#endif

#if defined(FEATURE_EEPROM)
#include <EEPROM.h>

#define AM_SIGNATURE    (((uint16_t)'A' << 8) | 'M')

#define EEPROM_OFFS_SIGNATURE       (0)
#if defined(FEATURE_I2C)
#define EEPROM_OFFS_I2C_ADDR        (2)
#endif
#define EEPROM_OFFS_SAMPLE_RATE     (4)
#if defined(FEATURE_MOUSE)
#define EEPROM_OFFS_ROTARY_PPR      (6)
#define EEPROM_OFFS_MOUSE_DPI       (8)
#endif
#endif


#if defined(FEATURE_JOYSTICK)
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, 0, 0,
  false, false, false,
  false, false, false,
  false, false, false, false, true); // Steering
#endif

volatile uint8_t* base_a;
volatile uint8_t* base_b;
volatile uint8_t mask_a;
volatile uint8_t mask_b;
#define DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)

volatile int32_t encoderCount = 0;

inline int32_t getEncoderCount()
{
    return encoderCount;
}

inline void setEncoderCount(int32_t value)
{
    encoderCount = value;
}

#if defined(FEATURE_COMM_DEVICE)
#define AM_SPININ_READ_VALUE                (0x00 | 0x00)
#define AM_SPININ_WRITE_VALUE               (0x40 | 0x01)
#define AM_SPININ_SET_MODE                  (0x40 | 0x02)
#define AM_SPININ_SET_SAMPLERATE            (0x40 | 0x03)
#if defined(FEATURE_JOYSTICK)
#define AM_SPININ_SET_MIN_VALUE             (0x40 | 0x04)
#define AM_SPININ_SET_MAX_VALUE             (0x40 | 0x05)
#endif
#if defined(FEATURE_EEPROM)
#define AM_SPININ_SET_I2C_ADDR              (0x40 | 0x0F)
#endif

int mode = 0;
#endif
unsigned long sampleInterval = 10;
volatile int32_t curValue = 0;
#if defined(FEATURE_JOYSTICK)
int32_t minValue = -10000;
int32_t maxValue = 10000;
#endif

#if defined(FEATURE_MOUSE)
#if defined(USE_A_B_BOTH_INTRP)
#define ROTARY_PPR_MUL1 (2)
#else
#define ROTARY_PPR_MUL1 (1)
#endif
#if defined(USE_CHANGE_INTRP)
#define ROTARY_PPR_MUL2 (2)
#else
#define ROTARY_PPR_MUL2 (1)
#endif
int32_t rotary_ppr;
int32_t mouse_dpi;
#endif

#if defined(FEATURE_CMD_SHELL)
// Command Shell용 전역변수
#define MAX_CMD_LEN 32
static char cmdBuf[MAX_CMD_LEN + 1];
static int cmdBufIdx = 0;
static unsigned long serialTime = 0;

// Command Shell을 수행
void processCmdShell()
{
    int availCnt = Serial.available();
    if (availCnt <= 0) {
        return;
    }

    unsigned long cur_time = millis();
    if (cur_time - serialTime > 1000) {
        cmdBufIdx = 0;
    }
    serialTime = cur_time;

    while (availCnt--) {
        char ch = Serial.read();
        if (ch == '\r') {
            cmdBuf[cmdBufIdx] = '\0';
            cmdBufIdx = 0;

            String cmdStr = String(cmdBuf);
            cmdStr.trim();
            if (cmdStr == "help") {
                Serial.print("suc:");
                Serial.print("help");
                Serial.print(",version");
                Serial.print(",val");
                Serial.print(",val:00000");
#if defined(FEATURE_MOUSE)
                Serial.print(",ppr");
                Serial.print(",ppr:00000");
                Serial.print(",dpi");
                Serial.print(",dpi:00000");
#endif
#if defined(FEATURE_JOYSTICK)
                Serial.print(",min");
                Serial.print(",min:00000");
                Serial.print(",max");
                Serial.print(",max:00000");
#endif
                Serial.print(",intv");
                Serial.print(",intv:00000");
                Serial.println("");
            } else if (cmdStr == "version") {
                Serial.print("suc:"); Serial.println(VERSION);
            } else if (cmdStr == "val") {
                Serial.print("suc:"); Serial.println(encoderCount);
            } else if (cmdStr.startsWith("val:")) {
                int v = cmdStr.substring(4).toInt();
                encoderCount = v;
                Serial.println("suc:Value Update OK");
#if defined(FEATURE_JOYSTICK)
            } else if (cmdStr == "min") {
                Serial.print("suc:"); Serial.println(minValue);
            } else if (cmdStr.startsWith("min:")) {
                int v = cmdStr.substring(4).toInt();
                if (v <= maxValue) {
                    minValue = v;
                    if (encoderCount < minValue) encoderCount = minValue;
                    Joystick.setSteeringRange(minValue, maxValue);
                    Serial.println("suc:Min Value Update OK");
                } else {
                    Serial.println("err:Invalide Min Value");
                }
            } else if (cmdStr == "max") {
                Serial.print("suc:"); Serial.println(maxValue);
            } else if (cmdStr.startsWith("max:")) {
                int v = cmdStr.substring(4).toInt();
                if (v >= minValue) {
                    maxValue = v;
                    if (encoderCount > maxValue) encoderCount = maxValue;
                    Joystick.setSteeringRange(minValue, maxValue);
                    Serial.println("suc:Max Value Update OK");
                } else {
                    Serial.println("err:Invalide Max Value");
                }
#endif
            } else if (cmdStr == "intv") {
                Serial.print("suc:"); Serial.println(sampleInterval);
            } else if (cmdStr.startsWith("intv:")) {
                int v = cmdStr.substring(5).toInt();
                if (v > 0 && v <= 1000) {
                    sampleInterval = v;
#if defined(FEATURE_EEPROM)                    
                    EEPROM.write(EEPROM_OFFS_SAMPLE_RATE, (v >> 8) & 0xFF);
                    EEPROM.write(EEPROM_OFFS_SAMPLE_RATE+1, v & 0xFF);
#endif                    
                    Serial.println("suc:Write OK");
                } else {
                    Serial.println("err:Wrong Value");
                }
#if defined(FEATURE_MOUSE)
            } else if (cmdStr == "ppr") {
                Serial.print("suc:"); Serial.println(rotary_ppr / (ROTARY_PPR_MUL1 * ROTARY_PPR_MUL2));
            } else if (cmdStr.startsWith("ppr:")) {
                int v = cmdStr.substring(4).toInt();
                if (v > 0 && v <= 32767) {
                    rotary_ppr = v * ROTARY_PPR_MUL1 * ROTARY_PPR_MUL2;
#if defined(FEATURE_EEPROM)                    
                    EEPROM.write(EEPROM_OFFS_ROTARY_PPR, (v >> 8) & 0xFF);
                    EEPROM.write(EEPROM_OFFS_ROTARY_PPR+1, v & 0xFF);
#endif                    
                    Serial.println("suc:Write OK");
                } else {
                    Serial.println("err:Wrong Value");
                }
            } else if (cmdStr == "dpi") {
                Serial.print("Suc:"); Serial.println(mouse_dpi);
            } else if (cmdStr.startsWith("dpi:")) {
                int v = cmdStr.substring(4).toInt();
                if (v > 0 && v <= 32767) {
                    mouse_dpi = v;
#if defined(FEATURE_EEPROM)                    
                    EEPROM.write(EEPROM_OFFS_MOUSE_DPI, (v >> 8) & 0xFF);
                    EEPROM.write(EEPROM_OFFS_MOUSE_DPI+1, v & 0xFF);
#endif                    
                    Serial.println("suc:Write OK");
                } else {
                    Serial.println("err:Wrong Value");
                }
#endif
            } else {
                if (cmdStr.length() > 0) {
                    Serial.println("err:Known Command");
                }
            }
        }
        else {
            if (cmdBufIdx >= MAX_CMD_LEN) {
                cmdBufIdx = 0;
            }
            cmdBuf[cmdBufIdx++] = ch;
        }
    }
}
#endif


#if defined(FEATURE_I2C)
static unsigned long i2c_time = 0;
static int i2c_state = 0;
static uint8_t i2c_cmd = 0;
static uint8_t i2c_next_temp_byte;

void recvWireEvent(int param)
{
    //noInterrupts();
    uint8_t oldSREG = SREG;
    cli();

    unsigned long cur_time = micros();
    if(cur_time - i2c_time > 100) {
        i2c_state = 0;
    }
    i2c_time = cur_time;

    while (param--){
        if (i2c_state == 0) {
            i2c_cmd = Wire.read();
        }

        if ((i2c_cmd & 0x40) == 0)  {
            switch(i2c_cmd) {
                case AM_SPININ_READ_VALUE:
                    i2c_state = -1;
                    // do nothing
                    break;
            }
        } else {
            int16_t v;
            if (i2c_state == 1){
                i2c_next_temp_byte = Wire.read();
            } else if(i2c_state == 2) {
                v = ((int16_t)Wire.read() << 8) | i2c_next_temp_byte;
                switch(i2c_cmd) {
                    case AM_SPININ_WRITE_VALUE:
                        encoderCount = v;
#if defined(FEATURE_JOYSTICK)
                        if (encoderCount < minValue) encoderCount = minValue;
                        else if (encoderCount > maxValue) encoderCount = maxValue;
#endif
                        break;
                    case AM_SPININ_SET_MODE:
                        mode = v;
                        break;
                    case AM_SPININ_SET_SAMPLERATE:
                        sampleInterval = v;
#if defined(FEATURE_EEPROM)                        
                        EEPROM.write(EEPROM_OFFS_SAMPLE_RATE, (sampleInterval >> 8) & 0xFF);
                        EEPROM.write(EEPROM_OFFS_SAMPLE_RATE+1, sampleInterval & 0xFF);
#endif                        
                        break;
#if defined(FEATURE_JOYSTICK)
                    case AM_SPININ_SET_MIN_VALUE:
                        if (v <= maxValue) {
                            minValue = v;
                            if (encoderCount < minValue) encoderCount = minValue;
                            Joystick.setSteeringRange(minValue, maxValue);
                        }
                        break;
                    case AM_SPININ_SET_MAX_VALUE:
                        if (v >= minValue) {
                            maxValue = v;
                            if (encoderCount > maxValue) encoderCount = maxValue;
                            Joystick.setSteeringRange(minValue, maxValue);
                        }
                        break;
#endif
#if defined(FEATURE_EEPROM)
                    case AM_SPININ_SET_I2C_ADDR:
                        {
                            if ((v & 0xff00) == 0x1200) {
                                v &= 0xFF;
                                if (v >= 0x03 && v <= 0x77) {
                                    EEPROM.write(EEPROM_OFFS_I2C_ADDR, v);
                                }
                            }
                        }
                        break;
#endif                        
                }
            }
        }

        if (++i2c_state > 2) {
            i2c_state = 0;
        }
    }

    //interrupts();
    SREG = oldSREG;
}

void reqWireEvent()
{
    //noInterrupts();
    uint8_t oldSREG = SREG;
    cli();

    int16_t v = 0;
    if (mode == 0) {
        v = encoderCount;
    } else if (mode == 1) {
        v = curValue;
    }

    Wire.write((const uint8_t *)&v, sizeof(int16_t));

    //interrupts();
    SREG = oldSREG;
}
#endif


#if defined(FEATURE_SPI)
static unsigned long spi_time = 0;
static int spi_state = 0;
static int8_t spi_cmd = 0;
static int8_t spi_next_temp_byte;

ISR(SPI_STC_vect)
{
    //noInterrupts();
    uint8_t oldSREG = SREG;
    cli();

    unsigned long cur_time = micros();
    if(cur_time - spi_time > 100) {
        spi_state = 0;
    }
    spi_time = cur_time;

    if (spi_state == 0) {
        spi_cmd = SPDR;
    }

    if ((spi_cmd & 0x40) == 0)  {
        int16_t v = 0;
        if (spi_state == 0){
            switch (spi_cmd) {
                case AM_SPININ_READ_VALUE:
                    if (mode == 0) {
                        v = encoderCount;
                    } else if (mode == 1) {
                        v = curValue;
                    }
                    break;
            }
            SPDR = (v >> 8) & 0xff;
            spi_next_temp_byte = v & 0xff;
        } else if(spi_state == 1) {
            SPDR = spi_next_temp_byte;
        } else {
            SPDR = 0;
        }
    } else {
        int16_t v;
        if (spi_state == 1){
            spi_next_temp_byte = SPDR;
        } else if(spi_state == 2) {
            v = (spi_next_temp_byte << 8) | SPDR;
            switch (spi_cmd) {
                case AM_SPININ_WRITE_VALUE:
                    encoderCount = v;
#if defined(FEATURE_JOYSTICK)
                    if (encoderCount < minValue) encoderCount = minValue;
                    else if (encoderCount > maxValue) encoderCount = maxValue;
#endif
                    break;
                case AM_SPININ_SET_MODE:
                    mode = v;
                    break;
                case AM_SPININ_SET_SAMPLERATE:
                    sampleInterval = v;
#if defined(FEATURE_EEPROM)                    
                    EEPROM.write(EEPROM_OFFS_SAMPLE_RATE, (v >> 8) & 0xFF);
                    EEPROM.write(EEPROM_OFFS_SAMPLE_RATE+1, v & 0xFF);
#endif                    
                    break;
#if defined(FEATURE_JOYSTICK)
                case AM_SPININ_SET_MIN_VALUE:
                    if (v <= maxValue) {
                        minValue = v;
                        if (encoderCount < minValue) encoderCount = minValue;
                        Joystick.setSteeringRange(minValue, maxValue);
                    }
                    break;
                case AM_SPININ_SET_MAX_VALUE:
                    if (v >= minValue) {
                        maxValue = v;
                        if (encoderCount > maxValue) encoderCount = maxValue;
                        Joystick.setSteeringRange(minValue, maxValue);
                    }
                    break;
#endif
#if defined(FEATURE_I2C) && defined(FEATURE_EEPROM)
                case AM_SPININ_SET_I2C_ADDR:
                    if ((v & 0xff00) == 0x1200) {
                        v &= 0xFF;
                        if (v >= 0x03 && v <= 0x77) {
                            EEPROM.write(EEPROM_OFFS_I2C_ADDR, v);
                        }
                    }
                    break;
#endif
            }
        }
    }

    if (++spi_state > 2) {
        spi_state = 0;
    }

    //interrupts();
    SREG = oldSREG;
}
#endif


#if defined(USE_CHANGE_INTRP) || defined(FEATURE_PC_INTRP)
//                        _______         _______
//               A ______|       |_______|       |______ A
// negative <---      _______         _______         __   ---> positive
//               B __|       |_______|       |_______|   B
//
//       new      new      old      old
//        A        B        A        B        Result
//       ---      ---      ---      ---       ------
//        0        0        0        0        no movement
//        0        0        0        1        +1
//        0        0        1        0        -1
//        0        0        1        1        +2  (assume A-pin edges only)
//        0        1        0        0        -1
//        0        1        0        1        no movement
//        0        1        1        0        -2  (assume A-pin edges only)
//        0        1        1        1        +1
//        1        0        0        0        +1
//        1        0        0        1        -2  (assume A-pin edges only)
//        1        0        1        0        no movement
//        1        0        1        1        -1
//        1        1        0        0        +2  (assume A-pin edges only)
//        1        1        0        1        -1
//        1        1        1        0        +1
//        1        1        1        1        no movement

#define MAX_STATE        (16)  /// 4bits
#define PREV_A_HI        (1 << 0)
#define PREV_B_HI        (1 << 1)
#define CURR_A_HI        (1 << 2)
#define CURR_B_HI        (1 << 3)
#define CONV_TO_PREV(a)  ((a) >> 2)

inline void update(void)
{
    static uint8_t state = (CURR_A_HI | CURR_B_HI);
    static const int8_t deltas[MAX_STATE] = {0, 1, -1, 2, -1, 0, -2, 1, 1, -2, 0, -1, 2, -1, 1, 0};

    uint8_t s = CONV_TO_PREV(state);
    if (DIRECT_PIN_READ(base_a, mask_a)) s |= CURR_A_HI;
    if (DIRECT_PIN_READ(base_b, mask_b)) s |= CURR_B_HI;
#if defined(FEATURE_JOYSTICK)
    int8_t delta = deltas[s];
    encoderCount += delta;
    if (delta > 0) {
        if (encoderCount > maxValue) encoderCount = maxValue;
    } else if (delta < 0) {
        if (encoderCount < minValue) encoderCount = minValue;
    }
#else
    encoderCount += deltas[s];
#endif
    state = s;
}
#endif

#if defined(FEATURE_EXT_INTRP)
#if defined(USE_CHANGE_INTRP)
#if defined(USE_A_B_BOTH_INTRP)
static void process_change_ab()
{
    //noInterrupts();
    uint8_t oldSREG = SREG;
    cli();

    update();

    //interrupts();
    SREG = oldSREG;
}
#else
static void process_change_a()
{
    //noInterrupts();
    uint8_t oldSREG = SREG;
    cli();

    update();

    //interrupts();
    SREG = oldSREG;
}
#endif
#else
static void process_falling_a(void)
{
    // To-do: The chattering problem should be fixed.
    //noInterrupts();
    uint8_t oldSREG = SREG;
    cli();

    uint8_t b = DIRECT_PIN_READ(base_b, mask_b);
    if (b){
#if defined(FEATURE_JOYSTICK)
        if (encoderCount > minValue) encoderCount--;
#else
        encoderCount--;
#endif
    } else {
#if defined(FEATURE_JOYSTICK)
        if (encoderCount < maxValue) encoderCount++;
#else
        encoderCount++;
#endif
    }

    //interrupts();
    SREG = oldSREG;
}

#if defined(USE_A_B_BOTH_INTRP)
static void process_falling_b(void)
{
    // To-do: The chattering problem should be fixed.
    //noInterrupts();
    uint8_t oldSREG = SREG;
    cli();

    uint8_t a = DIRECT_PIN_READ(base_a, mask_a);
    if (a){
#if defined(FEATURE_JOYSTICK)
        if (encoderCount < maxValue) encoderCount++;
#else
        encoderCount++;
#endif
    } else {
#if defined(FEATURE_JOYSTICK)
        if (encoderCount > minValue) encoderCount--;
#else
        encoderCount--;
#endif
    }

    //interrupts();
    SREG = oldSREG;
}
#endif
#endif
#endif

#if defined(FEATURE_PC_INTRP)
static void process_pc()
{
    //noInterrupts();
    uint8_t oldSREG = SREG;
    cli();

    update();

    //interrupts();
    SREG = oldSREG;

//    // I can't write the code below because of chattering problem
//    static uint8_t lastStateA = 1;
//    static uint8_t lastStateB = 1;
//
//     noInterrupts();
//     uint8_t a = DIRECT_PIN_READ(base_a, mask_a);
//     uint8_t b = DIRECT_PIN_READ(base_b, mask_b);
//     if (a != lastStateA){
//         lastStateA = a;
//         if (a == b) {
// #if defined(FEATURE_JOYSTICK)
//             if (encoderCount < maxValue) encoderCount++;
// #else
//             encoderCount++;
// #endif
//         } else {
// #if defined(FEATURE_JOYSTICK)
//             if (encoderCount > minValue) encoderCount--;
// #else
//             encoderCount--;
// #endif
//         }
//     } else if (b != lastStateB){
//         lastStateB = b;
//         if (a != b) {
// #if defined(FEATURE_JOYSTICK)
//             if (encoderCount < maxValue) encoderCount++;
// #else
//             encoderCount++;
// #endif
//         } else {
// #if defined(FEATURE_JOYSTICK)
//             if (encoderCount > minValue) encoderCount--;
// #else
//             encoderCount--;
// #endif
//         }
//     }
//     interrupts();
}

void pciSetup(byte pin)
{
    //noInterrupts();
    uint8_t oldSREG = SREG;
    cli();

#if defined(__AVR_ATmega328P__)
    *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR |= bit(digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR |= bit(digitalPinToPCICRbit(pin)); // enable interrupt for the group
#elif defined(__AVR_ATmega32U4__)
    PCICR |= bit(PCIE0);
    PCMSK0 |= bit(pin);
#elif defined(__AVR_ATtiny85__)
    PCMSK |= bit(pin);
    GIMSK |= bit(PCIE);
#endif

    //interrupts();
    SREG = oldSREG;
}
#endif


#if defined(__AVR_ATmega328P__)
#if defined(FEATURE_PC_INTRP)
ISR(PCINT2_vect)
{
    process_pc();
}
#endif
#elif defined(__AVR_ATmega32U4__)
#if defined(FEATURE_PC_INTRP)
ISR(PCINT0_vect)
{
    process_pc();
}
#endif
#elif defined(__AVR_ATtiny85__)
ISR(PCINT0_vect)
{
    process_pc();
}
#endif


void setup()
{
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);
#if defined(FEATURE_COMM_DEVICE)
    pinMode(ENCODER_INTR_PIN, OUTPUT);
    digitalWrite(ENCODER_INTR_PIN, HIGH);
#endif

    base_a = portInputRegister(digitalPinToPort(ENCODER_A_PIN));
    mask_a = digitalPinToBitMask(ENCODER_A_PIN);
    base_b = portInputRegister(digitalPinToPort(ENCODER_B_PIN));
    mask_b = digitalPinToBitMask(ENCODER_B_PIN);

#if defined(FEATURE_EXT_INTRP)
#if defined(USE_CHANGE_INTRP)
#if defined(USE_A_B_BOTH_INTRP)
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), process_change_ab, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), process_change_ab, CHANGE);
#else
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), process_change_a, CHANGE);
#endif
#else
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), process_falling_a, FALLING);
#if defined(USE_A_B_BOTH_INTRP)
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), process_falling_b, FALLING);
#endif
#endif
#elif defined(FEATURE_PC_INTRP)
    pciSetup(ENCODER_A_PIN);
    pciSetup(ENCODER_B_PIN);
#endif

#if defined(FEATURE_EEPROM)
    uint16_t sig = ((uint16_t)EEPROM.read(0) << 8) | EEPROM.read(1);

    // formatting EEPROM
    if (sig != AM_SIGNATURE) {
        EEPROM.write(EEPROM_OFFS_SIGNATURE, 'A'); EEPROM.write(EEPROM_OFFS_SIGNATURE+1, 'M');
#if defined(FEATURE_I2C)
        EEPROM.write(EEPROM_OFFS_I2C_ADDR, DEFAULT_I2C_ADDR); EEPROM.write(EEPROM_OFFS_I2C_ADDR+1, 0x00);
#endif
        EEPROM.write(EEPROM_OFFS_SAMPLE_RATE, (DEFAULT_SAMPLE_RATE >> 8) & 0xFF); EEPROM.write(EEPROM_OFFS_SAMPLE_RATE+1, DEFAULT_SAMPLE_RATE & 0xFF);
#if defined(FEATURE_MOUSE)
        EEPROM.write(EEPROM_OFFS_ROTARY_PPR, (DEFAULT_ROTARY_PPR >> 8) & 0xFF); EEPROM.write(EEPROM_OFFS_ROTARY_PPR+1, DEFAULT_ROTARY_PPR & 0xFF);
        EEPROM.write(EEPROM_OFFS_MOUSE_DPI, (DEFAULT_MOUSE_DPI >> 8) & 0xFF); EEPROM.write(EEPROM_OFFS_MOUSE_DPI, DEFAULT_I2C_ADDR & 0xFF);
#endif
    }
#endif

#if defined(FEATURE_I2C)
#if defined(FEATURE_EEPROM)
    uint8_t i2c_addr = EEPROM.read(EEPROM_OFFS_I2C_ADDR);
    if (i2c_addr < 0x03 || i2c_addr > 0x77) {
        i2c_addr = DEFAULT_I2C_ADDR;
    }
#else
    uint8_t i2c_addr = DEFAULT_I2C_ADDR;
#endif    
    Wire.begin(i2c_addr);
    Wire.onReceive(recvWireEvent);
    Wire.onRequest(reqWireEvent);
#endif

#if defined(FEATURE_EEPROM)
    sampleInterval = (EEPROM.read(EEPROM_OFFS_SAMPLE_RATE) << 8) | EEPROM.read(EEPROM_OFFS_SAMPLE_RATE+1);
    if (sampleInterval <= 0 || sampleInterval > 1000) {
        sampleInterval = DEFAULT_SAMPLE_RATE;
    }
#else
    sampleInterval = DEFAULT_SAMPLE_RATE;
#endif

#if defined(FEATURE_MOUSE)
#if defined(FEATURE_EEPROM)
    rotary_ppr = (EEPROM.read(EEPROM_OFFS_ROTARY_PPR) << 8) | EEPROM.read(EEPROM_OFFS_ROTARY_PPR+1);
    if (rotary_ppr <= 0 || rotary_ppr > 32767) {
        rotary_ppr = DEFAULT_ROTARY_PPR;
    }
#else
    rotary_ppr = DEFAULT_ROTARY_PPR;
#endif    
    rotary_ppr *= ROTARY_PPR_MUL1 * ROTARY_PPR_MUL2;
#if defined(FEATURE_EEPROM)
    mouse_dpi  = (EEPROM.read(EEPROM_OFFS_MOUSE_DPI) << 8) | EEPROM.read(EEPROM_OFFS_MOUSE_DPI+1);
    if (mouse_dpi <= 0 || mouse_dpi > 32767) {
        mouse_dpi = DEFAULT_MOUSE_DPI;
    }
#else
    mouse_dpi = DEFAULT_MOUSE_DPI;
#endif    
#endif

#if defined(FEATURE_SPI)
    pinMode(MISO, OUTPUT);
    pinMode(MOSI, INPUT);
    pinMode(SCK, INPUT);
    pinMode(SS, INPUT);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV16);
    SPI.attachInterrupt();
    SPCR &= ~_BV(MSTR);                      // select Slave Mode
    SPCR |= _BV(SPE);                        // SPI Enable
#endif

#if defined(FEATURE_JOYSTICK)
    Joystick.setSteeringRange(minValue, maxValue);
    Joystick.begin();
#endif
#if defined(FEATURE_MOUSE)
    Mouse.begin();
#endif

#if defined(FEATURE_DEBUG) || defined(FEATURE_CMD_SHELL)
    Serial.begin(9600);
#endif
}


void loop()
{
#if defined(FEATURE_CMD_SHELL)
    processCmdShell();
#endif

#if defined(FEATURE_COMM_DEVICE) || defined(FEATURE_HID_DEVICE) || defined(FEATURE_DEBUG)
    static unsigned long oldTime = 0;

    unsigned long ms = millis();
    if ((ms - oldTime) > sampleInterval) {
        oldTime = ms;

        int32_t val = getEncoderCount();

        if(val != curValue) {
            curValue = val;
#if defined(FEATURE_COMM_DEVICE)
            if (mode == 0) {
                // do nothing
            } else if (mode == 1) {
                setEncoderCount(0);
            }
            digitalWrite(ENCODER_INTR_PIN, LOW);
            digitalWrite(ENCODER_INTR_PIN, HIGH);
#endif
#if defined(FEATURE_HID_DEVICE)
            setEncoderCount(0);
#if defined(FEATURE_JOYSTICK)
            Joystick.setSteering(curValue);
#endif
#if defined(FEATURE_MOUSE)
            Mouse.move(curValue * mouse_dpi / rotary_ppr, 0, 0);
#endif
#endif
#if defined(FEATURE_DEBUG)
            Serial.println(curValue);
#endif
        }
    }
#endif
}
