# am_spinin

**Amos Spinner Input Firmware for Arduino &amp; ATtiny85**

---

<p align="center">
  <p align="center">
    <a href="README.md">English</a>
    ·
    <a href="README_kr.md">한국어</a>
  </p>
</p>

---
Games like Arkanoid use spinners as injection force devices.
To implement this spinner, you can use a device such as a Rotary Encoder. In this case, the rotary encoder to be used is not a single-phase type for RPM measurement, but a two-phase type or higher for bi-directional rotation, and the resolution must be at least 100 P/R. In fact, you don't need any higher resolution than that for classic arcade games.

Rotary Encoders are mainly sold in the following forms.

![Rotary Encoder](images/rotary_encoder.jpg)

There are a total of 4 output pins, and there are A and B pins in addition to the power pins of VCC and GND. The power supply is usually 5V to 24V.

In order to actually use this, a Rotary Encoder controller is required. It is best to use a counter chip like the LS7366R for the controller, but it is difficult to find one that is sold as a separate module, and if you purchase a separate encoder counter board, it is inappropriate for use in a game machine because it is a burdensome price for industrial use.
Because it receives human hand movements anyway, a high-speed, high-precision controller is not required, so you can make a controller for a spinner with the performance of an Arduino.

If you use Arduino Leonardo, it operates as a mouse device, so you can operate it without a separate driver by simply connecting it to the USB port of the Raspberry Pi.

![Spinner Connection Diagram](images/spinner_connect_leonardo.jpg)

In many cases, SparkFun's Promicro is used rather than the genuine Arduino Leonardo. It's cheaper and smaller in size, so it's more advantageous to be built into a game machine.

![Spinner Connection Diagram](images/spinner_connect_promicro.jpg)

There are many clones of Promicro. Unfortunately, these clones do not have good power supply parts. In the case of these products, the voltage of VCC is often less than 5V when a load is applied. In this case, the rotary encoder does not operate normally because sufficient voltage is not supplied to operate.

In this case, it is enough to receive the VCC of the Rotary Encoder from the 5V pin of the Raspberry Pi, not the Pro Micro.

![Spinner Connection Diagram](images/spinner_connect_promicro_clone.jpg)

The disadvantage of using an Arduino Leonardo is that it occupies one USB port. At the same time, the exposed rear part of the Raspberry Pi leads to design shortcomings.
Also, once the recording of the firmware is finished, there is a restriction in changing settings or changing functions afterwards.

In order to solve this problem, the Arduino is in charge of counting only the Rotary Encoder, and it is possible to think of a way to reproduce it with a real mouse device by transferring it to the Raspberry Pi through a driver. In this case, the production difficulty is slightly higher than in the previous case, but the device can be operated much more flexibly.

Actually, the board to be used as the controller is the Arduino Pro Mini.

![Arduino Pro Mini](images/arduino_pro_mini.jpg)

When the firmware upload is completed, start wiring to the corresponding control board.
The Rotary Encoder's A and B pins are respectively connected to GPIO pins 2 and 3 of the board.

A4 and A5 pins of the control board are I2C pins, and they are SDA and SCL pins, respectively.

When connecting to Raspberry Pi, connect to I2C pin or SPI pin.

![Spinner Connection Diagram](images/spinner_connect_promini.jpg)

However, if the Arduino is the 5V version, the Raspberry Pi uses 3.3V, so there may be a problem if you connect it directly. If the Arduino is a 3.3V version, there will be no problem with the Raspberry Pi, but instead, there is a possibility that there is a problem because the rotary encoder uses 5V.

Although it is a signal line, the magnitude of the current is not large, and when 3.3V, 5V, or 0 and 1 are clear as a signal level, the level values ​​can be recognized as the same, so it may seem to work well at the moment. However, if the over voltage is continuously applied, it can give a strain to the board in the long run, so it is better to use a level converter that converts the signal voltage level according to each other's voltage as much as possible.

There are several types of level converters: one-way, two-way, two-channel, four-channel, and so on.
When RX and TX are clearly separated like serial communication, unidirectional can be used, but when the same pin is in charge of both read and write like I2C, bidirectional should be used. Also, if they are mixed like SPI, you can mix them appropriately or just unify them in both directions.

Select the number of channels according to the number of lines used for the signal. Basically, serial communication requires 2 channels for I2C and 4 channels for SPI.
(It is different from the power regulator. Do not connect this to the power line, it should only be used for the signal line.)

![Level Converter](images/level_converter.jpg)

If the Arduino is the 5V version, place a signal level converter between the Arduino and the Raspberry Pi.

![Spinner Connection Diagram](images/spinner_connect_promini_5v.jpg)

If the Arduino is the 3.3V version, it should be connected directly to the Raspberry Pi and a signal level converter should be installed to convert the signal from the rotary encoder to 3.3V instead. (A unidirectional level converter may be used in this case.)

![Spinner Connection Diagram](images/spinner_connect_promini_3v3.jpg)

Since it is simply a voltage drop of a unidirectional signal, if there is no level converter, a resistance ratio can be used instead. The ratio of resistors is to lower 5V to 3.3V, so the ratio of R1:R2 should be 17:33.

![Spinner Connection Diagram](images/spinner_connect_promini_3v3_resistor.jpg)

In addition to the Arduino board, you can also use the ATtiny85, a one-chip solution.

![ATtiny85](images/attiny85.jpg)

![ATtiny85 Pinout](images/attiny85_pinout.png)

When using ATtiny85, it is advantageous in terms of size compared to when using an Arduino board. Also, in the case of the ATtiny85, the operating power supply can accommodate from 1.8V to 5.5V, so you can configure the circuit by selecting either 3.3V or 5V. Here, 3.3V was used as the standard. However, if the ATtiny85 is operated at 3.3V, the operating clock is lower than that of 5V, but it can be operated at 12MHz even at 3.3V, so it is sufficient for arcade game input.

![Spinner Connection Diagram](images/spinner_connect_attiny85_resistor.jpg)


Before building, adjust the following settings in src/main.cpp.
The target to be adjusted is the I2C address and the P/R value of the Rotary Encoder.
For reference, some of these values ​​can also be adjusted post-mortem using I2C or Serial commands.

```c++
// User Setting
//=========================================================================
#define USE_DEBUG                       /// for Debugging

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
```

This is a setting predefine that corresponds to the currently supported board.

| predefine   | support board                            | interface         | input pin |
|-------------|------------------------------------------|-------------------|-----------|
| uno         | Arduino Uno                              | I2C, SPI, Serial  | A:2, B:3  |
| promini_5v  | Arduino Pro Mini 5V                      | I2C, SPI, Serial  | A:2, B:3  |
| promini_3v3 | Arduino Pro Mini 3.3V                    | I2C, SPI, Serial  | A:2, B:3  |
| leonardo_5v | Arduino Leonardo, SparkFun Pro Micro 5V  | USB, Serial       | A:2, B:3  |
| attiny85    | ATtiny85                                 | I2C               | A:3, B:4  |


Supported commands for each interface are as follows.

### Serial

It is in string format, and each command ends with a CR character.

| command | R/W | Description | target |
|---------|-----|------|------|
| help | Read | Help
| version | Read | version
| val | Read | Read the current rotary count value
| val:00000 | Write | Write current rotary count value
| intv | Read | Read sampling period
| intv:00000 | Write | Write sampling cycle
| ppr | Read | Read Rotary Encoder Pulse/Revolution | mouse
| ppr:00000 | Write | Write Rotary Encoder Pulse/Revolution | mouse
| dpi | Read | Read Mouse Dot/Inch | mouse
| dpi:00000 | Write | Write Mouse Dot/Inch | mouse
| min | Read | Read Joystick Minimum | joystick
| min:00000 | Write | Write Joystick Minimum | joystick
| max | Read | Read Joystick Maximum | joystick
| max:00000 | Write | Write Joystick Maximum | joystick

The result value has the following format.

on success
```
suc: result or message
```

on failure
```
err: message
```

### I2C
- Read : Current count value

- Write: It has a command format consisting of 3 bytes.

| 0th | 1st | 2nd |
|:-------:|:----------:|:-----------:|
| command | high byte | low byte |

Description of each command.

| command | Description | target |
|---------|-----------------|---------|
| 0x41 | write value | |
| 0x42 | set mode | |
| 0x43 | set sample rate | |
| 0x44 | set min value | joystick |
| 0x45 | set max value | joystick |
| 0x4F | set i2c address | |

### SPI

The SPI interface consists of 3 bytes, and the format is as follows.

| 0th | 1st | 2nd |
|:-------:|:----------:|:-----------:|
| command | high byte | low byte |

Description of each command.

| command | Description | target |
|---------|-----------------|----------|
| 0x00 | read value | |
| 0x41 | write value | |
| 0x42 | set mode | |
| 0x43 | set sample rate | |
| 0x44 | set min value | joystick |
| 0x45 | set max value | joystick |
| 0x4F | set i2c address | |


If the USB interface is not used like the Arduino Leonardo series, a dedicated driver must exist in order to actually receive the count value of the Rotary Encoder and operate it as a mouse device.

In the case of Raspberry Pi, you can use it by installing a driver called am_joyin. The driver can be downloaded from the following link.

> [am_joyin Raspberry Pi Arcade Joystick Driver](https://github.com/amos42/am_joyin)

If the driver operates normally, the Rotary Encoder will act like a kind of mouse.
Therefore, to use it in the game, you need to adjust the settings so that mouse input is possible.

For example, in games such as Arkanoid, you can add mouse support in RetroArch settings.

![RetroArch input settings](images/retroarch_setting.png)
