/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

/* This variant requires the MattairTech SAM D|L|C Core for Arduino >= 1.6.18-beta-b1.
 * The format is different than the stock Arduino SAMD core,
 * which uses ARDUINO_SAMD_VARIANT_COMPLIANCE instead.
 */

#define MATTAIRTECH_ARDUINO_SAMD_VARIANT_COMPLIANCE 10618


#include <WVariant.h>

// General definitions
/* ----------------------------------------------------------------------------
 *Clock Configuration
 *----------------------------------------------------------------------------*/

/** Master clock frequency (also Fcpu frequency) */
#define VARIANT_MCK		(48000000ul)

/* If CLOCKCONFIG_HS_CRYSTAL is defined, then HS_CRYSTAL_FREQUENCY_HERTZ
 * must also be defined with the external crystal frequency in Hertz.
 */
#define HS_CRYSTAL_FREQUENCY_HERTZ	16000000UL
/* If the PLL is used (CLOCKCONFIG_32768HZ_CRYSTAL, or CLOCKCONFIG_HS_CRYSTAL
 * defined), then PLL_FRACTIONAL_ENABLED can be defined, which will result in
 * a more accurate 48MHz output frequency at the expense of increased jitter.
 */
//#define PLL_FRACTIONAL_ENABLED

/* If both PLL_FAST_STARTUP and CLOCKCONFIG_HS_CRYSTAL are defined, the crystal
 * will be divided down to 1MHz - 2MHz, rather than 32KHz - 64KHz, before being
 * multiplied by the PLL. This will result in a faster lock time for the PLL,
 * however, it will also result in a less accurate PLL output frequency if the
 * crystal is not divisible (without remainder) by 1MHz. In this case, define
 * PLL_FRACTIONAL_ENABLED as well.
 */
//#define PLL_FAST_STARTUP

/* The fine calibration value for DFLL open-loop mode is defined here.
 * The coarse calibration value is loaded from NVM OTP (factory calibration values).
 */
#define NVM_SW_CALIB_DFLL48M_FINE_VAL     (512)

// Pins
// ----

// Number of pins defined in PinDescription array
#define PINS_COUNT           (40u)
#define NUM_DIGITAL_PINS     (36u)
#define NUM_ANALOG_INPUTS    (4u)
#define NUM_ANALOG_OUTPUTS   (1u)

// Low-level pin register query macros
// -----------------------------------
#define digitalPinToPort(P)      (&(PORT->Group[g_APinDescription[P].ulPort]))
#define digitalPinToBitMask(P)   (1 << g_APinDescription[P].ulPin)
//#define analogInPinToBit(P)    ()
#define portOutputRegister(port) (&(port->OUT.reg))
#define portInputRegister(port)  (&(port->IN.reg))
#define portModeRegister(port)   (&(port->DIR.reg))
#define digitalPinHasPWM(P)      (g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER)
#define digitalPinToInterrupt(P) (g_APinDescription[P].ulExtInt)

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)


// LEDs
// ----
#define PIN_LED_23  (23u)
#define LED_BUILTIN PIN_LED_23

/*
 * Analog pins
 */
#define PIN_Y0               (2ul)
#define PIN_Y1               (3ul)
#define PIN_Y2               (4ul)
#define PIN_Y3               (5ul)
#define PIN_Y4               (6ul)
#define PIN_Y5               (7ul)
#define PIN_A3               (8ul)
#define PIN_A2               (9ul)
#define PIN_A1              (10ul)
#define PIN_A0              (11ul)
#define PIN_Y8              (32ul)
#define PIN_Y9              (33ul)
#define PIN_Y14             (34ul)
#define PIN_Y15             (35ul)

#define PIN_DAC0            (PIN_Y1)

static const uint8_t Y0  = PIN_Y0;
static const uint8_t Y1  = PIN_Y1;
static const uint8_t Y2  = PIN_Y2;
static const uint8_t Y3  = PIN_Y3;
static const uint8_t Y4  = PIN_Y4;
static const uint8_t Y5  = PIN_Y5;
static const uint8_t A3  = PIN_A3;
static const uint8_t A2  = PIN_A2;
static const uint8_t A1  = PIN_A1;
static const uint8_t A0  = PIN_A0;
static const uint8_t Y8  = PIN_Y8;
static const uint8_t Y9  = PIN_Y9;
static const uint8_t Y14  = PIN_Y14;
static const uint8_t Y15  = PIN_Y15;

static const uint8_t DAC0 = PIN_DAC0;

#define ADC_RESOLUTION		12

/* Set default analog voltage reference */
#define VARIANT_AR_DEFAULT	AR_DEFAULT

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (22u)
#define PIN_SPI_SCK          (19u)
#define PIN_SPI_MOSI         (18u)
#define PERIPH_SPI           sercom3
#define PAD_SPI_TX           SPI_PAD_0_SCK_2  // MOSI / SCK
#define PAD_SPI_RX           SERCOM_RX_PAD_1  // MISO

static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

// "external" public i2c interface
#define PIN_WIRE_SDA         (0u)
#define PIN_WIRE_SCL         (1u)
#define PERIPH_WIRE          sercom1
#define WIRE_IT_HANDLER      SERCOM1_Handler
static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

// USB
// ---
//#define PIN_USB_HOST_ENABLE (25ul)
#define PIN_USB_HOST_ENABLE_VALUE	0
#define PIN_USB_DM          (24ul)
#define PIN_USB_DP          (25ul)

// I2S Interfaces
// --------------
#define I2S_INTERFACES_COUNT 0

// Serial ports
// ------------
#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"

// Instances of SERCOM
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

// Serial1
extern Uart Serial1;
#define PIN_SERIAL1_RX       (13ul)
#define PIN_SERIAL1_TX       (12ul)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)
#endif // __cplusplus



// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_MONITOR         SerialUSB
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

// Alias Serial to SerialUSB
#define Serial                      SerialUSB
