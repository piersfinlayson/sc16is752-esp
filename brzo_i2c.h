/*
brzo_i2c.h -- A fast i2c master for the esp8266 written in assembly language

Copyright (c) 2016 Pascal Kurtansky (pascal at kurtansky dot ch).
All rights reserved.

This file is part of the library brzo_i2c.

Brzo_i2c is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Brzo_i2c is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef _BRZO_I2C_h
#define _BRZO_I2C_h

#ifdef ARDUINO
#include "Arduino.h"
#else
#include <c_types.h>

#ifdef OTB_V0_1

// SDA on GPIO12, SCL on GPIO13
#define BRZO_I2C_SDA_MUX PERIPHS_IO_MUX_GPIO5_U
#define BRZO_I2C_SCL_MUX PERIPHS_IO_MUX_GPIO4_U
#define BRZO_I2C_SDA_GPIO 5
#define BRZO_I2C_SCL_GPIO 4
#define BRZO_I2C_SDA_FUNC FUNC_GPIO5
#define BRZO_I2C_SCL_FUNC FUNC_GPIO4

#else

#ifdef OTB_TEST

#define BRZO_I2C_SDA_MUX PERIPHS_IO_MUX_GPIO0_U
#define BRZO_I2C_SCL_MUX PERIPHS_IO_MUX_GPIO2_U
#define BRZO_I2C_SDA_GPIO 0 
#define BRZO_I2C_SCL_GPIO 2 
#define BRZO_I2C_SDA_FUNC FUNC_GPIO0
#define BRZO_I2C_SCL_FUNC FUNC_GPIO2

#else 

#define BRZO_I2C_SDA_MUX PERIPHS_IO_MUX_GPIO4_U
#define BRZO_I2C_SCL_MUX PERIPHS_IO_MUX_GPIO5_U
#define BRZO_I2C_SDA_GPIO 4 
#define BRZO_I2C_SCL_GPIO 5
#define BRZO_I2C_SDA_FUNC FUNC_GPIO4
#define BRZO_I2C_SCL_FUNC FUNC_GPIO5

#endif // OTB_TEST

#endif

#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct brzo_i2c_info
{
  // Inputs
  uint8_t sda_pin;
  uint8_t scl_pin;
#ifndef ARDUINO
  uint8_t sda_pin_func;
  uint8_t scl_pin_func;
  uint32_t sda_pin_mux;
  uint32_t scl_pin_mux;
#endif
  uint32_t clock_stretch_time_out_usec;

  // Outputs
  uint8_t i2c_error;

  // Private data
  uint8_t i2c_slave_address;
  uint16_t sda_bitmask;
  uint16_t scl_bitmask;
  uint16_t iteration_scl_halfcycle;
  uint32_t iteration_scl_clock_stretch;
  uint16_t iteration_remove_spike;
  uint16_t ACK_polling_loop_usec;
  uint16_t i2c_SCL_frequency;
} brzo_i2c_info;

#ifdef ARDUINO
void brzo_i2c_setup(uint8_t sda, uint8_t scl, uint32_t clock_stretch_time_out_usec);
#else
void brzo_i2c_setup(uint32_t clock_stretch_time_out_usec);
#endif

void brzo_i2c_start_transaction(uint8_t slave_address, uint16_t SCL_frequency_KHz);
void brzo_i2c_write(uint8_t *data, uint32_t no_of_bytes, bool repeated_start);
void brzo_i2c_read(uint8_t *data, uint32_t nr_of_bytes, bool repeated_start);
void brzo_i2c_ACK_polling(uint16_t ACK_polling_time_out_usec);
uint8_t brzo_i2c_end_transaction();

void ICACHE_FLASH_ATTR brzo_i2c_setup_info(brzo_i2c_info *info);
void brzo_i2c_start_transaction_info(uint8_t slave_address, uint16_t SCL_frequency_KHz, brzo_i2c_info *info);
void brzo_i2c_write_info(uint8_t *data, uint32_t no_of_bytes, bool repeated_start, brzo_i2c_info *info);
void brzo_i2c_read_info(uint8_t *data, uint32_t nr_of_bytes, bool repeated_start, brzo_i2c_info *info);
void brzo_i2c_ACK_polling_info(uint16_t ACK_polling_time_out_usec, brzo_i2c_info *info);
uint8_t brzo_i2c_end_transaction_info(brzo_i2c_info *info);

#ifdef __cplusplus
}
#endif

#endif
