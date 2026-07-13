
//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all necessary GMP config macro in this file.
//
// WARNING: This file must be kept in the include search path during compilation.
//

#ifndef _FILE_XPLT_PERIPHERAL_H_
#define _FILE_XPLT_PERIPHERAL_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <gmp_core.h>

// controller settings
#include "ctrl_settings.h"

// select ADC PTR interface
#include <ctl/component/interface/adc_ptr_channel.h>
#include <ctl/component/interface/adc_channel.h>

//=================================================================================================
// definitions of peripheral

// DC power supply voltage and current feedback
extern adc_channel_t adc_vout;
extern adc_channel_t adc_iout;

// Raw ADC results and physical measurements for CCS Expressions
extern volatile uint16_t g_adc_vout_raw;
extern volatile uint16_t g_adc_iout_raw;

extern volatile float g_vout_meas_v;
extern volatile float g_iout_meas_a;
extern volatile float g_iout_meas_ma;

// inverter side voltage feedback
extern tri_ptr_adc_channel_t uuvw;
extern adc_gt uuvw_src[3];

// inverter side current feedback
extern tri_ptr_adc_channel_t iuvw;
extern adc_gt iuvw_src[3];

// grid side voltage feedback
extern tri_ptr_adc_channel_t vabc;
extern adc_gt vabc_src[3];

// grid side current feedback
extern tri_ptr_adc_channel_t iabc;
extern adc_gt iabc_src[3];

// DC bus current & voltage feedback
extern ptr_adc_channel_t udc;
extern adc_gt udc_src;
extern ptr_adc_channel_t idc;
extern adc_gt idc_src;

void reset_controller(void);

typedef enum
{
    BOARD_I2C_CLEAR_NOT_RUN = 0,
    BOARD_I2C_CLEAR_IN_PROGRESS = 1,
    BOARD_I2C_CLEAR_RELEASED = 2,
    BOARD_I2C_CLEAR_STILL_LOW = 3
} board_i2c_clear_result_t;

void board_i2c_bus_clear_begin(void);
board_i2c_clear_result_t board_i2c_bus_clear_step(void);
void board_i2c_restore_peripheral_mode(void);
void board_i2c_controller_reinit(void);
uint16_t board_i2c_read_sda_level(void);
uint16_t board_i2c_read_scl_level(void);

uint16_t SPI_readReg(uint16_t addr);
void SPI_writeReg(uint16_t addr, uint16_t data);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PERIPHERAL_H_
