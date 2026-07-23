//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all declarations of user objects in this file.
//
// WARNING: This file must be kept in the include search path during compilation.
//

#include <core/dev/at_device.h>

#include <core/pm/function_scheduler.h>


#ifndef _FILE_USER_MAIN_H_
#define _FILE_USER_MAIN_H_


#ifdef __cplusplus
extern "C"
{
#endif


//=================================================================================================
// SINV / Rectifier controller parameters
//
// Add by user for pgs_sinv_rc project
//
// These parameters are required by ctl_main.c
//


//------------------------------
// Current loop
//------------------------------

#ifndef SINV_CURRENT_LOOP_BANDWIDTH_HZ
#define SINV_CURRENT_LOOP_BANDWIDTH_HZ          1000.0f
#endif


//------------------------------
// FDRC controller parameters
//------------------------------

#ifndef SINV_FDRC_LEARNING_GAIN
#define SINV_FDRC_LEARNING_GAIN                0.05f
#endif


#ifndef SINV_FDRC_Q_FILTER_HZ
#define SINV_FDRC_Q_FILTER_HZ                   100.0f
#endif


#ifndef SINV_FDRC_FREEZE_ERROR_PU
#define SINV_FDRC_FREEZE_ERROR_PU               0.02f
#endif



//------------------------------
// Power loop
//------------------------------

#ifndef SINV_POWER_LOOP_KP
#define SINV_POWER_LOOP_KP                      0.5f
#endif


#ifndef SINV_POWER_LOOP_KI
#define SINV_POWER_LOOP_KI                      50.0f
#endif



//------------------------------
// DC bus voltage loop
//------------------------------

#ifndef SINV_DC_BUS_LOOP_KP
#define SINV_DC_BUS_LOOP_KP                     0.5f
#endif


#ifndef SINV_DC_BUS_LOOP_KI
#define SINV_DC_BUS_LOOP_KI                     50.0f
#endif



//------------------------------
// Outer loop
//------------------------------

#ifndef SINV_OUTER_LOOP_FREQUENCY_HZ
#define SINV_OUTER_LOOP_FREQUENCY_HZ            1000.0f
#endif



#ifndef SINV_OUTER_LOOP_POWER_LIMIT_PU
#define SINV_OUTER_LOOP_POWER_LIMIT_PU           1.0f
#endif



//------------------------------
// CIA402 state machine
//------------------------------

#ifndef SINV_CIA402_OPERATION_ENABLE_DELAY_MS
#define SINV_CIA402_OPERATION_ENABLE_DELAY_MS    1000
#endif



//------------------------------
// Initial voltage reference
//------------------------------

#ifndef SINV_LEVEL1_VOLTAGE_REF_PU
#define SINV_LEVEL1_VOLTAGE_REF_PU              0.5f
#endif



//=================================================================================================
// global variables


extern cia402_sm_t cia402_sm;



#ifndef SPECIFY_PC_TEST_ENV

#endif // SPECIFY_PC_TEST_ENV



//=================================================================================================
// global functions


//
// User should implement this 3 functions at least
//

void init(void);

void mainloop(void);

void setup_peripheral(void);


//
// For Controller projects user should implement the following functions
//

void ctl_init(void);

void ctl_mainloop(void);



gmp_task_status_t tsk_startup(gmp_task_t* tsk);



#ifdef __cplusplus
}
#endif


#endif // _FILE_USER_MAIN_H_