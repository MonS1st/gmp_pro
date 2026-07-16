
#ifndef _FILE_CTRL_SETTINGS_H_
#define _FILE_CTRL_SETTINGS_H_

//=================================================================================================
// Incremental Debug Options

// BUILD_LEVEL 1: hardware validate, VF, voltage open loop
// BUILD_LEVEL 2: IF, current close loop
// BUILD_LEVEL 3: current loop with actual angle
// BUILD_LEVEL 4: speed loop
// BUILD_LEVEL 5: position loop
// BUILD_LEVEL 6: communication mode
#define BUILD_LEVEL (2)

//=================================================================================================
// Select Board Pin definition
#define LAUNCHPAD 0
#define GMP_IRIS  1

#define BOARD_SELECTION GMP_IRIS


//=================================================================================================
// Controller basic parameters

// Startup Delay, ms
#define CTRL_STARTUP_DELAY (100)

// Controller Frequency
#define CONTROLLER_FREQUENCY (20e3)

// PWM depth
#define CTRL_PWM_CMP_MAX (2500 - 1)

// PWM dead band
#define CTRL_PWM_DEADBAND_CMP (100)

// System tick
#define DSP_C2000_DSP_TIME_DIV (100000 / CTRL_PWM_CMP_MAX / 2)

// ADC Voltage Reference
#define CTRL_ADC_VOLTAGE_REF (3.3f)

//=================================================================================================
// DC power supply ADC sampling parameters

#define PSU_ADC_VREF_V                 (3.3f)
#define PSU_ADC_RESOLUTION_BITS        (12U)
#define PSU_ADC_IQN                    (24U)

#define PSU_VOUT_BASE_V                (10.0f)
#define PSU_IOUT_BASE_A                (0.100f)

#define PSU_VOUT_SENSOR_GAIN           (0.25f)
#define PSU_IOUT_SENSOR_GAIN_V_PER_A   (22.0f)

// Application-layer measurement calibration. Raw ADC diagnostics remain
// unmodified; UI, mode control, and protection use the calibrated values.
#define PSU_VM_CALIBRATION_ADD_MV      (200U)
#define PSU_IM_CALIBRATION_SUB_MA      (0U)

// DAC command conversion parameters
#define PSU_DAC_VREF_MV                (3300UL)
#define PSU_DAC_RESOLUTION_BITS        (12U)
#define PSU_DAC_CODE_SCALE             (4096UL)
#define PSU_DAC_MAX_CODE               (4095U)

#define PSU_VOLTAGE_CMD_MAX_MV         (11000U)
#define PSU_CURRENT_CMD_MAX_MA         (105U)
#define PSU_CV_MODE_FIXED_ISET_MA      (105U)
#define PSU_CC_MODE_FIXED_VSET_MV      (11000U)

#if PSU_CV_MODE_FIXED_ISET_MA > PSU_CURRENT_CMD_MAX_MA
#error "CV-only fixed ISET exceeds the current command limit"
#endif

#if PSU_CC_MODE_FIXED_VSET_MV > PSU_VOLTAGE_CMD_MAX_MV
#error "CC-only fixed VSET exceeds the voltage command limit"
#endif
#define PSU_ENABLE_LOW_RANGE_BRINGUP_LIMITS (0)
// Measured hardware command gain: Vout / Vset = 3611 / 1000 = 3.611.
#define PSU_VOUT_PER_VSET_NUMERATOR    (3611UL)
#define PSU_VOUT_PER_VSET_DENOMINATOR  (1000UL)
#define PSU_ISET_GAIN_MV_PER_MA        (22UL)

// CV/CC mode hysteresis and consecutive-sample confirmation thresholds.
#define PSU_CC_ENTER_CURRENT_MARGIN_MA  (1U)
#define PSU_CC_ENTER_VOLTAGE_DROP_MV    (50U)
#define PSU_CV_RETURN_CURRENT_MARGIN_MA (3U)
#define PSU_CV_RETURN_VOLTAGE_MARGIN_MV (20U)
// Confirmation time is PSU_MODE_CONFIRM_COUNT / CONTROLLER_FREQUENCY:
// 200 samples are approximately 10 ms at the current 20 kHz controller rate.
#define PSU_MODE_CONFIRM_COUNT          (200U)

// Fast OVP/OCP confirmation: 20 kHz is approximately 50 us per sample, so
// five consecutive samples are approximately 250 us, well below the 10 ms
// CV/CC mode-confirmation interval.
#define PSU_OVP_TRIP_MV                 (10300U)
#define PSU_OCP_TRIP_MA                 (103U)
#define PSU_OVP_RELEASE_MV              (10000U)
#define PSU_OCP_RELEASE_MA              (100U)
#define PSU_OVP_CONFIRM_COUNT           (5U)
#define PSU_OCP_CONFIRM_COUNT           (5U)

// Temporary HT16K33 key IDs. The driver reports 1..39 (0 means released),
// but this board has no authoritative key-to-ID map in the repository.
// Adjust these values here after observing the actual key_id over serial.
#define PSU_KEY_VOLTAGE_UP_ID           (1U)
#define PSU_KEY_VOLTAGE_DOWN_ID         (2U)
#define PSU_KEY_CURRENT_UP_ID           (3U)
#define PSU_KEY_CURRENT_DOWN_ID         (4U)
#define PSU_KEY_OUTPUT_TOGGLE_ID        (5U)
#define PSU_KEY_FAULT_RESET_ID          (6U)
#define PSU_KEY_CONTROL_STRATEGY_ID      (7U)
// Preset keys: key IDs 16, 17 and 18 select M1, M2 and M3 respectively.
// g_last_raw_key_id remains available to confirm the physical mapping.
#define PSU_KEY_PRESET_1_ID              (16U)
#define PSU_KEY_PRESET_2_ID              (17U)
#define PSU_KEY_PRESET_3_ID              (18U)

#define PSU_VOLTAGE_STEP_MV             (100U)
#define PSU_CURRENT_STEP_MA             (1U)

#define PSU_ENABLE_ROTARY_ENCODER_UI       (1)

#define PSU_ENCODER_TASK_PERIOD_MS         (2U)
#define PSU_ENCODER_COUNTS_PER_DETENT      (4)
#define PSU_ENCODER_BUTTON_DEBOUNCE_COUNT  (10U)

#define PSU_ENCODER_MODE_VOLTAGE           (0U)
#define PSU_ENCODER_MODE_CURRENT           (1U)

// Must match eqep2.positionCounterMax in F280039_Iris_node.syscfg.
#define PSU_ENCODER_POSITION_COUNTER_MAX   (10000UL)

#define PSU_KEY_TASK_PERIOD_MS                    (20U)
#define PSU_KEY_TASK_INITIAL_DELAY_MS             (10U)
#define PSU_PRESET_AUTO_SAVE_DELAY_MS              (2000U)
#define PSU_HT16K33_DISPLAY_PERIOD_MS             (500U)
#define PSU_HT16K33_DISPLAY_INITIAL_DELAY_MS      (250U)
#define PSU_OLED_TASK_PERIOD_MS                   (100U)
#define PSU_OLED_TASK_INITIAL_DELAY_MS            (100U)
#define PSU_USE_INCREMENTAL_OLED_UI                (1)

#define PSU_I2C_TIMEOUT_TRIGGER_COUNT              (3U)
#define PSU_I2C_RECOVERY_INITIAL_DELAY_MS          (100U)
#define PSU_I2C_RECOVERY_SETTLE_MS                 (20U)
#define PSU_I2C_RECOVERY_RETRY_MS                  (500U)
#define PSU_I2C_RECOVERY_VERIFY_PERIOD_MS          (50U)
#define PSU_I2C_RECOVERY_VERIFY_OK_COUNT           (2U)
#define PSU_I2C_CLEAR_EDGE_DELAY_MS                 (1U)
#define PSU_I2C_CLEAR_MAX_CLOCKS                    (9U)
#define PSU_I2C_CLEAR_SETTLE_MS                     (2U)
#define PSU_I2C_CLEAR_RETRY_MS                      (1000U)

#define PSU_KEY_RELEASE_FILTER_COUNT              (2U)
#define PSU_KEY_ENABLE_REPEAT                     (0)
#define PSU_KEY_VOTE_REQUIRED_COUNT               (2U)
#define PSU_KEY_VOTE_WINDOW_COUNT                 (3U)

//=================================================================================================
// Complete control-board functional test profile
#define PSU_FULL_BOARD_TEST_PROFILE       (1)

#if PSU_FULL_BOARD_TEST_PROFILE

#define PSU_ENABLE_HT16K33_KEY             (1)
#define PSU_ENABLE_HT16K33_DISPLAY         (1)
#define PSU_ENABLE_OLED_DISPLAY            (1)
#define PSU_ENABLE_ANALOG_IO_TEST           (1)
#define PSU_ANALOG_IO_AUTO_FOLLOW_ENABLE    (1)
#define PSU_ANALOG_IO_AUTO_FOLLOW_DELAY_MS  (500U)
#define PSU_ENABLE_ANALOG_BOARD_BRINGUP             (1)
#define PSU_ANALOG_BOARD_USE_REAL_ADC               (1)
#define PSU_ANALOG_BOARD_VOLTAGE_LIMIT_MV           (2000U)
#define PSU_ANALOG_BOARD_CURRENT_LIMIT_MA            (20U)
#define PSU_ANALOG_BOARD_MIN_CURRENT_MA              (10U)
#define PSU_ANALOG_BOARD_ISET_PRECHARGE_MS           (100U)
#define PSU_ANALOG_BOARD_PROTECTION_GRACE_MS          (1500U)
#define PSU_ANALOG_BOARD_ADC_FAULT_CONFIRM_COUNT      (5U)
#define PSU_ANALOG_BOARD_FEEDBACK_VALID_CONFIRM_COUNT (20U)
#define PSU_ANALOG_BOARD_OCP_CONFIRM_COUNT            (2000U)
#define PSU_ANALOG_BOARD_OVP_CONFIRM_COUNT            (2000U)
#define PSU_ANALOG_BOARD_STARTUP_DELAY_MS            (500U)
#define PSU_ANALOG_BOARD_FEEDBACK_SETTLE_MS          (100U)
#define PSU_REAL_FEEDBACK_CONNECTED                   (1)
#define PSU_MODE_MONITOR_ENABLE                       (1)
#define PSU_MODE_MONITOR_TASK_PERIOD_MS               (10U)
#define PSU_MODE_MONITOR_CONFIRM_COUNT                (5U)
#define PSU_MODE_MONITOR_TEST_INJECTION               (1)
#define PSU_CONTROL_POLICY_ENABLE                     (1)
#define PSU_CONTROL_POLICY_TASK_PERIOD_MS             (10U)
#define PSU_CONTROL_POLICY_MISMATCH_CONFIRM_COUNT     (10U)
#define PSU_CONTROL_POLICY_DEFAULT_STRATEGY \
    PSU_CONTROL_STRATEGY_AUTO
#define PSU_CONTROL_POLICY_TEST_TRIP_ENABLE           (1)
#define PSU_ENABLE_CONSOLE_UI              (1)
#define PSU_ENABLE_MANUAL_COMMAND          (1)

#define PSU_ENABLE_STARTUP_LOG             (1)
#define PSU_ENABLE_KEY_EVENT_LOG           (1)
#define PSU_ENABLE_PERIODIC_STATUS_LOG     (1)
#define PSU_ENABLE_SAFE_SELF_TEST          (0)

#define PSU_PERIODIC_STATUS_PERIOD_MS      (1000U)

#define PSU_SAFE_BRINGUP                    (0)
#define PSU_SOFT_TEST_MODE                  (1)

#define PSU_ALLOW_PHYSICAL_DAC              (1)
#define PSU_ALLOW_PHYSICAL_PWM              (0)
#define PSU_ALLOW_PHYSICAL_OUTPUT_ENABLE    (1)
#define PSU_ALLOW_OUTPUT_REQUEST            (1)

#define PSU_ENABLE_BEEP                     (1)
#define PSU_ENABLE_STARTUP_BEEP             (0)
#define PSU_ENABLE_FAULT_BUZZER             (1)

#define PSU_FAULT_BUZZER_PULSE_MS           (300U)
#define PSU_FAULT_LED_TEST_DURATION_MS      (1000U)
#define PSU_FAULT_LED_SELECTED_NUMBER       (6U)
#define PSU_FAULT_LED_GPIO_NUMBER           (44U)

#define PSU_FAULT_BUZZER_TYPE_UNKNOWN        (0U)
#define PSU_FAULT_BUZZER_TYPE_ACTIVE         (1U)
#define PSU_FAULT_BUZZER_TYPE_PASSIVE        (2U)
#define PSU_FAULT_BUZZER_TYPE                PSU_FAULT_BUZZER_TYPE_ACTIVE
#define PSU_BUZZER_GPIO                      IRIS_GPIO1
#define PSU_BUZZER_ACTIVE_LEVEL              (1U)
#define PSU_BUZZER_INACTIVE_LEVEL            (0U)

#define PSU_ENABLE_LOGICAL_OUTPUT_SWITCH        (1)
#define PSU_OUTPUT_SWITCH_PRECHARGE_MS          (100U)
#define PSU_OUTPUT_SWITCH_REQUIRE_SETTLED       (1)
#define PSU_OUTPUT_SWITCH_PHYSICAL_RELAY_ENABLE (1)

#endif

#if PSU_ENABLE_LOW_RANGE_BRINGUP_LIMITS
#define PSU_COMMAND_VOLTAGE_LIMIT_MV PSU_ANALOG_BOARD_VOLTAGE_LIMIT_MV
#define PSU_COMMAND_CURRENT_LIMIT_MA PSU_ANALOG_BOARD_CURRENT_LIMIT_MA
#else
#define PSU_COMMAND_VOLTAGE_LIMIT_MV PSU_VOLTAGE_CMD_MAX_MV
#define PSU_COMMAND_CURRENT_LIMIT_MA PSU_CURRENT_CMD_MAX_MA
#endif

#if (PSU_SAFE_BRINGUP == 1) && \
    ((PSU_ALLOW_PHYSICAL_DAC == 1) || \
     (PSU_ALLOW_PHYSICAL_PWM == 1) || \
     (PSU_ALLOW_PHYSICAL_OUTPUT_ENABLE == 1) || \
     (PSU_ALLOW_OUTPUT_REQUEST == 1))
#error "SAFE_BRINGUP forbids all physical power-output permissions"
#endif

#if PSU_OUTPUT_SWITCH_PHYSICAL_RELAY_ENABLE && \
    ((!PSU_ALLOW_PHYSICAL_OUTPUT_ENABLE) || (!PSU_ALLOW_OUTPUT_REQUEST))
#error "Physical relay requires both output-enable and output-request permissions"
#endif

//=================================================================================================
// Hardware parameters

#define BOOSTXL_3PHGANINV_IS_DEFAULT_PARAM

// invoke motor parameters
#include <ctl/component/hardware_preset/pmsm_motor/TYI_5008_KV335.h>

// invoke motor controller parameters
#include <ctl/component/hardware_preset/inverter_3ph/TI_BOOSTXL_3PhGaNInv.h>

///////////////////////////////////////////////////////////
// Encoder Propeties

// Encoder Full scale
#define CTRL_POS_ENC_FS (8000)

// Speed & Position control division
#define CTRL_MECH_DIV (5)


///////////////////////////////////////////////////////////
// Controller Base value

// DC bus voltage
#define CTRL_DCBUS_VOLTAGE (80.0f)

// phase voltage base, SVPWM modulation
#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE / 1.73205081f)

// voltage base, SPWM modulation
//#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE/2.0f)

// Current base, 10 A
#define CTRL_CURRENT_BASE (10.0f)

///////////////////////////////////////////////////////////
// inverter side sensor

// Current sensor sensitivity, TMCS1133A2B, V/A
#define CTRL_INVERTER_CURRENT_SENSITIVITY (50e-3f)

// Current sensor bias, V
#define CTRL_INVERTER_CURRENT_BIAS (1.65f)

// Voltage sensor sensitivity, V/V
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (0.02738589f)

// Voltage sensor bias, V
#define CTRL_INVERTER_VOLTAGE_BIAS (0.0f)

///////////////////////////////////////////////////////////
// DC Bus side sensor

// Current sensor sensitivity, V/A
#define CTRL_DC_CURRENT_SENSITIVITY (24.75e-3f)

// Current sensor bias, V
#define CTRL_DC_CURRENT_BIAS (1.65f)

// Voltage sensor sensitivity, maximum 120V, V/V
#define CTRL_DC_VOLTAGE_SENSITIVITY (0.02738589f)

// Voltage sensor bias, V
#define CTRL_DC_VOLTAGE_BIAS (0.0f)

//=================================================================================================
// Controller Settings

// Use discrete PID controller
// Discrete controller may bring more smooth response.
//#define PMSM_CTRL_USING_DISCRETE_CTRL

// Enable Discrete PID controller anti-saturation algorithm
#define _USE_DEBUG_DISCRETE_PID

// Enable ADC Calibrate
#define SPECIFY_ENABLE_ADC_CALIBRATE

// SPLL Close loop criteria
#define CTRL_SPLL_EPSILON ((float2ctrl(0.005)))

// Using negative modulator logic
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC (1)

// Using three level modulator or two level modulator
//#define USING_NPC_MODULATOR

//=================================================================================================
// Board peripheral mapping

// Launchpad Board Pin Mapping
#if BOARD_SELECTION == LAUNCHPAD
#ifndef BOARD_PIN_MAPPING
#define BOARD_PIN_MAPPING

// PWM Channels
#define PHASE_U_BASE EPWM_J4_PHASE_U_BASE
#define PHASE_V_BASE EPWM_J4_PHASE_V_BASE
#define PHASE_W_BASE EPWM_J4_PHASE_W_BASE

// PWM Enable
#define PWM_ENABLE_PORT ENABLE_GATE
#define PWM_RESET_PORT  RESET_GATE

// DC Bus Voltage & Current
#define INV_VBUS_RESULT_BASE J3_VDC_RESULT_BASE
#define INV_IBUS_RESULT_BASE J7_VDC_RESULT_BASE

#define INV_VBUS J3_VDC
#define INV_IBUS J7_VDC

// Grid side Voltage & Current
#define INV_UA_RESULT_BASE J7_VU_RESULT_BASE
#define INV_UB_RESULT_BASE J7_VV_RESULT_BASE
#define INV_UC_RESULT_BASE J7_VW_RESULT_BASE

#define INV_UA J7_VU
#define INV_UB J7_VV
#define INV_UC J7_VW

#define INV_IA_RESULT_BASE J7_IU_RESULT_BASE
#define INV_IB_RESULT_BASE J7_IV_RESULT_BASE
#define INV_IC_RESULT_BASE J7_IW_RESULT_BASE

#define INV_IA J7_IU
#define INV_IB J7_IV
#define INV_IC J7_IW

// Converter side Voltage & Current
#define INV_UU_RESULT_BASE J3_VU_RESULT_BASE
#define INV_UV_RESULT_BASE J3_VV_RESULT_BASE
#define INV_UW_RESULT_BASE J3_VW_RESULT_BASE

#define INV_UU J3_VU
#define INV_UV J3_VV
#define INV_UW J3_VW

#define INV_IU_RESULT_BASE J3_IU_RESULT_BASE
#define INV_IV_RESULT_BASE J3_IV_RESULT_BASE
#define INV_IW_RESULT_BASE J3_IW_RESULT_BASE

#define INV_IU J3_IU
#define INV_IV J3_IV
#define INV_IW J3_IW

// System LED
#define SYSTEM_LED     LED_R
#define CONTROLLER_LED LED_G

#endif //BOARD_PIN_MAPPING

#else // BOARD_SELECTION == GMP_IRIS

#ifndef BOARD_PIN_MAPPING
#define BOARD_PIN_MAPPING

// PWM Channels
#define PHASE_U_BASE    IRIS_EPWM1_BASE
#define PHASE_V_BASE    IRIS_EPWM2_BASE
#define PHASE_W_BASE    IRIS_EPWM3_BASE

// GPIO58/IRIS_GPIO1 is reserved for the confirmed active beeper. The legacy
// motor-template PWM enable alias must not create a second owner for that pin.
#if !PSU_SAFE_BRINGUP
#define PWM_RESET_PORT  IRIS_GPIO3
#endif

// Vbus Voltage Channels
//#define MOTOR_VBUS_RESULT_BASE IRIS_ADCA_RESULT_BASE
//#define MOTOR_VBUS

// ADC Voltage Channels
//#define MOTOR_VU_RESULT_BASE IRIS_ADCA_RESULT_BASE
//#define MOTOR_VV_RESULT_BASE IRIS_ADCB_RESULT_BASE
//#define MOTOR_VW_RESULT_BASE IRIS_ADCC_RESULT_BASE

//#define MOTOR_VU
//#define MOTOR_VV
//#define MOTOR_VW

// ADC Current Channels
//#define MOTOR_IU_RESULT_BASE IRIS_ADCA_RESULT_BASE
//#define MOTOR_IV_RESULT_BASE IRIS_ADCB_RESULT_BASE
//#define MOTOR_IW_RESULT_BASE IRIS_ADCC_RESULT_BASE

//#define MOTOR_IU
//#define MOTOR_IV
//#define MOTOR_IW

// System LED
#define SYSTEM_LED      IRIS_LED1
#define CONTROLLER_LED  IRIS_LED2

#endif //BOARD_PIN_MAPPING

#endif // BOARD_PIN_MAPPING

#endif // _FILE_CTRL_SETTINGS_H_
