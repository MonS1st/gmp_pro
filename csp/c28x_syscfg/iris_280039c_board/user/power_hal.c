#include "power_hal.h"

#include <gmp_core.h>
#include <xplt.peripheral.h>
#include <ctrl_settings.h>
#include <driverlib.h>
#include <board.h>

static volatile bool s_power_output_hw_enabled = false;

volatile uint16_t g_epwm_fault_command = 0U;
volatile uint16_t g_epwm_fault_level = 0U;
volatile uint32_t g_epwm_fault_assert_count = 0U;
volatile uint32_t g_epwm_fault_clear_count = 0U;

#define POWER_FAULT_EPWM_BASE    (IRIS_EPWM4_BASE)
#define POWER_FAULT_EPWM_PERIOD  (IRIS_EPWM4_TBPRD)

#if !PSU_ENABLE_ANALOG_IO_TEST
static void power_dac_force_physical_zero(void)
{
    DAC_setShadowValue(IRIS_DACA_BASE, 0U);
    DAC_setShadowValue(IRIS_DACB_BASE, 0U);
}
#endif

static uint16_t power_limit_dac_code(uint32_t code)
{
    return (code > PSU_DAC_MAX_CODE) ? PSU_DAC_MAX_CODE : (uint16_t)code;
}

uint16_t power_voltage_mv_to_dac(uint16_t voltage_mv)
{
    uint32_t limited_mv = voltage_mv;
    uint32_t code;

    if (limited_mv > PSU_VOLTAGE_CMD_MAX_MV)
    {
        limited_mv = PSU_VOLTAGE_CMD_MAX_MV;
    }

    // Vset = Vout_cmd / 4; DAC code = Vset / 3300 mV * 4096.
    code = limited_mv * PSU_DAC_CODE_SCALE / (PSU_VSET_DIVIDER * PSU_DAC_VREF_MV);
    return power_limit_dac_code(code);
}

uint16_t power_current_ma_to_dac(uint16_t current_ma)
{
    uint32_t limited_ma = current_ma;
    uint32_t code;

    if (limited_ma > PSU_CURRENT_CMD_MAX_MA)
    {
        limited_ma = PSU_CURRENT_CMD_MAX_MA;
    }

    // V_ISET(mV) = 22 * I_cmd(mA); DAC code = V_ISET / 3300 mV * 4096.
    code = limited_ma * PSU_ISET_GAIN_MV_PER_MA * PSU_DAC_CODE_SCALE / PSU_DAC_VREF_MV;
    return power_limit_dac_code(code);
}

void power_dac_set_voltage_mv(uint16_t voltage_mv)
{
#if PSU_ENABLE_ANALOG_IO_TEST
    // The isolated calibration task is the only runtime owner of real DACs.
    (void)voltage_mv;
#elif PSU_SAFE_BRINGUP || !PSU_ALLOW_PHYSICAL_DAC || PSU_SOFT_TEST_MODE
    // SOFT_TEST does not automatically isolate the physical hardware. Force
    // both real DAC targets to zero explicitly on every command path.
    (void)voltage_mv;
    power_dac_force_physical_zero();
#else
    DAC_setShadowValue(IRIS_DACA_BASE, power_voltage_mv_to_dac(voltage_mv));
#endif
}

void power_dac_set_current_ma(uint16_t current_ma)
{
#if PSU_ENABLE_ANALOG_IO_TEST
    // The isolated calibration task is the only runtime owner of real DACs.
    (void)current_ma;
#elif PSU_SAFE_BRINGUP || !PSU_ALLOW_PHYSICAL_DAC || PSU_SOFT_TEST_MODE
    // SOFT_TEST does not automatically isolate the physical hardware. Force
    // both real DAC targets to zero explicitly on every command path.
    (void)current_ma;
    power_dac_force_physical_zero();
#else
    DAC_setShadowValue(IRIS_DACB_BASE, power_current_ma_to_dac(current_ma));
#endif
}

void power_dac_set_zero(void)
{
#if PSU_ENABLE_ANALOG_IO_TEST
    // Zeroing is owned by analog_io_test so power_app cannot overwrite a
    // manually armed calibration code from the 20 kHz control callback.
    return;
#else
    power_dac_force_physical_zero();
#endif
}

void power_fault_epwm_init(void)
{
    // Assert the fail-safe state before changing any time-base or AQ setting.
    EPWM_setActionQualifierContSWForceShadowMode(
        POWER_FAULT_EPWM_BASE, EPWM_AQ_SW_IMMEDIATE_LOAD);
    EPWM_setActionQualifierContSWForceAction(
        POWER_FAULT_EPWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
    EPWM_setActionQualifierContSWForceAction(
        POWER_FAULT_EPWM_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_OUTPUT_LOW);

    // Configure this channel independently of EPWM1. EPWM1 remains the ADC
    // SOCA/sample timebase; EPWM4A is an active-high OVP/OCP fault output.
    EPWM_setClockPrescaler(POWER_FAULT_EPWM_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    EPWM_selectPeriodLoadEvent(POWER_FAULT_EPWM_BASE,
                               EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);
    EPWM_setTimeBasePeriod(POWER_FAULT_EPWM_BASE,
                           POWER_FAULT_EPWM_PERIOD);
    EPWM_setTimeBaseCounter(POWER_FAULT_EPWM_BASE, 0U);
    EPWM_setTimeBaseCounterMode(POWER_FAULT_EPWM_BASE,
                                EPWM_COUNTER_MODE_UP);
    EPWM_setCountModeAfterSync(POWER_FAULT_EPWM_BASE,
                               EPWM_COUNT_MODE_UP_AFTER_SYNC);
    EPWM_disablePhaseShiftLoad(POWER_FAULT_EPWM_BASE);
    EPWM_setPhaseShift(POWER_FAULT_EPWM_BASE, 0U);
    EPWM_disableOneShotSync(POWER_FAULT_EPWM_BASE);
    EPWM_disableSyncOutPulseSource(POWER_FAULT_EPWM_BASE,
                                   EPWM_SYNC_OUT_PULSE_ON_ALL);

    EPWM_setCounterCompareValue(POWER_FAULT_EPWM_BASE,
                                EPWM_COUNTER_COMPARE_A, 0U);
    EPWM_setCounterCompareShadowLoadMode(POWER_FAULT_EPWM_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    EPWM_setActionQualifierAction(
        POWER_FAULT_EPWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(
        POWER_FAULT_EPWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(
        POWER_FAULT_EPWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(
        POWER_FAULT_EPWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(
        POWER_FAULT_EPWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(
        POWER_FAULT_EPWM_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

    // No complementary output path and no dead-band processing.
    EPWM_setDeadBandOutputSwapMode(POWER_FAULT_EPWM_BASE,
                                   EPWM_DB_OUTPUT_A, false);
    EPWM_setDeadBandOutputSwapMode(POWER_FAULT_EPWM_BASE,
                                   EPWM_DB_OUTPUT_B, false);
    EPWM_setDeadBandDelayMode(POWER_FAULT_EPWM_BASE, EPWM_DB_RED, false);
    EPWM_setDeadBandDelayMode(POWER_FAULT_EPWM_BASE, EPWM_DB_FED, false);

    EPWM_disableInterrupt(POWER_FAULT_EPWM_BASE);
    EPWM_disableADCTrigger(POWER_FAULT_EPWM_BASE, EPWM_SOC_A);
    EPWM_disableADCTrigger(POWER_FAULT_EPWM_BASE, EPWM_SOC_B);

    power_fault_epwm_set(false);
}

void power_fault_epwm_set(bool fault_active)
{
    uint16_t requested_level = fault_active ? 1U : 0U;
    bool level_changed = (g_epwm_fault_level != requested_level);

    g_epwm_fault_command = requested_level;

    if (!fault_active)
    {
        // Immediate continuous force makes the normal state independent of
        // AQ event collisions at TBCTR zero or a compare boundary.
        EPWM_setActionQualifierContSWForceAction(
            POWER_FAULT_EPWM_BASE, EPWM_AQ_OUTPUT_A,
            EPWM_AQ_SW_OUTPUT_LOW);
        EPWM_setCounterCompareValue(POWER_FAULT_EPWM_BASE,
                                    EPWM_COUNTER_COMPARE_A, 0U);
        g_epwm_fault_level = 0U;
        if (level_changed)
        {
            ++g_epwm_fault_clear_count;
        }
        return;
    }

    // Keep CMPA equal to TBPRD as the asserted-state representation, while
    // continuous HIGH guarantees no short low pulse at the period boundary.
    EPWM_setCounterCompareValue(POWER_FAULT_EPWM_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                POWER_FAULT_EPWM_PERIOD);
    EPWM_setActionQualifierContSWForceAction(
        POWER_FAULT_EPWM_BASE, EPWM_AQ_OUTPUT_A,
        EPWM_AQ_SW_OUTPUT_HIGH);
    g_epwm_fault_level = 1U;
    if (level_changed)
    {
        ++g_epwm_fault_assert_count;
    }
}

bool power_fault_epwm_get(void)
{
    return g_epwm_fault_level != 0U;
}

void power_output_hw_set(bool enable)
{
#if PSU_SAFE_BRINGUP || !PSU_ALLOW_PHYSICAL_OUTPUT_ENABLE || \
    !PSU_OUTPUT_SWITCH_PHYSICAL_RELAY_ENABLE
    (void)enable;
    s_power_output_hw_enabled = false;
#else
    s_power_output_hw_enabled = enable;
#endif
}

void power_output_hw_service(void)
{
    // Output state is software-only. No EPWM, FPGA SPI, or blocking work.
}

bool power_output_hw_get(void)
{
#if PSU_SAFE_BRINGUP || !PSU_ALLOW_PHYSICAL_OUTPUT_ENABLE || \
    !PSU_OUTPUT_SWITCH_PHYSICAL_RELAY_ENABLE
    return false;
#else
    return s_power_output_hw_enabled;
#endif
}

bool power_hal_safe_bringup_self_test(void)
{
#if PSU_SAFE_BRINGUP
    power_output_hw_set(true);
    power_dac_set_voltage_mv(PSU_VOLTAGE_CMD_MAX_MV);
    power_dac_set_current_ma(PSU_CURRENT_CMD_MAX_MA);

    return !power_output_hw_get() &&
           (DAC_getShadowValue(IRIS_DACA_BASE) == 0U) &&
           (DAC_getShadowValue(IRIS_DACB_BASE) == 0U);
#else
    return true;
#endif
}
