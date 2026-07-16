//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all definitions of peripheral objects in this file.
//
// User should implement the peripheral objects initialization in setup_peripheral function.
//
// This file is platform-related.
//

// GMP basic core header
#include <gmp_core.h>

#include "user_main.h"
#include <xplt.ctl_interface.h>
#include <xplt.peripheral.h>
#include <ctl/component/interface/gain_model.h>


//=================================================================================================
// definitions of peripheral

// DC power supply voltage and current feedback
adc_channel_t adc_vout;
adc_channel_t adc_iout;

volatile uint16_t g_adc_vout_raw = 0U;
volatile uint16_t g_adc_iout_raw = 0U;
volatile uint16_t g_fpga_spi_last_rx0 = 0xFFFFU;
volatile uint16_t g_fpga_spi_last_rx1 = 0xFFFFU;

volatile float g_vout_meas_v = 0.0f;
volatile float g_iout_meas_a = 0.0f;
volatile float g_iout_meas_ma = 0.0f;

// inverter side voltage feedback
tri_ptr_adc_channel_t uuvw;
adc_gt uuvw_src[3];

// inverter side current feedback
tri_ptr_adc_channel_t iuvw;
adc_gt iuvw_src[3];

// grid side voltage feedback
tri_ptr_adc_channel_t vabc;
adc_gt vabc_src[3];

// grid side current feedback
tri_ptr_adc_channel_t iabc;
adc_gt iabc_src[3];

// DC bus current & voltage feedback
ptr_adc_channel_t udc;
adc_gt udc_src;
ptr_adc_channel_t idc;
adc_gt idc_src;

//=================================================================================================
// peripheral setup function

extern iic_halt iic_bus;
extern gpio_halt user_led;
extern gpio_halt gpio_beep;
extern gpio_halt gpio_fault_led;

//
// Function to configure I2C A in FIFO mode.
//
void initI2C(void)
{
    //
    // Must put I2C into reset before configuring it
    //
    I2C_disableModule(I2CA_BASE);

    //
    // I2C configuration. Use a 400kHz I2CCLK with a 33% duty cycle.
    //
    I2C_initController(I2CA_BASE, DEVICE_SYSCLK_FREQ, 400000, I2C_DUTYCYCLE_33);
    I2C_setBitCount(I2CA_BASE, I2C_BITCOUNT_8);
    I2C_setTargetAddress(I2CA_BASE, 0x70);
    I2C_setEmulationMode(I2CA_BASE, I2C_EMULATION_FREE_RUN);

    //
    // Enable stop condition and register-access-ready interrupts
    I2C_enableInterrupt(I2CA_BASE, I2C_INT_STOP_CONDITION |
                                   I2C_INT_REG_ACCESS_RDY);

    //
    // FIFO configuration
    //
    I2C_enableFIFO(I2CA_BASE);
    I2C_clearInterruptStatus(I2CA_BASE, I2C_INT_RXFF | I2C_INT_TXFF);

    //
    // Configuration complete. Enable the module.
    //
    I2C_enableModule(I2CA_BASE);
}

typedef enum
{
    BOARD_I2C_CLEAR_PHASE_IDLE = 0,
    BOARD_I2C_CLEAR_PHASE_SETTLE,
    BOARD_I2C_CLEAR_PHASE_CLOCK_LOW_WAIT,
    BOARD_I2C_CLEAR_PHASE_CLOCK_HIGH_WAIT,
    BOARD_I2C_CLEAR_PHASE_STOP_LOW_WAIT,
    BOARD_I2C_CLEAR_PHASE_STOP_HIGH_WAIT,
    BOARD_I2C_CLEAR_PHASE_RESTORE,
    BOARD_I2C_CLEAR_PHASE_DONE
} board_i2c_clear_phase_t;

static board_i2c_clear_phase_t s_board_i2c_clear_phase =
    BOARD_I2C_CLEAR_PHASE_IDLE;
static board_i2c_clear_result_t s_board_i2c_clear_result =
    BOARD_I2C_CLEAR_NOT_RUN;
static time_gt s_board_i2c_clear_next_tick = 0U;

static bool board_i2c_clear_tick_due(time_gt deadline)
{
    time_gt now = gmp_base_get_system_tick();

    return ((int32_t)(now - deadline) >= 0);
}

void board_i2c_restore_peripheral_mode(void)
{
    GPIO_setPinConfig(IRIS_IIC_I2CSDA_PIN_CONFIG);
    GPIO_setPadConfig(IRIS_IIC_I2CSDA_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(IRIS_IIC_I2CSDA_GPIO, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(IRIS_IIC_I2CSCL_PIN_CONFIG);
    GPIO_setPadConfig(IRIS_IIC_I2CSCL_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(IRIS_IIC_I2CSCL_GPIO, GPIO_QUAL_ASYNC);

    g_i2c_clear_pinmux_restored = 1U;
}

void board_i2c_bus_clear_begin(void)
{
    I2C_disableModule(I2CA_BASE);

    // Preload released levels before GPIO takes ownership of the pins. With
    // open-drain enabled, writing one releases the line to the pull-up.
    GPIO_writePin(IRIS_IIC_I2CSDA_GPIO, 1U);
    GPIO_writePin(IRIS_IIC_I2CSCL_GPIO, 1U);

    GPIO_setPadConfig(IRIS_IIC_I2CSDA_GPIO,
                      GPIO_PIN_TYPE_OD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(IRIS_IIC_I2CSDA_GPIO, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(IRIS_IIC_I2CSDA_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPinConfig(GPIO_56_GPIO56);

    GPIO_setPadConfig(IRIS_IIC_I2CSCL_GPIO,
                      GPIO_PIN_TYPE_OD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(IRIS_IIC_I2CSCL_GPIO, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(IRIS_IIC_I2CSCL_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPinConfig(GPIO_57_GPIO57);

    ++g_i2c_clear_attempt_count;
    g_i2c_clear_state = BOARD_I2C_CLEAR_IN_PROGRESS;
    g_i2c_clear_clock_count = 0U;
    g_i2c_clear_sda_initial = 0U;
    g_i2c_clear_sda_final = 0U;
    g_i2c_clear_scl_final = 0U;
    g_i2c_clear_stop_generated = 0U;
    g_i2c_clear_pinmux_restored = 0U;

    s_board_i2c_clear_result = BOARD_I2C_CLEAR_IN_PROGRESS;
    s_board_i2c_clear_phase = BOARD_I2C_CLEAR_PHASE_SETTLE;
    s_board_i2c_clear_next_tick =
        gmp_base_get_system_tick() + (time_gt)PSU_I2C_CLEAR_SETTLE_MS;
}

board_i2c_clear_result_t board_i2c_bus_clear_step(void)
{
    if (s_board_i2c_clear_phase == BOARD_I2C_CLEAR_PHASE_IDLE)
    {
        return BOARD_I2C_CLEAR_NOT_RUN;
    }

    if (s_board_i2c_clear_phase == BOARD_I2C_CLEAR_PHASE_DONE)
    {
        return s_board_i2c_clear_result;
    }

    if (!board_i2c_clear_tick_due(s_board_i2c_clear_next_tick))
    {
        return BOARD_I2C_CLEAR_IN_PROGRESS;
    }

    switch (s_board_i2c_clear_phase)
    {
    case BOARD_I2C_CLEAR_PHASE_SETTLE:
        g_i2c_clear_sda_initial = board_i2c_read_sda_level();
        if (g_i2c_clear_sda_initial != 0U)
        {
            // SCL is already released high. Pull SDA low to begin STOP.
            GPIO_writePin(IRIS_IIC_I2CSDA_GPIO, 0U);
            s_board_i2c_clear_phase = BOARD_I2C_CLEAR_PHASE_STOP_LOW_WAIT;
        }
        else
        {
            // Begin the first recovery clock with one active-low edge.
            GPIO_writePin(IRIS_IIC_I2CSCL_GPIO, 0U);
            s_board_i2c_clear_phase = BOARD_I2C_CLEAR_PHASE_CLOCK_LOW_WAIT;
        }
        s_board_i2c_clear_next_tick =
            gmp_base_get_system_tick() +
            (time_gt)PSU_I2C_CLEAR_EDGE_DELAY_MS;
        return BOARD_I2C_CLEAR_IN_PROGRESS;

    case BOARD_I2C_CLEAR_PHASE_CLOCK_LOW_WAIT:
        // Open-drain one releases SCL; count each released high phase once.
        GPIO_writePin(IRIS_IIC_I2CSCL_GPIO, 1U);
        ++g_i2c_clear_clock_count;
        s_board_i2c_clear_phase = BOARD_I2C_CLEAR_PHASE_CLOCK_HIGH_WAIT;
        s_board_i2c_clear_next_tick =
            gmp_base_get_system_tick() +
            (time_gt)PSU_I2C_CLEAR_EDGE_DELAY_MS;
        return BOARD_I2C_CLEAR_IN_PROGRESS;

    case BOARD_I2C_CLEAR_PHASE_CLOCK_HIGH_WAIT:
        if ((board_i2c_read_sda_level() != 0U) ||
            (g_i2c_clear_clock_count >= PSU_I2C_CLEAR_MAX_CLOCKS))
        {
            // Leave SCL released high and pull SDA low to begin STOP.
            GPIO_writePin(IRIS_IIC_I2CSDA_GPIO, 0U);
            s_board_i2c_clear_phase = BOARD_I2C_CLEAR_PHASE_STOP_LOW_WAIT;
        }
        else
        {
            GPIO_writePin(IRIS_IIC_I2CSCL_GPIO, 0U);
            s_board_i2c_clear_phase = BOARD_I2C_CLEAR_PHASE_CLOCK_LOW_WAIT;
        }
        s_board_i2c_clear_next_tick =
            gmp_base_get_system_tick() +
            (time_gt)PSU_I2C_CLEAR_EDGE_DELAY_MS;
        return BOARD_I2C_CLEAR_IN_PROGRESS;

    case BOARD_I2C_CLEAR_PHASE_STOP_LOW_WAIT:
        // Releasing SDA while SCL is released high is the STOP edge.
        GPIO_writePin(IRIS_IIC_I2CSDA_GPIO, 1U);
        g_i2c_clear_stop_generated = 1U;
        s_board_i2c_clear_phase = BOARD_I2C_CLEAR_PHASE_STOP_HIGH_WAIT;
        s_board_i2c_clear_next_tick =
            gmp_base_get_system_tick() +
            (time_gt)PSU_I2C_CLEAR_EDGE_DELAY_MS;
        return BOARD_I2C_CLEAR_IN_PROGRESS;

    case BOARD_I2C_CLEAR_PHASE_STOP_HIGH_WAIT:
        // Keep pinmux restoration in its own scheduler step.
        s_board_i2c_clear_phase = BOARD_I2C_CLEAR_PHASE_RESTORE;
        return BOARD_I2C_CLEAR_IN_PROGRESS;

    case BOARD_I2C_CLEAR_PHASE_RESTORE:
        board_i2c_restore_peripheral_mode();
        g_i2c_clear_sda_final = board_i2c_read_sda_level();
        g_i2c_clear_scl_final = board_i2c_read_scl_level();

        if ((g_i2c_clear_sda_final != 0U) &&
            (g_i2c_clear_scl_final != 0U) &&
            (g_i2c_clear_stop_generated != 0U) &&
            (g_i2c_clear_pinmux_restored != 0U))
        {
            s_board_i2c_clear_result = BOARD_I2C_CLEAR_RELEASED;
        }
        else
        {
            s_board_i2c_clear_result = BOARD_I2C_CLEAR_STILL_LOW;
        }
        g_i2c_clear_state = (uint16_t)s_board_i2c_clear_result;
        s_board_i2c_clear_phase = BOARD_I2C_CLEAR_PHASE_DONE;
        return s_board_i2c_clear_result;

    case BOARD_I2C_CLEAR_PHASE_IDLE:
    case BOARD_I2C_CLEAR_PHASE_DONE:
    default:
        return s_board_i2c_clear_result;
    }
}

void board_i2c_controller_reinit(void)
{
    initI2C();
    iic_bus = I2CA_BASE;
}

uint16_t board_i2c_read_sda_level(void)
{
    return (uint16_t)GPIO_readPin(IRIS_IIC_I2CSDA_GPIO);
}

uint16_t board_i2c_read_scl_level(void)
{
    return (uint16_t)GPIO_readPin(IRIS_IIC_I2CSCL_GPIO);
}



// User should setup all the peripheral in this function.
void setup_peripheral(void)
{
#if PSU_SAFE_BRINGUP
    // SysConfig deliberately exposes no EPWM A/B pins in this build. Force
    // every internal PWM module into one-shot trip before other user setup.
    ctl_force_all_pwm_trip();
#endif

    parameter_gt vout_adc_gain = ctl_gain_calc_generic(PSU_ADC_VREF_V, PSU_VOUT_SENSOR_GAIN,
                                                       PSU_VOUT_BASE_V);
    parameter_gt iout_adc_gain = ctl_gain_calc_generic(PSU_ADC_VREF_V, PSU_IOUT_SENSOR_GAIN_V_PER_A,
                                                       PSU_IOUT_BASE_A);

    ctl_init_adc_channel(&adc_vout, float2ctrl(vout_adc_gain), float2ctrl(0.0f),
                         PSU_ADC_RESOLUTION_BITS, PSU_ADC_IQN);
    ctl_init_adc_channel(&adc_iout, float2ctrl(iout_adc_gain), float2ctrl(0.0f),
                         PSU_ADC_RESOLUTION_BITS, PSU_ADC_IQN);

    // Setup Debug Uart
    debug_uart = IRIS_UART_USB_BASE;

    // Test print function
    gmp_base_print(TEXT_STRING("Hello World!\r\n"));
    asm(" RPT #255 || NOP");

    //
    // Initialize GPIOs for use as SDA A and SCL A respectively
    //
    // GPIO_setPinConfig(DEVICE_GPIO_CFG_SDAA);
    GPIO_setPadConfig(IRIS_IIC_I2CSDA_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(IRIS_IIC_I2CSDA_GPIO, GPIO_QUAL_ASYNC);

    // GPIO_setPinConfig(DEVICE_GPIO_CFG_SCLA);
    GPIO_setPadConfig(IRIS_IIC_I2CSCL_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(IRIS_IIC_I2CSCL_GPIO, GPIO_QUAL_ASYNC);

    GPIO_setPadConfig(IRIS_IIC_I2CSDA_GPIO, GPIO_PIN_TYPE_PULLUP);

    //
    // Set I2C use, initializing it for FIFO mode
    //
    initI2C();

    iic_bus = I2CA_BASE;

    user_led = SYSTEM_LED;

#if PSU_ENABLE_BEEP && !PSU_SAFE_BRINGUP
    gpio_beep = IRIS_GPIO1;
#endif

    gpio_fault_led = IRIS_GPIO4;
    gmp_hal_gpio_write(gpio_fault_led, 0U);

}



//=================================================================================================
// ADC Interrupt ISR and controller related function

// ADC interrupt
interrupt void MainISR(void)
{
    ++g_main_isr_count;

    //
    // call GMP ISR  Controller operation callback function
    //
    gmp_base_ctl_step();

    //
    // Call GMP Timer
    //
    gmp_step_system_tick();

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(IRIS_ADCA_BASE, ADC_INT_NUMBER1);

    //
    // Check if overflow has occurred
    //
    if (true == ADC_getInterruptOverflowStatus(IRIS_ADCA_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(IRIS_ADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(IRIS_ADCA_BASE, ADC_INT_NUMBER1);
    }

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_IRIS_ADCA_1_INTERRUPT_ACK_GROUP);
}

void reset_controller(void)
{
#if PSU_SAFE_BRINGUP || !PSU_ALLOW_PHYSICAL_OUTPUT_ENABLE
    // PWM_RESET_PORT is an unconfirmed legacy mapping. Never toggle it in the
    // control-board-only safe build.
    return;
#else
    int i = 0;

    GPIO_WritePin(PWM_RESET_PORT, 0);

    for(i=0;i<10000;++i);

    GPIO_WritePin(PWM_RESET_PORT, 1);
#endif

}

//=================================================================================================
// communication functions and interrupt functions here

// 10000 -> 1.0
#define CAN_SCALE_FACTOR 10000

// 32 bit union
typedef union {
    int32_t i32;
    uint16_t u16[2]; // C2000中uint16_t占1个word，32位占用2个word
} can_data_t;

// CAN interrupt
interrupt void INT_IRIS_CAN_0_ISR(void)
{
    uint32_t status = CAN_getInterruptCause(IRIS_CAN_BASE);

    uint16_t rx_data[4];
    can_data_t recv_content[2];

    if (status == 1)
    {
        CAN_readMessage(IRIS_CAN_BASE, 1, rx_data);
        CAN_clearInterruptStatus(CANA_BASE, 1);

        // Control Flag, Enable System
//        if (rx_data[0] == 1)
//        {
//            cia402_send_cmd(&cia402_sm, CIA402_CMD_ENABLE_OPERATION);
//        }
//        if (rx_data[0] == 0)
//        {
//            cia402_send_cmd(&cia402_sm, CIA402_CMD_DISABLE_VOLTAGE);
//        }
    }
    else if (status == 2)
    {
        CAN_readMessage(IRIS_CAN_BASE, 2, (uint16_t*)recv_content);
        CAN_clearInterruptStatus(CANA_BASE, 2);

        // set target value
#if BUILD_LEVEL == 1
        // For level 1 Set target voltage
        ctl_set_gfl_inv_voltage_openloop(&inv_ctrl, float2ctrl((float)recv_content[0].i32 / CAN_SCALE_FACTOR),
                                         float2ctrl((float)recv_content[1].i32 / CAN_SCALE_FACTOR));

#endif // BUILD_LEVEL
    }

    //
    // Clear the interrupt flag
    //
    CAN_clearGlobalInterruptStatus(IRIS_CAN_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_IRIS_CAN_0_INTERRUPT_ACK_GROUP);
}

interrupt void INT_IRIS_CAN_1_ISR(void)
{
    // Nothing here

    //
    // Clear the interrupt flag
    //
    CAN_clearGlobalInterruptStatus(IRIS_CAN_BASE, CAN_GLOBAL_INT_CANINT1);

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_IRIS_CAN_1_INTERRUPT_ACK_GROUP);
}

void send_monitor_data(void)
{
    uint16_t rx_raw[4];
    can_data_t tran_content[2];

    // 0x201: Monitor Grid Voltage
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

    CAN_sendMessage(IRIS_CAN_BASE, 4, 8, (uint16_t*)tran_content);

    //0x202: Monitor inverter voltage
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

    CAN_sendMessage(IRIS_CAN_BASE, 5, 8, (uint16_t*)tran_content);

    // 0x203: Monitor grid current
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

    CAN_sendMessage(IRIS_CAN_BASE, 6, 8, (uint16_t*)tran_content);

    // 0x204: TODO Monitor inverter current
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

    CAN_sendMessage(IRIS_CAN_BASE, 7, 8, (uint16_t*)tran_content);

    // 0x205: TODO Monitor DC Voltage / Current
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

    CAN_sendMessage(IRIS_CAN_BASE, 8, 8, (uint16_t*)tran_content);

    // 0x206: Monitor Grid Voltage A and PLL output angle
//    tran_content[0].i32 = (int32_t)(inv_ctrl.vabc.dat[phase_A] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.pll.theta * CAN_SCALE_FACTOR);

    CAN_sendMessage(IRIS_CAN_BASE, 9, 8, (uint16_t*)tran_content);

    // 0x207: Monitor reserved
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

    CAN_sendMessage(IRIS_CAN_BASE, 10, 8, (uint16_t*)tran_content);
}

#if BOARD_SELECTION == GMP_IRIS

interrupt void INT_IRIS_UART_RS232_RX_ISR(void)
{
    // Nothing here

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_IRIS_UART_RS232_RX_INTERRUPT_ACK_GROUP);
}

#endif // BOARD_SELECTION == GMP_IRIS

//=================================================================================================
// Debug interface

// a local small cache size, capable of covering the depth of the hardware FIFO (typically 16 bytes)
#define ISR_LOCAL_BUF_SIZE 16

extern gmp_datalink_t dl;

void flush_dl_tx_buffer()
{
    // Send head
    gmp_hal_uart_write(IRIS_UART_USB_BASE, gmp_dev_dl_get_tx_hw_hdr_ptr(&dl), gmp_dev_dl_get_tx_hw_hdr_size(&dl), 10);

    // Send data body, if necessary
    if (gmp_dev_dl_get_tx_hw_pld_size(&dl) > 0)
    {
        gmp_hal_uart_write(IRIS_UART_USB_BASE, gmp_dev_dl_get_tx_hw_pld_ptr(&dl), gmp_dev_dl_get_tx_hw_pld_size(&dl),
                           10);
    }
}

void flush_dl_rx_buffer()
{
    uint16_t fifoLevel;
    data_gt rxBuf[ISR_LOCAL_BUF_SIZE];

    // read all FIFO messages
    fifoLevel = SCI_getRxFIFOStatus(IRIS_UART_USB_BASE);

    if (fifoLevel > 0)
    {
        SCI_readCharArray(IRIS_UART_USB_BASE, (uint16_t*)rxBuf, fifoLevel);

        // Lock-free ring queue pushed into the protocol stack (very fast, O(1))
        gmp_dev_dl_push_str(&dl, rxBuf, fifoLevel);
    }
}

interrupt void INT_IRIS_UART_USB_RX_ISR(void)
{
    flush_dl_rx_buffer();

    //
    // deal with overrun
    //
    if (SCI_getRxStatus(IRIS_UART_USB_BASE) & SCI_RXSTATUS_OVERRUN)
    {
        SCI_clearOverflowStatus(IRIS_UART_USB_BASE);
    }

    //
    // Clear interrupt flags
    //
    SCI_clearInterruptStatus(IRIS_UART_USB_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INT_IRIS_UART_USB_RX_INTERRUPT_ACK_GROUP);
}

////


//=========================================================
// 1. SPI 读写底层函数封装
//=========================================================

static void fpga_spi_clear_rx_fifo(void)
{
    while(SPI_getRxFIFOStatus(IRIS_SPI_FPGA_BRIDGE_BASE) !=
          SPI_FIFO_RXEMPTY)
    {
        (void)SPI_readDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE);
    }
}

// 向 FPGA 写入寄存器
// 协议: 帧1=[15位=1(写), 14:8=地址, 7:0=保留] -> 帧2=[16位数据]
void SPI_writeReg(uint16_t addr, uint16_t data)
{
    // 构造写命令，最高位为 0
    uint16_t cmd = 0x0000 | ((addr & 0x7F) << 8); // 最高位自然是 0

    fpga_spi_clear_rx_fifo();
    GPIO_writePin(IRIS_GPIO_SPI_CS, 0U);
    DEVICE_DELAY_US(2U);

    // 将两个 16-bit word 压入 TX FIFO 发送
    SPI_writeDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE, cmd);
    SPI_writeDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE, data);

    // 等待 FPGA 接收并返回两个 16-bit word
    // 虽然是写操作，但是 SPI 全双工会收到对方发回的废数据
    while(SPI_getRxFIFOStatus(IRIS_SPI_FPGA_BRIDGE_BASE) < SPI_FIFO_RX2);

    // 把接收到的这两个废数据读出，清空 RX FIFO，防止影响后续通信
    SPI_readDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE);
    SPI_readDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE);

    DEVICE_DELAY_US(2U);
    GPIO_writePin(IRIS_GPIO_SPI_CS, 1U);
    DEVICE_DELAY_US(2U);
}

// 从 FPGA 读取寄存器
// 协议: 帧1=[15位=0(读), 14:8=地址, 7:0=保留] -> 帧2=[16位占位符数据(0x0000)]
uint16_t SPI_readReg(uint16_t addr)
{
    // 构造读命令，最高位为 1
    uint16_t cmd = 0x8000 | ((addr & 0x7F) << 8); // 强制把最高位拉高
    uint16_t dummy_data = 0x0000; // 用于产生时钟的哑数据
    uint16_t rx_word0;
    uint16_t rx_word1;

    fpga_spi_clear_rx_fifo();
    GPIO_writePin(IRIS_GPIO_SPI_CS, 0U);
    DEVICE_DELAY_US(2U);

    // 压入命令帧和数据帧
    SPI_writeDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE, cmd);
    SPI_writeDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE, dummy_data);

    // 等待接收 2 个字
    while(SPI_getRxFIFOStatus(IRIS_SPI_FPGA_BRIDGE_BASE) < SPI_FIFO_RX2);

    // 读出的第一个字是发送命令帧时 FPGA 返回的（通常是状态位或全0，直接丢弃）
    rx_word0 = SPI_readDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE);

    // 读出的第二个字才是我们要的真实数据帧
    rx_word1 = SPI_readDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE);
    g_fpga_spi_last_rx0 = rx_word0;
    g_fpga_spi_last_rx1 = rx_word1;

    DEVICE_DELAY_US(2U);
    GPIO_writePin(IRIS_GPIO_SPI_CS, 1U);
    DEVICE_DELAY_US(2U);

    return rx_word1;
}


