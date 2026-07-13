
#include <gmp_core.h>

#include <oled_driver.h>
#include <oledfont.h>

#include <core/pm/function_scheduler.h>

#define TIMEOUT_SET 40
#define OLED_DISPLAY_WIDTH       128U
#define OLED_PAGE_COUNT          8U
#define OLED_TEXT_CELL_WIDTH     8U
#define OLED_FONT8_GLYPH_WIDTH   6U
#define OLED_PAGE_PAYLOAD_SIZE   (OLED_DISPLAY_WIDTH + 1U)

// Shared board-local buffers avoid placing full display lines on the C28x stack.
static data_gt s_oled_page_payload[OLED_PAGE_PAYLOAD_SIZE];
static data_gt s_oled_line_upper[OLED_DISPLAY_WIDTH];
#if FONT_SIZE == 16
static data_gt s_oled_line_lower[OLED_DISPLAY_WIDTH];
#endif

volatile ec_gt g_oled_position_result = GMP_EC_NOT_READY;
volatile ec_gt g_oled_data_result = GMP_EC_NOT_READY;
volatile uint16_t g_oled_fail_stage = OLED_FAIL_STAGE_NONE;
volatile uint16_t g_oled_fail_x = 0U;
volatile uint16_t g_oled_fail_page = 0U;
volatile uint16_t g_oled_fail_length = 0U;
volatile uint16_t g_oled_last_slave_address = OLED_IIC_7BIT_ADDR;
volatile ec_gt g_oled_probe_result = GMP_EC_NOT_READY;
volatile uint32_t g_oled_probe_ok_count = 0U;
volatile uint32_t g_oled_probe_error_count = 0U;

static void oled_prepare_diagnostics(void)
{
    g_oled_position_result = GMP_EC_NOT_READY;
    g_oled_data_result = GMP_EC_NOT_READY;
    g_oled_fail_stage = OLED_FAIL_STAGE_NONE;
    g_oled_fail_x = 0U;
    g_oled_fail_page = 0U;
    g_oled_fail_length = 0U;
    g_oled_last_slave_address = OLED_IIC_7BIT_ADDR;
}

static void oled_record_failure(uint16_t stage, uint16_t x,
                                uint16_t page, uint16_t length)
{
    g_oled_fail_stage = stage;
    g_oled_fail_x = x;
    g_oled_fail_page = page;
    g_oled_fail_length = length;
    g_oled_last_slave_address = OLED_IIC_7BIT_ADDR;
}

static ec_gt oled_write_byte_checked(uint8_t dat, uint8_t cmd)
{
    const time_gt timeout_ticks = TIMEOUT_SET;
    uint32_t control_byte = (cmd != 0U) ? 0x40U : 0x00U;

    g_oled_last_slave_address = OLED_IIC_7BIT_ADDR;
    return gmp_hal_iic_write_reg(
        iic_bus, OLED_IIC_7BIT_ADDR, control_byte, 1U,
        (uint32_t)dat, 1U, timeout_ticks);
}

static uint8_t oled_checked_printable_char(uint8_t chr)
{
    if ((chr < (uint8_t)' ') || (chr > (uint8_t)'~'))
    {
        return (uint8_t)'?';
    }

    return chr;
}


/**
 * @brief  Re-implementation of official OLED_WR_Byte using your custom HAL.
 * @param  dat: The actual byte of command or pixel data to be transmitted.
 * @param  cmd: 0 for Command (OLED_CMD), 1 for Pixel Data (OLED_DATA).
 */
void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
    (void)oled_write_byte_checked(dat, cmd);
}

/**
 * @brief  Optimized, low-overhead positional command function.
 * @note   Combines page and column positioning into a single I2C burst transaction.
 */
static ec_gt oled_set_position_raw(uint8_t x, uint8_t y_page)
{
    const time_gt timeout_ticks = TIMEOUT_SET;
    static data_gt pos_cmds[4];

    pos_cmds[0] = 0x00;                           /* Control Byte: Following are commands */
    pos_cmds[1] = (data_gt)(0xB0 + y_page);       /* Set Target Page Address */
    pos_cmds[2] = (data_gt)((((x + 2) & 0xF0) >> 4) | 0x10); /* Higher column nibble with +2 offset */
    pos_cmds[3] = (data_gt)((x + 2) & 0x0F);       /* Lower column nibble with +2 offset */

    /* Dispatch all 4 bytes consecutively to save bus time slices */
    return gmp_hal_iic_write_mem(
        iic_bus, OLED_IIC_7BIT_ADDR, 0, 0, pos_cmds, 4U, timeout_ticks);
}

ec_gt oled_set_position_checked(uint8_t x, uint8_t y_page)
{
    ec_gt ret;

    oled_prepare_diagnostics();
    if ((x >= OLED_DISPLAY_WIDTH) || (y_page >= OLED_PAGE_COUNT))
    {
        g_oled_position_result = GMP_EC_INVALID_PARAM;
        oled_record_failure(OLED_FAIL_STAGE_PARAMETER, x, y_page, 0U);
        return GMP_EC_INVALID_PARAM;
    }

    ret = oled_set_position_raw(x, y_page);
    g_oled_position_result = ret;
    if (ret != GMP_EC_OK)
    {
        oled_record_failure(OLED_FAIL_STAGE_POSITION, x, y_page, 0U);
    }

    return ret;
}

ec_gt oled_write_page_checked(uint8_t x, uint8_t y_page,
                              const data_gt *data, uint16_t length)
{
    const time_gt timeout_ticks = TIMEOUT_SET;
    uint16_t available;
    uint16_t write_length;
    uint16_t i;
    ec_gt ret;

    oled_prepare_diagnostics();
    if ((data == NULL) || (length == 0U) ||
        (x >= OLED_DISPLAY_WIDTH) || (y_page >= OLED_PAGE_COUNT))
    {
        g_oled_position_result = GMP_EC_INVALID_PARAM;
        oled_record_failure(OLED_FAIL_STAGE_PARAMETER, x, y_page, length);
        return GMP_EC_INVALID_PARAM;
    }

    available = (uint16_t)(OLED_DISPLAY_WIDTH - x);
    write_length = (length > available) ? available : length;
    ret = oled_set_position_checked(x, y_page);
    g_oled_position_result = ret;
    if (ret != GMP_EC_OK)
    {
        oled_record_failure(
            OLED_FAIL_STAGE_POSITION, x, y_page, write_length);
        return ret;
    }

    s_oled_page_payload[0] = 0x40U;
    for (i = 0U; i < write_length; ++i)
    {
        s_oled_page_payload[i + 1U] = data[i];
    }

    ret = gmp_hal_iic_write_mem(
        iic_bus, OLED_IIC_7BIT_ADDR, 0, 0,
        s_oled_page_payload, (size_gt)(write_length + 1U), timeout_ticks);
    g_oled_data_result = ret;
    if (ret != GMP_EC_OK)
    {
        oled_record_failure(OLED_FAIL_STAGE_DATA, x, y_page, write_length);
        return ret;
    }

    return GMP_EC_OK;
}

ec_gt oled_show_line_checked(uint8_t x, uint8_t y_page, const char *str)
{
    uint16_t max_width;
    uint16_t used = 0U;
    uint16_t char_index = 0U;
    uint16_t glyph_column;
    uint16_t font_index;
    uint8_t chr;
    ec_gt ret;

    if ((str == NULL) || (x >= OLED_DISPLAY_WIDTH) ||
        (y_page >= OLED_PAGE_COUNT))
    {
        oled_prepare_diagnostics();
        g_oled_position_result = GMP_EC_INVALID_PARAM;
        oled_record_failure(OLED_FAIL_STAGE_PARAMETER, x, y_page, 0U);
        return GMP_EC_INVALID_PARAM;
    }
#if FONT_SIZE == 16
    if ((uint16_t)y_page + 1U >= OLED_PAGE_COUNT)
    {
        oled_prepare_diagnostics();
        g_oled_position_result = GMP_EC_INVALID_PARAM;
        oled_record_failure(OLED_FAIL_STAGE_PARAMETER, x, y_page, 0U);
        return GMP_EC_INVALID_PARAM;
    }
#endif

    max_width = (uint16_t)(OLED_DISPLAY_WIDTH - x);
    while ((str[char_index] != '\0') && (used < max_width))
    {
        chr = oled_checked_printable_char((uint8_t)str[char_index]);
        font_index = (uint16_t)(chr - (uint8_t)' ');

        for (glyph_column = 0U;
             (glyph_column < OLED_TEXT_CELL_WIDTH) && (used < max_width);
             ++glyph_column)
        {
#if FONT_SIZE == 16
            s_oled_line_upper[used] =
                (data_gt)F8X16[(font_index * 16U) + glyph_column];
            s_oled_line_lower[used] =
                (data_gt)F8X16[(font_index * 16U) + glyph_column + 8U];
#else
            s_oled_line_upper[used] =
                (glyph_column < OLED_FONT8_GLYPH_WIDTH) ?
                    (data_gt)F6x8[font_index][glyph_column] : 0U;
#endif
            ++used;
        }

        ++char_index;
    }

    if (used == 0U)
    {
        oled_prepare_diagnostics();
        g_oled_position_result = GMP_EC_INVALID_PARAM;
        oled_record_failure(OLED_FAIL_STAGE_PARAMETER, x, y_page, 0U);
        return GMP_EC_INVALID_PARAM;
    }

    ret = oled_write_page_checked(x, y_page, s_oled_line_upper, used);
    if (ret != GMP_EC_OK)
    {
        return ret;
    }

#if FONT_SIZE == 16
    ret = oled_write_page_checked(
        x, (uint8_t)(y_page + 1U), s_oled_line_lower, used);
    if (ret != GMP_EC_OK)
    {
        return ret;
    }
#endif

    return GMP_EC_OK;
}

void oled_set_position(uint8_t x, uint8_t y_page)
{
    (void)oled_set_position_checked(x, y_page);
}


/**
 * @brief  Turns ON the OLED panel and internal charge pump in a single I2C transaction.
 */
void oled_display_on(void)
{
    const time_gt timeout_ticks = TIMEOUT_SET;
    static data_gt on_cmds[4];

    on_cmds[0] = 0x00; /* Control Byte: Following are commands */
    on_cmds[1] = 0x8D; /* Charge Pump Command Specifier */
    on_cmds[2] = 0x14; /* Enable Charge Pump */
    on_cmds[3] = 0xAF; /* Display ON */

    gmp_hal_iic_write_mem(iic_bus, OLED_IIC_7BIT_ADDR, 0, 0, on_cmds, 4, timeout_ticks);
}

/**
 * @brief  Turns OFF the OLED panel and internal charge pump cleanly.
 */
void oled_display_off(void)
{
    const time_gt timeout_ticks = TIMEOUT_SET;
    static data_gt off_cmds[4];

    off_cmds[0] = 0x00; /* Control Byte: Following are commands */
    off_cmds[1] = 0x8D; /* Charge Pump Command Specifier */
    off_cmds[2] = 0x10; /* Disable Charge Pump */
    off_cmds[3] = 0xAE; /* Display OFF */

    gmp_hal_iic_write_mem(iic_bus, OLED_IIC_7BIT_ADDR, 0, 0, off_cmds, 4, timeout_ticks);
}

/**
 * @brief  Optimized ultra-fast, non-blocking-friendly OLED clear function.
 * @note   Replaces 1024 separate I2C bursts with 8 continuous page bursts.
 */
ec_gt oled_clear_checked(void)
{
    static data_gt blank_page[OLED_DISPLAY_WIDTH] = {0U};
    uint8_t page;
    ec_gt ret;

    for (page = 0U; page < OLED_PAGE_COUNT; ++page)
    {
        ret = oled_write_page_checked(
            0U, page, blank_page, OLED_DISPLAY_WIDTH);
        if (ret != GMP_EC_OK)
        {
            return ret;
        }
    }

    return GMP_EC_OK;
}

void oled_clear(void)
{
    (void)oled_clear_checked();
}

/**
 * @brief  Ultra-efficient character rendering block without inner loop I2C overhead.
 * @param  x: Horizontal start column index (0 to 127).
 * @param  y_page: Vertical page index (0 to 7).
 * @param  chr: ASCII character to display.
 */
void oled_show_char(uint8_t x, uint8_t y_page, uint8_t chr)
{
    uint16_t i;

    const time_gt timeout_ticks = TIMEOUT_SET;
    uint8_t c_offset = chr - ' '; /* Calculate ASCII matrix offset index */

    /*
     * Temporary transmit buffer for the data stream.
     * Element [0] is reserved for the 0x40 Data Control Byte.
     * Max elements needed: 1 (Control) + 8 (Pixels) = 9.
     */
    static data_gt tx_payload[9];
    tx_payload[0] = 0x40; /* Control Byte: Following stream is graphic display RAM data */

    /* Auto wrap text boundaries just like your official code */
    if (x > 128 - 1)
    {
        x = 0;
        y_page = y_page + 2;
    }

    if (FONT_SIZE == 16)
    {
        uint16_t font_index = (uint16_t)c_offset * 16U;

        /* 1. Render Upper Half (8 pixels high, 8 pixels wide) */
        oled_set_position(x, y_page);
        for (i = 0; i < 8; i++)
        {
            tx_payload[i + 1] = (data_gt)F8X16[font_index + i];
        }
        /* Continuous push of 1 control byte + 8 data bytes via C2000 FIFO */
        gmp_hal_iic_write_mem(iic_bus, OLED_IIC_7BIT_ADDR, 0, 0, tx_payload, 9, timeout_ticks);

        /* 2. Render Lower Half (8 pixels high, 8 pixels wide) at the next vertical page slot */
        oled_set_position(x, y_page + 1);
        for (i = 0; i < 8; i++)
        {
            tx_payload[i + 1] = (data_gt)F8X16[font_index + i + 8];
        }
        gmp_hal_iic_write_mem(iic_bus, OLED_IIC_7BIT_ADDR, 0, 0, tx_payload, 9, timeout_ticks);
    }
    else
    {
        /* Render Small Font (8 pixels high, 6 pixels wide) */
        oled_set_position(x, y_page);
        for (i = 0; i < 6; i++)
        {
            tx_payload[i + 1] = (data_gt)F6x8[c_offset][i];
        }
        /* Continuous push of 1 control byte + 6 data bytes */
        gmp_hal_iic_write_mem(iic_bus, OLED_IIC_7BIT_ADDR, 0, 0, tx_payload, 7, timeout_ticks);
    }
}

/**
 * @brief  Displays a null-terminated string on the character grid.
 * @note   Leverages the highly optimized non-blocking oled_show_char underneath.
 */
void oled_show_str(uint8_t x, uint8_t y_page, const char *str)
{
    uint16_t j = 0;

    while (str[j] != '\0')
    {
        oled_show_char(x, y_page, (uint8_t)str[j]);

        /* Advance x by 8 pixels (width of F8X16 or font boundary spacing) */
        x += 8;
        if (x > 120)
        {
            x = 0;
            y_page += 2; /* Move down 2 pages for the 16-pixel high font height wrap */
        }
        j++;
    }
}

/**
 * @brief  Draws a BMP image using highly efficient page-burst transfers instead of byte-by-byte streaming.
 * @note   Optimized for C2000 FIFO architecture. Suitable for block initialization displays.
 * @param  x0: Starting column coordinate (0 to 127).
 * @param  y0: Starting page coordinate (0 to 7).
 * @param  x1: Ending column coordinate (1 to 128).
 * @param  y1: Ending page coordinate (1 to 8).
 * @param  BMP: Array containing the raw monochrome picture dot matrix data.
 */
void oled_show_bmp(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[])
{
    const time_gt timeout_ticks = TIMEOUT_SET;
    unsigned int bmp_idx = 0;
    unsigned char y_page;
    uint16_t x;

    /*
     * Temporary continuous transfer buffer.
     * Element [0] is reserved for the 0x40 Data Control Byte.
     * Maximum payload size: 1 (Control) + 128 (Max columns) = 129 words.
     */
    static data_gt page_payload[129];
    page_payload[0] = 0x40; /* Control Byte: Following stream is graphic display RAM data */

    /* Calculate horizontal segment width per burst */
    uint16_t chunk_width = (uint16_t)(x1 - x0);
    if (chunk_width > 128) chunk_width = 128;

    /* Loop through each targeted vertical page row sequentially */
    for (y_page = y0; y_page < y1; y_page++)
    {
        /* 1. Set the physical column and page address pointers on the OLED controller */
        oled_set_position(x0, y_page);

        /* 2. Assemble the current page row's pixel dataset into our linear buffer */
        for (x = 0; x < chunk_width; x++)
        {
            page_payload[x + 1] = (data_gt)BMP[bmp_idx++];
        }

        /*
         * 3. Send the entire line block in a single fast continuous I2C burst.
         * Total length = 1 control byte + chunk_width data bytes.
         * Your gmp_hal_iic_write_mem will manage the 16-byte hardware FIFO internally without stalls.
         */
        gmp_hal_iic_write_mem(iic_bus, OLED_IIC_7BIT_ADDR, 0, 0, page_payload, chunk_width + 1, timeout_ticks);
    }
}

/**
 * @brief  Initializes the SSD1306/SH1106 OLED module controller registers.
 * @details This function transmits a pre-defined hardware configuration macro sequence
 *          to setup internal clocks, multiplex ratio, display flip directions, and
 *          crucially activates the internal charge pump required to drive the panel VCC.
 *          It forces Page Addressing Mode and performs a clear screen layout at the end.
 *
 * @param  None
 * @return None
 *
 * @note   Ensure the physical hardware supply is connected to 5V prior to running this
 *         sequence to avoid sub-optimal low voltage NACK lockouts from the display controller.
 * @see    OLED_WR_Byte
 * @see    oled_clear
 * @see    oled_set_position
 */
ec_gt oled_init_checked(void)
{
    static const uint8_t init_commands[] = {
        0xAEU, 0x02U, 0x10U, 0x40U,
        0x81U, 0xCFU, 0xA1U, 0xC8U,
        0xA6U, 0xA8U, 0x3FU, 0xD3U,
        0x00U, 0xD5U, 0x80U, 0xD9U,
        0xF1U, 0xDAU, 0x12U, 0xDBU,
        0x40U, 0x20U, 0x02U, 0x8DU,
        0x14U, 0xA4U, 0xA6U, 0xAFU,
        0xAFU
    };
    uint16_t i;
    ec_gt ret;

    oled_prepare_diagnostics();
    for (i = 0U;
         i < (uint16_t)(sizeof(init_commands) / sizeof(init_commands[0]));
         ++i)
    {
        ret = oled_write_byte_checked(init_commands[i], OLED_CMD);
        if (ret != GMP_EC_OK)
        {
            return ret;
        }

        if (((i + 1U) % 4U) == 0U)
        {
            DEVICE_DELAY_US(200U);
        }
    }

    return GMP_EC_OK;
}

void oled_init(void)
{
    ec_gt ret = oled_init_checked();

    if (ret != GMP_EC_OK)
    {
        return;
    }

    ret = oled_clear_checked();
    if (ret != GMP_EC_OK)
    {
        return;
    }

    ret = oled_show_line_checked(0U, 0U, "OLED TEST");
    if (ret != GMP_EC_OK)
    {
        return;
    }

    (void)oled_show_line_checked(0U, 4U, "2026/07/01");
}

