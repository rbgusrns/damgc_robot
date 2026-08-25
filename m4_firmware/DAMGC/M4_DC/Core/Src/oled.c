#include "oled.h"
#include "i2c.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define OLED_ADDR ((uint16_t)(0x3CU << 1))
#define OLED_BUFFER_SIZE (OLED_WIDTH * OLED_PAGE_COUNT)
#define OLED_TX_CHUNK_SIZE (OLED_WIDTH + 1U)
#define OLED_TEXT_BUFFER_SIZE 32U
#define OLED_MULTIPLEX_RATIO (OLED_HEIGHT - 1U)
#if OLED_HEIGHT == 64U
#define OLED_COM_PINS 0x12U
#else
#define OLED_COM_PINS 0x02U
#endif

const uint8_t font5x7[95][5] = {
  {0x00,0x00,0x00,0x00,0x00}, // ' '
  {0x00,0x00,0x5F,0x00,0x00}, // '!'
  {0x00,0x07,0x00,0x07,0x00}, // '"'
  {0x14,0x7F,0x14,0x7F,0x14}, // '#'
  {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
  {0x23,0x13,0x08,0x64,0x62}, // '%'
  {0x36,0x49,0x55,0x22,0x50}, // '&'
  {0x00,0x05,0x03,0x00,0x00}, // '''
  {0x00,0x1C,0x22,0x41,0x00}, // '('
  {0x00,0x41,0x22,0x1C,0x00}, // ')'
  {0x14,0x08,0x3E,0x08,0x14}, // '*'
  {0x08,0x08,0x3E,0x08,0x08}, // '+'
  {0x00,0x50,0x30,0x00,0x00}, // ','
  {0x08,0x08,0x08,0x08,0x08}, // '-'
  {0x00,0x60,0x60,0x00,0x00}, // '.'
  {0x20,0x10,0x08,0x04,0x02}, // '/'
  {0x3E,0x51,0x49,0x45,0x3E}, // '0'
  {0x00,0x42,0x7F,0x40,0x00}, // '1'
  {0x42,0x61,0x51,0x49,0x46}, // '2'
  {0x21,0x41,0x45,0x4B,0x31}, // '3'
  {0x18,0x14,0x12,0x7F,0x10}, // '4'
  {0x27,0x45,0x45,0x45,0x39}, // '5'
  {0x3C,0x4A,0x49,0x49,0x30}, // '6'
  {0x01,0x71,0x09,0x05,0x03}, // '7'
  {0x36,0x49,0x49,0x49,0x36}, // '8'
  {0x06,0x49,0x49,0x29,0x1E}, // '9'
  {0x00,0x36,0x36,0x00,0x00}, // ':'
  {0x00,0x56,0x36,0x00,0x00}, // ';'
  {0x08,0x14,0x22,0x41,0x00}, // '<'
  {0x14,0x14,0x14,0x14,0x14}, // '='
  {0x00,0x41,0x22,0x14,0x08}, // '>'
  {0x02,0x01,0x51,0x09,0x06}, // '?'
  {0x32,0x49,0x79,0x41,0x3E}, // '@'
  {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
  {0x7F,0x49,0x49,0x49,0x36}, // 'B'
  {0x3E,0x41,0x41,0x41,0x22}, // 'C'
  {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
  {0x7F,0x49,0x49,0x49,0x41}, // 'E'
  {0x7F,0x09,0x09,0x09,0x01}, // 'F'
  {0x3E,0x41,0x49,0x49,0x7A}, // 'G'
  {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
  {0x00,0x41,0x7F,0x41,0x00}, // 'I'
  {0x20,0x40,0x41,0x3F,0x01}, // 'J'
  {0x7F,0x08,0x14,0x22,0x41}, // 'K'
  {0x7F,0x40,0x40,0x40,0x40}, // 'L'
  {0x7F,0x02,0x0C,0x02,0x7F}, // 'M'
  {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
  {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
  {0x7F,0x09,0x09,0x09,0x06}, // 'P'
  {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
  {0x7F,0x09,0x19,0x29,0x46}, // 'R'
  {0x46,0x49,0x49,0x49,0x31}, // 'S'
  {0x01,0x01,0x7F,0x01,0x01}, // 'T'
  {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
  {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
  {0x3F,0x40,0x38,0x40,0x3F}, // 'W'
  {0x63,0x14,0x08,0x14,0x63}, // 'X'
  {0x07,0x08,0x70,0x08,0x07}, // 'Y'
  {0x61,0x51,0x49,0x45,0x43}, // 'Z'
  {0x00,0x7F,0x41,0x41,0x00}, // '['
  {0x02,0x04,0x08,0x10,0x20}, // '\'
  {0x00,0x41,0x41,0x7F,0x00}, // ']'
  {0x04,0x02,0x01,0x02,0x04}, // '^'
  {0x40,0x40,0x40,0x40,0x40}, // '_'
  {0x00,0x01,0x02,0x04,0x00}, // '`'
  {0x20,0x54,0x54,0x54,0x78}, // 'a'
  {0x7F,0x48,0x44,0x44,0x38}, // 'b'
  {0x38,0x44,0x44,0x44,0x20}, // 'c'
  {0x38,0x44,0x44,0x48,0x7F}, // 'd'
  {0x38,0x54,0x54,0x54,0x18}, // 'e'
  {0x08,0x7E,0x09,0x01,0x02}, // 'f'
  {0x0C,0x52,0x52,0x52,0x3E}, // 'g'
  {0x7F,0x08,0x04,0x04,0x78}, // 'h'
  {0x00,0x44,0x7D,0x40,0x00}, // 'i'
  {0x20,0x40,0x44,0x3D,0x00}, // 'j'
  {0x7F,0x10,0x28,0x44,0x00}, // 'k'
  {0x00,0x41,0x7F,0x40,0x00}, // 'l'
  {0x7C,0x04,0x18,0x04,0x78}, // 'm'
  {0x7C,0x08,0x04,0x04,0x78}, // 'n'
  {0x38,0x44,0x44,0x44,0x38}, // 'o'
  {0x7C,0x14,0x14,0x14,0x08}, // 'p'
  {0x08,0x14,0x14,0x18,0x7C}, // 'q'
  {0x7C,0x08,0x04,0x04,0x08}, // 'r'
  {0x48,0x54,0x54,0x54,0x20}, // 's'
  {0x04,0x3F,0x44,0x40,0x20}, // 't'
  {0x3C,0x40,0x40,0x20,0x7C}, // 'u'
  {0x1C,0x20,0x40,0x20,0x1C}, // 'v'
  {0x3C,0x40,0x30,0x40,0x3C}, // 'w'
  {0x44,0x28,0x10,0x28,0x44}, // 'x'
  {0x0C,0x50,0x50,0x50,0x3C}, // 'y'
  {0x44,0x64,0x54,0x4C,0x44}, // 'z'
  {0x00,0x08,0x36,0x41,0x00}, // '{'
  {0x00,0x00,0x7F,0x00,0x00}, // '|'
  {0x00,0x41,0x36,0x08,0x00}, // '}'
  {0x08,0x08,0x2A,0x1C,0x08}, // '~'
};

static uint8_t draw_buffer[OLED_BUFFER_SIZE];
static uint8_t frame_buffer[OLED_BUFFER_SIZE];
static uint8_t oled_ready = 0U;

static void OLED_I2C_ClearErrorFlags(void);
static uint8_t OLED_I2C_Write(const uint8_t *data, uint16_t len, uint32_t timeout);
static uint8_t OLED_Cmd(uint8_t cmd);
static void OLED_DrawChar(uint8_t x, uint8_t page, char ch);
static uint8_t OLED_UpdateBlocking(void);

static void OLED_I2C_ClearErrorFlags(void)
{
    if (LL_I2C_IsActiveFlag_ADDR(I2C2) != 0U) {
        LL_I2C_ClearFlag_ADDR(I2C2);
    }
    if (LL_I2C_IsActiveFlag_NACK(I2C2) != 0U) {
        LL_I2C_ClearFlag_NACK(I2C2);
    }
    if (LL_I2C_IsActiveFlag_STOP(I2C2) != 0U) {
        LL_I2C_ClearFlag_STOP(I2C2);
    }
    if (LL_I2C_IsActiveFlag_BERR(I2C2) != 0U) {
        LL_I2C_ClearFlag_BERR(I2C2);
    }
    if (LL_I2C_IsActiveFlag_ARLO(I2C2) != 0U) {
        LL_I2C_ClearFlag_ARLO(I2C2);
    }
    if (LL_I2C_IsActiveFlag_OVR(I2C2) != 0U) {
        LL_I2C_ClearFlag_OVR(I2C2);
    }
}

static uint8_t OLED_I2C_Write(const uint8_t *data, uint16_t len, uint32_t timeout)
{
    uint32_t guard = timeout;

    if ((data == NULL) || (len == 0U))
    {
        return 0U;
    }

    if (LL_I2C_IsEnabled(I2C2) == 0U)
    {
        LL_I2C_Enable(I2C2);
    }

    while (LL_I2C_IsActiveFlag_BUSY(I2C2) != 0U)
    {
        if (guard-- == 0U)
        {
            return 0U;
        }
    }

    OLED_I2C_ClearErrorFlags();
    LL_I2C_HandleTransfer(I2C2, OLED_ADDR, LL_I2C_ADDRSLAVE_7BIT, len,
                          LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    for (uint16_t i = 0U; i < len; i++)
    {
        guard = timeout;
        while (LL_I2C_IsActiveFlag_TXIS(I2C2) == 0U)
        {
            if ((LL_I2C_IsActiveFlag_NACK(I2C2) != 0U) ||
                (LL_I2C_IsActiveFlag_BERR(I2C2) != 0U) ||
                (LL_I2C_IsActiveFlag_ARLO(I2C2) != 0U) ||
                (LL_I2C_IsActiveFlag_OVR(I2C2) != 0U))
            {
                LL_I2C_GenerateStopCondition(I2C2);
                OLED_I2C_ClearErrorFlags();
                return 0U;
            }
            if (guard-- == 0U)
            {
                LL_I2C_GenerateStopCondition(I2C2);
                return 0U;
            }
        }
        LL_I2C_TransmitData8(I2C2, data[i]);
    }

    guard = timeout;
    while (LL_I2C_IsActiveFlag_STOP(I2C2) == 0U)
    {
        if (LL_I2C_IsActiveFlag_NACK(I2C2) != 0U)
        {
            OLED_I2C_ClearErrorFlags();
            return 0U;
        }
        if (guard-- == 0U)
        {
            LL_I2C_GenerateStopCondition(I2C2);
            return 0U;
        }
    }

    LL_I2C_ClearFlag_STOP(I2C2);
    return 1U;
}

static uint8_t OLED_Cmd(uint8_t cmd)
{
    uint8_t data[2] = {0x00U, cmd};
    return OLED_I2C_Write(data, sizeof(data), 100000U);
}

void OLED_Init(void)
{
    LL_mDelay(100U);
    oled_ready = 0U;

    if ((OLED_Cmd(0xAEU) == 0U) ||
        (OLED_Cmd(0x20U) == 0U) ||
        (OLED_Cmd(0x00U) == 0U) ||
        (OLED_Cmd(0xB0U) == 0U) ||
        (OLED_Cmd(0xC0U) == 0U) ||
        (OLED_Cmd(0x00U) == 0U) ||
        (OLED_Cmd(0x10U) == 0U) ||
        (OLED_Cmd(0x40U) == 0U) ||
        (OLED_Cmd(0x81U) == 0U) ||
        (OLED_Cmd(0x7FU) == 0U) ||
        (OLED_Cmd(0xA0U) == 0U) ||
        (OLED_Cmd(0xA6U) == 0U) ||
        (OLED_Cmd(0xA8U) == 0U) ||
        (OLED_Cmd(OLED_MULTIPLEX_RATIO) == 0U) ||
        (OLED_Cmd(0xD3U) == 0U) ||
        (OLED_Cmd(0x00U) == 0U) ||
        (OLED_Cmd(0xD5U) == 0U) ||
        (OLED_Cmd(0x80U) == 0U) ||
        (OLED_Cmd(0xD9U) == 0U) ||
        (OLED_Cmd(0xF1U) == 0U) ||
        (OLED_Cmd(0xDAU) == 0U) ||
        (OLED_Cmd(OLED_COM_PINS) == 0U) ||
        (OLED_Cmd(0xDBU) == 0U) ||
        (OLED_Cmd(0x40U) == 0U) ||
        (OLED_Cmd(0x8DU) == 0U) ||
        (OLED_Cmd(0x14U) == 0U) ||
        (OLED_Cmd(0xAFU) == 0U)) {
        return;
    }

    oled_ready = 1U;

    OLED_Clear();

    while (OLED_IsBusy() != 0U)
    {
    }
}

void OLED_Clear(void)
{
    if (oled_ready == 0U) {
        return;
    }

    memset(draw_buffer, 0, sizeof(draw_buffer));
    OLED_Update();
}

uint8_t OLED_IsBusy(void)
{
    return 0U;
}

void OLED_Update(void)
{
    if (oled_ready == 0U) {
        return;
    }

    if (OLED_UpdateBlocking() == 0U) {
        oled_ready = 0U;
    }
}

void OLED_Print(uint8_t row, uint8_t col, const char *str)
{
    uint8_t x = (uint8_t)(col * 6U);

    if ((oled_ready == 0U) || (row >= OLED_PAGE_COUNT) || (str == NULL))
    {
        return;
    }

    while ((*str != '\0') && (x < (OLED_WIDTH - 6U)))
    {
        OLED_DrawChar(x, row, *str);
        x = (uint8_t)(x + 6U);
        str++;
    }
}

void OLED_PrintCentered(uint8_t row, const char *str)
{
    uint8_t length = 0U;

    if (str == NULL)
    {
        return;
    }

    while ((str[length] != '\0') && (length < OLED_TEXT_COLUMNS))
    {
        length++;
    }

    OLED_Print(row, (uint8_t)((OLED_TEXT_COLUMNS - length) / 2U), str);
}

void OLED_Printf(uint8_t row, uint8_t col, const char *fmt, ...)
{
    char buf[OLED_TEXT_BUFFER_SIZE];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    OLED_Print(row, col, buf);
    OLED_Update();
}


void OLED_ShowTextScreen(const char *title,
                         const char *line1,
                         const char *line2,
                         const char *line3)
{
    OLED_ClearBuffer();

    if (title != NULL)
    {
        OLED_PrintCentered(0U, title);
    }
    if (line1 != NULL)
    {
        OLED_Print(1U, 0U, line1);
    }
    if (line2 != NULL)
    {
        OLED_Print(2U, 0U, line2);
    }
    if (line3 != NULL)
    {
        OLED_Print(3U, 0U, line3);
    }

    OLED_Update();
}

static void OLED_DrawChar(uint8_t x, uint8_t page, char ch)
{
    uint16_t index;

    if ((page >= OLED_PAGE_COUNT) || (x >= OLED_WIDTH))
    {
        return;
    }

    if ((ch < 0x20) || (ch > 0x7E))
    {
        ch = '?';
    }

    index = (uint16_t)page * OLED_WIDTH + x;

    for (uint8_t col = 0U; col < 5U; col++)
    {
        if ((x + col) < OLED_WIDTH)
        {
            draw_buffer[index + col] = font5x7[(uint8_t)ch - 0x20U][col];
        }
    }

    if ((x + 5U) < OLED_WIDTH)
    {
        draw_buffer[index + 5U] = 0x00U;
    }
}

static uint8_t OLED_UpdateBlocking(void)
{
    uint8_t data[OLED_TX_CHUNK_SIZE];

    memcpy(frame_buffer, draw_buffer, sizeof(frame_buffer));

    for (uint8_t page = 0U; page < OLED_PAGE_COUNT; page++)
    {
        if ((OLED_Cmd((uint8_t)(0xB0U + page)) == 0U) ||
            (OLED_Cmd(0x00U) == 0U) ||
            (OLED_Cmd(0x10U) == 0U)) {
            return 0U;
        }

        data[0] = 0x40U;
        memcpy(&data[1], &frame_buffer[(uint16_t)page * OLED_WIDTH], OLED_WIDTH);
        if (OLED_I2C_Write(data, OLED_TX_CHUNK_SIZE, 100000U) == 0U) {
            return 0U;
        }
    }

    return 1U;
}

void OLED_DrawSensorBars(const float values[16])
{
    if (oled_ready == 0U) {
        return;
    }

    memset(draw_buffer, 0, sizeof(draw_buffer));

    for (uint8_t i = 0U; i < 16U; i++)
    {
        float val = values[i];
        if (val < 0.0f)
        {
            val = 0.0f;
        }
        if (val > 127.0f)
        {
            val = 127.0f;
        }

        uint8_t h = (uint8_t)((val * (float)OLED_HEIGHT) / 127.0f + 0.5f);
        if (h > OLED_HEIGHT)
        {
            h = OLED_HEIGHT;
        }

        uint8_t col_start = i * 8U;

        for (uint8_t x = col_start + 1U; x <= col_start + 6U; x++)
        {
            for (uint8_t page = 0U; page < OLED_PAGE_COUNT; page++)
            {
                uint8_t byte_val = 0U;
                for (uint8_t bit = 0U; bit < 8U; bit++)
                {
                    uint8_t y = (page * 8U) + bit;
                    if (y >= (OLED_HEIGHT - h))
                    {
                        byte_val |= (1U << bit);
                    }
                }
                draw_buffer[(uint16_t)page * OLED_WIDTH + x] = byte_val;
            }
        }
    }
    OLED_Update();
}

void OLED_ClearBuffer(void)
{
    memset(draw_buffer, 0, sizeof(draw_buffer));
}
