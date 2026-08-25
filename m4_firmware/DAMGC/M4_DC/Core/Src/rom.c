#include "rom.h"
#include "spi.h"
#include "usart.h"
#include "variable.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define ROM_PAGE_SIZE          256U
#define ROM_RECORD_PAGE_COUNT  17U
#define ROM_RECORD_SIZE        (ROM_PAGE_SIZE * ROM_RECORD_PAGE_COUNT)
#define ROM_MAGIC              0x4D344443UL
#define ROM_VERSION            1U

typedef enum {
    ROM_REC_MAXMIN = 0,
    ROM_REC_TURNVEL,
    ROM_REC_UNUSED_EXTVEL,
    ROM_REC_TURNMARK,
    ROM_REC_ACC,
    ROM_REC_HANDLE,
    ROM_REC_MARK,
    ROM_REC_FASTINFO,
    ROM_REC_COUNT
} rom_record_id_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t id;
    uint32_t length;
    uint32_t checksum;
} rom_record_header_t;

typedef struct {
    float min_value[16];
    float max_value[16];
} rom_maxmin_t;

typedef struct {
    float fp32_user_vel;
} rom_turnvel_t;

typedef struct {
    float turnmark_dist;
    uint16_t turnmark_limit;
    uint16_t turn_dist;
    float sen_valmax;
    float turn_threshold;
} rom_turnmark_t;

typedef struct {
    float user_acc;
    float end_acc;
    float end_vel;
    float endturn_acc;
    int32_t reserved_decel;
} rom_acc_t;

typedef struct {
    float out_corner_limit;
    float in_corner_limit;
    float out_corner_fast_limit;
    float in_corner_fast_limit;
} rom_handle_t;

typedef struct {
    int32_t total_cnt;
    int32_t mark_cnt;
} rom_mark_t;

typedef struct {
    uint16_t dist[256];
    uint16_t turn_way[256];
    float l_dist[256];
    float r_dist[256];
    float reserved_pos_integral[256];
} rom_fastinfo_t;

_Static_assert((sizeof(rom_record_header_t) + sizeof(rom_fastinfo_t)) <= ROM_RECORD_SIZE,
               "SPI ROM record area is too small");

static rom_fastinfo_t s_fastinfo_buffer;
static uint8_t s_record_buffer[ROM_RECORD_SIZE] __attribute__((aligned(4)));
static uint16_t s_spi_buffer[ROM_PAGE_SIZE];

static inline void SPI_CS_DELAY(void)
{
    volatile uint32_t count = 0x1000U;

    while (count-- != 0U) {
        __NOP();
        __NOP();
    }
}

static uint8_t SpiTx(uint8_t tx_data)
{
    if (LL_SPI_IsEnabled(SPI1) == 0U) {
        LL_SPI_Enable(SPI1);
    }

    while (LL_SPI_IsActiveFlag_TXE(SPI1) == 0U) {
    }
    LL_SPI_TransmitData8(SPI1, tx_data);

    while (LL_SPI_IsActiveFlag_RXNE(SPI1) == 0U) {
    }

    tx_data = LL_SPI_ReceiveData8(SPI1);
    while (LL_SPI_IsActiveFlag_BSY(SPI1) != 0U) {
    }

    return tx_data;
}

static void SpiBusyOK(void)
{
    uint8_t status_reg = 0U;

    while ((status_reg & 0x08U) != 0x08U) {
        LL_GPIO_SetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
        SPI_CS_DELAY();
        LL_GPIO_ResetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
        SPI_CS_DELAY();

        SpiTx(0xD7U);
        status_reg = SpiTx(0U);

        LL_GPIO_SetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
        SPI_CS_DELAY();
    }
}

static void SpiWriteRom(uint16_t page, uint8_t addr, uint16_t len, const uint16_t *buf)
{
    uint8_t add1;
    uint8_t add2;
    uint8_t add3;

    SpiBusyOK();

    add1 = (uint8_t)((page & 0x01FFU) >> 7);
    add2 = (uint8_t)(((page & 0x007FU) << 1) | ((addr & 0x01FFU) >> 8));
    add3 = (uint8_t)(addr & 0x00FFU);

    LL_GPIO_SetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
    SPI_CS_DELAY();
    LL_GPIO_ResetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
    SPI_CS_DELAY();

    SpiTx(0x82U);
    SpiTx(add1);
    SpiTx(add2);
    SpiTx(add3);

    for (uint16_t i = 0U; i < len; i++) {
        SpiTx((uint8_t)(buf[i] & 0x00FFU));
    }

    LL_GPIO_SetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
    SPI_CS_DELAY();
}

static void SpiReadRom(uint16_t page, uint8_t addr, uint16_t len, uint16_t *buf)
{
    uint8_t add1;
    uint8_t add2;
    uint8_t add3;

    SpiBusyOK();

    add1 = (uint8_t)((page & 0x01FFU) >> 7);
    add2 = (uint8_t)(((page & 0x007FU) << 1) | ((addr & 0x01FFU) >> 8));
    add3 = (uint8_t)(addr & 0x00FFU);

    LL_GPIO_SetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
    SPI_CS_DELAY();
    LL_GPIO_ResetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
    SPI_CS_DELAY();

    SpiTx(0xD2U);
    SpiTx(add1);
    SpiTx(add2);
    SpiTx(add3);
    SpiTx(0U);
    SpiTx(0U);
    SpiTx(0U);
    SpiTx(0U);

    for (uint16_t i = 0U; i < len; i++) {
        buf[i] = SpiTx(0U);
    }

    LL_GPIO_SetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
    SPI_CS_DELAY();
}

static uint16_t rom_record_page(rom_record_id_t id)
{
    return (uint16_t)id * ROM_RECORD_PAGE_COUNT;
}

static void rom_write_bytes(uint16_t first_page, const uint8_t *data, uint32_t length)
{
    uint16_t page = first_page;

    while (length != 0U) {
        uint16_t chunk = (length > ROM_PAGE_SIZE) ? ROM_PAGE_SIZE : (uint16_t)length;

        for (uint16_t i = 0U; i < chunk; i++) {
            s_spi_buffer[i] = data[i];
        }

        SpiWriteRom(page, 0U, chunk, s_spi_buffer);
        data += chunk;
        length -= chunk;
        page++;
    }
}

static void rom_read_bytes(uint16_t first_page, uint8_t *data, uint32_t length)
{
    uint16_t page = first_page;

    while (length != 0U) {
        uint16_t chunk = (length > ROM_PAGE_SIZE) ? ROM_PAGE_SIZE : (uint16_t)length;

        SpiReadRom(page, 0U, chunk, s_spi_buffer);
        for (uint16_t i = 0U; i < chunk; i++) {
            data[i] = (uint8_t)s_spi_buffer[i];
        }

        data += chunk;
        length -= chunk;
        page++;
    }
}

static uint32_t rom_checksum(const uint8_t *data, uint32_t length)
{
    uint32_t sum = 0x13572468UL;

    for (uint32_t i = 0U; i < length; i++) {
        sum = (sum << 5) | (sum >> 27);
        sum += data[i];
    }

    return sum;
}

static uint8_t rom_sensor_limit_is_valid(float min_value, float max_value)
{
    if ((isfinite(min_value) == 0) || (isfinite(max_value) == 0)) {
        return 0U;
    }

    if ((min_value < -1.0f) || (min_value > 4095.0f)) {
        return 0U;
    }

    if ((max_value < 0.0f) || (max_value > 4096.0f)) {
        return 0U;
    }

    if ((max_value - min_value) < 20.0f) {
        return 0U;
    }

    return 1U;
}

static uint8_t rom_write_record(rom_record_id_t id, const void *payload, uint32_t length)
{
    rom_record_header_t *header = (rom_record_header_t *)s_record_buffer;
    uint32_t total_length = sizeof(rom_record_header_t) + length;

    if ((payload == NULL) || (id >= ROM_REC_COUNT) || (total_length > ROM_RECORD_SIZE)) {
        return 0U;
    }

    memset(s_record_buffer, 0xFF, total_length);
    header->magic = ROM_MAGIC;
    header->version = ROM_VERSION;
    header->id = (uint16_t)id;
    header->length = length;
    header->checksum = rom_checksum((const uint8_t *)payload, length);
    memcpy(&s_record_buffer[sizeof(rom_record_header_t)], payload, length);

    rom_write_bytes(rom_record_page(id), s_record_buffer, total_length);
    return 1U;
}

static uint8_t rom_read_record(rom_record_id_t id, void *payload, uint32_t length)
{
    rom_record_header_t *header = (rom_record_header_t *)s_record_buffer;
    uint32_t total_length = sizeof(rom_record_header_t) + length;

    if ((payload == NULL) || (id >= ROM_REC_COUNT) || (total_length > ROM_RECORD_SIZE)) {
        return 0U;
    }

    rom_read_bytes(rom_record_page(id), s_record_buffer, sizeof(rom_record_header_t));

    if ((header->magic != ROM_MAGIC) ||
        (header->version != ROM_VERSION) ||
        (header->id != (uint16_t)id) ||
        (header->length != length)) {
        return 0U;
    }

    rom_read_bytes(rom_record_page(id), s_record_buffer, total_length);
    memcpy(payload, &s_record_buffer[sizeof(rom_record_header_t)], length);

    return (header->checksum == rom_checksum((const uint8_t *)payload, length)) ? 1U : 0U;
}

void maxmin_write_rom(void)
{
    rom_maxmin_t data;
    uint8_t ok;

    for (uint8_t i = 0U; i < 16U; i++) {
        data.min_value[i] = g_sen[i].fp32_4095_min_value;
        data.max_value[i] = g_sen[i].fp32_4095_max_value;
    }

    USART1_WriteString("ROMW\r\n");
    ok = rom_write_record(ROM_REC_MAXMIN, &data, sizeof(data));
    USART1_WriteString((ok != 0U) ? "ROMOK\r\n" : "ROMFAIL\r\n");
}

void maxmin_read_rom(void)
{
    rom_maxmin_t data;

    if (rom_read_record(ROM_REC_MAXMIN, &data, sizeof(data)) == 0U) {
        return;
    }

    for (uint8_t i = 0U; i < 16U; i++) {
        if (rom_sensor_limit_is_valid(data.min_value[i], data.max_value[i]) == 0U) {
            printf("ROM MAXMIN READ BAD\r\n");
            return;
        }
    }

    for (uint8_t i = 0U; i < 16U; i++) {
        g_sen[i].fp32_4095_min_value = data.min_value[i];
        g_sen[i].fp32_4095_max_value = data.max_value[i];
    }

    printf("ROM MAXMIN READ OK\r\n");
}

void turnvel_write_rom(void)
{
    rom_turnvel_t data = { g_fp32_user_vel };

    printf("ROM TURNVEL WRITE %s\r\n",
           (rom_write_record(ROM_REC_TURNVEL, &data, sizeof(data)) != 0U) ? "OK" : "FAIL");
}

void turnvel_read_rom(void)
{
    rom_turnvel_t data;

    if (rom_read_record(ROM_REC_TURNVEL, &data, sizeof(data)) != 0U) {
        g_fp32_user_vel = data.fp32_user_vel;
        printf("ROM TURNVEL READ OK\r\n");
    }
}

void turnmark_info_write_rom(void)
{
    rom_turnmark_t data = {
        g_fp32_turnmark_dist,
        g_u16_turnmark_limit,
        g_u16_turn_dist,
        g_fp32_sensor_value_max,
        g_fp32_turn_threshold
    };

    printf("ROM TURNMARK WRITE %s\r\n",
           (rom_write_record(ROM_REC_TURNMARK, &data, sizeof(data)) != 0U) ? "OK" : "FAIL");
}

void turnmark_info_read_rom(void)
{
    rom_turnmark_t data;

    if (rom_read_record(ROM_REC_TURNMARK, &data, sizeof(data)) == 0U) {
        return;
    }

    g_fp32_turnmark_dist = data.turnmark_dist;
    g_u16_turnmark_limit = data.turnmark_limit;
    g_u16_turn_dist = data.turn_dist;
    g_fp32_sensor_value_max = data.sen_valmax;
    g_fp32_turn_threshold = data.turn_threshold;
    g_fp32_straight_mark_dist = g_fp32_turnmark_dist + 80.0f;
    g_fp32_mark_dist = g_fp32_turnmark_dist;
    printf("ROM TURNMARK READ OK\r\n");
}

void acc_info_write_rom(void)
{
    rom_acc_t data = {
        g_fp32_user_acc,
        g_fp32_end_acc,
        g_fp32_end_vel,
        g_fp32_endturn_acc,
        g_i32_decel
    };

    printf("ROM ACC WRITE %s\r\n",
           (rom_write_record(ROM_REC_ACC, &data, sizeof(data)) != 0U) ? "OK" : "FAIL");
}

void acc_info_read_rom(void)
{
    rom_acc_t data;

    if (rom_read_record(ROM_REC_ACC, &data, sizeof(data)) == 0U) {
        return;
    }

    g_fp32_user_acc = data.user_acc;
    g_fp32_end_acc = data.end_acc;
    g_fp32_end_vel = data.end_vel;
    g_fp32_endturn_acc = data.endturn_acc;
    printf("ROM ACC READ OK\r\n");
}

void handle_write_rom(void)
{
    rom_handle_t data = {
        g_fp32_out_corner_limit,
        g_fp32_in_corner_limit,
        g_fp32_out_corner_fast_limit,
        g_fp32_in_corner_fast_limit
    };

    printf("ROM HANDLE WRITE %s\r\n",
           (rom_write_record(ROM_REC_HANDLE, &data, sizeof(data)) != 0U) ? "OK" : "FAIL");
}

void handle_read_rom(void)
{
    rom_handle_t data;

    if (rom_read_record(ROM_REC_HANDLE, &data, sizeof(data)) == 0U) {
        return;
    }

    g_fp32_out_corner_limit = data.out_corner_limit;
    g_fp32_in_corner_limit = data.in_corner_limit;
    g_fp32_out_corner_fast_limit = data.out_corner_fast_limit;
    g_fp32_in_corner_fast_limit = data.in_corner_fast_limit;
    printf("ROM HANDLE READ OK\r\n");
}

void mark_write_rom(void)
{
    rom_mark_t data = { g_i32_total_cnt, g_i32_mark_cnt };

    printf("ROM MARK WRITE %s\r\n",
           (rom_write_record(ROM_REC_MARK, &data, sizeof(data)) != 0U) ? "OK" : "FAIL");
}

void mark_read_rom(void)
{
    rom_mark_t data;

    if (rom_read_record(ROM_REC_MARK, &data, sizeof(data)) != 0U) {
        g_i32_total_cnt = data.total_cnt;
        g_i32_mark_cnt = data.mark_cnt;
        printf("ROM MARK READ OK total=%ld\r\n", (long)g_i32_total_cnt);
    }
}

void fast_infor_write_rom(void)
{
    for (uint16_t i = 0U; i < 256U; i++) {
        s_fastinfo_buffer.dist[i] = g_fast_info[i].u16_dist;
        s_fastinfo_buffer.turn_way[i] = g_fast_info[i].u16_turn_way;
        s_fastinfo_buffer.l_dist[i] = g_fast_info[i].fp32_left_dist;
        s_fastinfo_buffer.r_dist[i] = g_fast_info[i].fp32_right_dist;
    }

    printf("ROM FASTINFO WRITE %s\r\n",
           (rom_write_record(ROM_REC_FASTINFO,
                             &s_fastinfo_buffer,
                             sizeof(s_fastinfo_buffer)) != 0U) ? "OK" : "FAIL");
}

void fast_infor_read_rom(void)
{
    if (rom_read_record(ROM_REC_FASTINFO,
                        &s_fastinfo_buffer,
                        sizeof(s_fastinfo_buffer)) == 0U) {
        return;
    }

    for (uint16_t i = 0U; i < 256U; i++) {
        g_fast_info[i].u16_dist = s_fastinfo_buffer.dist[i];
        g_fast_info[i].u16_turn_way = s_fastinfo_buffer.turn_way[i];
        g_fast_info[i].fp32_left_dist = s_fastinfo_buffer.l_dist[i];
        g_fast_info[i].fp32_right_dist = s_fastinfo_buffer.r_dist[i];
    }

    printf("ROM FASTINFO READ OK\r\n");
}

void rom_load_all(void)
{
    
    maxmin_read_rom();
    turnvel_read_rom();
    acc_info_read_rom();
    handle_read_rom();
    turnmark_info_read_rom();
    mark_read_rom();
    fast_infor_read_rom();
}
