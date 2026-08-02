#include "run_log.h"

#include "board.h"

#include "stm32h7xx_hal.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define RUN_LOG_FLASH_ADDRESS 0x080E0000UL
#define RUN_LOG_FLASH_SECTOR_SIZE 0x20000UL
#define RUN_LOG_MAGIC 0x32474F4CUL
#define RUN_LOG_VERSION 2U
#define RUN_LOG_MAX_RECORDS 512U
#define RUN_LOG_SLOT_COUNT 3U

typedef struct {
    uint32_t timestamp_ms;
    uint32_t state;
    uint32_t fault;
    uint32_t index;
    float gyro_z_rad_s;
    float yaw_rad;
    float command_speed_m_s;
    float heading_correction_rad_s;
    float distance_m;
    float temperature_c;
    float wheel_rad_s[4];
    uint32_t can_tx_errors;
    uint32_t can_bus_offs;
    float cross_track_m;
    float cross_track_command_m_s;
    float actual_cross_speed_m_s;
    uint32_t event;
} run_log_record_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t record_count;
    uint32_t final_state;
    uint32_t fault;
    uint32_t sample_period_ms;
    uint32_t payload_crc32;
    uint32_t reserved;
} run_log_header_t;

_Static_assert(sizeof(run_log_record_t) == 80U, "run log record must be 80 bytes");
_Static_assert(sizeof(run_log_header_t) == 32U, "run log header must be 32 bytes");

#define RUN_LOG_SLOT_SIZE (sizeof(run_log_header_t) + \
                           RUN_LOG_MAX_RECORDS * sizeof(run_log_record_t))

_Static_assert(RUN_LOG_SLOT_COUNT * RUN_LOG_SLOT_SIZE <= RUN_LOG_FLASH_SECTOR_SIZE,
               "run log slots must fit in the flash sector");

static run_log_record_t records[RUN_LOG_MAX_RECORDS] __attribute__((aligned(32)));
static uint32_t record_count;
static bool saved;

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t length)
{
    size_t index;

    for (index = 0U; index < length; ++index) {
        uint32_t bit;
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc >> 1U) ^ (0xEDB88320UL & (0U - (crc & 1U)));
        }
    }
    return crc;
}

static uint32_t records_crc32(const run_log_record_t *data, uint32_t count)
{
    return crc32_update(0xFFFFFFFFUL, (const uint8_t *)data,
                        (size_t)count * sizeof(*data)) ^ 0xFFFFFFFFUL;
}

static uint32_t log_slot_address(uint32_t slot)
{
    return RUN_LOG_FLASH_ADDRESS + slot * RUN_LOG_SLOT_SIZE;
}

static const run_log_header_t *log_slot_header(uint32_t slot)
{
    return (const run_log_header_t *)log_slot_address(slot);
}

static const run_log_record_t *log_slot_records(uint32_t slot)
{
    return (const run_log_record_t *)(log_slot_address(slot) +
                                      sizeof(run_log_header_t));
}

static bool log_slot_valid(uint32_t slot)
{
    const run_log_header_t *header = log_slot_header(slot);
    const run_log_record_t *stored = log_slot_records(slot);

    return header->magic == RUN_LOG_MAGIC &&
           header->version == RUN_LOG_VERSION &&
           header->record_count <= RUN_LOG_MAX_RECORDS &&
           records_crc32(stored, header->record_count) == header->payload_crc32;
}

static bool log_slot_erased(uint32_t slot)
{
    const uint32_t *words = (const uint32_t *)log_slot_address(slot);
    const uint32_t word_count = RUN_LOG_SLOT_SIZE / sizeof(uint32_t);
    uint32_t index;

    for (index = 0U; index < word_count; ++index) {
        if (words[index] != 0xFFFFFFFFUL) {
            return false;
        }
    }
    return true;
}

static bool find_latest_log(uint32_t *slot, const run_log_header_t **header)
{
    uint32_t index;
    uint32_t latest_slot = 0U;
    uint32_t latest_sequence = 0U;
    bool found = false;

    for (index = 0U; index < RUN_LOG_SLOT_COUNT; ++index) {
        const run_log_header_t *candidate = log_slot_header(index);
        if (!log_slot_valid(index)) {
            continue;
        }
        if (!found || candidate->reserved >= latest_sequence) {
            latest_slot = index;
            latest_sequence = candidate->reserved;
            found = true;
        }
    }
    if (found) {
        if (slot != NULL) {
            *slot = latest_slot;
        }
        if (header != NULL) {
            *header = log_slot_header(latest_slot);
        }
    }
    return found;
}

static uint32_t next_log_sequence(void)
{
    uint32_t index;
    uint32_t latest_sequence = 0U;
    bool found = false;

    for (index = 0U; index < RUN_LOG_SLOT_COUNT; ++index) {
        const run_log_header_t *header = log_slot_header(index);
        if (!log_slot_valid(index)) {
            continue;
        }
        if (!found || header->reserved >= latest_sequence) {
            latest_sequence = header->reserved;
            found = true;
        }
    }
    return found ? latest_sequence + 1U : 1U;
}

static bool find_next_log_slot(uint32_t *slot)
{
    uint32_t latest_slot = 0U;
    const run_log_header_t *latest_header = NULL;
    uint32_t index;

    if (slot == NULL) {
        return false;
    }
    if (find_latest_log(&latest_slot, &latest_header)) {
        for (index = 1U; index <= RUN_LOG_SLOT_COUNT; ++index) {
            uint32_t candidate = (latest_slot + index) % RUN_LOG_SLOT_COUNT;
            if (log_slot_erased(candidate)) {
                *slot = candidate;
                return true;
            }
        }
    } else {
        for (index = 0U; index < RUN_LOG_SLOT_COUNT; ++index) {
            if (log_slot_erased(index)) {
                *slot = index;
                return true;
            }
        }
    }
    return false;
}

static bool flash_program_words(uint32_t address, const void *data, size_t length)
{
    const uint8_t *bytes = data;
    size_t offset;

    if ((length % 32U) != 0U) {
        return false;
    }
    for (offset = 0U; offset < length; offset += 32U) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address + (uint32_t)offset,
                              (uint32_t)(uintptr_t)(bytes + offset)) != HAL_OK) {
            return false;
        }
    }
    return true;
}

static int32_t scaled(float value, float factor)
{
    const float result = value * factor;
    return result >= 0.0f ? (int32_t)(result + 0.5f) : (int32_t)(result - 0.5f);
}

void run_log_reset(void)
{
    memset(records, 0, sizeof(records));
    record_count = 0U;
    saved = false;
}

void run_log_sample(uint32_t timestamp_ms, uint32_t state, uint32_t fault,
                    float gyro_z_rad_s, float yaw_rad, float command_speed_m_s,
                    float heading_correction_rad_s, float distance_m,
                    float temperature_c, const float wheel_rad_s[4],
                    float cross_track_m, float cross_track_command_m_s,
                    float actual_cross_speed_m_s, uint32_t event)
{
    run_log_record_t *record;

    if (record_count >= RUN_LOG_MAX_RECORDS || wheel_rad_s == NULL) {
        return;
    }
    record = &records[record_count];
    record->timestamp_ms = timestamp_ms;
    record->state = state;
    record->fault = fault;
    record->index = record_count;
    record->gyro_z_rad_s = gyro_z_rad_s;
    record->yaw_rad = yaw_rad;
    record->command_speed_m_s = command_speed_m_s;
    record->heading_correction_rad_s = heading_correction_rad_s;
    record->distance_m = distance_m;
    record->temperature_c = temperature_c;
    memcpy(record->wheel_rad_s, wheel_rad_s, sizeof(record->wheel_rad_s));
    record->can_tx_errors = g_fdcan_tx_error_count;
    record->can_bus_offs = g_fdcan_bus_off_count;
    record->cross_track_m = cross_track_m;
    record->cross_track_command_m_s = cross_track_command_m_s;
    record->actual_cross_speed_m_s = actual_cross_speed_m_s;
    record->event = event;
    record_count++;
}

bool run_log_save(uint32_t final_state, uint32_t fault)
{
    FLASH_EraseInitTypeDef erase = {0};
    run_log_header_t header __attribute__((aligned(32))) = {0};
    uint32_t sector_error = 0U;
    uint32_t slot = 0U;
    uint32_t slot_address;
    size_t payload_length;
    size_t programmed_length;
    bool ok;

    if (saved) {
        return true;
    }
    header.magic = RUN_LOG_MAGIC;
    header.version = RUN_LOG_VERSION;
    header.record_count = record_count;
    header.final_state = final_state;
    header.fault = fault;
    header.sample_period_ms = RUN_LOG_SAMPLE_PERIOD_MS;
    header.payload_crc32 = records_crc32(records, record_count);
    header.reserved = next_log_sequence();

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = FLASH_SECTOR_7;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }
    ok = find_next_log_slot(&slot);
    if (!ok) {
        ok = HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK;
        slot = 0U;
    }
    slot_address = log_slot_address(slot);
    if (ok && record_count > 0U) {
        payload_length = (size_t)record_count * sizeof(records[0]);
        programmed_length = (payload_length + 31U) & ~(size_t)31U;
        ok = flash_program_words(slot_address + sizeof(header), records,
                                 programmed_length);
    }
    if (ok) {
        ok = flash_program_words(slot_address, &header, sizeof(header));
    }
    (void)HAL_FLASH_Lock();
    SCB_InvalidateICache();
    saved = ok;
    return ok;
}

void run_log_dump_stored(void)
{
    uint32_t slot = 0U;
    const run_log_header_t *header = NULL;
    const run_log_record_t *stored;
    char line[256];
    uint32_t index;
    int length;

    if (!find_latest_log(&slot, &header)) {
        board_uart1_write("LOG,NONE\r\n");
        return;
    }
    stored = log_slot_records(slot);
    length = snprintf(line, sizeof(line),
                      "LOG,BEGIN,slot=%lu,seq=%lu,count=%lu,state=%lu,fault=%lu,period_ms=%lu,record_bytes=%lu\r\n",
                      (unsigned long)slot,
                      (unsigned long)header->reserved,
                      (unsigned long)header->record_count,
                      (unsigned long)header->final_state,
                      (unsigned long)header->fault,
                      (unsigned long)header->sample_period_ms,
                      (unsigned long)sizeof(run_log_record_t));
    if (length > 0) {
        board_uart1_write(line);
    }
    board_uart1_write("index,time_ms,state,fault,gyro_mrad_s,yaw_mrad,speed_mm_s,correction_mrad_s,distance_mm,temp_mC,fl_mrad_s,fr_mrad_s,rl_mrad_s,rr_mrad_s,can_tx_errors,can_bus_offs,cross_mm,cross_cmd_mm_s,cross_speed_mm_s,event\r\n");
    for (index = 0U; index < header->record_count; ++index) {
        const run_log_record_t *record = &stored[index];
        length = snprintf(
            line, sizeof(line),
            "%lu,%lu,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%lu,%lu,%ld,%ld,%ld,%lu\r\n",
            (unsigned long)record->index, (unsigned long)record->timestamp_ms,
            (unsigned long)record->state, (unsigned long)record->fault,
            (long)scaled(record->gyro_z_rad_s, 1000.0f),
            (long)scaled(record->yaw_rad, 1000.0f),
            (long)scaled(record->command_speed_m_s, 1000.0f),
            (long)scaled(record->heading_correction_rad_s, 1000.0f),
            (long)scaled(record->distance_m, 1000.0f),
            (long)scaled(record->temperature_c, 1000.0f),
            (long)scaled(record->wheel_rad_s[0], 1000.0f),
            (long)scaled(record->wheel_rad_s[1], 1000.0f),
            (long)scaled(record->wheel_rad_s[2], 1000.0f),
            (long)scaled(record->wheel_rad_s[3], 1000.0f),
            (unsigned long)record->can_tx_errors,
            (unsigned long)record->can_bus_offs,
            (long)scaled(record->cross_track_m, 1000.0f),
            (long)scaled(record->cross_track_command_m_s, 1000.0f),
            (long)scaled(record->actual_cross_speed_m_s, 1000.0f),
            (unsigned long)record->event);
        if (length > 0) {
            board_uart1_write(line);
        }
    }
    board_uart1_write("LOG,END\r\n");
}
