#ifndef RC_PROTOCOL_H
#define RC_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RC_MAX_CHANNELS 16U
#define RC_IBUS_FRAME_SIZE 32U
#define RC_SBUS_FRAME_SIZE 25U
/* Parser-level electrical sanity limits; configured control channels are stricter. */
#define RC_PHYSICAL_CHANNEL_MIN_US 750U
#define RC_PHYSICAL_CHANNEL_MAX_US 2250U
/* Frame bytes are ~0.1 ms apart; 2 ms safely identifies an interrupted frame. */
#define RC_STREAM_GAP_RESET_MS 2U

typedef enum {
    RC_PROTOCOL_IBUS = 1,
    RC_PROTOCOL_SBUS = 2
} rc_protocol_kind_t;

typedef enum {
    RC_PARSE_NONE = 0,
    RC_PARSE_VALID_FRAME = 1,
    RC_PARSE_RANGE_ERROR = -1,
    RC_PARSE_FRAME_LOST = -2,
    RC_PARSE_FAILSAFE = -3
} rc_parse_result_t;

typedef struct {
    uint16_t channels[RC_MAX_CHANNELS]; /* microsecond-equivalent, nominally 1000..2000 */
    uint8_t channel_count;
    bool frame_lost;
    bool failsafe;
    uint32_t timestamp_ms;
} rc_frame_t;

typedef struct {
    uint32_t last_valid_ms;
    uint32_t valid_frame_count;
    uint32_t checksum_error_count;
    uint32_t framing_error_count;
    uint32_t range_error_count;
    uint32_t lost_frame_count;
    uint32_t failsafe_frame_count;
    uint32_t byte_count;
    uint32_t gap_reset_count;
} rc_parser_stats_t;

typedef struct {
    rc_protocol_kind_t kind;
    uint16_t channel_min;
    uint16_t channel_max;
    uint8_t buffer[RC_IBUS_FRAME_SIZE];
    uint8_t index;
    uint32_t last_byte_ms;
    bool has_last_byte;
    rc_parser_stats_t stats;
} rc_parser_t;

void rc_parser_init(rc_parser_t *parser, rc_protocol_kind_t kind,
                     uint16_t channel_min, uint16_t channel_max);
/* Drop a partial frame after a UART error while preserving all statistics. */
void rc_parser_reset_stream(rc_parser_t *parser);
rc_parse_result_t rc_parser_feed(rc_parser_t *parser, uint8_t byte,
                                 uint32_t now_ms, rc_frame_t *out_frame);
bool rc_sbus_end_byte_is_valid(uint8_t byte);

#endif
