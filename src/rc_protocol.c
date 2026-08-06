#include "rc_protocol.h"

#include <string.h>

static void reset_or_resync(rc_parser_t *parser, uint8_t frame_size, bool two_byte_header)
{
    uint8_t i;
    const uint8_t first = two_byte_header ? 0x20U : 0x0FU;

    for (i = 1U; i < frame_size; ++i) {
        if (parser->buffer[i] == first) {
            if (!two_byte_header ||
                ((i + 1U) < frame_size && parser->buffer[i + 1U] == 0x40U)) {
                const uint8_t kept = (uint8_t)(frame_size - i);
                memmove(parser->buffer, &parser->buffer[i], kept);
                parser->index = kept;
                return;
            }
        }
    }

    if (two_byte_header && parser->buffer[frame_size - 1U] == 0x20U) {
        parser->buffer[0] = 0x20U;
        parser->index = 1U;
    } else {
        parser->index = 0U;
    }
}

static bool channels_in_range(const rc_parser_t *parser, const rc_frame_t *frame)
{
    uint8_t i;
    for (i = 0U; i < frame->channel_count; ++i) {
        if (frame->channels[i] < parser->channel_min ||
            frame->channels[i] > parser->channel_max) {
            return false;
        }
    }
    return true;
}

static rc_parse_result_t finish_ibus(rc_parser_t *parser, uint32_t now_ms,
                                     rc_frame_t *out_frame)
{
    uint32_t sum = 0U;
    uint16_t received_checksum;
    uint16_t expected_checksum;
    rc_frame_t frame = {0};
    uint8_t i;

    for (i = 0U; i < 30U; ++i) {
        sum += parser->buffer[i];
    }
    expected_checksum = (uint16_t)(0xFFFFU - sum);
    received_checksum = (uint16_t)parser->buffer[30] |
                        ((uint16_t)parser->buffer[31] << 8);
    if (received_checksum != expected_checksum) {
        parser->stats.checksum_error_count++;
        reset_or_resync(parser, RC_IBUS_FRAME_SIZE, true);
        return RC_PARSE_NONE;
    }

    frame.channel_count = 14U;
    frame.timestamp_ms = now_ms;
    for (i = 0U; i < frame.channel_count; ++i) {
        const uint8_t offset = (uint8_t)(2U + (2U * i));
        frame.channels[i] = (uint16_t)parser->buffer[offset] |
                            ((uint16_t)parser->buffer[offset + 1U] << 8);
    }
    parser->index = 0U;

    if (!channels_in_range(parser, &frame)) {
        parser->stats.range_error_count++;
        if (out_frame != NULL) {
            *out_frame = frame;
        }
        return RC_PARSE_RANGE_ERROR;
    }

    parser->stats.last_valid_ms = now_ms;
    parser->stats.valid_frame_count++;
    if (out_frame != NULL) {
        *out_frame = frame;
    }
    return RC_PARSE_VALID_FRAME;
}

bool rc_sbus_end_byte_is_valid(uint8_t byte)
{
    return byte == 0x00U || byte == 0x04U || byte == 0x14U ||
           byte == 0x24U || byte == 0x34U;
}

static uint16_t sbus_to_us(uint16_t raw)
{
    const int32_t numerator = ((int32_t)raw - 172) * 1000;
    const int32_t rounded = numerator >= 0 ? numerator + 819 : numerator - 819;
    const int32_t value = 1000 + rounded / 1639;
    return (uint16_t)value;
}

static rc_parse_result_t finish_sbus(rc_parser_t *parser, uint32_t now_ms,
                                     rc_frame_t *out_frame)
{
    rc_frame_t frame = {0};
    uint8_t channel;

    if (!rc_sbus_end_byte_is_valid(parser->buffer[24])) {
        parser->stats.framing_error_count++;
        reset_or_resync(parser, RC_SBUS_FRAME_SIZE, false);
        return RC_PARSE_NONE;
    }

    frame.channel_count = 16U;
    frame.timestamp_ms = now_ms;
    frame.frame_lost = (parser->buffer[23] & 0x04U) != 0U;
    frame.failsafe = (parser->buffer[23] & 0x08U) != 0U;
    for (channel = 0U; channel < 16U; ++channel) {
        const uint16_t bit = (uint16_t)channel * 11U;
        const uint8_t offset = (uint8_t)(1U + (bit / 8U));
        const uint8_t shift = (uint8_t)(bit % 8U);
        const uint32_t packed = (uint32_t)parser->buffer[offset] |
                                ((uint32_t)parser->buffer[offset + 1U] << 8) |
                                ((uint32_t)parser->buffer[offset + 2U] << 16);
        frame.channels[channel] = sbus_to_us((uint16_t)((packed >> shift) & 0x07FFU));
    }
    parser->index = 0U;

    if (out_frame != NULL) {
        *out_frame = frame;
    }
    if (frame.failsafe) {
        parser->stats.failsafe_frame_count++;
        if (frame.frame_lost) {
            parser->stats.lost_frame_count++;
        }
        return RC_PARSE_FAILSAFE;
    }
    if (frame.frame_lost) {
        parser->stats.lost_frame_count++;
        return RC_PARSE_FRAME_LOST;
    }
    if (!channels_in_range(parser, &frame)) {
        parser->stats.range_error_count++;
        return RC_PARSE_RANGE_ERROR;
    }

    parser->stats.last_valid_ms = now_ms;
    parser->stats.valid_frame_count++;
    return RC_PARSE_VALID_FRAME;
}

void rc_parser_init(rc_parser_t *parser, rc_protocol_kind_t kind,
                    uint16_t channel_min, uint16_t channel_max)
{
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->kind = kind;
    parser->channel_min = channel_min;
    parser->channel_max = channel_max;
}

void rc_parser_reset_stream(rc_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }
    parser->index = 0U;
    parser->last_byte_ms = 0U;
    parser->has_last_byte = false;
    memset(parser->buffer, 0, sizeof(parser->buffer));
}

rc_parse_result_t rc_parser_feed(rc_parser_t *parser, uint8_t byte,
                                 uint32_t now_ms, rc_frame_t *out_frame)
{
    if (parser == NULL) {
        return RC_PARSE_NONE;
    }
    if (parser->index != 0U && parser->has_last_byte &&
        (uint32_t)(now_ms - parser->last_byte_ms) >= RC_STREAM_GAP_RESET_MS) {
        rc_parser_reset_stream(parser);
        parser->stats.gap_reset_count++;
    }
    parser->last_byte_ms = now_ms;
    parser->has_last_byte = true;
    parser->stats.byte_count++;

    if (parser->kind == RC_PROTOCOL_IBUS) {
        if (parser->index == 0U) {
            if (byte == 0x20U) {
                parser->buffer[0] = byte;
                parser->index = 1U;
            }
            return RC_PARSE_NONE;
        }
        if (parser->index == 1U) {
            if (byte != 0x40U) {
                parser->stats.framing_error_count++;
                parser->index = 0U;
                if (byte == 0x20U) {
                    parser->buffer[0] = byte;
                    parser->index = 1U;
                }
                return RC_PARSE_NONE;
            }
            parser->buffer[1] = byte;
            parser->index = 2U;
            return RC_PARSE_NONE;
        }
        parser->buffer[parser->index++] = byte;
        if (parser->index == RC_IBUS_FRAME_SIZE) {
            return finish_ibus(parser, now_ms, out_frame);
        }
        return RC_PARSE_NONE;
    }

    if (parser->kind == RC_PROTOCOL_SBUS) {
        if (parser->index == 0U) {
            if (byte == 0x0FU) {
                parser->buffer[0] = byte;
                parser->index = 1U;
            }
            return RC_PARSE_NONE;
        }
        parser->buffer[parser->index++] = byte;
        if (parser->index == RC_SBUS_FRAME_SIZE) {
            return finish_sbus(parser, now_ms, out_frame);
        }
    }
    return RC_PARSE_NONE;
}
