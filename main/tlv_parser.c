#include <string.h>
#include <stdbool.h>
#include <esp_log.h>
#include "tlv_parser.h"

const uint8_t MAGIC_WORD[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};

uint32_t parse_uint32_le(const uint8_t *data)
{
    return ((uint32_t)data[0]) | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

float parse_float_le(const uint8_t *data)
{
    uint32_t int_val = parse_uint32_le(data);
    return *(float *)&int_val;
}

int16_t parse_int16_le(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0]) | ((uint16_t)data[1] << 8));
}

void parse_header(const uint8_t *buffer, mmwHeader *header)
{
    header->version = parse_uint32_le(&buffer[0]);
    header->totalPacketLen = parse_uint32_le(&buffer[4]);
    header->platform = parse_uint32_le(&buffer[8]);
    header->frameNumber = parse_uint32_le(&buffer[12]);
    header->timeCpuCycles = parse_uint32_le(&buffer[16]);
    header->numDetectedObj = parse_uint32_le(&buffer[20]);
    header->numTLVs = parse_uint32_le(&buffer[24]);
    header->subFrameNumber = parse_uint32_le(&buffer[28]);

    ESP_LOGI("PARSER", "Header:");
    ESP_LOGI("PARSER", "  version        = %ld", header->version);
    ESP_LOGI("PARSER", "  totalPacketLen = %ld", header->totalPacketLen);
    ESP_LOGI("PARSER", "  platform       = %ld", header->platform);
    ESP_LOGI("PARSER", "  frameNumber    = %ld", header->frameNumber);
    ESP_LOGI("PARSER", "  cpuCycles      = %ld", header->timeCpuCycles);
    ESP_LOGI("PARSER", "  numDetectedObj = %ld", header->numDetectedObj);
    ESP_LOGI("PARSER", "  numTLVs        = %ld", header->numTLVs);
    ESP_LOGI("PARSER", "  subFrame       = %ld", header->subFrameNumber);

}

void parse_tlv(const uint8_t *buffer, int numTLVs, int offset, int total_len, radarFrame_t *frame)
{
    for (int i = 0; i < numTLVs; i++)
    {
        if (offset + 8 > total_len)
        {
            ESP_LOGI("PARSER", "[ERROR] TLV header extends beyond buffer");
            return;
        }

        uint32_t type = parse_uint32_le(&buffer[offset]);
        uint32_t length = parse_uint32_le(&buffer[offset + 4]);
        offset += 8;

        if (offset + length > total_len)
        {
            ESP_LOGI("PARSER", "[ERROR] TLV payload extends beyond buffer");
            return;
        }

        const uint8_t *payload = &buffer[offset];
        ESP_LOGI("PARSER", "TLV %d: type=%ld, length=%ld", i, type, length);

        switch (type)
        {
        case TLV_POINT_CLOUD:
        {
            if (length < 20)
            {
                ESP_LOGI("PARSER", "[ERROR] Point cloud TLV too short");
                break;
            }

            frame->units.elevationUnit = parse_float_le(&payload[0]);
            frame->units.azimuthUnit   = parse_float_le(&payload[4]);
            frame->units.dopplerUnit   = parse_float_le(&payload[8]);
            frame->units.rangeUnit     = parse_float_le(&payload[12]);
            frame->units.snrUnit       = parse_float_le(&payload[16]);

            ESP_LOGI("PARSER", "  Units:");
            ESP_LOGI("PARSER", "    elevationUnit = %f", frame->units.elevationUnit);
            ESP_LOGI("PARSER", "    azimuthUnit   = %f", frame->units.azimuthUnit);
            ESP_LOGI("PARSER", "    dopplerUnit   = %f", frame->units.dopplerUnit);
            ESP_LOGI("PARSER", "    rangeUnit     = %f", frame->units.rangeUnit);
            ESP_LOGI("PARSER", "    snrUnit       = %f", frame->units.snrUnit);

            const uint8_t *point_data = payload + 20;
            int point_payload_len = length - 20;
            int numPoints = point_payload_len / 8;

            if (numPoints > MAX_POINTS)
            {
                ESP_LOGI("PARSER", "[WARNING] Truncating points to %d", MAX_POINTS);
                numPoints = MAX_POINTS;
            }

            frame->numPoints = numPoints;
            ESP_LOGI("PARSER", "  numPoints = %d", numPoints);

            for (int p = 0; p < numPoints; p++)
            {
                const uint8_t *pt_data = point_data + (p * 8);
                frame->points[p].elevation = (int8_t)pt_data[0];
                frame->points[p].azimuth   = (int8_t)pt_data[1];
                frame->points[p].doppler   = parse_int16_le(&pt_data[2]);
                frame->points[p].range     = parse_int16_le(&pt_data[4]);
                frame->points[p].snr       = parse_int16_le(&pt_data[6]);

                ESP_LOGI("PARSER", "    Point[%d]: elev=%d, az=%d, doppler=%d, range=%d, snr=%d",
                         p,
                         frame->points[p].elevation,
                         frame->points[p].azimuth,
                         frame->points[p].doppler,
                         frame->points[p].range,
                         frame->points[p].snr);
            }
            break;
        }
        case TLV_DETECTED_POINTS:
        {
            frame->numPoints = length / sizeof(pointObj);
            if (frame->numPoints > MAX_POINTS)
                frame->numPoints = MAX_POINTS;
            memcpy(frame->points, payload, frame->numPoints * sizeof(pointObj));

            ESP_LOGI("PARSER", "  Detected Points: %d", frame->numPoints);
            for (int p = 0; p < frame->numPoints; p++)
            {
                ESP_LOGI("PARSER", "    Point[%d]: elev=%d, az=%d, doppler=%d, range=%d, snr=%d",
                         p,
                         frame->points[p].elevation,
                         frame->points[p].azimuth,
                         frame->points[p].doppler,
                         frame->points[p].range,
                         frame->points[p].snr);
            }
            break;
        }
        case TLV_OBJ_LIST:
        {
            frame->numObjects = length / sizeof(listTlv);
            if (frame->numObjects > MAX_OBJECTS)
                frame->numObjects = MAX_OBJECTS;
            memcpy(frame->objects, payload, frame->numObjects * sizeof(listTlv));

            ESP_LOGI("PARSER", "  Object List: %d objects", frame->numObjects);
            break;
        }
        case TLV_INDEX:
        {
            frame->numIndices = length / sizeof(indexTlv);
            if (frame->numIndices > MAX_POINTS)
                frame->numIndices = MAX_POINTS;
            memcpy(frame->indices, payload, frame->numIndices * sizeof(indexTlv));

            ESP_LOGI("PARSER", "  Indices: %d", frame->numIndices);
            break;
        }
        case TLV_HEIGHT:
        {
            frame->numHeights = length / sizeof(heightTlv);
            if (frame->numHeights > MAX_OBJECTS)
                frame->numHeights = MAX_OBJECTS;
            memcpy(frame->heights, payload, frame->numHeights * sizeof(heightTlv));

            ESP_LOGI("PARSER", "  Heights: %d", frame->numHeights);
            break;
        }
        case TLV_PRESENCE:
        {
            if (length >= sizeof(presenceTlv))
            {
                frame->presence.present = parse_uint32_le(payload);
                frame->hasPresence = 1;
                ESP_LOGI("PARSER", "  Presence detected: %ld", frame->presence.present);
            }
            break;
        }
        default:
            ESP_LOGI("PARSER", "[WARNING] Unknown TLV type %ld", type);
            break;
        }
        offset += length;
    }
}

bool find_magic_word(const uint8_t *buffer, int length, int*pos)
{
    for (int i = 0; i <= length - 8; i++)
    {
        if (memcmp(&buffer[i], MAGIC_WORD, 8) == 0){
            *pos = i;
            return true;

        }
    }
    return false;
}