/*
 * SimpleLPP - Minimal CayenneLPP encoder for Zephyr
 *
 * Implements a subset of CayenneLPP format for telemetry encoding.
 * Supports the sensor types used in ZephCore's EnvironmentSensorManager.
 *
 * Format: [channel:1][type:1][value:N bytes big-endian]
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SIMPLE_LPP_H
#define SIMPLE_LPP_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>

/* LPP Type Codes (from CayenneLPP spec) */
#define LPP_ANALOG_INPUT        2     /* 2 bytes, 0.01 signed */
#define LPP_TEMPERATURE         103   /* 2 bytes, 0.1°C signed */
#define LPP_RELATIVE_HUMIDITY   104   /* 1 byte, 0.5% unsigned */
#define LPP_BAROMETRIC_PRESSURE 115   /* 2 bytes, 0.1 hPa unsigned */
#define LPP_VOLTAGE             116   /* 2 bytes, 0.01V unsigned */
#define LPP_CURRENT             117   /* 2 bytes, 0.001A unsigned */
#define LPP_ALTITUDE            121   /* 2 bytes, 1m signed */
#define LPP_POWER               128   /* 2 bytes, 1W unsigned */
#define LPP_DISTANCE            130   /* 4 bytes, 0.001m unsigned */
#define LPP_GPS                 136   /* 9 bytes: 3 lat, 3 lon, 3 alt */

/* Multipliers */
#define LPP_TEMPERATURE_MULT         10
#define LPP_HUMIDITY_MULT            2
#define LPP_PRESSURE_MULT            10
#define LPP_VOLTAGE_MULT             100
#define LPP_CURRENT_MULT             1000
#define LPP_DISTANCE_MULT            1000
#define LPP_ANALOG_MULT              100
#define LPP_GPS_LAT_LON_MULT         10000
#define LPP_GPS_ALT_MULT             100

/**
 * SimpleLPP - Minimal CayenneLPP encoder
 *
 * Usage:
 *   uint8_t buffer[64];
 *   SimpleLPP lpp(buffer, sizeof(buffer));
 *   lpp.addVoltage(0, 3.85f);       // Battery voltage
 *   lpp.addTemperature(0, 25.5f);   // Temperature
 *   // Use lpp.getBuffer() and lpp.getSize() to get encoded data
 */
class SimpleLPP {
public:
    /**
     * Constructor
     * @param buffer   Buffer to write LPP data into
     * @param max_size Maximum buffer size
     */
    SimpleLPP(uint8_t *buffer, size_t max_size)
        : _buffer(buffer), _maxsize(max_size), _cursor(0) {}

    /** Reset the buffer cursor for reuse */
    void reset() { _cursor = 0; }

    /** Get current encoded data size */
    size_t getSize() const { return _cursor; }

    /** Get pointer to buffer */
    uint8_t *getBuffer() { return _buffer; }

    /** Get const pointer to buffer */
    const uint8_t *getBuffer() const { return _buffer; }

    /**
     * Add temperature reading
     * @param channel Channel number (0 = self)
     * @param celsius Temperature in degrees Celsius
     * @return Number of bytes written, or 0 on overflow
     */
    uint8_t addTemperature(uint8_t channel, float celsius) {
        int16_t val = (int16_t)(celsius * LPP_TEMPERATURE_MULT);
        return addField2Signed(channel, LPP_TEMPERATURE, val);
    }

    /**
     * Add relative humidity reading
     * @param channel Channel number
     * @param percent Humidity in percent (0-100)
     * @return Number of bytes written, or 0 on overflow
     */
    uint8_t addRelativeHumidity(uint8_t channel, float percent) {
        uint8_t val = (uint8_t)(percent * LPP_HUMIDITY_MULT);
        return addField1(channel, LPP_RELATIVE_HUMIDITY, val);
    }

    /**
     * Add barometric pressure reading
     * @param channel Channel number
     * @param hpa Pressure in hectopascals (hPa / mbar)
     * @return Number of bytes written, or 0 on overflow
     */
    uint8_t addBarometricPressure(uint8_t channel, float hpa) {
        uint16_t val = (uint16_t)(hpa * LPP_PRESSURE_MULT);
        return addField2Unsigned(channel, LPP_BAROMETRIC_PRESSURE, val);
    }

    /**
     * Add voltage reading
     * @param channel Channel number
     * @param volts Voltage in volts
     * @return Number of bytes written, or 0 on overflow
     */
    uint8_t addVoltage(uint8_t channel, float volts) {
        uint16_t val = (uint16_t)(volts * LPP_VOLTAGE_MULT);
        return addField2Unsigned(channel, LPP_VOLTAGE, val);
    }

    /**
     * Add current reading
     * @param channel Channel number
     * @param amps Current in amperes
     * @return Number of bytes written, or 0 on overflow
     */
    uint8_t addCurrent(uint8_t channel, float amps) {
        uint16_t val = (uint16_t)(amps * LPP_CURRENT_MULT);
        return addField2Unsigned(channel, LPP_CURRENT, val);
    }

    /**
     * Add power reading
     * @param channel Channel number
     * @param watts Power in watts
     * @return Number of bytes written, or 0 on overflow
     */
    uint8_t addPower(uint8_t channel, float watts) {
        uint16_t val = (uint16_t)watts;
        return addField2Unsigned(channel, LPP_POWER, val);
    }

    /**
     * Add altitude reading
     * @param channel Channel number
     * @param meters Altitude in meters
     * @return Number of bytes written, or 0 on overflow
     */
    uint8_t addAltitude(uint8_t channel, float meters) {
        int16_t val = (int16_t)meters;
        return addField2Signed(channel, LPP_ALTITUDE, val);
    }

    /**
     * Add distance reading
     * @param channel Channel number
     * @param meters Distance in meters
     * @return Number of bytes written, or 0 on overflow
     */
    uint8_t addDistance(uint8_t channel, float meters) {
        uint32_t val = (uint32_t)(meters * LPP_DISTANCE_MULT);
        return addField4(channel, LPP_DISTANCE, val);
    }

    /**
     * Add analog input (generic sensor value)
     * @param channel Channel number
     * @param value Analog value (scaled by 0.01)
     * @return Number of bytes written, or 0 on overflow
     */
    uint8_t addAnalogInput(uint8_t channel, float value) {
        int16_t val = (int16_t)(value * LPP_ANALOG_MULT);
        return addField2Signed(channel, LPP_ANALOG_INPUT, val);
    }

    /**
     * Add GPS location
     * @param channel Channel number
     * @param latitude Latitude in degrees
     * @param longitude Longitude in degrees
     * @param altitude Altitude in meters
     * @return Number of bytes written, or 0 on overflow
     */
    uint8_t addGPS(uint8_t channel, float latitude, float longitude, float altitude) {
        /* GPS format: 3 bytes lat, 3 bytes lon, 3 bytes alt (all signed, big-endian) */
        if (_cursor + 11 > _maxsize) return 0;

        int32_t lat = (int32_t)(latitude * LPP_GPS_LAT_LON_MULT);
        int32_t lon = (int32_t)(longitude * LPP_GPS_LAT_LON_MULT);
        int32_t alt = (int32_t)(altitude * LPP_GPS_ALT_MULT);

        _buffer[_cursor++] = channel;
        _buffer[_cursor++] = LPP_GPS;

        /* Latitude - 3 bytes signed */
        _buffer[_cursor++] = (lat >> 16) & 0xFF;
        _buffer[_cursor++] = (lat >> 8) & 0xFF;
        _buffer[_cursor++] = lat & 0xFF;

        /* Longitude - 3 bytes signed */
        _buffer[_cursor++] = (lon >> 16) & 0xFF;
        _buffer[_cursor++] = (lon >> 8) & 0xFF;
        _buffer[_cursor++] = lon & 0xFF;

        /* Altitude - 3 bytes signed */
        _buffer[_cursor++] = (alt >> 16) & 0xFF;
        _buffer[_cursor++] = (alt >> 8) & 0xFF;
        _buffer[_cursor++] = alt & 0xFF;

        return 11;
    }

private:
    uint8_t *_buffer;
    size_t _maxsize;
    size_t _cursor;

    /* Add 1-byte unsigned field */
    uint8_t addField1(uint8_t channel, uint8_t type, uint8_t value) {
        if (_cursor + 3 > _maxsize) return 0;
        _buffer[_cursor++] = channel;
        _buffer[_cursor++] = type;
        _buffer[_cursor++] = value;
        return 3;
    }

    /* Add 2-byte unsigned field (big-endian) */
    uint8_t addField2Unsigned(uint8_t channel, uint8_t type, uint16_t value) {
        if (_cursor + 4 > _maxsize) return 0;
        _buffer[_cursor++] = channel;
        _buffer[_cursor++] = type;
        _buffer[_cursor++] = (value >> 8) & 0xFF;
        _buffer[_cursor++] = value & 0xFF;
        return 4;
    }

    /* Add 2-byte signed field (big-endian) */
    uint8_t addField2Signed(uint8_t channel, uint8_t type, int16_t value) {
        if (_cursor + 4 > _maxsize) return 0;
        _buffer[_cursor++] = channel;
        _buffer[_cursor++] = type;
        _buffer[_cursor++] = (value >> 8) & 0xFF;
        _buffer[_cursor++] = value & 0xFF;
        return 4;
    }

    /* Add 4-byte unsigned field (big-endian) */
    uint8_t addField4(uint8_t channel, uint8_t type, uint32_t value) {
        if (_cursor + 6 > _maxsize) return 0;
        _buffer[_cursor++] = channel;
        _buffer[_cursor++] = type;
        _buffer[_cursor++] = (value >> 24) & 0xFF;
        _buffer[_cursor++] = (value >> 16) & 0xFF;
        _buffer[_cursor++] = (value >> 8) & 0xFF;
        _buffer[_cursor++] = value & 0xFF;
        return 6;
    }
};

#endif /* SIMPLE_LPP_H */
