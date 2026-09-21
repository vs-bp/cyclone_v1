/* -------------------------------------------------------------------------- */
/*                              Packed Structures                             */
/* -------------------------------------------------------------------------- */
#pragma once
#include <Arduino.h>

struct LoRaPacketPacked { uint8_t buffer[8]; };
struct LoRaPacketUnpacked {
    bool flag_debug;
    bool flag_launch;
    bool flag_burnout;
    bool flag_apogee;
    bool cont1;
    bool cont2;
    bool cont3;
    bool cont4;
    float latitude;
    float longitude;
    float altitude;
    float battery_voltage;
    uint8_t siv;
};

LoRaPacketUnpacked unpack_lora(LoRaPacketPacked in) {
    LoRaPacketUnpacked output;

    // First byte is all the flags packed bit-by-bit along with continuities.
    output.flag_debug    = (in.buffer[0] & 0b00000001) != 0;
    output.flag_launch   = (in.buffer[0] & 0b00000010) != 0;
    output.flag_burnout  = (in.buffer[0] & 0b00000100) != 0;
    output.flag_apogee   = (in.buffer[0] & 0b00001000) != 0;
    output.cont1         = (in.buffer[0] & 0b00010000) != 0;
    output.cont2         = (in.buffer[0] & 0b00100000) != 0;
    output.cont3         = (in.buffer[0] & 0b01000000) != 0;
    output.cont4         = (in.buffer[0] & 0b10000000) != 0;
    // Following four bytes are GPS, with 16 bits for latitude and longitude each.
    // GPS values are relative to a "center" latitude and longitude and are sent as integers
    // where each increment of "1" represents 1/10^5 degrees, giving a total range of 15/10^5 = 0.32768 degrees or 36km in each direction,
    // along with a resolution of 1/10^5 = 0.00001 degrees or 1.11m.
    uint16_t packed_gps_lat, packed_gps_long;
    memcpy(&packed_gps_lat, &in.buffer[1], 2);
    memcpy(&packed_gps_long, &in.buffer[3], 2);
    output.latitude = ((float)packed_gps_lat / 100000.0) + GPS_OFFSET_LAT;
    output.longitude = ((float)packed_gps_long / 100000.0) + GPS_OFFSET_LONG;
    // Following two bytes are altitude, truncated to 1 meter resolution and fit into 16 bytes.
    // This gives a maximum range of 65536 m.
    uint16_t packed_alt;
    memcpy(&packed_alt, &in.buffer[5], 2);
    output.altitude = (float)packed_alt;
    // Following byte is 4-bit SIV and 4-bit battery voltage.
    output.siv = (in.buffer[7] & 0b11110000) >> 4;
    output.battery_voltage = (float)(in.buffer[7] & 0b00001111) * 10.0 + 3.2;

    return output;
}

LoRaPacketPacked pack_lora(LoRaPacketUnpacked in) {
    // Transmit heavily packed lat/long/alt, flags, battery, and SIV.
    // This is the only time the flight vehicle transmits on 915 Mhz, no command byte is needed.
    //
    // Pack state and flags into a single byte.
    // 4 States can pack into two bits.
    uint8_t packed_flags = 0b00000000;
    // Following bits represent one of each for all relevant flags.
    if (in.flag_debug)   packed_flags |= 0b00000001;
    if (in.flag_launch)  packed_flags |= 0b00000010;
    if (in.flag_burnout) packed_flags |= 0b00000100;
    if (in.flag_apogee)  packed_flags |= 0b00001000;
    if (in.cont1)        packed_flags |= 0b00010000;
    if (in.cont2)        packed_flags |= 0b00100000;
    if (in.cont3)        packed_flags |= 0b01000000;
    if (in.cont4)        packed_flags |= 0b10000000;
    // Following four bytes are GPS, with 16 bits for latitude and longitude each.
    // GPS values are relative to a "center" latitude and longitude and are sent as integers
    // where each increment of "1" represents 1/10^5 degrees, giving a total range of 15/10^5 = 0.32768 degrees or 36km in each direction,
    // along with a resolution of 1/10^5 = 0.00001 degrees or 1.11m.
    uint16_t packed_gps_lat = (uint16_t)((in.latitude - GPS_OFFSET_LAT) * 100000.0);
    uint16_t packed_gps_long = (uint16_t)((in.longitude - GPS_OFFSET_LONG) * 100000.0);
    // Following two bytes are altitude, truncated to 1 meter resolution and fit into 16 bytes.
    // This gives a maximum range of 65536 m.
    //
    // Negative values are assumed to be zero.
    float temp = in.altitude;
    if (temp < 0.0) temp = 0.0;
    uint16_t packed_alt = (uint16_t)(temp);
    // Last byte is 4-bit SIV and then 4-bit battey voltage.
    // Battery voltage is such that each increment of "1" represents 0.1 volts starting from 3.2 volts, giving a max of 4.7 volts.
    // SIV is clamped to 0-15, if above 15 then it just stays at 15.
    uint8_t packed_siv = in.siv;
    if (packed_siv > 15) packed_siv = 15;
    uint8_t packed_volt = (uint8_t)((in.battery_voltage - 3.2) * 10.0);
    uint8_t packed_siv_volt = ((packed_siv << 4) & 0b11110000) | (packed_volt & 0b00001111);

    LoRaPacketPacked output;
    memcpy(&output.buffer[0], &packed_flags, 1);
    memcpy(&output.buffer[1], &packed_gps_lat, 2);
    memcpy(&output.buffer[3], &packed_gps_long, 2);
    memcpy(&output.buffer[5], &packed_alt, 2);
    memcpy(&output.buffer[7], &packed_siv_volt, 1);
    return output;
}