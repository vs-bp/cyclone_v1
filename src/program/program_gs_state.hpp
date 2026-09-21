#pragma once
#include <driver/driver.hpp>

/* -------------------------------------------------------------------------- */
/*                                Global State                                */
/* -------------------------------------------------------------------------- */
struct ProgramStateGS {
    uint32_t flag_display_mode = CMD_DISPLAY_MODE_NONE;
    ProgramStateFV rx_wifi;
    LoRaPacketUnpacked rx_lora;
};