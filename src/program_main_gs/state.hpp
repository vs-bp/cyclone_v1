#pragma once
#include <driver_main/_driver.hpp>
#include <program_main_fv/state.hpp>

/* -------------------------------------------------------------------------- */
/*                                Global State                                */
/* -------------------------------------------------------------------------- */
struct ProgramStateGSMain {
    uint32_t flag_display_mode = CMD_DISPLAY_MODE_NONE;
    ProgramStateFVMain rx_wifi;
    LoRaPacketUnpacked rx_lora;
};