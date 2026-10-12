#pragma once
#include <driver_main/_driver.hpp>
#include <program_main_fv/state.hpp>

/* -------------------------------------------------------------------------- */
/*                               Flight Complete                              */
/* -------------------------------------------------------------------------- */
void program_fv_disarmed() {
    while (true) {
        esp_sleep_enable_timer_wakeup(1000000ULL * 60ULL * 60ULL * 24ULL);
        esp_deep_sleep_start();
    }
}