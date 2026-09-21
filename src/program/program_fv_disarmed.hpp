#pragma once
#include <driver/driver.hpp>

/* -------------------------------------------------------------------------- */
/*                               Flight Complete                              */
/* -------------------------------------------------------------------------- */
void program_fv_disarmed() {
    while (true) {
        esp_sleep_enable_timer_wakeup(1000000 * 60 * 60 * 24);
        esp_deep_sleep_start();
    }
}