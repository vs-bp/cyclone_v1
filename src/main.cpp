/* -------------------------------------------------------------------------- */
/*                                   Preface                                  */
/* -------------------------------------------------------------------------- */
//
/* --------------------------- Naming Conventions --------------------------- */
// In general cases:
// hwm = Main MCU-specific interface functions
// hwc = Camera MCU-specific interface functions
//
// mfv = Main MCU, flight vehicle.
// mgs = Main MCU, ground station
// cfv = Camera MCU, flight vehicle.
// cgs = Camera MCU, ground station

#ifdef ENV_FV_MAIN
    #include <program_mfv.hpp>
    void setup() { program_mfv(); }
    void loop() { ESP.restart(); }
#endif
#ifdef ENV_GS_MAIN
    #include <program_mgs.hpp>
    void setup() { program_mgs(); }
    void loop() { ESP.restart(); }
#endif
#ifdef ENV_FV_CAM
    #include <program_cfv.hpp>
    void setup() { program_cfv(); }
    void loop() { ESP.restart(); }
#endif
#ifdef ENV_GS_CAM
    #include <program_cgs.hpp>
    void setup() { program_cgs(); }
    void loop() { ESP.restart(); }
#endif