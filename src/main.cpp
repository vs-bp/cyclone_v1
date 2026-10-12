#include <program_main_fv/_program.hpp>
#include <program_main_gs/_program.hpp>
#include <program_cam/_program.hpp>

void setup() {
    #ifdef ENV_FV_MAIN
    
        ProgramStateFVMain state_fv;
        program_fv_initialization(state_fv);
        program_fv_loop(state_fv, true);
        program_fv_loop(state_fv, false);
        program_fv_disarmed();
    #endif

    #ifdef ENV_GS_MAIN
        ProgramStateGSMain state_gs;
        program_gs_initialization(state_gs);
        program_gs_loop(state_gs);
    #endif

    #ifdef ENV_CAM
        program_cam_initialization();
        program_cam_loop();
    #endif
}
void loop() {
    ESP.restart();
}