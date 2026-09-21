#include <program/program.hpp>

void setup() {
    #ifdef ENV_FV_MAIN
        ProgramStateFV state_fv;
        program_fv_initialization(state_fv);
        program_fv_loop(state_fv, true);
        program_fv_loop(state_fv, false);
    #endif

    #ifdef ENV_GS_MAIN
        ProgramStateGS state_gs;
        program_gs_initialization(state_gs);
        program_gs_loop(state_gs);
    #endif
}
void loop() {
    ESP.restart();
}