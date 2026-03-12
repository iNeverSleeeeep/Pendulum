#include "core_framework.h"
#include "runtime_state.h"

static void Module_SMC_Update(float dt_s, void *user_ctx)
{
    float e[4] = {0};
    float u_eq,u_sw;
    float s,sat_s;
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    if (state == 0) {
        return;
    }

    e[0] = state->x0 - state->x0_target;
    e[1] = state->x1 - state->x1_target;
    e[2] = state->x2 - state->x2_target;
    e[3] = state->x3 - state->x3_target;
    
    // [0, 1, 20, 1]
    s = 0.0*e[0] + 1*e[1] + 20*e[2] + 1*e[3];
    if (abs(s) <= 0.3)
        sat_s = s / 0.3;
    else
        sat_s = s > 0 ? 1 : -1;

    // [0,12.888888888888890,1.927841953677244,1.657363137313881]
    u_eq = 0.0*e[0] + 12.888888888888890*e[1] + 1.927841953677244*e[2] + 1.657363137313881*e[3];
    u_eq -= 0.0*state->x0_target + 12.888888888888890*state->x1_target + 1.927841953677244*state->x2_target + 1.657363137313881*state->x3_target;
    u_sw = 10 * sat_s;

    state->y_w = u_eq + u_sw;
}

static FrameworkModuleDescriptor g_module_smc =
{
    "smc",
    0.005f,
    Priority_Controller_W,
    Module_SMC_Update,
    0,
    0
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_smc)