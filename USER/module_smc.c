#include "core_framework.h"
#include "runtime_state.h"

static void Module_SMC_Update(float dt_s, void *user_ctx)
{
    float e[4] = {0};
    float u_eq,u_sw;
    float s,sat_s;
    float delta = 0.20f;
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    if (state == 0) {
        return;
    }

    e[0] = state->x[0] - state->x_d[0];
    e[1] = state->x[1] - state->x_d[1];
    e[2] = state->x[2] - state->x_d[2];
    e[3] = state->x[3] - state->x_d[3];
    
    // [0, 1, 20, 1]
    s = 0.0*e[0] + 1*e[1] + 20*e[2] + 1*e[3];
    if (s > -delta && s < delta)
        sat_s = s / delta;
    else
        sat_s = s > 0.0f ? 1 : -1;

    // [0,12.888888888888895,24.043180849068400,1.657363137313881]
    u_eq = 0.0*e[0] + 12.888888888888895*e[1] + 24.043180849068400*e[2] + 11.186579654870580*e[3];
    u_sw = 10.0f * sat_s * sat_s * sat_s;

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
