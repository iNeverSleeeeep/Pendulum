#include "core_framework.h"
#include "runtime_state.h"

static void Module_LQR_Update(float dt_s, void *user_ctx)
{
    // Q = [12000 0 0 0; 0 0 0 0; 0 0 2000 0; 0 0 0 0]; % Q ¾ØÕó
    // [-89.859598943918700,-64.170706672410760,-1.570566429215020e+02,-24.837820582264910]
    float k0 = -89.859598943918700;
    float k1 = -64.170706672410760;
    float k2 = -1.570566429215020e+02;
    float k3 = -24.837820582264910;

    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    if (state == 0) {
        return;
    }

    state->y_w = k0 * state->x[0] + k1 * state->x[1] + k2 * state->x[2] + k3 * state->x[3];
    state->y_w = -state->y_w;
}

static FrameworkModuleDescriptor g_module_lqr =
{
    "lqr",
    0.005f,
    Priority_Controller_W,
    Module_LQR_Update,
    0,
    0
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_lqr)