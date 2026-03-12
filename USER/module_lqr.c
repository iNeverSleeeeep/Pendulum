#include "core_framework.h"
#include "runtime_state.h"

static void Module_LQR_Update(float dt_s, void *user_ctx)
{
		float k0 = 36.3167;
		float k1 = 46.1784;
		float k2 = -138.1564;
		float k3 = -22.6988;

    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    if (state == 0) {
        return;
    }

    state->y_w = k0 * state->x0 + k1 * state->x1 + k2 * state->x2 + k3 * state->x3;
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