#include "core_framework.h"
#include "runtime_state.h"

static void Module_PID_Update(float dt_s, void *user_ctx)
{
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    if (state == 0) {
        return;
    }

    state->y_pos = 10.0f * state->x0 + 0.0f * state->x1 + 0.0f * state->x2 + 0.0f * state->x3;
}

static FrameworkModuleDescriptor g_module_pid =
{
    "pid",
    0.05f,
    Priority_Controller_Pos,
    Module_PID_Update,
    0,
    0
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_pid)
