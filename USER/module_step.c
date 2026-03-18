#include "core_framework.h"
#include "runtime_state.h"

typedef struct {
    float time_after_reset;
} StepData;

StepData g_step_data = {0};

static void Module_Step_Reset(void *user_ctx)
{
    StepData* step_data = (StepData*)user_ctx;
    step_data->time_after_reset = 0;
}

static void Module_Step_Update(float dt_s, void *user_ctx)
{
    StepData* step_data = (StepData*)user_ctx;
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    if (state == 0) {
        return;
    }

    if (state->x_d[0] > 0.0f)
    {
        state->x_d[0] = 0.0f;
    }
    else
    {
        state->x_d[0] = 0.2f;
    }
}

static FrameworkModuleDescriptor g_module_pid =
{
    "step",
    5.0f,
    Priority_Input,
    Module_Step_Update,
    Module_Step_Reset,
    &g_step_data
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_pid)