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

    step_data->time_after_reset += dt_s;
    if (step_data->time_after_reset > 1.0f)
    {
        state->y_pos = 5;
    }
}

static FrameworkModuleDescriptor g_module_pid =
{
    "step",
    0.05f,
    Priority_Controller_Pos,
    Module_Step_Update,
    Module_Step_Reset,
    &g_step_data
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_pid)