#include "core_framework.h"
#include "runtime_state.h"

typedef struct {
    float time_after_reset;
} StepData;

StepData g_step_xd0_data = {0};

static void Module_Step_XD0_Reset(void *user_ctx)
{
    StepData* step_data = (StepData*)user_ctx;
    step_data->time_after_reset = 0;
}

static void Module_Step_XD0_Update(float dt_s, void *user_ctx)
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

static FrameworkModuleDescriptor g_module_step_xd0 =
{
    "step_xd0",
    5.0f,
    Priority_Input,
    Module_Step_XD0_Update,
    Module_Step_XD0_Reset,
    &g_step_xd0_data
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_step_xd0)

StepData g_step_xd2_data = {0};

static void Module_Step_XD2_Reset(void *user_ctx)
{
    StepData* step_data = (StepData*)user_ctx;
    step_data->time_after_reset = 0;
}

static void Module_Step_XD2_Update(float dt_s, void *user_ctx)
{
    StepData* step_data = (StepData*)user_ctx;
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    if (state == 0) {
        return;
    }

    step_data->time_after_reset += dt_s;
    if (step_data->time_after_reset > 0.99f)
    {
        state->x_d[2] = 0.1f;
    }
    if (step_data->time_after_reset > 1.49f)
    {
        state->x_d[2] = 0.0f;
    }
}

static FrameworkModuleDescriptor g_module_step_xd2 =
{
    "step_xd2",
    0.001f,
    Priority_Input,
    Module_Step_XD2_Update,
    Module_Step_XD2_Reset,
    &g_step_xd2_data
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_step_xd2)