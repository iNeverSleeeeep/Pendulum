#include "core_framework.h"
#include "runtime_state.h"

static void Module_DeadZone_Compensation_Update(float dt_s, void *user_ctx)
{
    float y_raw;
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    if (state == 0) {
        return;
    }
    
    y_raw = state->y_angle + state->y_pos;
    if (y_raw > -1e-6 && y_raw < 1e-6)
    {
        state->y_compensation = -y_raw;
    }
    else if (y_raw > 0 && y_raw < 1.3)
    {
        state->y_compensation = 1.3 - y_raw;
    }
    else if (y_raw < 0 && y_raw > -1.3)
    {
        state->y_compensation = -1.3 - y_raw;
    }
}

static FrameworkModuleDescriptor g_module_deadzone_compensation =
{
    "deadzone_compensation",
    0.001f,
    Priority_DeadZoneCompensation,
    Module_DeadZone_Compensation_Update,
    0,
    0
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_deadzone_compensation)
