#include "core_framework.h"
#include "runtime_state.h"

static void Module_HInf_Update(float dt_s, void *user_ctx)
{
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    if (state == 0) {
        return;
    }

}

static FrameworkModuleDescriptor g_module_hinf =
{
    "hinf",
    0.005f,
    Priority_Controller_Pos,
    Module_HInf_Update,
    0,
    0
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_hinf)