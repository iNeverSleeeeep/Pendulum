#include "core_framework.h"
#include "runtime_state.h"

static void Module_Scope_Update(float dt_s, void *user_ctx)
{
    delay_flag = 0; // 50ms中断精准延时标志位
    (void)dt_s;
    (void)user_ctx;
}

static FrameworkModuleDescriptor g_module_Scope =
{
    "scope",
    0.05f,
    Priority_Scope,
    Module_Scope_Update,
    0,
    0
};

// FRAMEWORK_AUTO_REGISTER_MODULE(g_module_Scope)
