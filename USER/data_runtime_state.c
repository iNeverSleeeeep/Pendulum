#include "core_framework.h"
#include "runtime_state.h"
#include <string.h>

static RuntimeState g_runtime_state = {0};

static void RuntimeState_Reset(void *data_ptr, u16 size)
{
    if ((data_ptr == 0) || (size == 0))
    {
        return;
    }
    memset(data_ptr, 0, size);
}

static FrameworkDataDescriptor g_runtime_state_desc =
{
    "runtime_state",
    &g_runtime_state,
    sizeof(g_runtime_state),
    RuntimeState_Reset
};

FRAMEWORK_AUTO_REGISTER_DATA(g_runtime_state_desc)
