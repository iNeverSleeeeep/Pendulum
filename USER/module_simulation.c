#include "core_framework.h"
#include "runtime_state.h"

typedef struct {
    u8 last_valid;
    float x[4];
} LastState;

typedef struct
{
    /* 状态矩阵 A */
    float A[4][4];
    /* 输入矩阵 B */
    float B[4];
} StateSpaceConfig;


static StateSpaceConfig g_state_space_cfg =
{
    {
        {0.00f, 1.00f, 0.00f, 0.000f},
        {0.00f, -129.8502214711504f, -1.440558467105710, 0.000f},
        {0.00f, 0.00f, 0.00f, 1.00f},
        {0.00f, 285.3851021343966f, 24.704524103529035f, 1.00f},
    },
    {0.00f, 10.074586148623741f, 0.00f, -22.141947579392840f},
};

static void Module_Simulation_Reset(void *user_ctx)
{
    LastState* last_state = (LastState*)user_ctx;
    last_state->last_valid = 0;
}

static void Module_Simulation_Update(float dt_s, void *user_ctx)
{
    LastState* last_state = (LastState*)user_ctx;
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    float u;
    float dx0;
    float dx1;
    float dx2;
    float dx3;
    if (state == 0) {
        return;
    }

    if (last_state->last_valid != 0)
    {
        u = state->y_pos + state->y_w;

        // 状态更新（欧拉积分，基于状态方程 dx/dt = Ax + Bu）
        // dx = A * x + B * u;  
        // x = x + dx * dt;  

        dx0 = g_state_space_cfg.A[0][0] * last_state->x[0]
            + g_state_space_cfg.A[0][1] * last_state->x[1]
            + g_state_space_cfg.A[0][2] * last_state->x[2]
            + g_state_space_cfg.A[0][3] * last_state->x[3]
            + g_state_space_cfg.B[0] * u;
        dx1 = g_state_space_cfg.A[1][0] * last_state->x[0]
            + g_state_space_cfg.A[1][1] * last_state->x[1]
            + g_state_space_cfg.A[1][2] * last_state->x[2]
            + g_state_space_cfg.A[1][3] * last_state->x[3]
            + g_state_space_cfg.B[1] * u;
        dx2 = g_state_space_cfg.A[2][0] * last_state->x[0]
            + g_state_space_cfg.A[2][1] * last_state->x[1]
            + g_state_space_cfg.A[2][2] * last_state->x[2]
            + g_state_space_cfg.A[2][3] * last_state->x[3]
            + g_state_space_cfg.B[2] * u;
        dx3 = g_state_space_cfg.A[3][0] * last_state->x[0]
            + g_state_space_cfg.A[3][1] * last_state->x[1]
            + g_state_space_cfg.A[3][2] * last_state->x[2]
            + g_state_space_cfg.A[3][3] * last_state->x[3]
            + g_state_space_cfg.B[3] * u;

        last_state->x[0] += dx0 * dt_s;
        last_state->x[1] += dx1 * dt_s;
        last_state->x[2] += dx2 * dt_s;
        last_state->x[3] += dx3 * dt_s;
    }

    last_state->x[0] = state->x[0];
    last_state->x[1] = state->x[1];
    last_state->x[2] = state->x[2];
    last_state->x[3] = state->x[3];
    last_state->last_valid = 1;
}

LastState g_last_state = {0};

static FrameworkModuleDescriptor g_module_simulation =
{
    "sim",
    0.001f,
    Priority_Simulation,
    Module_Simulation_Update,
    0,
    &g_last_state
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_simulation)