#include "core_framework.h"
#include "matrix_lite.h"
#include "runtime_state.h"
#include <string.h>

#define MU_STATE_DIM 6
#define MU_INPUT_DIM STATE_DIM

typedef struct {
    // 1. 定义控制器状态
    float x_ctrl[MU_STATE_DIM];
    float x_ctrl_next[MU_STATE_DIM];

    // 2. 离线算好的矩阵数组 (从 MATLAB 导出)
    float Ak[MU_STATE_DIM][MU_STATE_DIM];
    float Bk[MU_STATE_DIM][MU_INPUT_DIM];
    float Ck[1][MU_STATE_DIM];
    float Dk[1][MU_INPUT_DIM];
} MuStateData;

MuStateData g_mu_stata_data = {
    {0},
    {0},
    {{0}},
    {{0}},
    {{0}},
    {{0}},
};

static void Module_Mu_Reset(void *user_ctx)
{
    MuStateData* mu_stata_data = (MuStateData*)user_ctx;
    memset(mu_stata_data->x_ctrl, 0, sizeof(mu_stata_data->x_ctrl));
    memset(mu_stata_data->x_ctrl_next, 0, sizeof(mu_stata_data->x_ctrl_next));
}

static void Module_Mu_Update(float dt_s, void *user_ctx)
{
    float u = 0.0f;
    float ak_x[MU_STATE_DIM];
    float bk_x[MU_STATE_DIM];
    MuStateData* mu_stata_data = (MuStateData*)user_ctx;
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    (void)dt_s;

    if (state == 0) {
        return;
    }

    // 当前控制输出 u = Ck * x_ctrl + Dk * x_measure
    u = MatLite_StateSpaceOutput(&mu_stata_data->Ck[0][0],
                                 MU_STATE_DIM,
                                 mu_stata_data->x_ctrl,
                                 &mu_stata_data->Dk[0][0],
                                 MU_INPUT_DIM,
                                 state->x);

    // 控制器内部状态更新 x_ctrl(k+1) = Ak * x_ctrl(k) + Bk * x_measure(k)
    MatLite_MatVecMul(&mu_stata_data->Ak[0][0],
                      MU_STATE_DIM,
                      MU_STATE_DIM,
                      mu_stata_data->x_ctrl,
                      ak_x);
    MatLite_MatVecMul(&mu_stata_data->Bk[0][0],
                      MU_STATE_DIM,
                      MU_INPUT_DIM,
                      state->x,
                      bk_x);
    MatLite_VecAdd(ak_x, bk_x, mu_stata_data->x_ctrl_next, MU_STATE_DIM);
    memcpy(mu_stata_data->x_ctrl, mu_stata_data->x_ctrl_next, sizeof(mu_stata_data->x_ctrl));

    state->y_angle = u;
}

static FrameworkModuleDescriptor g_module_mu =
{
    "mu",
    0.005f,
    Priority_Controller_Angle,
    Module_Mu_Update,
    Module_Mu_Reset,
    &g_mu_stata_data
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_mu)
