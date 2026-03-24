#include "core_framework.h"
#include "runtime_state.h"
#include <string.h>

#define MU_STATE_DIM 4

typedef struct {
    // 1. 定义控制器状态 (假设降阶后是 4 阶控制器)
    float x_ctrl[MU_STATE_DIM];
    float x_ctrl_next[MU_STATE_DIM];

    // 2. 离线算好的矩阵数组 (从 MATLAB 导出)
    float Ak[MU_STATE_DIM][MU_STATE_DIM];
    float Bk[MU_STATE_DIM][MU_STATE_DIM]; // 这里的输入通常是误差 e = ref - x
    float Ck[1][MU_STATE_DIM];
    float Dk[1][MU_STATE_DIM];
} MuStateData;

MuStateData g_mu_stata_data = {
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
    },
    {
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
    },
    {
        {0, 0, 0, 0},
    },
    {
        {0, 0, 0, 0},
    },
};

static float Mu_RowVecMul(const float row[1][MU_STATE_DIM], const float vec[MU_STATE_DIM])
{
    u8 i;
    float sum = 0.0f;

    for (i = 0; i < MU_STATE_DIM; ++i) {
        sum += row[0][i] * vec[i];
    }

    return sum;
}

static void Mu_MatVecMul(const float mat[MU_STATE_DIM][MU_STATE_DIM],
                         const float vec[MU_STATE_DIM],
                         float out[MU_STATE_DIM])
{
    u8 i;
    u8 j;

    for (i = 0; i < MU_STATE_DIM; ++i) {
        float sum = 0.0f;
        for (j = 0; j < MU_STATE_DIM; ++j) {
            sum += mat[i][j] * vec[j];
        }
        out[i] = sum;
    }
}

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
    u8 i;
    MuStateData* mu_stata_data = (MuStateData*)user_ctx;
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    (void)dt_s;

    if (state == 0) {
        return;
    }

    // --- 在线执行矩阵计算 ---

    // 计算当前输出 u = Ck * x_ctrl + Dk * x_measure
    // 这一步就是你要的控制量 u
    u = Mu_RowVecMul(mu_stata_data->Ck, mu_stata_data->x_ctrl) +
        Mu_RowVecMul(mu_stata_data->Dk, state->x);

    // 更新控制器内部状态 x_ctrl = Ak * x_ctrl + Bk * x_measure
    // 这就是“执行状态空间计算”，为了给下一次中断做准备
    Mu_MatVecMul(mu_stata_data->Ak, mu_stata_data->x_ctrl, ak_x);
    Mu_MatVecMul(mu_stata_data->Bk, state->x, bk_x);
    for (i = 0; i < MU_STATE_DIM; ++i) {
        mu_stata_data->x_ctrl_next[i] = ak_x[i] + bk_x[i];
    }
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
