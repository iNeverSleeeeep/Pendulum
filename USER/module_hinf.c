#include "core_framework.h"
#include "matrix_lite.h"
#include "runtime_state.h"
#include <string.h>

#define HINF_STATE_DIM 6
#define HINF_INPUT_DIM STATE_DIM

typedef struct {
    // 1. 定义控制器状态
    float x_ctrl[HINF_STATE_DIM];
    float x_ctrl_next[HINF_STATE_DIM];

    // 2. 离线算好的矩阵数组 (从 MATLAB 导出)
    float Ak[HINF_STATE_DIM][HINF_STATE_DIM];
    float Bk[HINF_STATE_DIM][HINF_INPUT_DIM];
    float Ck[1][HINF_STATE_DIM];
    float Dk[1][HINF_INPUT_DIM];
} HinfStateData;

HinfStateData g_hinf_stata_data = {
    {0},
    {0},
    {    
        {1.001544, 0.005741, 0.001866, 0.000543, 0.000475, -0.000015, },
        {0.770428, 1.373579, 0.897464, 0.138303, 0.237059, -0.007592, },
        {0.006651, 0.003497, 1.005802, -0.003445, 0.002047, -0.000066, },
        {-2.744871, -1.329460, -3.031512, 0.452954, -0.844592, 0.027049, },
        {-0.194551, -0.075674, -0.151172, -0.074412, 0.939142, 0.002102, },
        {0.007421, 0.003748, 0.027292, -0.003887, 0.002284, 0.994939, },
    },
    {
        {0.000003, 0.000007, -0.000038, -0.000240, },
        {0.001231, -0.000363, 0.002068, 0.013065, },
        {0.000011, -0.000275, 0.001548, 0.009747, },
        {-0.004387, -0.000230, 0.001283, 0.008171, },
        {0.010072, -0.001428, 0.008075, 0.050966, },
        {0.000012, -0.000153, 0.000855, 0.005391, },
    },
    {
        {235.201597, 126.953327, 277.819920, 41.711186, 72.371145, -2.317792, },
    },
    {
        {0.375905, -0.126346, 0.714393, 4.508940, },
    },
};

static void Module_HInf_Reset(void *user_ctx)
{
    HinfStateData* hinf_stata_data = (HinfStateData*)user_ctx;
    memset(hinf_stata_data->x_ctrl, 0, sizeof(hinf_stata_data->x_ctrl));
    memset(hinf_stata_data->x_ctrl_next, 0, sizeof(hinf_stata_data->x_ctrl_next));
}

static void Module_HInf_Update(float dt_s, void *user_ctx)
{
    float u = 0.0f;
    float ak_x[HINF_STATE_DIM];
    float bk_x[HINF_STATE_DIM];
    HinfStateData* hinf_stata_data = (HinfStateData*)user_ctx;
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    (void)dt_s;

    if (state == 0) {
        return;
    }

    // 当前控制输出 u = Ck * x_ctrl + Dk * x_measure
    u = MatLite_StateSpaceOutput(&hinf_stata_data->Ck[0][0],
                                 HINF_STATE_DIM,
                                 hinf_stata_data->x_ctrl,
                                 &hinf_stata_data->Dk[0][0],
                                 HINF_INPUT_DIM,
                                 state->x);

    // 控制器内部状态更新 x_ctrl(k+1) = Ak * x_ctrl(k) + Bk * x_measure(k)
    MatLite_MatVecMul(&hinf_stata_data->Ak[0][0],
                      HINF_STATE_DIM,
                      HINF_STATE_DIM,
                      hinf_stata_data->x_ctrl,
                      ak_x);
    MatLite_MatVecMul(&hinf_stata_data->Bk[0][0],
                      HINF_STATE_DIM,
                      HINF_INPUT_DIM,
                      state->x,
                      bk_x);
    MatLite_VecAdd(ak_x, bk_x, hinf_stata_data->x_ctrl_next, HINF_STATE_DIM);
    memcpy(hinf_stata_data->x_ctrl, hinf_stata_data->x_ctrl_next, sizeof(hinf_stata_data->x_ctrl));

    state->y_angle = u;
}

static FrameworkModuleDescriptor g_module_hinf =
{
    "hinf",
    0.005f,
    Priority_Controller_Angle,
    Module_HInf_Update,
    Module_HInf_Reset,
    &g_hinf_stata_data
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_hinf)
