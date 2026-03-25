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
        {1.010271, 0.007979, 0.006666, 0.001344, -0.006860, -0.002969, },
        {6.185617, 3.693185, 3.828527, 0.779054, -4.131276, -1.788408, },
        {0.193760, 0.088188, 1.110439, 0.008345, -0.129409, -0.056020, },
        {-21.752494, -9.459385, -13.309012, -1.816415, 14.528147, 6.289161, },
        {-0.000000, -0.000000, -0.000000, -0.000000, 0.999500, 0.000000, },
        {0.094491, 0.042397, 0.075752, 0.003903, -0.063109, 0.967693, },
    },
    {
        {-0.000017, 0.001483, -0.000273, 0.000053, },
        {-0.010328, -0.005775, 0.019221, 0.061559, },
        {-0.000324, -0.004007, 0.008648, 0.022963, },
        {0.036321, 0.008775, -0.049477, -0.139320, },
        {0.004999, -0.000000, 0.000000, -0.000000, },
        {-0.000158, -0.001344, 0.002258, 0.008981, },
    },
    {
        {1894.719323, 838.020771, 1176.292047, 237.607944, -1265.453123, -547.808242, },
    },
    {
        {-3.163692, -1.950233, 6.186288, 19.889620, },
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
