#include "core_framework.h"
#include "runtime_state.h"
#include <string.h>
#include "api.h"
#include "utils.h"

#ifndef DAQP_SINGLE_PRECISION
#error DAQP_SINGLE_PRECISION must be enabled for module_mpc.c
#endif

#define MPC_STATE_DIM 3
#define MPC_INPUT_DIM 1
#define MPC_PREDICTION_HORIZON 10
#define MPC_QP_ITERS 12
#define MPC_STATUS_SETUP_BASE   (-100)
#define MPC_STATUS_UPDATE_BASE  (-200)
#define MPC_STATUS_SOLVE_BASE   (-300)
#define MPC_STATUS_INPUT_ERROR  (-400)
#define MPC_STATUS_STATE_ERROR  (-401)

#define MPC_STATUS_SETUP_INFEASIBLE     (MPC_STATUS_SETUP_BASE + 1)
#define MPC_STATUS_SETUP_CYCLE          (MPC_STATUS_SETUP_BASE + 2)
#define MPC_STATUS_SETUP_UNBOUNDED      (MPC_STATUS_SETUP_BASE + 3)
#define MPC_STATUS_SETUP_ITERLIMIT      (MPC_STATUS_SETUP_BASE + 4)
#define MPC_STATUS_SETUP_NONCONVEX      (MPC_STATUS_SETUP_BASE + 5)
#define MPC_STATUS_SETUP_OVERDET        (MPC_STATUS_SETUP_BASE + 6)
#define MPC_STATUS_SETUP_UNKNOWN        (MPC_STATUS_SETUP_BASE)

#define MPC_STATUS_UPDATE_INFEASIBLE    (MPC_STATUS_UPDATE_BASE + 1)
#define MPC_STATUS_UPDATE_CYCLE         (MPC_STATUS_UPDATE_BASE + 2)
#define MPC_STATUS_UPDATE_UNBOUNDED     (MPC_STATUS_UPDATE_BASE + 3)
#define MPC_STATUS_UPDATE_ITERLIMIT     (MPC_STATUS_UPDATE_BASE + 4)
#define MPC_STATUS_UPDATE_NONCONVEX     (MPC_STATUS_UPDATE_BASE + 5)
#define MPC_STATUS_UPDATE_OVERDET       (MPC_STATUS_UPDATE_BASE + 6)
#define MPC_STATUS_UPDATE_UNKNOWN       (MPC_STATUS_UPDATE_BASE)

#define MPC_STATUS_SOLVE_ZERO           (MPC_STATUS_SOLVE_BASE)
#define MPC_STATUS_SOLVE_INFEASIBLE     (MPC_STATUS_SOLVE_BASE + 1)
#define MPC_STATUS_SOLVE_CYCLE          (MPC_STATUS_SOLVE_BASE + 2)
#define MPC_STATUS_SOLVE_UNBOUNDED      (MPC_STATUS_SOLVE_BASE + 3)
#define MPC_STATUS_SOLVE_ITERLIMIT      (MPC_STATUS_SOLVE_BASE + 4)
#define MPC_STATUS_SOLVE_NONCONVEX      (MPC_STATUS_SOLVE_BASE + 5)
#define MPC_STATUS_SOLVE_OVERDET        (MPC_STATUS_SOLVE_BASE + 6)

static int Mpc_MapStageStatus(int stage_base, int raw_code)
{
    if (raw_code > 0) {
        return raw_code;
    }

    switch (raw_code) {
    case 0:
        return stage_base;
    case EXIT_INFEASIBLE:
        return stage_base + 1;
    case EXIT_CYCLE:
        return stage_base + 2;
    case EXIT_UNBOUNDED:
        return stage_base + 3;
    case EXIT_ITERLIMIT:
        return stage_base + 4;
    case EXIT_NONCONVEX:
        return stage_base + 5;
    case EXIT_OVERDETERMINED_INITIAL:
        return stage_base + 6;
    default:
        return stage_base;
    }
}

typedef struct
{
    /* 离散状态矩阵 A */
    float A[MPC_STATE_DIM][MPC_STATE_DIM];
    /* 离散输入矩阵 B，单输入时按列向量展开 */
    float B[MPC_STATE_DIM];
    /* 预测区间状态权重矩阵 Q */
    float Q[MPC_STATE_DIM][MPC_STATE_DIM];
    /* 终端状态权重矩阵 F */
    float F[MPC_STATE_DIM][MPC_STATE_DIM];
    /* 控制输入权重 R */
    float R;
    /* 控制量约束上限，满足 |u| <= u_max */
    float u_max;
} MpcConfig;

typedef struct
{
    /* 预测矩阵和 H 是否已经初始化 */
    u8 is_initialized;
    /* MATLAB 中的 M，按行堆叠 A^1 到 A^Np */
    float M[MPC_PREDICTION_HORIZON * MPC_STATE_DIM][MPC_STATE_DIM];
    /* MATLAB 中的 C，表示未来输入对未来状态的映射 */
    float C[MPC_PREDICTION_HORIZON * MPC_STATE_DIM][MPC_PREDICTION_HORIZON];
    /* MATLAB 中的 Q_bar */
    float Q_bar[MPC_PREDICTION_HORIZON * MPC_STATE_DIM][MPC_PREDICTION_HORIZON * MPC_STATE_DIM];
    /* 二次规划 Hessian 矩阵 H，按行优先连续存放 */
    float H[MPC_PREDICTION_HORIZON][MPC_PREDICTION_HORIZON];
    /* DAQP 需要的 Hessian 上三角压缩存储 */
    /* 当前控制序列 U */
    float u_seq[MPC_PREDICTION_HORIZON];
    /* 预计算阶段临时矩阵 A^k */
    float powers[MPC_PREDICTION_HORIZON + 1][MPC_STATE_DIM][MPC_STATE_DIM];
    /* 预计算阶段临时向量 */
    float tmp_vec[MPC_STATE_DIM];
    /* 求解阶段临时向量 M * x */
    float mx[MPC_PREDICTION_HORIZON * MPC_STATE_DIM];
    /* 求解阶段临时向量 Q_bar * M * x */
    float qmx[MPC_PREDICTION_HORIZON * MPC_STATE_DIM];
    /* 求解阶段线性项 */
    float f[MPC_PREDICTION_HORIZON];
    /* 输入上下界 */
    float lb[MPC_PREDICTION_HORIZON];
    float ub[MPC_PREDICTION_HORIZON];
} MpcWorkspace;

typedef struct
{
    /* 最大迭代次数 */
    u16 max_iters;
} QuadProgOptions;

typedef struct
{
    /* DAQP 是否已经完成 setup */
    u8 is_setup;
    /* DAQP 设置 */
    DAQPSettings settings;
    /* DAQP 问题描述 */
    DAQPProblem problem;
    /* DAQP 工作区 */
    DAQPWorkspace workspace;
    /* DAQP 求解结果 */
    DAQPResult result;
    /* DAQP 输出的最优控制序列 */
    c_float primal[MPC_PREDICTION_HORIZON];
    /* DAQP 输出的对偶变量 */
    c_float dual[MPC_PREDICTION_HORIZON];
    /* 控制量下界 */
    c_float lower[MPC_PREDICTION_HORIZON];
    /* 控制量上界 */
    c_float upper[MPC_PREDICTION_HORIZON];
    /* 约束类型，0 表示普通不等式/边界 */
    int sense[MPC_PREDICTION_HORIZON];
} MpcDaqpContext;

static MpcConfig g_mpc_cfg =
{
    {
        {1.00f, 0.05f, 0.009f},
        {0.00f, 1.00f, 0.3097f},
        {0.00f, 0.00f, 0.3679f}
    },
    {0.0032f, 0.1803f, 0.6321f},
    {
        {100.0f/0.36f, 0.0f, 0.0f},
        {0.0f, 10.0f/0.36f, 0.0f},
        {0.0f, 0.0f, 100.0f/0.25f}
    },
    {
        {7.9643e3f, 5.2318e3f, 2.3612e3f},
        {5.2318e3f, 7.1424e3f, 3.3240e3f},
        {2.3612e3f, 3.3240e3f, 2.0178e3f}
    },
    20000.0f,
    0.5f
};

static MpcWorkspace g_mpc_ws = {0};
static MpcDaqpContext g_mpc_qp = {0};

static float Mpc_Clamp(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static void Mpc_Mat3Mul(const float a[MPC_STATE_DIM][MPC_STATE_DIM],
                        const float b[MPC_STATE_DIM][MPC_STATE_DIM],
                        float out[MPC_STATE_DIM][MPC_STATE_DIM])
{
    u8 i;
    u8 j;
    u8 k;
    for (i = 0; i < MPC_STATE_DIM; ++i) {
        for (j = 0; j < MPC_STATE_DIM; ++j) {
            float sum = 0.0f;
            for (k = 0; k < MPC_STATE_DIM; ++k) {
                sum += a[i][k] * b[k][j];
            }
            out[i][j] = sum;
        }
    }
}

static void Mpc_Mat3VecMul(const float a[MPC_STATE_DIM][MPC_STATE_DIM],
                           const float b[MPC_STATE_DIM],
                           float out[MPC_STATE_DIM])
{
    u8 i;
    u8 k;
    for (i = 0; i < MPC_STATE_DIM; ++i) {
        float sum = 0.0f;
        for (k = 0; k < MPC_STATE_DIM; ++k) {
            sum += a[i][k] * b[k];
        }
        out[i] = sum;
    }
}

static void Mpc_MatIdentity(float out[MPC_STATE_DIM][MPC_STATE_DIM])
{
    u8 i;
    u8 j;
    for (i = 0; i < MPC_STATE_DIM; ++i) {
        for (j = 0; j < MPC_STATE_DIM; ++j) {
            out[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}




static int Mpc_DaqpSetup(void)
{
    int setup_flag;
    u8 i;

    if (g_mpc_qp.is_setup != 0) {
        return 0;
    }

    memset(&g_mpc_qp, 0, sizeof(g_mpc_qp));

    daqp_default_settings(&g_mpc_qp.settings);
    g_mpc_qp.settings.iter_limit = MPC_QP_ITERS;

    for (i = 0; i < MPC_PREDICTION_HORIZON; ++i) {
        g_mpc_qp.lower[i] = -g_mpc_cfg.u_max;
        g_mpc_qp.upper[i] = g_mpc_cfg.u_max;
        g_mpc_qp.sense[i] = 0;
    }

    g_mpc_qp.problem.n = MPC_PREDICTION_HORIZON;
    g_mpc_qp.problem.m = MPC_PREDICTION_HORIZON;
    g_mpc_qp.problem.ms = MPC_PREDICTION_HORIZON;
    g_mpc_qp.problem.H = &g_mpc_ws.H[0][0];
    g_mpc_qp.problem.f = g_mpc_ws.f;
    g_mpc_qp.problem.A = 0;
    g_mpc_qp.problem.bupper = g_mpc_qp.upper;
    g_mpc_qp.problem.blower = g_mpc_qp.lower;
    g_mpc_qp.problem.sense = g_mpc_qp.sense;
    g_mpc_qp.problem.break_points = 0;
    g_mpc_qp.problem.nh = 0;

    g_mpc_qp.workspace.settings = &g_mpc_qp.settings;
    setup_flag = setup_daqp(&g_mpc_qp.problem, &g_mpc_qp.workspace, 0);
    if (setup_flag < 0) {
        return setup_flag;
    }

    g_mpc_qp.result.x = g_mpc_qp.primal;
    g_mpc_qp.result.lam = g_mpc_qp.dual;
    g_mpc_qp.is_setup = 1;
    return 0;
}

static void Mpc_WorkspaceInit_S1(void)
{
    u8 i;
    u8 j;
    u8 r;
    u8 c;

    Mpc_MatIdentity(g_mpc_ws.powers[0]);
    for (i = 1; i <= MPC_PREDICTION_HORIZON; ++i) {
        Mpc_Mat3Mul(g_mpc_ws.powers[i - 1], g_mpc_cfg.A, g_mpc_ws.powers[i]);
    }

    for (r = 0; r < MPC_PREDICTION_HORIZON * MPC_STATE_DIM; ++r) {
        for (c = 0; c < MPC_STATE_DIM; ++c) {
            g_mpc_ws.M[r][c] = 0.0f;
        }
    }
    for (r = 0; r < MPC_PREDICTION_HORIZON * MPC_STATE_DIM; ++r) {
        for (c = 0; c < MPC_PREDICTION_HORIZON; ++c) {
            g_mpc_ws.C[r][c] = 0.0f;
        }
    }
    for (r = 0; r < MPC_PREDICTION_HORIZON * MPC_STATE_DIM; ++r) {
        for (c = 0; c < MPC_PREDICTION_HORIZON * MPC_STATE_DIM; ++c) {
            g_mpc_ws.Q_bar[r][c] = 0.0f;
        }
    }

    for (i = 0; i < MPC_PREDICTION_HORIZON; ++i) {
        for (r = 0; r < MPC_STATE_DIM; ++r) {
            for (c = 0; c < MPC_STATE_DIM; ++c) {
                g_mpc_ws.M[i * MPC_STATE_DIM + r][c] = g_mpc_ws.powers[i + 1][r][c];
            }
        }
        for (j = 0; j <= i; ++j) {
            Mpc_Mat3VecMul(g_mpc_ws.powers[i - j], g_mpc_cfg.B, g_mpc_ws.tmp_vec);
            for (r = 0; r < MPC_STATE_DIM; ++r) {
                g_mpc_ws.C[i * MPC_STATE_DIM + r][j] = g_mpc_ws.tmp_vec[r];
            }
        }
        for (r = 0; r < MPC_STATE_DIM; ++r) {
            for (c = 0; c < MPC_STATE_DIM; ++c) {
                if (i == (MPC_PREDICTION_HORIZON - 1)) {
                    g_mpc_ws.Q_bar[i * MPC_STATE_DIM + r][i * MPC_STATE_DIM + c] = g_mpc_cfg.F[r][c];
                } else {
                    g_mpc_ws.Q_bar[i * MPC_STATE_DIM + r][i * MPC_STATE_DIM + c] = g_mpc_cfg.Q[r][c];
                }
            }
        }
    }

    for (r = 0; r < MPC_PREDICTION_HORIZON; ++r) {
        for (c = 0; c < MPC_PREDICTION_HORIZON; ++c) {
            float sum = 0.0f;
            u8 k;
            u8 m;

            for (k = 0; k < MPC_PREDICTION_HORIZON * MPC_STATE_DIM; ++k) {
                for (m = 0; m < MPC_PREDICTION_HORIZON * MPC_STATE_DIM; ++m) {
                    sum += g_mpc_ws.C[k][r] * g_mpc_ws.Q_bar[k][m] * g_mpc_ws.C[m][c];
                }
            }
            if (r == c) {
                sum += g_mpc_cfg.R;
            }
            g_mpc_ws.H[r][c] = 2.0f * sum;
        }
    }

    for (r = 0; r < MPC_PREDICTION_HORIZON; ++r) {
        g_mpc_ws.u_seq[r] = 0.0f;
    }

    g_mpc_ws.is_initialized = 1;
}

static void Mpc_WorkspaceInit_S2(void)
{
    (void)Mpc_DaqpSetup();
}

static void Mpc_WorkspaceInit(void)
{
    if (g_mpc_ws.is_initialized == 0) {
        Mpc_WorkspaceInit_S1();
    }
    Mpc_WorkspaceInit_S2();
}

static void Mpc_ModuleReset(void *user_ctx)
{
    (void)user_ctx;
    Mpc_WorkspaceInit();
}

static int quadprog(const float H[MPC_PREDICTION_HORIZON][MPC_PREDICTION_HORIZON],
                    const float f[MPC_PREDICTION_HORIZON],
                    const float lb[MPC_PREDICTION_HORIZON],
                    const float ub[MPC_PREDICTION_HORIZON],
                    const QuadProgOptions *options,
                    float U_k[MPC_PREDICTION_HORIZON],
                    float *FVal)
{
    int exitflag;
    u8 i;
    (void)H;

    exitflag = Mpc_DaqpSetup();
    if (exitflag < 0) {
        return Mpc_MapStageStatus(MPC_STATUS_SETUP_BASE, exitflag);
    }

    if (options != 0 && options->max_iters > 0) {
        g_mpc_qp.workspace.settings->iter_limit = options->max_iters;
    } else {
        g_mpc_qp.workspace.settings->iter_limit = MPC_QP_ITERS;
    }

    for (i = 0; i < MPC_PREDICTION_HORIZON; ++i) {
        g_mpc_qp.lower[i] = lb[i];
        g_mpc_qp.upper[i] = ub[i];
    }

    g_mpc_qp.problem.f = (c_float *)f;
    exitflag = update_ldp(UPDATE_v + UPDATE_d, &g_mpc_qp.workspace, &g_mpc_qp.problem);
    if (exitflag < 0) {
        return Mpc_MapStageStatus(MPC_STATUS_UPDATE_BASE, exitflag);
    }

    daqp_solve(&g_mpc_qp.result, &g_mpc_qp.workspace);
    if (g_mpc_qp.result.exitflag <= 0) {
        return Mpc_MapStageStatus(MPC_STATUS_SOLVE_BASE, g_mpc_qp.result.exitflag);
    }

    for (i = 0; i < MPC_PREDICTION_HORIZON; ++i) {
        U_k[i] = (float)g_mpc_qp.primal[i];
    }

    if (FVal != 0) {
        *FVal = (float)g_mpc_qp.result.fval;
    }

    return g_mpc_qp.result.exitflag;
}

static int Mpc_Solve(const float x[MPC_STATE_DIM], float *u0_out)
{
    /* 当前二次规划目标函数值 */
    float f_val = 0.0f;
    /* quadprog 求解选项 */
    QuadProgOptions options;
    int qp_status;
    u8 i;
    u8 j;

    if ((x == 0) || (u0_out == 0)) {
        return MPC_STATUS_INPUT_ERROR;
    }

    if (g_mpc_ws.is_initialized == 0) {
        Mpc_WorkspaceInit();
    }

    for (i = 0; i < MPC_PREDICTION_HORIZON * MPC_STATE_DIM; ++i) {
        float sum = 0.0f;
        for (j = 0; j < MPC_STATE_DIM; ++j) {
            sum += g_mpc_ws.M[i][j] * x[j];
        }
        g_mpc_ws.mx[i] = sum;
    }

    for (i = 0; i < MPC_PREDICTION_HORIZON * MPC_STATE_DIM; ++i) {
        float sum = 0.0f;
        for (j = 0; j < MPC_PREDICTION_HORIZON * MPC_STATE_DIM; ++j) {
            sum += g_mpc_ws.Q_bar[i][j] * g_mpc_ws.mx[j];
        }
        g_mpc_ws.qmx[i] = sum;
    }

    for (i = 0; i < MPC_PREDICTION_HORIZON; ++i) {
        float sum = 0.0f;
        for (j = 0; j < MPC_PREDICTION_HORIZON * MPC_STATE_DIM; ++j) {
            sum += g_mpc_ws.C[j][i] * g_mpc_ws.qmx[j];
        }
        g_mpc_ws.f[i] = 2.0f * sum;
    }

    for (i = 0; i < MPC_PREDICTION_HORIZON; ++i) {
        g_mpc_ws.lb[i] = -g_mpc_cfg.u_max;
        g_mpc_ws.ub[i] = g_mpc_cfg.u_max;
        g_mpc_ws.u_seq[i] = Mpc_Clamp(g_mpc_ws.u_seq[i], g_mpc_cfg.u_max);
    }

    options.max_iters = MPC_QP_ITERS;
    qp_status = quadprog(g_mpc_ws.H, g_mpc_ws.f, g_mpc_ws.lb, g_mpc_ws.ub, &options, g_mpc_ws.u_seq, &f_val);
    if (qp_status <= 0) {
        g_mpc_ws.u_seq[0] = Mpc_Clamp(g_mpc_ws.u_seq[0], g_mpc_cfg.u_max);
    }

    *u0_out = g_mpc_ws.u_seq[0];
    return qp_status;
}

static void Module_MPC_Update(float dt_s, void *user_ctx)
{
    /* 当前状态误差向量 x = [x0-x0_ref, x1-x1_ref, x2-x2_ref] */
    float x[MPC_STATE_DIM];
    /* 优化后得到的首个控制量 */
    float u_k = 0.0f;
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);

    (void)dt_s;
    (void)user_ctx;

    if (state == 0) {
        return;
    }

    /* 小车位置误差 */
    x[0] = state->x[0] - state->x_d[0];
    /* 小车速度误差 */
    x[1] = state->x[1] - state->x_d[1];
    /* 摆杆角度误差 */
    x[2] = state->x[2] - state->x_d[2];

    Mpc_Solve(x, &u_k);

    /* 对应 MATLAB 输出 x_d(1) */
    state->x_d[0] = 0.0f;
    /* 对应 MATLAB 输出 x_d(2) */
    state->x_d[1] = 0.0f;
    /* 对应 MATLAB 输出 x_d(3) = u_k */
    state->x_d[2] = u_k;
    /* 对应 MATLAB 输出 x_d(4) */
    state->x_d[3] = 0.0f;
}

static FrameworkModuleDescriptor g_module_mpc =
{
    "mpc",
    0.05f,
    Priority_Controller_Pos,
    Module_MPC_Update,
    Mpc_ModuleReset,
    0
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_mpc)
