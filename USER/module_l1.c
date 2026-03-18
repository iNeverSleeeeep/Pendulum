#include "core_framework.h"
#include "runtime_state.h"

typedef struct {
    /* 上一次 L1 模块执行时记录的 x_sim_e，用于和本次做差。 */
    float x_sim_e_last[4];
    /* 对总模型失配/总扰动的在线估计。 */
    float sigma_hat;
    /* 经过低通后的自适应补偿输出，最终叠加到 y_w。 */
    float u_ad;
} L1Data;

L1Data g_l1_data = {0};

static void Module_L1_Reset(void *user_ctx)
{
    L1Data *l1_data = (L1Data *)user_ctx;
    l1_data->sigma_hat = 0.0f;
    l1_data->u_ad = 0.0f;
    l1_data->x_sim_e_last[0] = 0.0f;
    l1_data->x_sim_e_last[1] = 0.0f;
    l1_data->x_sim_e_last[2] = 0.0f;
    l1_data->x_sim_e_last[3] = 0.0f;
}

static void Module_L1_Update(float dt_s, void *user_ctx)
{
    float residual;
    float sigma_dot;
    float u_cmd;
    float u_ad_dot;
    float delta_x_sim_e[4];
    /* 扰动估计限幅，避免 sigma_hat 长时间积累后失控。 */
    const float sigma_limit = 6.0f;
    /* 补偿输出限幅，避免 L1 补偿把主控制器顶得过猛。 */
    const float u_limit = 3.0f;
    /* 自适应增益，越大说明 sigma_hat 跟踪 residual 越快。 */
    const float gamma = 18.0f;
    /* 低通滤波带宽，决定 u_ad 跟踪理想补偿 u_cmd 的快慢。 */
    const float filter_bw = 35.0f;
    /* 当前这个 L1 周期内，各状态维度上的平均模型误差。 */
    float e[4];
    L1Data *l1_data = (L1Data *)user_ctx;
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    if (state == 0) {
        return;
    }

    if (dt_s <= 0.0f) {
        return;
    }

    /*
     * 现在 x_sim_e 的定义已经改成：
     * x_sim_e += (x - x_sim)
     *
     * 也就是说它保存的不是“误差时间积分”，而是“误差样本累加”。
     * 因此本次 L1 周期内的新增量：
     * delta_x_sim_e = x_sim_e_now - x_sim_e_last
     *
     * 表示的是这段时间里所有 1 kHz 子步误差的求和。
     */
    delta_x_sim_e[0] = state->x_sim_e[0] - l1_data->x_sim_e_last[0];
    delta_x_sim_e[1] = state->x_sim_e[1] - l1_data->x_sim_e_last[1];
    delta_x_sim_e[2] = state->x_sim_e[2] - l1_data->x_sim_e_last[2];
    delta_x_sim_e[3] = state->x_sim_e[3] - l1_data->x_sim_e_last[3];

    e[0] = delta_x_sim_e[0] / dt_s;
    e[1] = delta_x_sim_e[1] / dt_s;
    e[2] = delta_x_sim_e[2] / dt_s;
    e[3] = delta_x_sim_e[3] / dt_s;

    /*
     * 把四维状态误差按经验权重合成为一个标量 residual。
     * 这里的思路是：把位置、速度、角度、角速度上的模型失配
     * 投影成一个“等效总扰动”，再交给后面的自适应律去估计。
     *
     * 权重越大，说明该状态误差对补偿输出的影响越大。
     * 当前参数里对摆角 e[2] 权重最大，说明更关注摆角通道的失配。
     */
    residual = 2.5f * e[0] + 0.2f * e[1] + 35.0f * e[2] + 1.0f * e[3];

    /*
     * 一阶自适应律：
     * sigma_dot = gamma * (residual - sigma_hat)
     *
     * 含义：
     * residual  = 当前由模型误差推出来的“目标扰动估计”
     * sigma_hat = 模块内部当前保存的扰动估计
     * sigma_dot = 扰动估计的变化率
     *
     * 如果 residual 持续存在，sigma_hat 会逐步向 residual 靠拢；
     * residual 变小后，sigma_hat 也会逐步回落。
     */
    sigma_dot = gamma * (residual - l1_data->sigma_hat);
    l1_data->sigma_hat += sigma_dot * dt_s;
    if (l1_data->sigma_hat > sigma_limit) {
        l1_data->sigma_hat = sigma_limit;
    } else if (l1_data->sigma_hat < -sigma_limit) {
        l1_data->sigma_hat = -sigma_limit;
    }

    /*
     * 理想补偿量 u_cmd 直接取 -sigma_hat，目的就是抵消 sigma_hat
     * 代表的那部分等效扰动/模型失配。
     */
    u_cmd = -l1_data->sigma_hat;

    /*
     * L1 的关键不是只做“快速估计”，而是：
     * 先快速估计 sigma_hat，再通过低通滤波器生成可执行的平滑补偿 u_ad。
     *
     * 如果没有这个低通，补偿会跟着估计值快速变化，容易把高频噪声也打进控制量。
     * 所以最终真正加到系统上的不是 u_cmd，而是经过滤波后的 u_ad。
     */
    u_ad_dot = filter_bw * (u_cmd - l1_data->u_ad);
    l1_data->u_ad += u_ad_dot * dt_s;
    if (l1_data->u_ad > u_limit) {
        l1_data->u_ad = u_limit;
    } else if (l1_data->u_ad < -u_limit) {
        l1_data->u_ad = -u_limit;
    }

    /* 把 L1 自适应补偿量叠加到原有角度控制输出上。 */
    state->y_w += l1_data->u_ad;

    /* 保存本次读到的累计误差，供下一个 L1 周期做差分。 */
    l1_data->x_sim_e_last[0] = state->x_sim_e[0];
    l1_data->x_sim_e_last[1] = state->x_sim_e[1];
    l1_data->x_sim_e_last[2] = state->x_sim_e[2];
    l1_data->x_sim_e_last[3] = state->x_sim_e[3];
}

static FrameworkModuleDescriptor g_module_l1 =
{
    "l1",
    0.005f,
    Priority_Controller_W_Comp,
    Module_L1_Update,
    Module_L1_Reset,
    &g_l1_data,
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_l1)