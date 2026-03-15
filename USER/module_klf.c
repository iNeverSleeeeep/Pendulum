#include "core_framework.h"
#include "runtime_state.h"

#define STEP 0.001f

typedef struct {
    float X[2];
    float P[2][2];
    float Q1;
    float Q2;
    float R;
    float dt;
} KalmanFilterPendulum;

void KalmanFilterPendulum_Init(KalmanFilterPendulum *kf, float dt, float Q1, float Q2, float R, float init_angle) {
    kf->X[0] = init_angle;
    kf->X[1] = 0.0f;
    
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;
    
    kf->Q1 = Q1;
    kf->Q2 = Q2;
    kf->R = R;
    kf->dt = dt;
}

void KalmanFilterPendulum_Predict(KalmanFilterPendulum *kf) {
		float P00;
	  float P01;
	  float P10;
	  float P11;
    float x0_pred = kf->X[0] + kf->X[1] * kf->dt;
    float x1_pred = kf->X[1];                      
    kf->X[0] = x0_pred;
    kf->X[1] = x1_pred;
    
    P00 = kf->P[0][0] + kf->P[1][0] * kf->dt;
    P01 = kf->P[0][1] + kf->P[1][1] * kf->dt;
    P10 = kf->P[1][0];
    P11 = kf->P[1][1];
	
    kf->P[0][0] = P00 + P01 * kf->dt;
    kf->P[0][1] = P01;
    kf->P[1][0] = P10 + P11 * kf->dt;
    kf->P[1][1] = P11;
	
    kf->P[0][0] += kf->Q1;
    kf->P[1][1] += kf->Q2;
}

void KalmanFilterPendulum_Update(KalmanFilterPendulum *kf, float z, float *filtered_angle, float *filtered_omega) {
									
    float P00;															 
    float P01;															 
    float P10;															 
    float P11;															 
    float deno = kf->P[0][0] + kf->R;
    float K0 = kf->P[0][0] / deno;
    float K1 = kf->P[1][0] / deno;
    
    float err = z - kf->X[0];
    kf->X[0] += K0 * err;
    kf->X[1] += K1 * err;
    
    P00 = kf->P[0][0] - K0 * kf->P[0][0];
    P01 = kf->P[0][1] - K0 * kf->P[0][1];
    P10 = kf->P[1][0] - K1 * kf->P[0][0];
    P11 = kf->P[1][1] - K1 * kf->P[0][1];
    kf->P[0][0] = P00;
    kf->P[0][1] = P01;
    kf->P[1][0] = P10;
    kf->P[1][1] = P11;
    
    *filtered_angle = kf->X[0];
    *filtered_omega = kf->X[1];
}


// 卡尔曼角度和角速度估计
KalmanFilterPendulum kf_w = {0};
static void Module_Klf_Reset_W(void *user_ctx)
{
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    if (state == 0) {
        return;
    }
    KalmanFilterPendulum_Init(user_ctx, STEP, 0.0001f, 0.002f, 0.0001f, state->x_init[2]);
    (void)user_ctx;
}

static void Module_Klf_Update_W(float dt_s, void *user_ctx)
{
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    if (state == 0) {
        return;
    }
    
	KalmanFilterPendulum_Predict(user_ctx);
	KalmanFilterPendulum_Update(user_ctx, state->x_raw[2], &state->x[2], &state->x[3]);
    (void)dt_s;
    (void)user_ctx;
}

static FrameworkModuleDescriptor g_module_klf_w =
{
    "klf_w",
    STEP,
    Priority_State_Observer,
    Module_Klf_Update_W,
    Module_Klf_Reset_W,
    &kf_w
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_klf_w)

// 卡尔曼位置和速度估计
KalmanFilterPendulum kf_pos = {0};
static void Module_Klf_Reset_Pos(void *user_ctx)
{
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    if (state == 0) {
        return;
    }
    KalmanFilterPendulum_Init(user_ctx, STEP, 0.0001f, 0.002f, 0.0001f, 0);
    (void)user_ctx;
}
static void Module_Klf_Update_Pos(float dt_s, void *user_ctx)
{
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    if (state == 0) {
        return;
    }
    KalmanFilterPendulum_Predict(user_ctx);
    KalmanFilterPendulum_Update(user_ctx, state->x_raw[0] - state->x_init[0], &state->x[0], &state->x[1]);
    (void)dt_s;
    (void)user_ctx;
}

static FrameworkModuleDescriptor g_module_klf_pos =
{
    "klf_pos",
    STEP,
    Priority_State_Observer,
    Module_Klf_Update_Pos,
    Module_Klf_Reset_Pos,
    &kf_pos
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_klf_pos)
