#include "core_framework.h"
#include <string.h>

#define FRAMEWORK_CPU_CLOCK_HZ 72000000UL

typedef struct
{
    FrameworkModuleDescriptor descriptor;
    float last_run_s;
    float next_run_s;
    u8 enabled;
} FrameworkModuleState;

static FrameworkModuleState g_modules[FRAMEWORK_MAX_MODULES];
static u8 g_module_count = 0;

static FrameworkDataDescriptor g_data_items[FRAMEWORK_MAX_DATA_ITEMS];
static u8 g_data_count = 0;

static volatile float g_framework_time_s = 0.0f;
static u8 g_framework_started = 0;
static volatile FrameworkRunOnceTimingStats g_run_once_timing_stats = {0};
static volatile unsigned long long g_run_once_total_ticks = 0;

static u32 Framework_TimingReadTicks(void)
{
    return SysTick->VAL;
}

static u32 Framework_TimingTicksToUs(u32 ticks)
{
    u32 systick_clk_hz;

    if ((SysTick->CTRL & SysTick_CTRL_CLKSOURCE_Msk) != 0)
    {
        systick_clk_hz = FRAMEWORK_CPU_CLOCK_HZ;
    }
    else
    {
        systick_clk_hz = FRAMEWORK_CPU_CLOCK_HZ / 8U;
    }

    if (systick_clk_hz == 0U)
    {
        return 0U;
    }

    return (u32)(((unsigned long long)ticks * 1000000ULL) / (unsigned long long)systick_clk_hz);
}

static u32 Framework_TimingElapsedTicks(u32 start_ticks, u32 end_ticks)
{
    u32 reload_ticks = SysTick->LOAD + 1U;

    if (start_ticks >= end_ticks)
    {
        return start_ticks - end_ticks;
    }

    return start_ticks + (reload_ticks - end_ticks);
}

static void Framework_TimingUpdate(u32 elapsed_ticks)
{
    g_run_once_timing_stats.last_cycles = elapsed_ticks;
    g_run_once_timing_stats.last_us = Framework_TimingTicksToUs(elapsed_ticks);

    if (elapsed_ticks > g_run_once_timing_stats.max_cycles)
    {
        g_run_once_timing_stats.max_cycles = elapsed_ticks;
        g_run_once_timing_stats.max_us = g_run_once_timing_stats.last_us;
    }

    ++g_run_once_timing_stats.sample_count;
    g_run_once_total_ticks += elapsed_ticks;
    g_run_once_timing_stats.avg_us = Framework_TimingTicksToUs(
        (u32)(g_run_once_total_ticks / g_run_once_timing_stats.sample_count));
}

static int Framework_StrEq(const char *a, const char *b)
{
    if ((a == 0) || (b == 0))
    {
        return 0;
    }
    return strcmp(a, b) == 0;
}

static void Framework_SortModulesByPriority(void)
{
    u8 i;
    for (i = 0; i + 1 < g_module_count; ++i)
    {
        u8 j;
        for (j = (u8)(i + 1); j < g_module_count; ++j)
        {
            if (g_modules[j].descriptor.priority < g_modules[i].descriptor.priority)
            {
                FrameworkModuleState tmp = g_modules[i];
                g_modules[i] = g_modules[j];
                g_modules[j] = tmp;
            }
        }
    }
}

void Framework_Init(void)
{
    Framework_SortModulesByPriority();
    Framework_Reset();
}

void Framework_Reset(void)
{
    u8 i;
    float now = Framework_GetTimeS();

    for (i = 0; i < g_data_count; ++i)
    {
        if (g_data_items[i].reset != 0)
        {
            g_data_items[i].reset(g_data_items[i].ptr, g_data_items[i].size);
        }
    }

    for (i = 0; i < g_module_count; ++i)
    {
        g_modules[i].last_run_s = now;
        g_modules[i].next_run_s = now;
        if (g_modules[i].descriptor.reset != 0)
        {
            g_modules[i].descriptor.reset(g_modules[i].descriptor.user_ctx);
        }
    }

    g_framework_started = 1;
}

void Framework_RunOnce(void)
{
    u8 i;
    float now;
    u32 start_ticks;

    if (g_framework_started == 0)
    {
        return;
    }

    start_ticks = Framework_TimingReadTicks();
    now = Framework_GetTimeS();

    for (i = 0; i < g_module_count; ++i)
    {
        FrameworkModuleState *state = &g_modules[i];
        if ((state->enabled == 0) || (state->descriptor.period_s <= 0.0f))
        {
            continue;
        }

        if (now >= state->next_run_s - 0.0001f)
        {
            float dt = now - state->last_run_s;
            state->last_run_s = now;

            do
            {
                state->next_run_s += state->descriptor.period_s;
            } while (state->next_run_s <= now);

            if (state->descriptor.update != 0)
            {
                state->descriptor.update(dt, state->descriptor.user_ctx);
            }
        }
    }

    Framework_TimingUpdate(Framework_TimingElapsedTicks(start_ticks, Framework_TimingReadTicks()));
}

void Framework_TickFromISR(float delta_s)
{
    g_framework_time_s += delta_s;
}

float Framework_GetTimeS(void)
{
    return g_framework_time_s;
}

void Framework_GetRunOnceTimingStats(FrameworkRunOnceTimingStats *stats_out)
{
    if (stats_out == 0)
    {
        return;
    }
    *stats_out = g_run_once_timing_stats;
}

void Framework_ResetRunOnceTimingStats(void)
{
    memset((void *)&g_run_once_timing_stats, 0, sizeof(g_run_once_timing_stats));
    g_run_once_total_ticks = 0;
}

int Framework_RegisterModule(const FrameworkModuleDescriptor *descriptor)
{
    u8 i;

    if (descriptor == 0)
    {
        return -1;
    }
    if ((descriptor->name == 0) || (descriptor->period_s <= 0.0f) || (descriptor->update == 0))
    {
        return -2;
    }

    for (i = 0; i < g_module_count; ++i)
    {
        if (Framework_StrEq(g_modules[i].descriptor.name, descriptor->name))
        {
            return -3;
        }
    }

    if (g_module_count >= FRAMEWORK_MAX_MODULES)
    {
        return -4;
    }

    g_modules[g_module_count].descriptor = *descriptor;
    g_modules[g_module_count].last_run_s = 0.0f;
    g_modules[g_module_count].next_run_s = 0.0f;
    g_modules[g_module_count].enabled = 0;
    ++g_module_count;
    return 0;
}

int Framework_ModuleSetEnabled(const char *name, u8 enabled)
{
    u8 i;

    if (name == 0)
    {
        return -1;
    }

    for (i = 0; i < g_module_count; ++i)
    {
        if (Framework_StrEq(g_modules[i].descriptor.name, name))
        {
            u8 normalized_enabled = (enabled != 0) ? 1 : 0;
            if ((g_modules[i].enabled == 0) && (normalized_enabled != 0))
            {
                float now = Framework_GetTimeS();
                g_modules[i].last_run_s = now;
                g_modules[i].next_run_s = now;
            }
            g_modules[i].enabled = normalized_enabled;
            return 0;
        }
    }

    return -2;
}
int Framework_RegisterData(const FrameworkDataDescriptor *descriptor)
{
    u8 i;
    if (descriptor == 0)
    {
        return -1;
    }
    if ((descriptor->name == 0) || (descriptor->ptr == 0) || (descriptor->size == 0))
    {
        return -2;
    }
    if (g_data_count >= FRAMEWORK_MAX_DATA_ITEMS)
    {
        return -3;
    }

    for (i = 0; i < g_data_count; ++i)
    {
        if (Framework_StrEq(g_data_items[i].name, descriptor->name))
        {
            return -4;
        }
    }

    g_data_items[g_data_count] = *descriptor;
    ++g_data_count;
    return 0;
}

void *Framework_DataGet(const char *name, u16 *size_out)
{
    u8 i;
    for (i = 0; i < g_data_count; ++i)
    {
        if (Framework_StrEq(g_data_items[i].name, name))
        {
            if (size_out != 0)
            {
                *size_out = g_data_items[i].size;
            }
            return g_data_items[i].ptr;
        }
    }
    return 0;
}

int Framework_DataWrite(const char *name, const void *src, u16 size)
{
    u16 registered_size = 0;
    void *dst = Framework_DataGet(name, &registered_size);
    if ((dst == 0) || (src == 0))
    {
        return -1;
    }
    if (size > registered_size)
    {
        return -2;
    }
    memcpy(dst, src, size);
    return 0;
}

int Framework_DataRead(const char *name, void *dst, u16 size)
{
    u16 registered_size = 0;
    void *src = Framework_DataGet(name, &registered_size);
    if ((src == 0) || (dst == 0))
    {
        return -1;
    }
    if (size > registered_size)
    {
        return -2;
    }
    memcpy(dst, src, size);
    return 0;
}
