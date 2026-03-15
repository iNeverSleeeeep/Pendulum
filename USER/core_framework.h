#ifndef __CORE_FRAMEWORK_H
#define __CORE_FRAMEWORK_H

#include "sys.h"

#define FRAMEWORK_MAX_MODULES 16
#define FRAMEWORK_MAX_DATA_ITEMS 32

typedef void (*FrameworkModuleUpdateFn)(float dt_s, void *user_ctx);
typedef void (*FrameworkModuleResetFn)(void *user_ctx);
typedef void (*FrameworkDataResetFn)(void *data_ptr, u16 size);

typedef struct
{
    const char *name;
    float period_s;
    u8 priority;
    FrameworkModuleUpdateFn update;
    FrameworkModuleResetFn reset;
    void *user_ctx;
} FrameworkModuleDescriptor;

typedef struct
{
    const char *name;
    void *ptr;
    u16 size;
    FrameworkDataResetFn reset;
} FrameworkDataDescriptor;

typedef struct
{
    u32 last_cycles;
    u32 max_cycles;
    u32 last_us;
    u32 max_us;
    u32 avg_us;
    u32 sample_count;
} FrameworkRunOnceTimingStats;

void Framework_Init(void);
void Framework_Reset(void);
void Framework_RunOnce(void);
void Framework_TickFromISR(float delta_s);
float Framework_GetTimeS(void);
void Framework_GetRunOnceTimingStats(FrameworkRunOnceTimingStats *stats_out);
void Framework_ResetRunOnceTimingStats(void);

int Framework_RegisterModule(const FrameworkModuleDescriptor *descriptor);
int Framework_RegisterData(const FrameworkDataDescriptor *descriptor);
int Framework_ModuleSetEnabled(const char *name, u8 enabled);

void *Framework_DataGet(const char *name, u16 *size_out);
int Framework_DataWrite(const char *name, const void *src, u16 size);
int Framework_DataRead(const char *name, void *dst, u16 size);

#if defined(__GNUC__) || defined(__clang__) || defined(__CC_ARM)
#define FRAMEWORK_CONSTRUCTOR __attribute__((constructor))
#else
#define FRAMEWORK_CONSTRUCTOR
#endif

#define FRAMEWORK_AUTO_REGISTER_MODULE(descriptor_symbol)                    \
    static void FRAMEWORK_CONSTRUCTOR framework_auto_reg_module_##descriptor_symbol(void) \
    {                                                                        \
        (void)Framework_RegisterModule(&(descriptor_symbol));                \
    }

#define FRAMEWORK_AUTO_REGISTER_DATA(descriptor_symbol)                      \
    static void FRAMEWORK_CONSTRUCTOR framework_auto_reg_data_##descriptor_symbol(void) \
    {                                                                        \
        (void)Framework_RegisterData(&(descriptor_symbol));                  \
    }

#endif
