#ifndef __MTK_GPU_UTILITY_H__
#define __MTK_GPU_UTILITY_H__

#include <linux/types.h>

#define MTK_GPU_DVFS_TYPE_LIST { \
    MTK_GPU_DVFS_TYPE_ITEM(NONE) \
    MTK_GPU_DVFS_TYPE_ITEM(SMARTBOOST) \
    MTK_GPU_DVFS_TYPE_ITEM(VSYNCBASED) \
    MTK_GPU_DVFS_TYPE_ITEM(FALLBACK) \
    MTK_GPU_DVFS_TYPE_ITEM(TIMERBASED) \
    MTK_GPU_DVFS_TYPE_ITEM(FASTDVFS) \
    MTK_GPU_DVFS_TYPE_ITEM(TOUCHBOOST) \
    MTK_GPU_DVFS_TYPE_ITEM(THERMAL) \
    MTK_GPU_DVFS_TYPE_ITEM(CUSTOMIZATION)}

typedef enum MTK_GPU_DVFS_TYPE_TAG
#define MTK_GPU_DVFS_TYPE_ITEM(type) MTK_GPU_DVFS_TYPE_##type,
MTK_GPU_DVFS_TYPE_LIST
#undef MTK_GPU_DVFS_TYPE_ITEM
MTK_GPU_DVFS_TYPE;

#ifdef __cplusplus
extern "C"
{
#endif

/* returning false indicated no implement */

/* unit: x bytes */
bool mtk_get_gpu_memory_usage(unsigned int *pMemUsage);
bool mtk_get_gpu_page_cache(unsigned int *pPageCache);

/* unit: 0~100 % */
bool mtk_get_gpu_loading(unsigned int *pLoading);
bool mtk_get_gpu_block(unsigned int *pBlock);
bool mtk_get_gpu_idle(unsigned int *pIlde);


bool mtk_get_gpu_GP_loading(unsigned int *pLoading);
bool mtk_get_gpu_PP_loading(unsigned int *pLoading);
bool mtk_get_gpu_power_loading(unsigned int *pLoading);

bool mtk_enable_gpu_dvfs_timer(bool bEnable);
bool mtk_boost_gpu_freq(void);
bool mtk_set_bottom_gpu_freq(unsigned int ui32FreqLevel);

/* ui32FreqLevel: 0=>lowest freq, count-1=>highest freq */
bool mtk_custom_get_gpu_freq_level_count(unsigned int *pui32FreqLevelCount);
bool mtk_custom_boost_gpu_freq(unsigned int ui32FreqLevel);
bool mtk_custom_upbound_gpu_freq(unsigned int ui32FreqLevel);
bool mtk_get_custom_boost_gpu_freq(unsigned int *pui32FreqLevel);
bool mtk_get_custom_upbound_gpu_freq(unsigned int *pui32FreqLevel);

bool mtk_dump_gpu_memory_usage(void);
#ifdef __cplusplus
}
#endif

#endif
