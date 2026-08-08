/**
 * @file dal_registry.h
 * @brief 通用设备/模块注册表框架（类型安全、零冗余版）
 * @note  提供静态内存分配的注册表管理，适用于嵌入式系统。
 *        所有操作时间复杂度为 O(n)，n 为容量（线性扫描），
 *        适用于设备数量较少的场景（通常 < 32）。
 *
 * @warning 本框架不提供内部互斥保护。
 *          - register / unregister 必须在任务上下文调用
 *          - find / foreach 为只读操作，可在中断上下文调用
 *            （前提是调用期间不会有并发的 register / unregister）
 *
 * @author xserein
 * @version v1.0
 */
#ifndef DAL_REGISTRY_H
#define DAL_REGISTRY_H

#include "dal_err.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                               注册表条目结构                                */
/* ========================================================================== */

/**
 * @brief 注册表条目
 * @note  槽位空闲判定：name == NULL。
 *        不存在其他冗余标志，避免状态不一致。
 *        unregister 时仅将 name 置 NULL（标记删除，不压缩数组），
 *        保证已有索引/指针的稳定性。
 */
typedef struct dal_reg_entry {
    const char *name;   ///< 设备名称（唯一标识）；NULL 表示槽位空闲
    const void *ops;    ///< 操作集指针（可为 NULL）
    void       *ctx;    ///< 设备上下文（可为 NULL）
} dal_reg_entry_t;

/* ========================================================================== */
/*                               注册表句柄                                    */
/* ========================================================================== */

/**
 * @brief 注册表句柄
 * @note  由外部静态分配条目数组，框架本身不做动态内存分配。
 */
typedef struct {
    dal_reg_entry_t *entries; ///< 条目数组（外部静态分配）
    uint16_t         capacity;///< 数组最大容量
    uint16_t         count;   ///< 当前有效条目数
} dal_registry_t;

/* ========================================================================== */
/*                                公共 API                                     */
/* ========================================================================== */

/**
 * @brief 初始化注册表
 * @param reg   注册表句柄（不能为 NULL）
 * @param buf   预先分配的 dal_reg_entry_t 数组（不能为 NULL）
 * @param cap   数组容量（必须 > 0）
 * @retval DAL_OK               成功
 * @retval DAL_ERR_PARAM_INVALID 参数无效
 */
dal_err_t dal_registry_init(dal_registry_t *reg, dal_reg_entry_t *buf, uint16_t cap);

/**
 * @brief 注册设备
 * @param reg  注册表句柄
 * @param name 设备名称（必须以 '\0' 结尾，全局唯一；生命周期由调用方保证）
 * @param ops  操作集指针（可为 NULL）
 * @param ctx  设备上下文（可为 NULL）
 * @retval DAL_OK                成功
 * @retval DAL_ERR_PARAM_INVALID name 为 NULL 或空字符串
 * @retval DAL_ERR_FULL          注册表已满
 * @retval DAL_ERR_DUPLICATE     名称已存在
 *
 * @note 注册时会在首个 name == NULL 的空闲槽位写入数据，并递增 count。
 */
dal_err_t dal_registry_register(dal_registry_t *reg, const char *name,
                                const void *ops, void *ctx);

/**
 * @brief 注销设备
 * @param reg  注册表句柄
 * @param name 设备名称
 * @retval DAL_OK                成功
 * @retval DAL_ERR_PARAM_INVALID 参数无效
 * @retval DAL_ERR_NOT_FOUND     未找到
 *
 * @note 采用标记删除策略：仅将对应槽位的 name 置 NULL 并递减 count，
 *       不移动后续元素。因此遍历期间注销不会导致索引漂移，
 *       但遍历可能遇到已注销的槽位（foreach 内部会自动跳过）。
 */
dal_err_t dal_registry_unregister(dal_registry_t *reg, const char *name);

/**
 * @brief 查找设备
 * @param reg      注册表
 * @param name     设备名
 * @param out_ops  [可选] 返回 ops 指针，不需要时传 NULL
 * @param out_ctx  [可选] 返回 ctx 指针，不需要时传 NULL
 * @retval DAL_OK                成功
 * @retval DAL_ERR_NOT_FOUND     未找到
 * @retval DAL_ERR_PARAM_INVALID 参数无效
 *
 * @note 此函数只读，可在中断上下文调用（无并发写时）。
 */
dal_err_t dal_registry_find(const dal_registry_t *reg, const char *name,
                            const void **out_ops, void **out_ctx);

/**
 * @brief 获取当前有效设备数量
 * @param reg 注册表句柄
 * @return 有效条目数（reg 为 NULL 时返回 0）
 */
uint16_t dal_registry_count(const dal_registry_t *reg);

/**
 * @brief 按物理索引获取条目（原始访问）
 * @param reg      注册表
 * @param index    物理索引（0 ~ capacity-1）
 * @param out_name [可选] 返回名称
 * @param out_ops  [可选] 返回 ops
 * @param out_ctx  [可选] 返回 ctx
 * @retval DAL_OK                成功
 * @retval DAL_ERR_PARAM_INVALID 越界或 reg 为 NULL
 *
 * @note 此函数暴露内部数组布局，仅用于需要直接索引的场景。
 *       调用方需自行判断 name != NULL 以确认槽位有效。
 *       一般遍历请优先使用 dal_registry_foreach()。
 */
dal_err_t dal_registry_get_entry(const dal_registry_t *reg, uint16_t index,
                                 const char **out_name,
                                 const void **out_ops,
                                 void **out_ctx);

/**
 * @brief 安全遍历所有有效设备
 * @param reg 注册表
 * @param cb  回调函数，参数为 (name, ops, ctx, user_data)；
 *            返回 true 继续遍历，false 提前终止
 * @param ud  用户上下文
 *
 * @warning 遍历期间禁止对同一注册表调用 register / unregister，
 *          否则可能遍历到不一致状态（如重复访问、遗漏条目）。
 *          若需并发修改，请由调用方在外部加锁。
 */
void dal_registry_foreach(const dal_registry_t *reg,
                          bool (*cb)(const char *name, const void *ops,
                                     void *ctx, void *ud),
                          void *ud);

/* ========================================================================== */
/*                          类型安全辅助（标准 C 可移植）                       */
/* ========================================================================== */

/**
 * @brief 注册设备（自动添加 const void* 转换，消除 -Wcast-qual 等警告）
 * @param reg      注册表句柄
 * @param name     设备名称
 * @param ops_ptr  操作集指针
 * @param ctx_ptr  设备上下文指针
 * @retval 同 dal_registry_register()
 */
#define DAL_REGISTRY_REGISTER(reg, name, ops_ptr, ctx_ptr) \
    dal_registry_register((reg), (name), (const void *)(ops_ptr), (ctx_ptr))

/**
 * @brief 查找设备并返回指定类型的 ops 指针（内部辅助）
 * @param reg      注册表
 * @param name     设备名
 * @param out_ctx  [出参] 上下文指针变量（void * 类型），不需要时传 NULL
 * @return ops 指针（const void *）；未找到返回 NULL
 *
 * @note 此函数为 static inline，无额外调用开销。
 *       调用方收到返回值后需自行强制转换为具体 ops 类型。
 *       一般推荐使用 DAL_REGISTRY_FIND_OPS() 宏以获得类型安全。
 */
static inline const void *
dal_registry_find_ops(const dal_registry_t *reg, const char *name, void **out_ctx)
{
    const void *ops = NULL;
    void *ctx = NULL;
    dal_err_t err = dal_registry_find(reg, name, &ops, &ctx);
    if (out_ctx) {
        *out_ctx = ctx;
    }
    return (err == DAL_OK) ? ops : NULL;
}

/**
 * @brief 查找设备并自动转换 ops 为指定指针类型
 * @param reg       注册表
 * @param dev_name  设备名
 * @param ops_type  期望的 ops 指针类型（如 const my_ops_t *）
 * @param ctx_out   [出参] 上下文指针变量（void * 类型）
 * @return 转换后的 ops 指针；未找到返回 NULL
 *
 * @usage
 *   void *ctx = NULL;
 *   const my_ops_t *ops = DAL_REGISTRY_FIND_OPS(&reg, "key0", const my_ops_t *, &ctx);
 *   if (ops) { ops->start(ctx); }
 */
#define DAL_REGISTRY_FIND_OPS(reg, dev_name, ops_type, ctx_out) \
    ((ops_type)dal_registry_find_ops((reg), (dev_name), (ctx_out)))

#ifdef __cplusplus
}
#endif

#endif /* DAL_REGISTRY_H */