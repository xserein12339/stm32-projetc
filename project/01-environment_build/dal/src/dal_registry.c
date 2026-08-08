/**
 * @file dal_registry.c
 * @brief 通用设备注册表实现
 * @note  配合 dal_registry.h 使用，提供静态注册表管理。
 * 
 * @author xserein
 * @version v1.0
 */
#include "dal_registry.h"
#include "dal_err.h"
#include <string.h>

/* ==========================================
 * API 实现
 * ========================================== */

dal_err_t dal_registry_init(dal_registry_t *reg, dal_reg_entry_t *buf, uint16_t cap)
{
    if (reg == NULL || buf == NULL || cap == 0) {
        return DAL_ERR_PARAM_INVALID;
    }
    reg->entries = buf;
    reg->capacity = cap;
    reg->count = 0;

    for (uint16_t i = 0; i < cap; i++) {
        buf[i].name = NULL;
        buf[i].ops = NULL;
        buf[i].ctx = NULL;
    }
    return DAL_OK;
}

dal_err_t dal_registry_register(dal_registry_t *reg, const char *name,
                                const void *ops, void *ctx)
{
    if (reg == NULL || name == NULL || name[0] == '\0') {
        return DAL_ERR_PARAM_INVALID;
    }
    if (reg->entries == NULL) {
        return DAL_ERR_NOT_READY;   ///< 未初始化
    }

    /* 查重（线性扫描） */
    for (uint16_t i = 0; i < reg->capacity; i++) {
        if (reg->entries[i].name != NULL &&
            strcmp(reg->entries[i].name, name) == 0) {
            return DAL_ERR_DUPLICATE;
        }
    }

    /* 找首个空闲槽位 */
    for (uint16_t i = 0; i < reg->capacity; i++) {
        if (reg->entries[i].name == NULL) {
            reg->entries[i].name = name;
            reg->entries[i].ops = ops;
            reg->entries[i].ctx = ctx;
            reg->count++;
            return DAL_OK;
        }
    }
    return DAL_ERR_FULL;
}

dal_err_t dal_registry_unregister(dal_registry_t *reg, const char *name)
{
    if (reg == NULL || name == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (reg->entries == NULL || reg->count == 0) {
        return DAL_ERR_NOT_FOUND;
    }

    for (uint16_t i = 0; i < reg->capacity; i++) {
        if (reg->entries[i].name != NULL &&
            strcmp(reg->entries[i].name, name) == 0) {
            /* 标记删除，不移动元素 */
            reg->entries[i].name = NULL;
            reg->entries[i].ops = NULL;
            reg->entries[i].ctx = NULL;
            reg->count--;
            return DAL_OK;
        }
    }
    return DAL_ERR_NOT_FOUND;
}

dal_err_t dal_registry_find(const dal_registry_t *reg, const char *name,
                            const void **out_ops, void **out_ctx)
{
    if (reg == NULL || name == NULL || name[0] == '\0') {
        return DAL_ERR_PARAM_INVALID;
    }
    if (reg->entries == NULL || reg->count == 0) {
        return DAL_ERR_NOT_FOUND;
    }

    for (uint16_t i = 0; i < reg->capacity; i++) {
        const dal_reg_entry_t *e = &reg->entries[i];
        if (e->name != NULL && strcmp(e->name, name) == 0) {
            if (out_ops) *out_ops = e->ops;
            if (out_ctx) *out_ctx = e->ctx;
            return DAL_OK;
        }
    }
    return DAL_ERR_NOT_FOUND;
}

uint16_t dal_registry_count(const dal_registry_t *reg)
{
    return (reg != NULL) ? reg->count : 0;
}

dal_err_t dal_registry_get_entry(const dal_registry_t *reg, uint16_t index,
                                 const char **out_name,
                                 const void **out_ops,
                                 void **out_ctx)
{
    if (reg == NULL || index >= reg->capacity) {
        return DAL_ERR_PARAM_INVALID;
    }
    const dal_reg_entry_t *e = &reg->entries[index];
    if (out_name) *out_name = e->name;
    if (out_ops)  *out_ops  = e->ops;
    if (out_ctx)  *out_ctx  = e->ctx;
    return DAL_OK;
}

void dal_registry_foreach(const dal_registry_t *reg,
                          bool (*cb)(const char *, const void *, void *, void *),
                          void *ud)
{
    if (reg == NULL || cb == NULL || reg->entries == NULL) {
        return;
    }

    for (uint16_t i = 0; i < reg->capacity; i++) {
        const dal_reg_entry_t *e = &reg->entries[i];
        if (e->name == NULL) {
            continue;   ///< 空闲槽位跳过 
        }
        if (!cb(e->name, e->ops, e->ctx, ud)) {
            break;      ///< 回调要求提前终止
        }
    }
}