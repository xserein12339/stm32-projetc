/**
 * @file    fw_version.h
 * @brief   固件版本定义（SRS NFR-05）
 *
 * 版本号规则（语义化）：MAJOR 不兼容的接口/行为变更；
 * MINOR 向下兼容的功能新增；PATCH 缺陷修复。
 * 发布（烧录出厂固件）时更新；每次调试构建不强制更新。
 *
 * @author  xserein
 * @version 1.0.0
 */

#ifndef __FW_VERSION_H__
#define __FW_VERSION_H__

#ifdef __cplusplus
extern "C" {
#endif

#define FW_VERSION_MAJOR    1U
#define FW_VERSION_MINOR    0U
#define FW_VERSION_PATCH    0U

/** 字符串形式（供日志/协议上报） */
#define FW_VERSION_STRING   "1.0.0"

#ifdef __cplusplus
}
#endif

#endif /* __FW_VERSION_H__ */
