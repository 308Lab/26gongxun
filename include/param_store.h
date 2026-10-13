/**
 * @file    param_store.h
 * @brief   参数持久化：把标定结果存进主 Flash 末尾页（STM32F1 没有片内 EEPROM）
 *
 * 设计要点：
 *   · **A/B 双页轮换**（0x0807F000 / 0x0807F800，各 2KB）。每次保存写【较旧】的那一页 ——
 *     擦除瞬间掉电最多丢掉这一次更新，旧记录完好。单页方案在擦除时掉电会丢掉全部标定，
 *     而重标一次要上机摆臂，代价不小。
 *   · 记录带 magic + version + seq + CRC32。校验不过 = 无有效记录 → 上层退回出厂值。
 *     这是**安全退化**：本模块或数据坏掉时，系统行为与"没启用标定"完全一致，机械臂不会乱动。
 *   · 擦一页在 F1 上要 20~40ms，期间 CPU 从 Flash 取指会停摆 —— 详见 .c 里的说明与约束。
 *
 * ⚠️ 本模块只管"存取字节"，不理解零位的语义（那是 arm_zero.c 的事）。
 */

#ifndef __PARAM_STORE_H
#define __PARAM_STORE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 记录 = 20 字节，逐半字编程。字段顺序刻意排成**无 padding**（见 .c 的静态断言）。
 * 布局：magic(2) version(2) seq(4) zero_pwm(6) calibrated(1) reserved(1) crc(4) */
typedef struct {
    uint16_t magic;        /* 'ZX' = 0x5A58 */
    uint16_t version;      /* 结构版本；改布局时必须 +1 */
    uint32_t seq;          /* 单调递增；A/B 两页之间取大者 */
    uint16_t zero_pwm[3];  /* 记录的零位原始 PWM（ID 1/2/3） */
    uint8_t  calibrated;   /* 1=有效标定；0=未标定 */
    uint8_t  reserved;     /* 对齐用，恒 0 */
    uint32_t crc;          /* 对 crc 之前的全部字节做 CRC32 */
} param_blob_t;

#define PARAM_MAGIC     0x5A58u
#define PARAM_VERSION   1u

/**
 * @brief 取一条有效记录；A/B 都有效时取 seq 较大者
 * @param out 解析结果（无有效记录时被清零）
 * @retval 1=取到有效记录；0=无有效记录
 */
int ParamStore_Load(param_blob_t *out);

/**
 * @brief 存一条记录：自动挑较旧的那页、递增 seq、填好 magic/version/crc（reserved 恒 0）
 * @param in 只提供数据字段（zero_pwm / calibrated）
 * @retval 0=成功；<0=失败（擦除或编程出错）
 */
int ParamStore_Save(const param_blob_t *in);

#ifdef __cplusplus
}
#endif

#endif /* __PARAM_STORE_H */
