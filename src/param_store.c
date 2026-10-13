/**
 * @file    param_store.c
 * @brief   参数持久化实现（设计说明见 param_store.h）
 */

#include "param_store.h"

#include <stddef.h>
#include <string.h>

#include "main.h"                        /* HAL + FLASH 寄存器定义 */
#include "stm32f1xx_hal_flash_ex.h"      /* HAL_FLASHEx_Erase */

/* 主 Flash 末尾两页。
 *   STM32F103VET6 = 512KB，高密度产品（≥256KB）页大小 2KB。
 *   固件当前约 248KB（结束于 0x0803E16C），距这里还有约 250KB 余量 —— 见 .ld 的说明。 */
#define PARAM_PAGE_A   0x0807F000u
#define PARAM_PAGE_B   0x0807F800u

/* 布局自检：必须正好 20 字节、无 padding，否则逐半字编程会错位 */
_Static_assert(sizeof(param_blob_t) == 20u, "param_blob_t 必须正好 20 字节（逐半字编程依赖这个）");

/* CRC 覆盖到 crc 字段之前为止 */
#define PARAM_CRC_LEN  ((uint32_t)offsetof(param_blob_t, crc))

/* ---- CRC32（IEEE 802.3 / zlib 同多项式 0xEDB88320）----
 * 用位运算版而不是 1KB 查表版：只在人工标定时算一次，不值得为它占表空间。 */
static uint32_t crc32_of(const uint8_t *p, uint32_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;

    while (n > 0u)
    {
        crc ^= (uint32_t)(*p++);
        for (i = 0u; i < 8u; i++)
        {
            crc = (crc >> 1) ^ (((crc & 1u) != 0u) ? 0xEDB88320u : 0u);
        }
        n--;
    }
    return ~crc;
}

/* 从 Flash 逐半字读到 RAM。
 * 不直接 memcpy：这里要的就是明确的半字访问，行为可预测、将来换型号也好改。 */
static void flash_read(uint32_t addr, param_blob_t *out)
{
    const uint16_t *src = (const uint16_t *)addr;
    uint16_t       *dst = (uint16_t *)out;
    uint32_t        i;

    for (i = 0u; i < ((uint32_t)sizeof(param_blob_t) / 2u); i++)
    {
        dst[i] = src[i];
    }
}

/* 擦一页 + 逐半字写入。返回 0=成功 / -1=擦除失败 / -2=编程失败。
 *
 * ★ F1 的 Flash 擦写在 20~40ms 内会 stall CPU 取指 —— 期间所有驻留 Flash 的中断
 *   （FreeRTOS 时基、串口接收）都被推迟，HAL_GetTick 会跳变，串口可能丢字节。
 *   本工程的用法是人工按键触发、车静止、任务没在跑，所以可接受；
 *   **不要在任务运行中调用它**。 */
static int flash_write(uint32_t addr, const param_blob_t *rec)
{
    FLASH_EraseInitTypeDef er;
    uint32_t               page_err = 0u;
    const uint16_t        *src      = (const uint16_t *)rec;
    uint32_t               i;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

    er.TypeErase   = FLASH_TYPEERASE_PAGES;
    er.PageAddress = addr;
    er.NbPages     = 1u;
    if (HAL_FLASHEx_Erase(&er, &page_err) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return -1;
    }

    for (i = 0u; i < ((uint32_t)sizeof(param_blob_t) / 2u); i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i * 2u, src[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return -2;
        }
    }

    HAL_FLASH_Lock();
    return 0;
}

/* magic + version + CRC 三重校验；任一不过就是无效记录 */
static uint8_t blob_valid(const param_blob_t *r)
{
    if (r->magic   != PARAM_MAGIC)   return 0u;
    if (r->version != PARAM_VERSION) return 0u;
    if (crc32_of((const uint8_t *)r, PARAM_CRC_LEN) != r->crc) return 0u;
    return 1u;
}

int ParamStore_Load(param_blob_t *out)
{
    param_blob_t a;
    param_blob_t b;
    uint8_t      va;
    uint8_t      vb;

    flash_read(PARAM_PAGE_A, &a);
    flash_read(PARAM_PAGE_B, &b);

    va = blob_valid(&a);
    vb = blob_valid(&b);

    if ((va != 0u) && (vb != 0u))
    {
        *out = (a.seq >= b.seq) ? a : b;   /* 两页都有效：取更新的 */
    }
    else if (va != 0u)
    {
        *out = a;
    }
    else if (vb != 0u)
    {
        *out = b;
    }
    else
    {
        (void)memset(out, 0, sizeof(*out));
        return 0;
    }
    return 1;
}

int ParamStore_Save(const param_blob_t *in)
{
    param_blob_t a;
    param_blob_t b;
    param_blob_t rec;
    uint32_t     target;
    uint32_t     max_seq;
    uint8_t      va;
    uint8_t      vb;

    flash_read(PARAM_PAGE_A, &a);
    flash_read(PARAM_PAGE_B, &b);
    va = blob_valid(&a);
    vb = blob_valid(&b);

    /* 选要写的那一页：优先写【无效】的；两页都有效时写 seq 较小的（较旧）——
     * 于是"较新"的那页在整次擦写期间始终完好，掉电不丢已有标定。 */
    if (va == 0u)
    {
        target = PARAM_PAGE_A;
    }
    else if (vb == 0u)
    {
        target = PARAM_PAGE_B;
    }
    else
    {
        target = (a.seq <= b.seq) ? PARAM_PAGE_A : PARAM_PAGE_B;
    }

    max_seq = 0u;
    if (va != 0u)                        max_seq = a.seq;
    if ((vb != 0u) && (b.seq > max_seq)) max_seq = b.seq;

    rec          = *in;
    rec.magic    = PARAM_MAGIC;
    rec.version  = PARAM_VERSION;
    rec.seq      = max_seq + 1u;
    rec.reserved = 0u;
    rec.crc      = crc32_of((const uint8_t *)&rec, PARAM_CRC_LEN);

    return flash_write(target, &rec);
}
