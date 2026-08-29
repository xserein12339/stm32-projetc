/**
 * @file    bsp_dbg.c
 * @brief   调试日志输出通道 BSP 层实现（USART2 / PA2）v2.0
 *
 * 数据通路：CPU -> USART2 TX(PA2, 115200 8N1) -> USB 转串口 -> 上位机
 * 串口调试助手。`_write` 覆盖 newlib nosys 桩，使 printf（含
 * fault_handlers 取证）与 mw_log 输出均重定向到该通道。
 *
 * v1.0 走 ITM/SWO(PB3)，需调试器开 SWV 才可见；v2.0 板改走 USART2，
 * 无调试器依赖，HardFault 取证也可直接在串口助手上观察。
 *
 * WHY 寄存器级轮询而非 bsp_uart 框架：
 *   1. HardFault 上下文（fault_handlers printf）不可用互斥量/信号量，
 *      框架的 HAL_UART_Transmit 亦非异常安全；
 *   2. 日志通道独占 USART2：不经 bsp_uart_open 打开，避免中断/DMA
 *      资源与框架实例(s_instances[1])冲突；
 *   3. TXE 单字节轮询无锁：任务级并发调用仅可能字符交错（调试通道
 *      可接受，正式遥测走 svc_comm）。
 *
 * @note  必须在系统时钟配置完成后调用（波特率依赖 PCLK1=36MHz）。
 *
 * @author  xserein
 * @version v2.0
 */
#include "bsp_dbg.h"
#include "board_v1_config.h"
#include "stm32f1xx_hal.h"
#include <string.h>

/* ========================================================================== */
/*                          USART2 初始化                                       */
/* ========================================================================== */

bsp_err_t bsp_dbg_init(void)
{
    /* --- 1. GPIO：PA2 = USART2_TX 复用推挽（RX PA3 未用，仅 TX 输出日志） --- */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = BSP_LOG_USART_TX_PIN;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BSP_LOG_USART_TX_PORT, &gpio);

    /* --- 2. USART2 外设时钟 + 寄存器配置（115200 8N1，无中断无 DMA） --- */
    __HAL_RCC_USART2_CLK_ENABLE();

    BSP_LOG_USART_PORT->CR1 = 0U;                    /* 先关 UE 再改配置 */
    BSP_LOG_USART_PORT->CR2 = 0U;                    /* 1 停止位 */
    BSP_LOG_USART_PORT->CR3 = 0U;                    /* 无硬件流控 / 无 DMA */

    /* BRR 按实际 PCLK1 计算（USART2 挂 APB1）：mantissa+fraction 舍入 */
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t brr   = (pclk1 + (BSP_LOG_USART_BAUDRATE / 2U))
                     / BSP_LOG_USART_BAUDRATE;
    if (brr == 0U) {
        return BSP_ERR_PARAM;
    }
    BSP_LOG_USART_PORT->BRR = brr;

    /* UE + TE（使能收发器仅 TX；RE 不开，避免浮空 RX 引脚产生错误帧） */
    BSP_LOG_USART_PORT->CR1 = USART_CR1_UE | USART_CR1_TE;

    /* WHY 此处不再打印复位原因：上一现场打印 12 字节（~1.04ms）后 CPU 停死，
     * 且无 HardFault 输出。首条输出移至 board_v1.c 的 P:dbg（关中断包裹，
     * 二分"异步中断 vs 时钟/总线"两种死因），复位原因延后到 main 打印 */
    return BSP_OK;
}

/**
 * @brief   延后打印复位原因（raw hex，零 libc 依赖）
 * @note    放在系统完全起来后的稳定窗口调用
 */
void bsp_dbg_report_reset_cause(void)
{
    static const char k_hex[] = "0123456789ABCDEF";
    char line[17] = "reset:0x";
    const uint32_t csr = RCC->CSR;

    for (uint32_t i = 0U; i < 8U; i++) {
        line[8U + i] = k_hex[(csr >> (28U - 4U * i)) & 0xFU];
    }
    line[16] = '\n';
    (void)bsp_dbg_write(line, (uint32_t)sizeof(line));
}

/* ========================================================================== */
/*                          输出原语                                            */
/* ========================================================================== */

int32_t bsp_dbg_write(const char *data, uint32_t len)
{
    USART_TypeDef *uart = BSP_LOG_USART_PORT;
    uint32_t written = 0;

    /* 未初始化（UE 未置位）直接丢弃，避免 fault 上下文死等 */
    if ((uart->CR1 & USART_CR1_UE) == 0U) {
        return 0;
    }

    for (uint32_t i = 0; i < len; i++) {
        /* TXE 就绪轮询：115200 下单字节约 87us；HardFault 上下文中
         * USART 硬件始终在跑，不会死等（无超时上限属可接受取舍） */
        while ((uart->SR & USART_SR_TXE) == 0U) {
            /* busy wait */
        }
        uart->DR = (uint8_t)data[i];
        written++;
    }

    /* 等待最后字节移出（TC）：返回后若立刻崩溃/复位，在途字节不丢失 */
    if (written > 0U) {
        while ((uart->SR & USART_SR_TC) == 0U) {
            /* busy wait */
        }
    }

    return (int32_t)written;
}

void bsp_dbg_sink(const char *buf, uint32_t len)
{
    (void)bsp_dbg_write(buf, len);
}

/* ========================================================================== */
/*                          newlib 系统调用重定向                                */
/* ========================================================================== */

/**
 * @brief   newlib `_write` 覆盖（nosys 桩替换）
 * @note    printf/fputs 等输出最终走此函数；fd 非法仍按 -1 返回，
 *          stdout(1)/stderr(2) 统一走 USART2。
 */
int _write(int fd, char *buf, int len)
{
    if (buf == NULL || len <= 0) {
        return -1;
    }
    if (fd != 1 && fd != 2) {
        return -1;
    }

    int32_t written = bsp_dbg_write(buf, (uint32_t)len);
    return (written > 0) ? (int)written : (int)len; /* 半写也按成功计，避免重试 */
}
