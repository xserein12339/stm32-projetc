/**
 * @file    fault_handlers.c
 * @brief   HardFault 取证处理器 v2.1
 *
 * v2.1：处理器改用独立的有界 TX 输出（不依赖 bsp_dbg_write 的无限
 *       TXE 轮询）：若原故障本身与 USART/总线相关，处理器的寄存器
 *       访问可能再次 fault -> 嵌套 fault 在 CM3 上是 LOCKUP（CPU 永停，
 *       无任何输出）。有界轮询 + 最小调用链降低该风险，并先点亮
 *       PC13 LED 作为"发生过 fault"的硬件级指示。
 *
 * v2.0：全部输出改为直写 + 手写十六进制，去掉 printf（newlib nosys
 *       下 stdout 全缓冲，printf 取证永不落地）。
 *
 * @note  停机策略：纯 while(1)，IWDG 兜底复位（如已启用）。
 */
#include "stm32f1xx.h"

/* ========================================================================== */
/*                 独立有界 TX 原语（不复用 bsp_dbg_write）                       */
/* ========================================================================== */

#define FAULT_TX_SPIN_MAX   (2000U)   /* 每字节约 28us@72MHz，有界防嵌套死等 */

/**
 * @brief   USART2 直写（硬编码地址级配置假设已由 bsp_dbg_init 完成）
 * @note    单字节 TXE 最多轮询 2000 次后放弃，保证 fault 上下文不死等
 */
static void fw(const char *s, uint32_t len)
{
    USART_TypeDef *u = USART2;

    if ((u->CR1 & USART_CR1_UE) == 0U) {
        return;    /* 通道未初始化（极早期 fault）：放弃输出 */
    }
    for (uint32_t i = 0U; i < len; i++) {
        uint32_t spin = 0U;
        while (((u->SR & USART_SR_TXE) == 0U) && (spin < FAULT_TX_SPIN_MAX)) {
            spin++;
        }
        if (spin >= FAULT_TX_SPIN_MAX) {
            return;    /* 通道异常：立即放弃剩余输出 */
        }
        u->DR = (uint8_t)s[i];
    }
    {
        uint32_t spin = 0U;
        while (((u->SR & USART_SR_TC) == 0U) && (spin < FAULT_TX_SPIN_MAX)) {
            spin++;
        }
    }
}

#define RAW_PUTS(s) fw("" s "", (uint32_t)(sizeof(s) - 1U))

/**
 * @brief   输出 "标签:0xXXXXXXXX" 一行（无 libc）
 */
static void raw_line(const char *label, uint32_t label_len, uint32_t value)
{
    static const char k_hex[] = "0123456789ABCDEF";
    char buf[16];   /* 标签最多 7 字符 + ":0x" + 8 hex + '\n' */

    for (uint32_t i = 0U; i < label_len; i++) {
        buf[i] = label[i];
    }
    buf[label_len]     = ':';
    buf[label_len + 1] = '0';
    buf[label_len + 2] = 'x';
    for (uint32_t j = 0U; j < 8U; j++) {
        buf[label_len + 3U + j] = k_hex[(value >> (28U - 4U * j)) & 0xFU];
    }
    buf[label_len + 11U] = '\n';

    fw(buf, label_len + 12U);
}

/**
 * @brief   PC13 LED 点亮（硬件级"发生过 fault"指示，零依赖）
 * @note    直接寄存器操作：即使串口死了，LED 常亮即证明进了 fault
 */
static void fault_led_on(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH &= ~(0xFU << 20);
    GPIOC->CRH |= (0x2U << 20);
    GPIOC->ODR &= ~(1U << 13);
}

/* ========================================================================== */
/*                          故障解析与打印                                       */
/* ========================================================================== */

/**
 * @brief 解析并打印 CFSR/HFSR 故障原因（按置位逐条输出）
 */
static void Print_FaultReason(void)
{
    uint32_t cfsr = SCB->CFSR;

    /* MemManage Fault */
    if (cfsr & SCB_CFSR_MMARVALID_Msk)   raw_line("MMAR", 4, SCB->MMFAR);
    if (cfsr & SCB_CFSR_DACCVIOL_Msk)    RAW_PUTS("MM:DataAccVio\n");
    if (cfsr & SCB_CFSR_IACCVIOL_Msk)    RAW_PUTS("MM:InstrAccV\n");
    if (cfsr & SCB_CFSR_MSTKERR_Msk)     RAW_PUTS("MM:StkPushErr\n");
    if (cfsr & SCB_CFSR_MUNSTKERR_Msk)   RAW_PUTS("MM:StkPopErr\n");

    /* Bus Fault */
    if (cfsr & SCB_CFSR_BFARVALID_Msk)   raw_line("BFAR", 4, SCB->BFAR);
    if (cfsr & SCB_CFSR_PRECISERR_Msk)   RAW_PUTS("BUS:Precise\n");
    if (cfsr & SCB_CFSR_IMPRECISERR_Msk) RAW_PUTS("BUS:Imprecise\n");
    if (cfsr & SCB_CFSR_STKERR_Msk)      RAW_PUTS("BUS:StkErr\n");
    if (cfsr & SCB_CFSR_UNSTKERR_Msk)    RAW_PUTS("BUS:UnstkErr\n");
    if (cfsr & SCB_CFSR_IBUSERR_Msk)     RAW_PUTS("BUS:IBusErr\n");

    /* Usage Fault */
    if (cfsr & SCB_CFSR_DIVBYZERO_Msk)   RAW_PUTS("USE:Div0\n");
    if (cfsr & SCB_CFSR_UNALIGNED_Msk)   RAW_PUTS("USE:Unalign\n");
    if (cfsr & SCB_CFSR_NOCP_Msk)        RAW_PUTS("USE:NoCP\n");
    if (cfsr & SCB_CFSR_INVPC_Msk)       RAW_PUTS("USE:InvPC\n");
    if (cfsr & SCB_CFSR_INVSTATE_Msk)    RAW_PUTS("USE:InvState\n");
    if (cfsr & SCB_CFSR_UNDEFINSTR_Msk)  RAW_PUTS("USE:UndefInstr\n");

    /* Hard Fault Status */
    if (SCB->HFSR & SCB_HFSR_FORCED_Msk) RAW_PUTS("HFSR:Forced\n");
    if (SCB->HFSR & SCB_HFSR_VECTTBL_Msk) RAW_PUTS("HFSR:VectTbl\n");
}

/**
 * @brief 故障处理核心逻辑（由汇编包装调用）
 * @param stack_frame  异常自动压栈的 R0,R1,R2,R3,R12,LR,PC,xPSR
 * @param exc_return   LR 中的 EXC_RETURN 值
 */
void HardFault_Process(uint32_t *stack_frame, uint32_t exc_return)
{
    fault_led_on();      /* 先点灯：串口死了也能证明发生过 fault */

    RAW_PUTS("\n!!! HARD FAULT !!!\n");
    (void)exc_return;

    /* 故障原因 */
    Print_FaultReason();

    /* 核心寄存器快照 */
    raw_line("R0",  2, stack_frame[0]);
    raw_line("R1",  2, stack_frame[1]);
    raw_line("R2",  2, stack_frame[2]);
    raw_line("R3",  2, stack_frame[3]);
    raw_line("R12", 3, stack_frame[4]);
    raw_line("LR",  2, stack_frame[5]);
    raw_line("PC",  2, stack_frame[6]);
    raw_line("xPSR", 4, stack_frame[7]);
    raw_line("MSP", 3, __get_MSP());
    raw_line("PSP", 3, __get_PSP());
    raw_line("CFSR", 4, SCB->CFSR);
    raw_line("HFSR", 4, SCB->HFSR);

    /* 写1清零 CFSR/HFSR，防止复位后残留标志干扰下次诊断 */
    SCB->CFSR = SCB->CFSR;
    SCB->HFSR = SCB->HFSR;

    /* 纯停机：IWDG 兜底（如已启用）；bkpt 无调试器时成环，弃用 */
    while (1) {
        __NOP();
    }
}

/**
 * @brief HardFault_Handler 汇编包装
 *        根据 EXC_RETURN 自动选择 MSP/PSP，再跳转 C 函数
 */
__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "tst lr, #4        \n"
        "ite eq            \n"
        "mrseq r0, msp     \n"
        "mrsne r0, psp     \n"
        "mov r1, lr        \n"
        "b HardFault_Process \n"
    );
}
