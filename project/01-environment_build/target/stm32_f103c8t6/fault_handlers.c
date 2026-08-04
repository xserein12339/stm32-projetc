#include "stm32f1xx.h" 
#include <stdio.h>   

/**
 * @brief 解析并打印 CFSR/HFSR 故障原因
 */
static void Print_FaultReason(void)
{
    uint32_t cfsr = SCB->CFSR;
    uint32_t hfsr = SCB->HFSR;

    /* MemManage Fault */
    if (cfsr & SCB_CFSR_MMARVALID_Msk)
        printf("[MM] Address: 0x%08lX\n", SCB->MMFAR);
    if (cfsr & SCB_CFSR_DACCVIOL_Msk)  printf("[MM] Data access violation\n");
    if (cfsr & SCB_CFSR_IACCVIOL_Msk)  printf("[MM] Instr access violation\n");
    if (cfsr & SCB_CFSR_MSTKERR_Msk)   printf("[MM] Stack push error\n");
    if (cfsr & SCB_CFSR_MUNSTKERR_Msk) printf("[MM] Stack pop error\n");

    /* Bus Fault */
    if (cfsr & SCB_CFSR_BFARVALID_Msk)
        printf("[Bus] Address: 0x%08lX\n", SCB->BFAR);
    if (cfsr & SCB_CFSR_PRECISERR_Msk)   printf("[Bus] Precise data bus error\n");
    if (cfsr & SCB_CFSR_IMPRECISERR_Msk) printf("[Bus] Imprecise data bus error\n");
    if (cfsr & SCB_CFSR_STKERR_Msk)      printf("[Bus] Stack push bus error\n");
    if (cfsr & SCB_CFSR_UNSTKERR_Msk)    printf("[Bus] Stack pop bus error\n");
    if (cfsr & SCB_CFSR_IBUSERR_Msk)     printf("[Bus] Instruction bus error\n");

    /* Usage Fault */
    if (cfsr & SCB_CFSR_DIVBYZERO_Msk)  printf("[Usage] Divide by zero\n");
    if (cfsr & SCB_CFSR_UNALIGNED_Msk)  printf("[Usage] Unaligned access\n");
    if (cfsr & SCB_CFSR_NOCP_Msk)       printf("[Usage] No coprocessor\n");
    if (cfsr & SCB_CFSR_INVPC_Msk)      printf("[Usage] Invalid PC load\n");
    if (cfsr & SCB_CFSR_INVSTATE_Msk)   printf("[Usage] Invalid EPSR state\n");
    if (cfsr & SCB_CFSR_UNDEFINSTR_Msk) printf("[Usage] Undefined instruction\n");

    /* Hard Fault Status */
    if (hfsr & SCB_HFSR_FORCED_Msk)     printf("[Hard] Escalated from configurable fault\n");
    if (hfsr & SCB_HFSR_VECTTBL_Msk)    printf("[Hard] Vector table read error\n");
}

/**
 * @brief 故障处理核心逻辑（由汇编包装调用）
 * @param stack_frame  异常自动压栈的 R0,R1,R2,R3,R12,LR,PC,xPSR
 * @param exc_return   LR 中的 EXC_RETURN 值
 */
void HardFault_Process(uint32_t *stack_frame, uint32_t exc_return)
{
    printf("\n===== HARD FAULT =====\n");

    /* 打印故障原因 */
    Print_FaultReason();

    /* 打印核心寄存器快照*/
    printf("R0  = 0x%08lX | R1  = 0x%08lX\n", stack_frame[0], stack_frame[1]);
    printf("R2  = 0x%08lX | R3  = 0x%08lX\n", stack_frame[2], stack_frame[3]);
    printf("R12 = 0x%08lX | LR  = 0x%08lX\n", stack_frame[4], stack_frame[5]);
    printf("PC  = 0x%08lX | xPSR= 0x%08lX\n", stack_frame[6], stack_frame[7]);
    printf("MSP = 0x%08lX | PSP = 0x%08lX\n", __get_MSP(), __get_PSP());
    printf("EXC_RETURN = 0x%08lX\n", exc_return);

    printf("======================\n");

    /* 写1清零 CFSR/HFSR，防止复位后残留标志干扰下次诊断 */
    SCB->CFSR = SCB->CFSR;
    SCB->HFSR = SCB->HFSR;

    /* 触发硬件断点；无调试器则死循环等待看门狗 */
    __asm volatile("bkpt #0");
    while (1) {}
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