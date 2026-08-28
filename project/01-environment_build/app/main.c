#include "board_v1.h"
#include "FreeRTOS.h"
#include "task.h"


/* ========================================================================== */
/*                                main                                         */
/* ========================================================================== */
int main(void)
{
    if (bsp_init() != BSP_OK) Error_Handler();

    vTaskStartScheduler();
    Error_Handler();
    return 0;
}