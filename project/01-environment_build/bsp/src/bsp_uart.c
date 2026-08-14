/**
 * @file    bsp_uart.c
 * @brief   BSP UART 多实例驱动实现（阻塞/DMA/非阻塞）v1.4
 * @note    - [v1.4] MspInit/MspDeInit 改为 static + 函数指针注入，避免全局符号冲突
 *          - [v1.4] IDLE 中断停止 DMA 后同步重置 HAL RxState，防止后续 recv_dma 返回 BUSY
 *          - [v1.3] 统一使用 HAL_UART_MspInit 配置 GPIO/时钟，与 bsp_timer 架构一致
 *          - [v1.3] DMA 中断号显式映射，修复 DMA_Channel_TypeDef 无 IRQn 成员的问题
 *          - [v1.2] GPIO 配置缺失时编译报错，杜绝静默失败
 *          - [v1.2] IDLE 回调中自动停止 DMA，防止后续接收状态悬挂
 *          - [v1.1] 修复阻塞模式与异步标志冲突
 *          - [v1.1] abort() 正确计算已传输字节数
 * @author  xserein
 * @version v1.4
 */

#include "board_v1.h"
#include "board_v1_config.h"
#include "bsp_uart.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

/* ========================================================================== */
/*                    编译期硬件完整性校验                                      */
/* ========================================================================== */

#if !defined(BSP_USART3_TX_PORT) || !defined(BSP_USART3_RX_PORT)
  #error "BSP_USART3 GPIO must be defined in board_v1_config.h"
#endif

/* ========================================================================== */
/*                         编译期配置                                          */
/* ========================================================================== */

#define BSP_UART_MAX_INSTANCES          3U

/* ========================================================================== */
/*                         硬件映射表                                          */
/* ========================================================================== */

typedef struct {
    USART_TypeDef       *periph;
    IRQn_Type            irqn;
    DMA_Channel_TypeDef *dma_tx_channel;
    DMA_Channel_TypeDef *dma_rx_channel;
    IRQn_Type            dma_tx_irqn;
    IRQn_Type            dma_rx_irqn;
} bsp_uart_hw_map_t;

static const bsp_uart_hw_map_t s_hw_map[BSP_UART_MAX_INSTANCES] = {
    {
        .periph         = USART1,
        .irqn           = USART1_IRQn,
        .dma_tx_channel = DMA1_Channel4,
        .dma_rx_channel = DMA1_Channel5,
        .dma_tx_irqn    = DMA1_Channel4_IRQn,
        .dma_rx_irqn    = DMA1_Channel5_IRQn,
    },
    {
        .periph         = USART2,
        .irqn           = USART2_IRQn,
        .dma_tx_channel = DMA1_Channel7,
        .dma_rx_channel = DMA1_Channel6,
        .dma_tx_irqn    = DMA1_Channel7_IRQn,
        .dma_rx_irqn    = DMA1_Channel6_IRQn,
    },
    {
        .periph         = USART3,
        .irqn           = USART3_IRQn,
        .dma_tx_channel = DMA1_Channel2,
        .dma_rx_channel = DMA1_Channel3,
        .dma_tx_irqn    = DMA1_Channel2_IRQn,
        .dma_rx_irqn    = DMA1_Channel3_IRQn,
    },
};

/* ========================================================================== */
/*                         内部状态结构                                        */
/* ========================================================================== */

typedef struct {
    UART_HandleTypeDef  huart;
    DMA_HandleTypeDef   hdma_tx;
    DMA_HandleTypeDef   hdma_rx;

    bsp_uart_config_t   config;
    bool                in_use;
    bool                is_initialized;

    volatile bool       tx_busy;
    const uint8_t      *tx_buf;
    uint16_t            tx_len;

    volatile bool       rx_busy;
    bool                rx_timeout_enabled;
    uint8_t            *rx_buf;
    uint16_t            rx_len;

    bsp_uart_callback_t callback;
    void               *callback_arg;

    SemaphoreHandle_t   tx_sem;
    SemaphoreHandle_t   rx_sem;
} bsp_uart_inst_t;

static bsp_uart_inst_t s_instances[BSP_UART_MAX_INSTANCES];

/* ========================================================================== */
/*              [v1.4] Static MspInit / MspDeInit（避免全局符号冲突）            */
/* ========================================================================== */

/**
 * @brief [v1.4] 静态 MSP 初始化，通过函数指针注入 huart，不占用全局弱符号
 * @note  HAL_UART_Init 内部检查 huart->MspInitCallback，若非 NULL 则调用之，
 *        否则才回退到全局 HAL_UART_MspInit。此设计确保：
 *        1. 不与 CubeMX 生成的 stm32f1xx_hal_msp.c 冲突
 *        2. 每个 BSP 驱动独立管理自己的底层资源
 *        3. 支持多实例差异化配置
 */
static void _uart_msp_init(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};

    if (huart->Instance == USART1) {
#if defined(BSP_USART1_TX_PORT) && defined(BSP_USART1_RX_PORT)
        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpio.Pin   = BSP_USART1_TX_PIN;
        gpio.Mode  = GPIO_MODE_AF_PP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(BSP_USART1_TX_PORT, &gpio);

        gpio.Pin  = BSP_USART1_RX_PIN;
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(BSP_USART1_RX_PORT, &gpio);

        __HAL_RCC_USART1_CLK_ENABLE();
        HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
#endif
    } else if (huart->Instance == USART2) {
#if defined(BSP_USART2_TX_PORT) && defined(BSP_USART2_RX_PORT)
        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpio.Pin   = BSP_USART2_TX_PIN;
        gpio.Mode  = GPIO_MODE_AF_PP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(BSP_USART2_TX_PORT, &gpio);

        gpio.Pin  = BSP_USART2_RX_PIN;
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(BSP_USART2_RX_PORT, &gpio);

        __HAL_RCC_USART2_CLK_ENABLE();
        HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
#endif
    } else if (huart->Instance == USART3) {
        __HAL_RCC_GPIOB_CLK_ENABLE();

        gpio.Pin   = BSP_USART3_TX_PIN;
        gpio.Mode  = GPIO_MODE_AF_PP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(BSP_USART3_TX_PORT, &gpio);

        gpio.Pin  = BSP_USART3_RX_PIN;
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(BSP_USART3_RX_PORT, &gpio);

        __HAL_RCC_USART3_CLK_ENABLE();
        HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}

/**
 * @brief [v1.4] 静态 MSP 反初始化
 */
static void _uart_msp_deinit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        HAL_NVIC_DisableIRQ(USART1_IRQn);
        __HAL_RCC_USART1_CLK_DISABLE();
    } else if (huart->Instance == USART2) {
        HAL_NVIC_DisableIRQ(USART2_IRQn);
        __HAL_RCC_USART2_CLK_DISABLE();
    } else if (huart->Instance == USART3) {
        HAL_NVIC_DisableIRQ(USART3_IRQn);
        __HAL_RCC_USART3_CLK_DISABLE();
    }
}

/* ========================================================================== */
/*                         内部辅助函数                                        */
/* ========================================================================== */

static inline bsp_uart_inst_t *_huart_to_inst(UART_HandleTypeDef *huart)
{
    if (huart == NULL) return NULL;
    ptrdiff_t idx = (uint8_t *)huart - (uint8_t *)s_instances;
    if (idx < 0 || (size_t)idx >= sizeof(s_instances)) return NULL;
    if ((size_t)idx % sizeof(bsp_uart_inst_t) != 0) return NULL;

    bsp_uart_inst_t *inst = (bsp_uart_inst_t *)((uint8_t *)s_instances + idx);
    return inst->in_use ? inst : NULL;
}

static bool _handle_valid(bsp_uart_handle_t handle)
{
    if (handle == NULL) return false;
    ptrdiff_t idx = (bsp_uart_inst_t *)handle - s_instances;
    return (idx >= 0 && idx < (ptrdiff_t)BSP_UART_MAX_INSTANCES && s_instances[idx].in_use);
}

static bsp_uart_inst_t *_get_inst(bsp_uart_handle_t handle)
{
    return _handle_valid(handle) ? (bsp_uart_inst_t *)handle : NULL;
}

static void _config_dma(bsp_uart_inst_t *inst, uint8_t id)
{
    const bsp_uart_hw_map_t *map = &s_hw_map[id - 1];

    inst->hdma_tx.Instance                 = map->dma_tx_channel;
    inst->hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    inst->hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    inst->hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    inst->hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    inst->hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    inst->hdma_tx.Init.Mode                = DMA_NORMAL;
    inst->hdma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&inst->hdma_tx);
    __HAL_LINKDMA(&inst->huart, hdmatx, inst->hdma_tx);

    inst->hdma_rx.Instance                 = map->dma_rx_channel;
    inst->hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    inst->hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    inst->hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    inst->hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    inst->hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    inst->hdma_rx.Init.Mode                = DMA_NORMAL;
    inst->hdma_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&inst->hdma_rx);
    __HAL_LINKDMA(&inst->huart, hdmarx, inst->hdma_rx);

    HAL_NVIC_SetPriority(map->dma_tx_irqn, 5, 0);
    HAL_NVIC_EnableIRQ(map->dma_tx_irqn);
    HAL_NVIC_SetPriority(map->dma_rx_irqn, 5, 0);
    HAL_NVIC_EnableIRQ(map->dma_rx_irqn);
}

/* ========================================================================== */
/*                   中断与 DMA 回调处理                                       */
/* ========================================================================== */

/**
 * @brief [v1.4] IDLE 中断：停止 DMA + 重置 HAL RxState + 触发回调
 *
 * @par v1.3 遗留问题
 *      __HAL_DMA_DISABLE 仅停止硬件 DMA 通道，HAL 句柄的 RxState 仍为
 *      HAL_UART_STATE_BUSY_RX。下次调用 HAL_UART_Receive_DMA 时，
 *      HAL 内部检查 RxState != READY 直接返回 HAL_BUSY，导致接收永久锁死。
 *
 * @par v1.4 修正
 *      在 ISR 中直接将 RxState 重置为 READY。这是安全的，因为：
 *      1. DMA 已被禁用，不会有新的数据写入
 *      2. 我们已拥有完整的 received 计数
 *      3. 此操作仅为寄存器级赋值，< 1μs
 */
static void _uart_irq_handler(bsp_uart_inst_t *inst)
{
    UART_HandleTypeDef *huart = &inst->huart;

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        if (inst->rx_busy && inst->rx_timeout_enabled) {
            /* Step 1: 停止 DMA 硬件 */
            __HAL_DMA_DISABLE(&inst->hdma_rx);

            /* Step 2: 计算已接收字节数 */
            uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&inst->hdma_rx);
            uint16_t received  = inst->rx_len - remaining;

            /* Step 3: [v1.4] 重置 HAL 状态机，使后续 recv_dma 可正常启动 */
            huart->RxState = HAL_UART_STATE_READY;

            /* Step 4: 更新 BSP 层状态 */
            inst->rx_busy = false;

            /* Step 5: 通知上层 */
            if (inst->callback) {
                bsp_uart_evt_info_t info = {
                    .event      = BSP_UART_EVT_RX_TIMEOUT,
                    .bytes_done = received,
                    .buf        = inst->rx_buf,
                    .err_detail = {0}
                };
                inst->callback((bsp_uart_handle_t)inst, &info, inst->callback_arg);
            }
        }
    }

    HAL_UART_IRQHandler(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    bsp_uart_inst_t *inst = _huart_to_inst(huart);
    if (!inst) return;

    inst->tx_busy = false;
    if (inst->callback) {
        bsp_uart_evt_info_t info = {
            .event      = BSP_UART_EVT_TX_COMPLETE,
            .bytes_done = inst->tx_len,
            .buf        = inst->tx_buf,
            .err_detail = {0}
        };
        inst->callback((bsp_uart_handle_t)inst, &info, inst->callback_arg);
    }
    if (inst->tx_sem) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(inst->tx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    bsp_uart_inst_t *inst = _huart_to_inst(huart);
    if (!inst) return;

    inst->rx_busy = false;
    if (inst->callback) {
        bsp_uart_evt_info_t info = {
            .event      = BSP_UART_EVT_RX_COMPLETE,
            .bytes_done = inst->rx_len,
            .buf        = inst->rx_buf,
            .err_detail = {0}
        };
        inst->callback((bsp_uart_handle_t)inst, &info, inst->callback_arg);
    }
    if (inst->rx_sem) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(inst->rx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    bsp_uart_inst_t *inst = _huart_to_inst(huart);
    if (!inst) return;

    inst->tx_busy = false;
    inst->rx_busy = false;

    if (inst->callback) {
        bsp_uart_err_detail_t err = {0};
        if (huart->ErrorCode & HAL_UART_ERROR_ORE) err.overrun = true;
        if (huart->ErrorCode & HAL_UART_ERROR_FE)  err.framing = true;
        if (huart->ErrorCode & HAL_UART_ERROR_NE)  err.noise   = true;
        if (huart->ErrorCode & HAL_UART_ERROR_PE)  err.parity  = true;

        bsp_uart_evt_info_t info = {
            .event      = BSP_UART_EVT_ERROR,
            .bytes_done = 0,
            .buf        = NULL,
            .err_detail = err
        };
        inst->callback((bsp_uart_handle_t)inst, &info, inst->callback_arg);
    }
}

/* ========================================================================== */
/*                        DMA / UART 中断服务函数                              */
/* ========================================================================== */

void DMA1_Channel4_IRQHandler(void) { HAL_DMA_IRQHandler(&s_instances[0].hdma_tx); }
void DMA1_Channel5_IRQHandler(void) { HAL_DMA_IRQHandler(&s_instances[0].hdma_rx); }
void DMA1_Channel7_IRQHandler(void) { HAL_DMA_IRQHandler(&s_instances[1].hdma_tx); }
void DMA1_Channel6_IRQHandler(void) { HAL_DMA_IRQHandler(&s_instances[1].hdma_rx); }
void DMA1_Channel2_IRQHandler(void) { HAL_DMA_IRQHandler(&s_instances[2].hdma_tx); }
void DMA1_Channel3_IRQHandler(void) { HAL_DMA_IRQHandler(&s_instances[2].hdma_rx); }

void USART1_IRQHandler(void) { if (s_instances[0].in_use) _uart_irq_handler(&s_instances[0]); }
void USART2_IRQHandler(void) { if (s_instances[1].in_use) _uart_irq_handler(&s_instances[1]); }
void USART3_IRQHandler(void) { if (s_instances[2].in_use) _uart_irq_handler(&s_instances[2]); }

/* ========================================================================== */
/*                         公共 API 实现                                       */
/* ========================================================================== */

bsp_err_t bsp_uart_open(uint8_t id, const bsp_uart_config_t *cfg,
                        bsp_uart_handle_t *handle)
{
    if (cfg == NULL || handle == NULL) return BSP_ERR_PARAM;
    if (id < 1 || id > BSP_UART_MAX_INSTANCES) return BSP_ERR_PARAM;

    uint8_t idx = id - 1;
    bsp_uart_inst_t *inst = &s_instances[idx];
    if (inst->in_use) return BSP_ERR_BUSY;

    memset(inst, 0, sizeof(bsp_uart_inst_t));
    inst->in_use = true;
    inst->config = *cfg;

    /* [v1.4] 注入静态 MSP 回调，避免全局符号冲突 */
    inst->huart.Instance          = s_hw_map[idx].periph;
    inst->huart.MspInitCallback   = _uart_msp_init;
    inst->huart.MspDeInitCallback = _uart_msp_deinit;

    inst->huart.Init.BaudRate     = cfg->baudrate;
    inst->huart.Init.StopBits     = (cfg->stop_bits == BSP_UART_STOP_BITS_2) ?
                                    UART_STOPBITS_2 : UART_STOPBITS_1;
    inst->huart.Init.Parity       = (cfg->parity == BSP_UART_PARITY_NONE) ?
                                    UART_PARITY_NONE :
                                    ((cfg->parity == BSP_UART_PARITY_ODD) ?
                                     UART_PARITY_ODD : UART_PARITY_EVEN);
    inst->huart.Init.Mode         = UART_MODE_TX_RX;
    inst->huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    inst->huart.Init.OverSampling = UART_OVERSAMPLING_16;

    switch (cfg->data_bits) {
        case BSP_UART_DATA_BITS_8: inst->huart.Init.WordLength = UART_WORDLENGTH_8B; break;
        case BSP_UART_DATA_BITS_9: inst->huart.Init.WordLength = UART_WORDLENGTH_9B; break;
        default: return BSP_ERR_UNSUPPORT;
    }

    if (HAL_UART_Init(&inst->huart) != HAL_OK) {
        inst->in_use = false;
        return BSP_ERR_IO;
    }

    _config_dma(inst, id);
    __HAL_UART_ENABLE_IT(&inst->huart, UART_IT_IDLE);

    inst->tx_sem = xSemaphoreCreateBinary();
    inst->rx_sem = xSemaphoreCreateBinary();
    if (inst->tx_sem == NULL || inst->rx_sem == NULL) {
        HAL_UART_DeInit(&inst->huart);
        inst->in_use = false;
        return BSP_ERR_NOMEM;
    }

    inst->is_initialized = true;
    *handle = (bsp_uart_handle_t)inst;
    return BSP_OK;
}

bsp_err_t bsp_uart_close(bsp_uart_handle_t handle)
{
    bsp_uart_inst_t *inst = _get_inst(handle);
    if (!inst) return BSP_ERR_PARAM;

    bsp_uart_abort(handle, BSP_UART_DIR_TX | BSP_UART_DIR_RX);
    HAL_UART_DeInit(&inst->huart);

    if (inst->tx_sem) vSemaphoreDelete(inst->tx_sem);
    if (inst->rx_sem) vSemaphoreDelete(inst->rx_sem);

    inst->in_use = false;
    inst->is_initialized = false;
    return BSP_OK;
}

bsp_err_t bsp_uart_send(bsp_uart_handle_t handle,
                        const uint8_t *data, uint16_t len,
                        uint32_t timeout_ms)
{
    bsp_uart_inst_t *inst = _get_inst(handle);
    if (!inst || data == NULL || len == 0) return BSP_ERR_PARAM;
    if (!inst->is_initialized) return BSP_ERR_NOT_INIT;

    HAL_StatusTypeDef status = HAL_UART_Transmit(&inst->huart,
                                                 (uint8_t *)data, len,
                                                 timeout_ms);
    if (status == HAL_OK)      return BSP_OK;
    if (status == HAL_TIMEOUT) return BSP_ERR_TIMEOUT;
    return BSP_ERR_IO;
}

bsp_err_t bsp_uart_recv(bsp_uart_handle_t handle,
                        uint8_t *buf, uint16_t len,
                        uint16_t *recv_len, uint32_t timeout_ms)
{
    bsp_uart_inst_t *inst = _get_inst(handle);
    if (!inst || buf == NULL || len == 0 || recv_len == NULL)
        return BSP_ERR_PARAM;
    if (!inst->is_initialized) return BSP_ERR_NOT_INIT;

    HAL_StatusTypeDef status = HAL_UART_Receive(&inst->huart,
                                                buf, len, timeout_ms);
    if (status == HAL_OK) {
        *recv_len = len;
        return BSP_OK;
    } else if (status == HAL_TIMEOUT) {
        *recv_len = len - inst->huart.RxXferCount;
        return BSP_ERR_TIMEOUT;
    }
    *recv_len = 0;
    return BSP_ERR_IO;
}

bsp_err_t bsp_uart_send_dma(bsp_uart_handle_t handle,
                            const uint8_t *data, uint16_t len)
{
    bsp_uart_inst_t *inst = _get_inst(handle);
    if (!inst || data == NULL || len == 0) return BSP_ERR_PARAM;
    if (!inst->is_initialized) return BSP_ERR_NOT_INIT;
    if (inst->tx_busy) return BSP_ERR_BUSY;

    inst->tx_buf  = data;
    inst->tx_len  = len;
    inst->tx_busy = true;

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&inst->huart,
                                                     (uint8_t *)data, len);
    if (status != HAL_OK) {
        inst->tx_busy = false;
        return BSP_ERR_IO;
    }
    return BSP_OK;
}

bsp_err_t bsp_uart_recv_dma(bsp_uart_handle_t handle,
                            uint8_t *buf, uint16_t len)
{
    bsp_uart_inst_t *inst = _get_inst(handle);
    if (!inst || buf == NULL || len == 0) return BSP_ERR_PARAM;
    if (!inst->is_initialized) return BSP_ERR_NOT_INIT;
    if (inst->rx_busy) return BSP_ERR_BUSY;

    inst->rx_buf             = buf;
    inst->rx_len             = len;
    inst->rx_busy            = true;
    inst->rx_timeout_enabled = (inst->config.rx_timeout_ms > 0);

    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(&inst->huart, buf, len);
    if (status != HAL_OK) {
        inst->rx_busy = false;
        return BSP_ERR_IO;
    }
    return BSP_OK;
}

bsp_err_t bsp_uart_set_callback(bsp_uart_handle_t handle,
                                bsp_uart_callback_t cb,
                                void *user_data)
{
    bsp_uart_inst_t *inst = _get_inst(handle);
    if (!inst) return BSP_ERR_PARAM;
    inst->callback     = cb;
    inst->callback_arg = user_data;
    return BSP_OK;
}

bsp_err_t bsp_uart_is_busy(bsp_uart_handle_t handle,
                           bsp_uart_dir_t dir, bool *busy)
{
    bsp_uart_inst_t *inst = _get_inst(handle);
    if (!inst || busy == NULL) return BSP_ERR_PARAM;
    if (dir & BSP_UART_DIR_TX)      *busy = inst->tx_busy;
    else if (dir & BSP_UART_DIR_RX) *busy = inst->rx_busy;
    else return BSP_ERR_PARAM;
    return BSP_OK;
}

bsp_err_t bsp_uart_abort(bsp_uart_handle_t handle, bsp_uart_dir_t dir)
{
    bsp_uart_inst_t *inst = _get_inst(handle);
    if (!inst || dir == 0) return BSP_ERR_PARAM;

    if (dir & BSP_UART_DIR_TX) {
        uint16_t tx_remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&inst->hdma_tx);
        uint16_t tx_done = inst->tx_len - tx_remaining;

        HAL_UART_AbortTransmit(&inst->huart);
        inst->tx_busy = false;

        if (inst->callback) {
            bsp_uart_evt_info_t info = {
                .event      = BSP_UART_EVT_ABORT,
                .bytes_done = tx_done,
                .buf        = inst->tx_buf,
                .err_detail = {0}
            };
            inst->callback(handle, &info, inst->callback_arg);
        }
    }

    if (dir & BSP_UART_DIR_RX) {
        uint16_t rx_remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&inst->hdma_rx);
        uint16_t rx_done = inst->rx_len - rx_remaining;

        HAL_UART_AbortReceive(&inst->huart);
        inst->rx_busy = false;

        if (inst->callback) {
            bsp_uart_evt_info_t info = {
                .event      = BSP_UART_EVT_ABORT,
                .bytes_done = rx_done,
                .buf        = inst->rx_buf,
                .err_detail = {0}
            };
            inst->callback(handle, &info, inst->callback_arg);
        }
    }

    return BSP_OK;
}

bsp_err_t bsp_uart_get_dma_max_len(bsp_uart_handle_t handle, uint16_t *max_len)
{
    bsp_uart_inst_t *inst = _get_inst(handle);
    if (!inst || max_len == NULL) return BSP_ERR_PARAM;
    *max_len = 0xFFFF;
    return BSP_OK;
}