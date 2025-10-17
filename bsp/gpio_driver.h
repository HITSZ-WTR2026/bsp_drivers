/**
 * @file    gpio_driver.h
 * @author  syhanjin
 * @date    2025-09-10
 * @brief   gpio driver
 */
#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H
#include "main.h"

#define USE_EXTI

typedef struct
{
    GPIO_TypeDef* port;
    uint16_t pin;
} GPIO_t;

#ifdef USE_EXTI
#include "cmsis_compiler.h"
typedef void (*EXTI_Callback)(GPIO_t* gpio, uint32_t counter, void* data);
static struct
{
    GPIO_t* gpio;           ///< 被注册的 GPIO
    uint32_t counter;       ///< 触发次数
    void* data;             ///< 回调对应的数据
    EXTI_Callback callback; ///< 回调
} EXTI_CallbackMap[16];
#endif

static inline GPIO_PinState GPIO_ReadPin(GPIO_t* hgpio) { return HAL_GPIO_ReadPin(hgpio->port, hgpio->pin); }

static inline void GPIO_WritePin(GPIO_t* hgpio, const GPIO_PinState PinState)
{
    HAL_GPIO_WritePin(hgpio->port, hgpio->pin, PinState);
}

static inline void GPIO_SetPin(GPIO_t* hgpio) { GPIO_WritePin(hgpio, GPIO_PIN_SET); }

static inline void GPIO_ResetPin(GPIO_t* hgpio) { GPIO_WritePin(hgpio, GPIO_PIN_RESET); }

static inline void GPIO_TogglePin(GPIO_t* hgpio) { HAL_GPIO_TogglePin(hgpio->port, hgpio->pin); }

#ifdef USE_EXTI
static inline size_t gpio_pin_to_index(uint16_t pin)
{
#if defined(__GNUC__)
    return __builtin_ctz(pin);
#else
    // 通用 CMSIS 实现
    return __CLZ(__RBIT(pin));
#endif
}

/**
 * 清零 EXTI 触发计数器
 * @param gpio gpio
 */
static inline void GPIO_EXTI_ResetCounter(const GPIO_t* gpio)
{
    const size_t index              = gpio_pin_to_index(gpio->pin);
    EXTI_CallbackMap[index].counter = 0;
}

/**
 * 注册 EXTI 回调
 * @param gpio gpio
 * @param callback 回调函数
 */
static inline void GPIO_EXTI_RegisterCallback(GPIO_t* gpio, const EXTI_Callback callback)
{
    const size_t index               = gpio_pin_to_index(gpio->pin);
    EXTI_CallbackMap[index].gpio     = gpio;
    EXTI_CallbackMap[index].callback = callback;
    EXTI_CallbackMap[index].counter  = 0;
}

/**
 * 取消注册 EXTI 回调
 * @param gpio gpio
 */
static inline void GPIO_EXTI_UnregisterCallback(GPIO_t* gpio)
{
    const size_t index               = gpio_pin_to_index(gpio->pin);
    EXTI_CallbackMap[index].gpio     = NULL;
    EXTI_CallbackMap[index].callback = NULL;
    EXTI_CallbackMap[index].counter  = 0;
}

/**
 * EXTI 回调处理函数
 *
 * @note 在 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin); 中调用本函数
 *       因为 EXTI 的回调无法被注册
 * @param GPIO_Pin
 */
static inline void GPIO_EXTI_Callback(const uint16_t GPIO_Pin)
{
    const size_t index = gpio_pin_to_index(GPIO_Pin);
    if (EXTI_CallbackMap[index].gpio != NULL && EXTI_CallbackMap[index].callback != NULL)
    {
        EXTI_CallbackMap[index].counter++;
        EXTI_CallbackMap[index].callback(EXTI_CallbackMap[index].gpio, EXTI_CallbackMap[index].counter,
                                         EXTI_CallbackMap[index].data);
    }
}
#endif

#endif // GPIO_DRIVER_H
