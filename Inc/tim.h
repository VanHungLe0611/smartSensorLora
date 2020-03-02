/**
 * @author      : mehdi (kaf@ad-hoc.com)
 * @file        : tim
 * @created     : Monday Feb 17, 2020 12:02:46 CET
 */

#ifndef TIM_H

#define TIM_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "gpio.h"
#include <stdint.h>
/*!
 * ADC object type definition
 */
typedef struct
{
    Gpio_t TimPIns;
}Tim_t;


void TimInit(Tim_t *  obj);

/**
 * @brief initialize the TIMER internal parameters
 */
void TimConfig(uint32_t frequency);

/**
 * @brief starts timer
 */
void TimStart(void);
/**
 * @brief stops timer
 */
void TimStop(void);

/**
 * @brief reset timer
 */
void TimReset(void);
/**
 * @brief get elapsed time in ms
 */
uint32_t TimGetElapsedTimeInUs(void);

#ifdef __cplusplus
}
#endif
#endif /* end of include guard TIM_H */

