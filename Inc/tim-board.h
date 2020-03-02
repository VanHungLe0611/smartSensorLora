/**
 * @author      : mehdi (kaf@ad-hoc.com)
 * @file        : tim-board
 * @created     : Monday Feb 17, 2020 11:08:58 CET
 */

#ifndef __TIM_BOARD_H

#define __TIM_BOARD_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "tim.h"

/**
 * @brief initalizes the timer object
 *
 * @param obj Timer object
 */
void TimMcuInit( Tim_t *obj);

/**
 * @brief initialize the TIMER internal parameters
 */
void TimMcuConfig(uint32_t frequency);

/**
 * @brief starts timer
 */
void TimMcuStart(void);
/**
 * @brief stops timer
 */
void TimMcuStop(void);

/**
 * @brief reset cnt register
 */
void TimMcuReset(void);
/**
 * @brief get elapsed time in ms
 */
uint32_t TimMcuGetElapsedTimeInUs(void);
#ifdef __cplusplus
}
#endif
#endif /* end of include guard TIM_BOARD_H */

