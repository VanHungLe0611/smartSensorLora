/**
 * @author      : mehdi (kaf@ad-hoc.com)
 * @file        : tim
 * @created     : Monday Feb 17, 2020 12:04:26 CET
 */

#include <stdbool.h>
#include "tim-board.h"

static bool TimInitialized = false;

void TimInit(Tim_t *  obj){
    if( TimInitialized == false )
    {
        TimInitialized = true;

        TimMcuInit( obj);
    }
}
/**
 * @brief initialize the TIMER internal parameters
 */
void TimConfig(uint32_t frequency){
    TimMcuConfig(frequency);
}

/**
 * @brief starts timer
 */
void TimStart(void){
    TimMcuStart();
}
/**
 * @brief stops timer
 */
void TimStop(void){
    TimMcuStop();
}
void TimReset(void){
    TimMcuReset();
}
/**
 * @brief get elapsed time in ms
 */
uint32_t TimGetElapsedTimeInUs(void){
    return TimMcuGetElapsedTimeInUs();
}
/**
 * @brief initialize the TIMER internal parameters
 */
