// timer.c
#include "timer.h"

// Array to hold the overflow counts for each timer

volatile static uint32_t overflowCounts[MAX_TIMERS] = {0};

// Start a timer and reset its overflow count
void timer_start(Timer *timer) {
    overflowCounts[timer->index] = 0; // Reset overflow count for this timer
    __HAL_TIM_SET_COUNTER(timer->htim, 0);  // Reset the counter
    __HAL_TIM_CLEAR_FLAG(timer->htim, TIM_FLAG_UPDATE); // Clear the update flag
    HAL_TIM_Base_Start_IT(timer->htim);    // Start the timer with interrupt
}
// Start a timer and reset its overflow count
void timer_stop(Timer *timer) {
    HAL_TIM_Base_Stop_IT(timer->htim);    // Start the timer with interrupt
    overflowCounts[timer->index] = 0; // Reset overflow count for this timer
    __HAL_TIM_SET_COUNTER(timer->htim, 0);  // Reset the counter
    __HAL_TIM_CLEAR_FLAG(timer->htim, TIM_FLAG_UPDATE); // Clear the update flag
}

// Get the elapsed time for a timer in ticks
float timer_GetElapsedTime(Timer *timer) {
    uint32_t timerMaxCount = __HAL_TIM_GET_AUTORELOAD(timer->htim) + 1;
    uint32_t currentCount = __HAL_TIM_GET_COUNTER(timer->htim);
    // timer counts to 64000, clock freq is 64Mhz and we want to return time in microseconds 
    return ((overflowCounts[timer->index] * timerMaxCount) + currentCount)/ 64.0;
}

// Timer period elapsed callback - called on timer overflow
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    int timerIndex = -1;
    extern TIM_HandleTypeDef htim3;
    if (htim == &htim3) {
        timerIndex = 0;
    // ... additional else if statements for other timers ...
    }
    if (timerIndex == -1) 
    {
        print_uart("ERR: Timer not found\r\n");
        return; // Timer not found, shouldn't happen
    }
        
    overflowCounts[timerIndex]++; // Increment overflow count for this timer

}