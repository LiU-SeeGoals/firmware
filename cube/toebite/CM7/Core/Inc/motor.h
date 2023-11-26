#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32h7xx_hal.h"

typedef struct 
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t reversing;
    uint16_t reversePin;
    uint16_t breakPin;
    uint16_t readSpeedPin;
    GPIO_TypeDef *reversePinPort;
    GPIO_TypeDef *breakPinPort;
    GPIO_TypeDef *readSpeedPinPort;
} MotorPWM;

void startMotor(MotorPWM *motor);
void setSpeed(MotorPWM *motor, float percent);
void breakMotor(MotorPWM *motor);
float readSpeed(MotorPWM *motor);

#endif