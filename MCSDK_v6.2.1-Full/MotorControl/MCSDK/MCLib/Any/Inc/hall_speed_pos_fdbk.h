/**
  ******************************************************************************
  * @file    hall_speed_pos_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          hall Speed & Position Feedback component of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup Encoder
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HALL_SPEEDNPOSFDBK_H
#define HALL_SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
   * @{
   */

/** @addtogroup Encoder
  * @{
  */

/* Exported constants --------------------------------------------------------*/

#define GPIO_NoRemap_TIMx ((uint32_t)(0))
#define ENC_DMA_PRIORITY DMA_Priority_High
#define ENC_SPEED_ARRAY_SIZE  ((uint8_t)16)    /* 2^4 */

#define HALL_ARRAY_SIZE     1

/**
  * @brief  ENCODER class parameters definition
  */
typedef struct
{
  SpeednPosFdbk_Handle_t _Super;                     /*!< SpeednPosFdbk  handle definition. */
  TIM_TypeDef *TIMx;                                 /*!< Timer used for ENCODER sensor management.*/
  uint32_t SpeedSamplingFreqUnit;                    /*!< Frequency at which motor speed is to be
                                                          computed. It must be equal to the frequency
                                                          at which function SPD_CalcAvrgMecSpeedUnit is called.*/
  int32_t DeltaCapturesBuffer[ENC_SPEED_ARRAY_SIZE]; /*!< Buffer used to store captured variations of timer counter*/
  uint32_t U32MAXdivPulseNumber;                     /*!< It stores U32MAX/hPulseNumber */
  uint16_t SpeedSamplingFreqHz;                      /*!< Frequency (Hz) at which motor speed is to be computed. */

  /* SW Settings */
  uint16_t PulseNumber;                              /*!< Number of pulses per revolution, provided by each
                                                          of the two encoder signals, multiplied by 4 */
  volatile uint16_t TimerOverflowNb;                 /*!< Number of overflows occurred since
                                                          last speed measurement event*/
  uint16_t PreviousCapture;                          /*!< Timer counter value captured during
                                                          previous speed measurement event*/
  uint8_t SpeedBufferSize;                           /*!< Size of the buffer used to calculate the average
                                                          speed. It must be <= 16.*/
  bool SensorIsReliable;                             /*!< Flag to indicate sensor/decoding is not properly working.*/
  uint32_t ICx_Filter;                                /*!< Input Capture filter selection */
  volatile uint8_t DeltaCapturesIndex;               /*!< Buffer index */
  bool TimerOverflowError;                           /*!< true if the number of overflow  occurred is greater than
                                                          'define' ENC_MAX_OVERFLOW_NB*/
  uint16_t adcRawValue[HALL_ARRAY_SIZE];
} HALL_Handle_t;


/* IRQ implementation of the TIMER ENCODER */
void *HALL_IRQHandler(void *pHandleVoid);

/* It initializes the hardware peripherals (TIMx, GPIO and NVIC)
 * required for the speed position sensor management using ENCODER
 * sensors */
void HALL_Init(HALL_Handle_t *pHandle);

/* Clear software FIFO where are "pushed" rotor angle variations captured */
void HALL_Clear(HALL_Handle_t *pHandle);

/* It calculates the rotor electrical and mechanical angle, on the basis
 * of the instantaneous value of the timer counter */
int16_t HALL_CalcAngle(HALL_Handle_t *pHandle);

/* The method generates a capture event on a channel, computes & stores average mechanical speed */
bool HALL_CalcAvrgMecSpeedUnit(HALL_Handle_t *pHandle, int16_t *pMecSpeedUnit);

/* It set instantaneous rotor mechanical angle */
void HALL_SetMecAngle(HALL_Handle_t *pHandle, int16_t hMecAngle);


/**
  * @}
  */

/**
  * @}
  */

/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*ENCODER_SPEEDNPOSFDBK_H*/

/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
