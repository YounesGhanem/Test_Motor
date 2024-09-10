/**
  ******************************************************************************
  * @file    encoder_speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the Encoder component of the Motor Control SDK:
  *           - computes and stores average mechanical speed
  *           - computes and stores average mechanical acceleration
  *           - computes and stores  the instantaneous electrical speed
  *           - calculates the rotor electrical and mechanical angle
  *
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

/* Includes ------------------------------------------------------------------*/
#include "hall_speed_pos_fdbk.h"
#include "mc_type.h"
#include "math.h"
#include "stdlib.h"  // initialize pointer

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @defgroup Encoder Encoder Speed & Position Feedback
  * @brief Quadrature Encoder based Speed & Position Feedback implementation
  *
  * This component is used in applications controlling a motor equipped with a quadrature encoder.
  *
  * This component uses the output of a quadrature encoder to provide a measure of the speed and
  * the motor rotor position.
  *
  * More detail in [Encoder Speed & Position Feedback module](rotor_speed_pos_feedback_qenc.md).
  *
  * @{
  */

/* Private defines -----------------------------------------------------------*/


/**
  * @brief  It initializes the hardware peripherals
            required for the speed position sensor management using ENCODER
            sensors.
  * @param  pHandle: handler of the current instance of the encoder component
  */
__weak void HALL_Init(HALL_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
    if (NULL == pHandle)
    {
        /* Nothing to do */
    }
    else
    {
#endif
pHandle->clarkTransform = malloc(sizeof(ClarkTransform_t));
if (pHandle->clarkTransform == NULL)
{
    // Gérer l'erreur d'allocation mémoire
}

pHandle->hallSignals = malloc(sizeof(HALL_Signals_t));
if (pHandle->hallSignals == NULL)
{
    // Gérer l'erreur d'allocation mémoire
}
        
        /* Configure ADC parameter if necessary */
        for (uint8_t i = 0; i < HALL_ARRAY_SIZE; i++)
        {
            pHandle->adcRawValue[i] = 0;  // Réinitialiser les valeurs du buffer ADC
        }

        /* Configure sample frequency */
        pHandle->SpeedSamplingFreqUnit = ((uint32_t)pHandle->SpeedSamplingFreqHz * (uint32_t)SPEED_UNIT);  //1000 x 10

        /* Falg inits */
        pHandle->SensorIsReliable = true;  

        /* Other initialization necessary ? */

        /* End */
        
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
    }
#endif
}


/**
  * @brief  Clear software FIFO where the captured rotor angle variations are stored.
  *         This function must be called before starting the motor to initialize
  *         the speed measurement process.
  * @param  pHandle: handler of the current instance of the encoder component
  */
__weak void HALL_Clear(HALL_Handle_t *pHandle)
{

    pHandle->SensorIsReliable = true;

    

}

void HALL_ClarkeTransform(const HALL_Signals_t* hallSignals, ClarkTransform_t *clarkeTransform) {
    // Les coefficients de la transformation de Clarke dépendent du système de référence
    // Ici, nous utilisons une transformation typique pour des signaux triphasés équilibrés

    clarkeTransform->alpha = (2.0f / 3.0f) * (hallSignals->Hall_a - 0.5f * (hallSignals->Hall_b + hallSignals->Hall_c));
    clarkeTransform->beta = (2.0f / 3.0f) * ((hallSignals->Hall_b - hallSignals->Hall_c) * sqrtf(3.0f) / 2.0f);
}

/**
  * @brief  It calculates the rotor electrical and mechanical angle, on the basis
  *         of the instantaneous value of the timer counter.
  * @param  pHandle: handler of the current instance of the encoder component
  * @retval Measured electrical angle in [s16degree](measurement_units.md) format.
  */
__weak int16_t HALL_CalcAngle(HALL_Handle_t *pHandle)
{
    int16_t elAngle;  /* s16degree format */
    int16_t mecAngle; /* s16degree format */

    // 1. Get Hall sensor state and convert to electrical angle
    uint8_t hallState = getHallState(pHandle->hallSignals);
    elAngle = HallStateToAngle(hallState);  // Use a lookup table or logic to map hallState to angle

    // 2. Convert elAngle from radians to s16degree if needed
    elAngle = (int16_t)((elAngle * 65536) / (2 * M_PI));

    // 3. Compute mechanical angle using the ratio
    mecAngle = elAngle / (int16_t)(pHandle->_Super.bElToMecRatio);

    // Save previous angle
    int16_t hMecAnglePrev = pHandle->_Super.hMecAngle;

    // Update current mechanical angle
    pHandle->_Super.hMecAngle = mecAngle;

    // Calculate instantaneous mechanical speed
    int16_t hMecSpeedDpp = mecAngle - hMecAnglePrev;
    pHandle->_Super.wMecAngle += ((int32_t)hMecSpeedDpp);

    // Update electrical angle in structure
    pHandle->_Super.hElAngle = elAngle;

    // Return the electrical angle
    return elAngle;
}


__weak bool HALL_CalcAvrgMecSpeedUnit(HALL_Handle_t *pHandle, int16_t *pMecSpeedUnit)
{
    bool bReliability;
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
    if ((NULL == pHandle) || (NULL == pMecSpeedUnit))
    {
        bReliability = false;
    }
    else
    {
#endif
        int32_t wOverallAngleVariation = 0;
        uint8_t bBufferIndex;
        uint8_t bBufferSize = pHandle->SpeedBufferSize;  
        int32_t wtemp1;
       

        pHandle->AngleCapturesBuffer[pHandle->AngleDeltaCapturesIndex] = pHandle->rotorAngle - pHandle->previousRotorAngle;  // TODO : Unit ?

        /* Asum up ADC values to calculate average speed */
        for (bBufferIndex = 0U; bBufferIndex < bBufferSize; bBufferIndex++)
        {
            //wOverallAngleVariation += pHandle->adcRawValue[bBufferIndex];
            //wOverallAngleVariation += pHandle->DeltaCapturesBuffer[bBufferIndex];
            wOverallAngleVariation +=pHandle->AngleCapturesBuffer[bBufferIndex];

        }

        /* Calculate average speed */
      wtemp1 = wOverallAngleVariation * ((int32_t)pHandle->SpeedSamplingFreqUnit);    //variation per second
      wtemp1 /= (int32_t)(pHandle->SpeedBufferSize);          //rev/s -> Hz (tenth of Hz)



      *pMecSpeedUnit = (int16_t)wtemp1; // tenth of Hz

      /* Computes & stores average mechanical acceleration */
        pHandle->_Super.hMecAccelUnitP = (int16_t)(wtemp1 - pHandle->_Super.hAvrMecSpeedUnit);

      /* Stores average mechanical speed */
        pHandle->_Super.hAvrMecSpeedUnit = (int16_t)wtemp1;

      /* Computes and store tje instantaneous electrical speed[dpp], var wtemp1*/
      wtemp1 = pHandle->AngleCapturesBuffer[pHandle->AngleDeltaCapturesIndex] * ((int32_t)pHandle->SpeedSamplingFreqHz) * ((int32_t)pHandle->_Super.bElToMecRatio);
      wtemp1 *= ((int32_t)pHandle->_Super.DPPConvFactor);
      wtemp1 /= ((int32_t)pHandle->_Super.hMeasurementFrequency);



      pHandle->previousRotorAngle = pHandle->rotorAngle;


      /* Update buffer index */
      pHandle->AngleDeltaCapturesIndex++;
      if (pHandle->AngleDeltaCapturesIndex >= pHandle->SpeedBufferSize)
      {
        pHandle->AngleDeltaCapturesIndex = 0U;
      }

      /* Vérification de la fiabilité du capteur */
      bReliability = SPD_IsMecSpeedReliable(&pHandle->_Super, pMecSpeedUnit);

#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
    }
#endif
    return bReliability;
}


/**
  * @brief  It set instantaneous rotor mechanical angle.
  *         As a consequence, timer counter is computed and updated.
  * @param  pHandle: handler of the current instance of the encoder component
  * @param  hMecAngle new value of rotor mechanical angle in [s16degree](measurement_units.md) format.
  */

// S16degree  1turn = 65536 s16 degrees
// 1 s16 corrresponds to 360 / 65536 = 0.005493 degree or 2*pi / 655536 radians
__weak void HALL_SetMecAngle(HALL_Handle_t *pHandle, int16_t hMecAngle)
{
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    //TIM_TypeDef *TIMx = pHandle->TIMx;


    int16_t localhMecAngle = hMecAngle;  // Instantaneous measure of rotor mechanical angle
    uint16_t hMecAngleuint;

    pHandle->_Super.hMecAngle = localhMecAngle;
    pHandle->_Super.hElAngle = localhMecAngle * (int16_t)pHandle->_Super.bElToMecRatio;
    if (localhMecAngle < 0)
    {
      localhMecAngle *= -1;
      hMecAngleuint = ((uint16_t)65535 - ((uint16_t)localhMecAngle));
    }
    else
    {
      hMecAngleuint = (uint16_t)localhMecAngle;
    }

    //hAngleCounts = (uint16_t)((((uint32_t)hMecAngleuint) * ((uint32_t)pHandle->PulseNumber)) / 65535U);  //TODO

    //TIMx->CNT = (uint16_t)hAngleCounts;
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  TIMER ENCODER Overflow interrupt counter update
  * @param  pHandleVoid: handler of the current instance of the encoder component
  */
__weak void *HALL_IRQHandler(void *pHandleVoid)
{
  //HALL_Handle_t *pHandle = (HALL_Handle_t *)pHandleVoid; //cstat !MISRAC2012-Rule-11.5 //TODO

  /* Updates the number of overflows occurred */
  /* The handling of overflow error is done in ENC_CalcAvrgMecSpeedUnit */
  //pHandle->TimerOverflowNb += 1U;  // TODO

  return (MC_NULL);
}
/**
  * @}
  */

/**
  * @}
  */

/** @} */


/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
