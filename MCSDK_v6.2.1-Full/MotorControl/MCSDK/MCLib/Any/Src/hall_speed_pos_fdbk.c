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

    uint8_t index = 0;

    for (index =0U; index < pHandle->SpeedBufferSize ; index++)
    {
      pHandle->AngleCapturesBuffer[index] = 0;
    }

    pHandle->SensorIsReliable = true;

    

}

// Define constants for ADC conversion and voltage divider
#define VREF 3.3f
#define ADC_MAX 4095.0f  // For 12-bit ADC
#define R1 10000.0f  // Resistance in ohms
#define R2 30000.0f  // Resistance in ohms
// #define R3 10000.0f // Resistance in ohms

// Constants for converting angle to s16 representation
#define S16_MAX 32767  // Max value for s16degree format (representing +/-180 degrees)
#define TWO_PI 6.28318530718f  // 2*pi
#define PI 3.14159265359f  // Pi constant

int16_t  HALL_ClarkeTransform(const HALL_Signals_t* hallSignals, ClarkTransform_t *clarkeTransform) {
    // Convert raw ADC values to voltages (Vadc)
    float Vadc_a = (hallSignals->Hall_a / ADC_MAX) * VREF;
    float Vadc_b = (hallSignals->Hall_b / ADC_MAX) * VREF ;
    float Vadc_c = (hallSignals->Hall_c / ADC_MAX) * VREF;

    // Apply the voltage divider to get the actual analog voltages (Vanalog_hall)
    float Vhall_a = Vadc_a * ( R1 / R2 + 1);
    float Vhall_b = Vadc_b * ( R1 / R2 + 1);
    float Vhall_c = Vadc_c * ( R1 / R2 + 1);


    // Clarke Transformation using the corrected analog voltages
    clarkeTransform->alpha = (2.0f / 3.0f) * (Vhall_a - 0.5f * (Vhall_b + Vhall_c));
    clarkeTransform->beta = (Vhall_b - Vhall_c) * sqrtf(3.0f) / 3.0f;

    // Calculate the angle using atan2 and convert it to s16degree representation
    float angle_radians = atan2f(clarkeTransform->beta, clarkeTransform->alpha);

      // Convert from [-π, π] to [0, 2π] by adding 2π to negative angles
     if (angle_radians < 0) 
     {
         angle_radians += TWO_PI;
     }
     else
     {

     }

    // // Map the angle from [0, π] to [0, 32767], and from [π, 2π] to [-32767, 0]
     int16_t angle_s16;
     if (angle_radians <= PI) 
     {
       // From 0 to π: positive s16degree values
       angle_s16 = (int16_t)((angle_radians * S16_MAX) / PI);
    } 
     else 
     {
     // From π to 2π: negative s16degree values
     angle_s16 = (int16_t)(((angle_radians - TWO_PI) * S16_MAX) / PI);
     }


    //int16_t angle_S16 = (int16_t)(angle_radians * 65536 / TWO_PI );

    return angle_s16;

    
}


__weak void HALL_CalcAngle(HALL_Handle_t *pHandle) 
{
    int16_t elAngle;  // s16degree format
    int16_t mecAngle; // s16degree format
    alphabeta_t Ialphabeta;

    // Call the Clarke Transform function to calculate alpha and beta
    // Get the electrical angle from Clarke Transform (in s16degree format)
    // Assign the calculated electrical angle to hElAngle in SpeednPosFdbk_Handle_t
    //Ialphabeta = MCM_Clarke(Iab);
    elAngle = HALL_ClarkeTransform(pHandle->hallSignals, pHandle->clarkTransform);  // all signals already calculated in Timer

    // Calculate mechanical angle by dividing electrical angle by the ratio
    pHandle->_Super.hElAngle = elAngle;
    mecAngle = elAngle / (int16_t)(pHandle->_Super.bElToMecRatio);

    // Save the previous mechanical angle
    int16_t hMecAnglePrev = pHandle->_Super.hMecAngle;

    // Update current mechanical angle
    pHandle->_Super.hMecAngle = mecAngle;

    // Instantaneous mechanical speed (difference in mechanical angle)
    int16_t hMecSpeedDpp = mecAngle - hMecAnglePrev;

    // Accumulate mechanical angle in 32-bit to avoid overflow
    pHandle->_Super.wMecAngle += ((int32_t)hMecSpeedDpp);


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
// #ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
//   if (NULL == pHandle)
//   {
//     /* Nothing to do */
//   }
//   else
//   {
// #endif
    //TIM_TypeDef *TIMx = pHandle->TIMx;


    int16_t localhMecAngle = hMecAngle;  // Instantaneous measure of rotor mechanical angle
    //uint16_t hMecAngleuint;

    pHandle->_Super.hMecAngle = localhMecAngle;
    pHandle->_Super.hElAngle = localhMecAngle * (int16_t)pHandle->_Super.bElToMecRatio;
    // if (localhMecAngle < 0)
    // {
    //   localhMecAngle *= -1;
    //   hMecAngleuint = ((uint16_t)65535 - ((uint16_t)localhMecAngle));
    // }
    // else
    // {
    //   hMecAngleuint = (uint16_t)localhMecAngle;
    // }

    //hAngleCounts = (uint16_t)((((uint32_t)hMecAngleuint) * ((uint32_t)pHandle->PulseNumber)) / 65535U);  //TODO

    //TIMx->CNT = (uint16_t)hAngleCounts;
// #ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
//   }
// #endif
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
