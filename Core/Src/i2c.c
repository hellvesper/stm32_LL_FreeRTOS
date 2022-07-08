/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#define CMD_1 0xA0
#define CMD_2 0xB0
#define CMD_3 0xC0
#define CMD_4 0xF0
#define PWM_W 0xD0


LL_I2C_InitTypeDef I2C_InitStruct = {0};
/**
  * @brief Variables related to Slave process
  */
//const char *aSlaveInfo[] = {
//        "STM32F103RBT6",
//        "1.2.3"};

//uint8_t aSlaveReceiveBuffer[CHANNELS] = {0}; // receive buf
uint8_t pSlaveTransmitBuffer[CHANNELS*2] = {0};   // transmit buf
__IO uint8_t ubSlaveNbDataToTransmit = 0;   // transmit bytes index
//uint8_t       ubSlaveInfoIndex          = 0xFF;// sample data index
//__IO uint8_t ubSlaveReceiveIndex = 0;   // receive bytes index
//__IO uint8_t ubSlaveReceiveComplete = 0;
__IO uint8_t RxBytesCounter = 0;    // receive bytes index
__IO uint8_t lastCMD = 0;
__IO uint8_t RxData = 0;

uint8_t Buffercmp8(uint8_t *pBuffer1, uint8_t *pBuffer2, uint8_t BufferLength);

void FlushBuffer8(uint8_t *pBuffer1);
/* USER CODE END 0 */

/* I2C1 init function */
void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

//    LL_I2C_InitTypeDef I2C_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
    /**I2C1 GPIO Configuration
    PB6   ------> I2C1_SCL
    PB7   ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

    /* I2C1 interrupt Init */
    NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(I2C1_ER_IRQn);

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */

    /** I2C Initialization
    */
    LL_I2C_DisableClockStretching(I2C1);
    LL_I2C_DisableOwnAddress2(I2C1);
    LL_I2C_DisableGeneralCall(I2C1);
    I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
    I2C_InitStruct.ClockSpeed = 100000;
    I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
    I2C_InitStruct.OwnAddress1 = 144;
    I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
    I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
    LL_I2C_Init(I2C1, &I2C_InitStruct);
    LL_I2C_SetOwnAddress2(I2C1, 0);
    /* USER CODE BEGIN I2C1_Init 2 */
    LL_I2C_EnableIT_EVT(I2C1);

    /* USER CODE END I2C1_Init 2 */

}

/* USER CODE BEGIN 1 */

/**
  * @brief  Function called from I2C IRQ Handler when TXE flag is set
  *         Function is in charge of transmit a byte on I2C lines.
  * @param  None
  * @retval None
  */
void Slave_Ready_To_Transmit_Callback(void) {
    static uint8_t bCount = 0;
    if (bCount < ubSlaveNbDataToTransmit) {
        /* Send the Byte requested by the Master */
        LL_I2C_TransmitData8(I2C1, pSlaveTransmitBuffer[bCount]);

        bCount++;
    } else {
        /* Send the NULL Byte until Master stop the communication */
        /* This is needed due to Master don't know how many data slave will sent */
        ubSlaveNbDataToTransmit = 0;
        bCount = 0;
        LL_I2C_TransmitData8(I2C1, 0x11);
    }

}

/**
  * @brief  Function called from I2C IRQ Handler when RXNE flag is set
  *         Function is in charge of retrieving received byte on I2C lines.
  * @param  None
  * @retval None
  */
void Slave_Reception_Callback(void) {
    /* Read character in Receive Data register.
    RXNE flag is cleared by reading data in RXDR register */
    RxData = LL_I2C_ReceiveData8(I2C1);
    RxBytesCounter++;

    /* Check Command code & prepare data for transmit*/
    switch (RxData) {
        case (uint8_t) CMD_1: {
            lastCMD = RxData;
            pSlaveTransmitBuffer[0] = 0xAA;
            ubSlaveNbDataToTransmit = 1;
        }
            break;
        case (uint8_t) CMD_2:
            lastCMD = RxData;
            pSlaveTransmitBuffer[0] = 0xBB;
            ubSlaveNbDataToTransmit = 1;
            break;
        case (uint8_t) CMD_3:
            lastCMD = RxData;
            for (ubSlaveNbDataToTransmit = 0; ubSlaveNbDataToTransmit < 0x10; ++ubSlaveNbDataToTransmit) {
                pSlaveTransmitBuffer[ubSlaveNbDataToTransmit] = ubSlaveNbDataToTransmit;
            }
            break;
        case (uint8_t) CMD_4:
            lastCMD = RxData;
            for (int i = 0; i < 8; ++i) {
                pSlaveTransmitBuffer[i * 2] = TicksSec[i] << 8;
                pSlaveTransmitBuffer[i * 2 + 1] = (TicksSec[i] << 8) >> 8;
            }
        case (uint8_t) PWM_W:
            lastCMD = RxData;
            ubSlaveNbDataToTransmit = 0;
            break;
        default:
            if (lastCMD == PWM_W)
                PWM_DutyCycle = RxData;
            ubSlaveNbDataToTransmit = 0;
            break;
    }
}

/**
  * @brief  Function called from I2C IRQ Handler when STOP flag is set
  *         Function is in charge of checking data received,
  *         LED2 is On if data are correct.
  * @param  None
  * @retval None
  */
void Slave_Complete_Callback(void) {
    /* Check Command code & prepare data for transmit (moved to receive callback)*/

    /* Clear and Reset process variables and arrays */
//    ubSlaveReceiveIndex = 0;
//    ubSlaveReceiveComplete = 0;
    RxBytesCounter = 0;
//    FlushBuffer8(aSlaveReceiveBuffer);
}

/**
  * @brief  Function called in case of error detected in I2C IT Handler
  * @param  None
  * @retval None
  */
void Error_Callback(void) {
    /* Disable I2C1_EV_IRQn */
    NVIC_DisableIRQ(I2C1_EV_IRQn);

    /* Disable I2C1_ER_IRQn */
    NVIC_DisableIRQ(I2C1_ER_IRQn);

}

/**
  * @brief  Flush 8-bit buffer.
  * @param  pBuffer1: pointer to the buffer to be flushed.
  * @retval None
  */
void FlushBuffer8(uint8_t *pBuffer1) {
    uint8_t Index = 0;

    for (Index = 0; Index < sizeof(pBuffer1); Index++) {
        pBuffer1[Index] = 0;
    }
}

/**
  * @brief  Compares two 8-bit buffers and returns the comparison result.
  * @param  pBuffer1: pointer to the source buffer to be compared to.
  * @param  pBuffer2: pointer to the second source buffer to be compared to the first.
  * @param  BufferLength: buffer's length.
  * @retval 0: Comparison is OK (the two Buffers are identical)
  *         Value different from 0: Comparison is NOK (Buffers are different)
  */
uint8_t Buffercmp8(uint8_t *pBuffer1, uint8_t *pBuffer2, uint8_t BufferLength) {
    while (BufferLength--) {
        if (*pBuffer1 != *pBuffer2) {
            return 1;
        }

        pBuffer1++;
        pBuffer2++;
    }

    return 0;
}

/* USER CODE END 1 */
