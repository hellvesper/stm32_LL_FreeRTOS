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
LL_I2C_InitTypeDef I2C_InitStruct = {0};
/**
  * @brief Variables related to Slave process
  */
const char *aSlaveInfo[] = {
        "STM32F103RBT6",
        "1.2.3"};

uint8_t aSlaveReceiveBuffer[0xF] = {0}; // receive buf
uint8_t pSlaveTransmitBuffer[0xF] = {0};   // transmit buf
__IO uint8_t ubSlaveNbDataToTransmit = 0;   // transmit bytes index
//uint8_t       ubSlaveInfoIndex          = 0xFF;// sample data index
__IO uint8_t ubSlaveReceiveIndex = 0;   // receive bytes index
__IO uint8_t ubSlaveReceiveComplete = 0;
__IO uint8_t lastCMD = 0;

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
void I2C_EvHandler() {
    static uint8_t rwBytesCounter = 0;
    /* Check ADDR flag value in ISR register */
//    if (LL_I2C_IsActiveFlag_ADDR(I2C1)) {
//        /* Verify the slave transfer direction, a read direction, Slave enters receiver mode */
//        if (LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_READ) {
//            /* Enable Buffer Interrupts */
//            LL_I2C_EnableIT_BUF(I2C1);
//
//            /* Clear ADDR flag value in ISR register */
//            LL_I2C_ClearFlag_ADDR(I2C1);
//        } else if (LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_WRITE) {
//            /* Enable Buffer Interrupts */
//            LL_I2C_EnableIT_BUF(I2C1);
//
//            /* Clear ADDR flag value in ISR register */
//            LL_I2C_ClearFlag_ADDR(I2C1);
//        }
//    }
//    if (LL_I2C_IsActiveFlag_STOP(I2C1)) {
//
//    }
//    if (LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_READ) {
//        while (!LL_I2C_IsActiveFlag_RXNE(I2C1));
//        RxBuf[rwBytesCounter] = LL_I2C_ReceiveData8(I2C1);
//        rwBytesCounter++;
//    } else { /* LL_I2C_DIRECTION_WRITE */
//
//    }


}

/**
  * @brief  Function called from I2C IRQ Handler when TXE flag is set
  *         Function is in charge of transmit a byte on I2C lines.
  * @param  None
  * @retval None
  */
void Slave_Ready_To_Transmit_Callback(void) {
    if (ubSlaveNbDataToTransmit > 0) {
        /* Send the Byte requested by the Master */
        LL_I2C_TransmitData8(I2C1, pSlaveTransmitBuffer[ubSlaveNbDataToTransmit]);

        ubSlaveNbDataToTransmit--;
    } else {
        /* Send the NULL Byte until Master stop the communication */
        /* This is needed due to Master don't know how many data slave will sent */
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
//    aSlaveReceiveBuffer[ubSlaveReceiveIndex] = LL_I2C_ReceiveData8(I2C1);
    lastCMD = LL_I2C_ReceiveData8(I2C1);

    /* Check Command code & prepare data for transmit*/
    switch (lastCMD) {
        case (uint8_t)0xA0:
        {
            pSlaveTransmitBuffer[0] = 0xAA;
            __NOP();
            ubSlaveNbDataToTransmit = (uint8_t) 1;
        }
            break;
        case (uint8_t)0xB0:
            pSlaveTransmitBuffer[0] = 0xBB;
            ubSlaveNbDataToTransmit = 1;
            break;
        case (uint8_t)0xC0:
            for (ubSlaveNbDataToTransmit = 0; ubSlaveNbDataToTransmit < 0xF; ++ubSlaveNbDataToTransmit) {
                pSlaveTransmitBuffer[ubSlaveNbDataToTransmit] = ubSlaveNbDataToTransmit;
            }
            break;
        default:
            __NOP();
            break;
    }
    ubSlaveReceiveIndex++;
}

/**
  * @brief  Function called from I2C IRQ Handler when STOP flag is set
  *         Function is in charge of checking data received,
  *         LED2 is On if data are correct.
  * @param  None
  * @retval None
  */
void Slave_Complete_Callback(void) {
    /* Check Command code & prepare data for transmit*/
//    switch (aSlaveReceiveBuffer[0]) {
//        case 0xA0:
//            pSlaveTransmitBuffer[0] = 0xAA;
//            break;
//        case 0xB0:
//            pSlaveTransmitBuffer[0] = 0xBB;
//            break;
//        case 0xC0:
//            for (int i = 0; i < 0xF; ++i) {
//                pSlaveTransmitBuffer[i] = 0;
//            }
//    }
//    HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, GPIO_PIN_RESET); // ON
    __NOP();
    /* Clear and Reset process variables and arrays */
    ubSlaveReceiveIndex = 0;
    ubSlaveReceiveComplete = 0;
    FlushBuffer8(aSlaveReceiveBuffer);
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
