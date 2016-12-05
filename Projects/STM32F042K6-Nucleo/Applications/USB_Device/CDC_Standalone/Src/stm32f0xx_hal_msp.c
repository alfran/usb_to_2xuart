/**
******************************************************************************
* @file    USB_Device/CDC_StandaloneSrc/stm32f0xx_hal_msp.c
* @author  MCD Application Team
* @version V1.5.0
* @date    29-January-2016
* @brief   HAL MSP module.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

DMA_HandleTypeDef hdma_rx;
DMA_HandleTypeDef hdmaY_rx;

/**
* @brief UART MSP Initialization
*        This function configures the hardware resources used in this example:
*           - Peripheral's clock enable
*           - Peripheral's GPIO Configuration
*           - DMA configuration for transmission request by peripheral
*           - NVIC configuration for DMA interrupt request enable
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    static DMA_HandleTypeDef hdma_tx;
    static DMA_HandleTypeDef hdmaY_tx;
    GPIO_InitTypeDef  GPIO_InitStruct;

    if (huart->Instance == USARTx)
    {
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* Enable GPIO clock */
        USARTx_TX_GPIO_CLK_ENABLE();
        USARTx_RX_GPIO_CLK_ENABLE();

        /* Enable USARTx clock */
        USARTx_CLK_ENABLE();

        /* Enable DMA clock */
        DMAx_CLK_ENABLE();

        /*##-2- Configure peripheral GPIO ##########################################*/
        /* UART TX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = USARTx_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = USARTx_TX_AF;

        HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

        /*##-3- Configure the NVIC for UART ########################################*/
        HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(USARTx_IRQn);

        /* UART RX GPIO pin configuration  */
        GPIO_InitStruct.Pin = USARTx_RX_PIN;
        GPIO_InitStruct.Alternate = USARTx_RX_AF;

        HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

        hdma_rx.Instance                 = USARTx_RX_DMA_STREAM;
        hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_rx.Init.Mode                = DMA_CIRCULAR;
        hdma_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;

        HAL_DMA_Init(&hdma_rx);

        /* Associate the initialized DMA handle to the the UART handle */
        __HAL_LINKDMA(huart, hdmarx, hdma_rx);

    }
    else if (huart->Instance == USARTy)
    {
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* Enable GPIO clock */
        USARTy_TX_GPIO_CLK_ENABLE();
        USARTy_RX_GPIO_CLK_ENABLE();

        /* Enable USARTy clock */
        USARTy_CLK_ENABLE();

        /* Enable DMA clock */
        DMAx_CLK_ENABLE();

        /*##-2- Configure peripheral GPIO ##########################################*/
        /* UARTy TX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = USARTy_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = USARTy_TX_AF;

        HAL_GPIO_Init(USARTy_TX_GPIO_PORT, &GPIO_InitStruct);

        /*##-3- Configure the NVIC for UART ########################################*/
        HAL_NVIC_SetPriority(USARTy_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(USARTy_IRQn);

        /* UARTy RX GPIO pin configuration  */
        GPIO_InitStruct.Pin = USARTy_RX_PIN;
        GPIO_InitStruct.Alternate = USARTy_RX_AF;

        HAL_GPIO_Init(USARTy_RX_GPIO_PORT, &GPIO_InitStruct);

        hdmaY_rx.Instance                 = USARTy_RX_DMA_STREAM;
        hdmaY_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdmaY_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdmaY_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdmaY_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdmaY_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdmaY_rx.Init.Mode                = DMA_CIRCULAR;
        hdmaY_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;

        HAL_DMA_Init(&hdmaY_rx);

        /* Associate the initialized DMA handle to the the UART handle */
        __HAL_LINKDMA(huart, hdmarx, hdmaY_rx);
    }
    else
    {
        //ERROR//
    }

    /*##-6- Enable TIM peripherals Clock #######################################*/
    TIMx_CLK_ENABLE();

    /*##-7- Configure the NVIC for TIMx ########################################*/
    /* Set Interrupt Group Priority */
    HAL_NVIC_SetPriority(TIMx_IRQn, 5, 0);

    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIMx_IRQn);
}

/**
* @brief UART MSP De-Initialization
*        This function frees the hardware resources used in this example:
*          - Disable the Peripheral's clock
*          - Revert GPIO, DMA and NVIC configuration to their default state
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USARTx)
    {
        /*##-1- Reset peripherals ##################################################*/
        USARTx_FORCE_RESET();
        USARTx_RELEASE_RESET();

        /*##-2- Disable peripherals and GPIO Clocks #################################*/
        /* Configure UART Tx as alternate function  */
        HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
        /* Configure UART Rx as alternate function  */
        HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);

        /*##-4- Disable the NVIC for DMA ###########################################*/
        HAL_NVIC_DisableIRQ(USARTx_IRQn);
    }
    else if (huart->Instance == USARTy)
    {
        /*##-1- Reset peripherals ##################################################*/
        USARTy_FORCE_RESET();
        USARTy_RELEASE_RESET();

        /*##-2- Disable peripherals and GPIO Clocks #################################*/
        /* Configure UART Tx as alternate function  */
        HAL_GPIO_DeInit(USARTy_TX_GPIO_PORT, USARTy_TX_PIN);
        /* Configure UART Rx as alternate function  */
        HAL_GPIO_DeInit(USARTy_RX_GPIO_PORT, USARTy_RX_PIN);

        /*##-4- Disable the NVIC for DMA ###########################################*/
        HAL_NVIC_DisableIRQ(USARTy_IRQn);
    }
    else
    {
        //ERROR
    }
}

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
