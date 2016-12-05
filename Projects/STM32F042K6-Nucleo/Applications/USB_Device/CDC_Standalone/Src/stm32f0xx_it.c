/**
******************************************************************************
* @file    USB_Device/CDC_Standalone/Src/stm32f0xx_it.c
* @author  MCD Application Team
* @version V1.5.0
* @date    29-January-2016
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and
*          peripherals interrupt service routine.
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V.
* All rights reserved.</center></h2>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted, provided that the following conditions are met:
*
* 1. Redistribution of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
* 3. Neither the name of STMicroelectronics nor the names of other
*    contributors to this software may be used to endorse or promote products
*    derived from this software without specific written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern PCD_HandleTypeDef hpcd;
extern USBD_HandleTypeDef USBD_Device;
/* UART handler declared in "usbd_cdc_interface.c" file */
extern UART_HandleTypeDef UartHandleX;
extern UART_HandleTypeDef UartHandleY;

/* TIM handler declared in "usbd_cdc_interface.c" file */
extern TIM_HandleTypeDef TimHandle;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
* @brief   This function handles NMI exception.
* @param  None
* @retval None
*/
void NMI_Handler(void)
{
}

/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}


/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval None
*/
void SVC_Handler(void)
{
}


/**
* @brief  This function handles PendSVC exception.
* @param  None
* @retval None
*/
void PendSV_Handler(void)
{
}

/**
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*/
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/

/**
* @brief  This function handles USB Handler.
* @param  None
* @retval None
*/
void USB_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd);
}


/**
* @brief  This function handles UART interrupt request.
* @param  None
* @retval None
*/
void USARTx_IRQHandler(void)
{
    HAL_UART_IRQHandler(&UartHandleX);
}

/**
* @brief  This function handles UART interrupt request.
* @param  None
* @retval None
*/
void USARTy_IRQHandler(void)
{
    HAL_UART_IRQHandler(&UartHandleY);
}


/**
* @brief  This function handles TIM interrupt request.
* @param  None
* @retval None
*/
void TIMx_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TimHandle);
}

/**
* @brief  This function handles PPP interrupt request.
* @param  None
* @retval None
*/
/*void PPP_IRQHandler(void)
{
}*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
