/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Src/main.c
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    29-January-2016
  * @brief   USB device CDC demo main file
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
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_HandleTypeDef USBD_Device;
volatile uint8_t timer_expired = 0;
uint32_t UserTxBufPtrInX = 0;/* Increment this pointer or roll it back to
start address when data are received over USART */
uint32_t UserTxBufPtrOutX = 0; /* Increment this pointer or roll it back to
start address when data are sent over USB */
uint32_t UserTxBufPtrInY = 0;/* Increment this pointer or roll it back to
start address when data are received over USART */
uint32_t UserTxBufPtrOutY = 0; /* Increment this pointer or roll it back to
start address when data are sent over USB */

extern DMA_HandleTypeDef hdma_rx;
extern DMA_HandleTypeDef hdmaY_rx;
extern uint8_t UserRxBuffer[2][APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
extern uint8_t UserTxBuffer[2][APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */
extern uint8_t UART_RxBuffer[2][APP_TX_DATA_SIZE];

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    /* STM32F0xx HAL library initialization:
         - Configure the Flash prefetch, Flash preread and Buffer caches
         - Systick timer is configured by default as source of time base, but user
               can eventually implement his proper time base source (a general purpose
               timer for example or other time source), keeping in mind that Time base
               duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
               handled in milliseconds basis.
         - Low Level Initialization
       */
    HAL_Init();

    /* Configure the system clock to get correspondent USB clock source */
    SystemClock_Config();

    /* Init Device Library */
    USBD_Init(&USBD_Device, &VCP_Desc, 0);

    /* Add Supported Class */
    USBD_RegisterClass(&USBD_Device, &USBD_CDC);

    /* Add CDC Interface Class */
    USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops[0]);

    /* Start Device Process */
    USBD_Start(&USBD_Device);

    while (1)
    {
        uint8_t status;

        if(timer_expired)
        {
            uint16_t DmaCounter, length;

            timer_expired = 0;

            if(__HAL_DMA_GET_FLAG(&hdma_rx, __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_rx)) == RESET)
            {
                DmaCounter = APP_TX_DATA_SIZE - __HAL_DMA_GET_COUNTER(&hdma_rx);
                UserTxBufPtrInX = DmaCounter;

                if(UserTxBufPtrOutX != UserTxBufPtrInX)
                {
                    if (UserTxBufPtrOutX > UserTxBufPtrInX)
                    {
                        memcpy(&UserTxBuffer[0][0], &UART_RxBuffer[0][UserTxBufPtrOutX], (APP_TX_DATA_SIZE - UserTxBufPtrOutX));
                        memcpy(&UserTxBuffer[0][(APP_TX_DATA_SIZE - UserTxBufPtrOutX)], &UART_RxBuffer[0][0], UserTxBufPtrInX);
                        length = APP_TX_DATA_SIZE + UserTxBufPtrInX - UserTxBufPtrOutX;
                    }
                    else
                    {
                        length = UserTxBufPtrInX - UserTxBufPtrOutX;
                        memcpy(&UserTxBuffer[0][0], &UART_RxBuffer[0][UserTxBufPtrOutX], length);
                    }

                    USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t*)&UserTxBuffer[0][0], length, CDC_IN_EP1);

                    status = USBD_CDC_TransmitPacket(&USBD_Device, CDC_IN_EP1);

                    if(status == USBD_OK)
                    {
                        UserTxBufPtrOutX = UserTxBufPtrInX;
                    }
                }
            }

            if(__HAL_DMA_GET_FLAG(&hdmaY_rx, __HAL_DMA_GET_TE_FLAG_INDEX(&hdmaY_rx)) == RESET)
            {
                DmaCounter = APP_TX_DATA_SIZE - __HAL_DMA_GET_COUNTER(&hdmaY_rx);
                UserTxBufPtrInY = DmaCounter;

                if(UserTxBufPtrOutY != UserTxBufPtrInY)
                {
                    if (UserTxBufPtrOutY > UserTxBufPtrInY)
                    {
                        memcpy(&UserTxBuffer[1][0], &UART_RxBuffer[1][UserTxBufPtrOutY], (APP_TX_DATA_SIZE - UserTxBufPtrOutY));
                        memcpy(&UserTxBuffer[1][(APP_TX_DATA_SIZE - UserTxBufPtrOutY)], &UART_RxBuffer[1][0], UserTxBufPtrInY);
                        length = APP_TX_DATA_SIZE + UserTxBufPtrInY - UserTxBufPtrOutY;
                    }
                    else
                    {
                        length = UserTxBufPtrInY - UserTxBufPtrOutY;
                        memcpy(&UserTxBuffer[1][0], &UART_RxBuffer[1][UserTxBufPtrOutY], length);
                    }

                    USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t*)&UserTxBuffer[1][0], length, CDC_IN_EP2);

                    status = USBD_CDC_TransmitPacket(&USBD_Device, CDC_IN_EP2);

                    if(status == USBD_OK)
                    {
                        UserTxBufPtrOutY = UserTxBufPtrInY;
                    }
                }
            }
        }

        __WFI();
    }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow:
  *
  *            HSI48 used as USB clock source (USE_USB_CLKSOURCE_CRSHSI48 defined in main.h)
  *              - System Clock source            = HSI48
  *              - SYSCLK(Hz)                     = 48000000
  *              - HCLK(Hz)                       = 48000000
  *              - AHB Prescaler                  = 1
  *              - APB1 Prescaler                 = 1
  *              - Flash Latency(WS)              = 1
  *
  *              - PLL(HSE) used as USB clock source (USE_USB_CLKSOURCE_PLL defined in main.h)
  *              - System Clock source            = PLL (HSE)
  *              - SYSCLK(Hz)                     = 48000000
  *              - HCLK(Hz)                       = 48000000
  *              - AHB Prescaler                  = 1
  *              - APB1 Prescaler                 = 1
  *              - HSE Frequency(Hz)              = 8000000
  *              - PREDIV                         = 1
  *              - PLLMUL                         = 6
  *              - Flash Latency(WS)              = 1
  *
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

#if defined (USE_USB_CLKSOURCE_CRSHSI48)
    static RCC_CRSInitTypeDef RCC_CRSInitStruct;
#endif

#if defined (USE_USB_CLKSOURCE_CRSHSI48)

    /* Enable HSI48 Oscillator to be used as system clock source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* Select HSI48 as USB clock source */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    /* Select HSI48 as system clock source and configure the HCLK and PCLK1
    clock dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

    /*Configure the clock recovery system (CRS)**********************************/

    /*Enable CRS Clock*/
    __HAL_RCC_CRS_CLK_ENABLE();

    /* Default Synchro Signal division factor (not divided) */
    RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;

    /* Set the SYNCSRC[1:0] bits according to CRS_Source value */
    RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;

    /* HSI48 is synchronized with USB SOF at 1KHz rate */
    RCC_CRSInitStruct.ReloadValue =  __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
    RCC_CRSInitStruct.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;

    /* Set the TRIM[5:0] to the default value*/
    RCC_CRSInitStruct.HSI48CalibrationValue = 0x20;

    /* Start automatic synchronization */
    HAL_RCCEx_CRSConfig (&RCC_CRSInitStruct);

#elif defined (USE_USB_CLKSOURCE_PLL)

    /* Enable HSE Oscillator and activate PLL with HSE as source
    PLLCLK = (8 * 6) / 1) = 48 MHz */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /*Select PLL 48 MHz output as USB clock source */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    /* Select PLL as system clock source and configure the HCLK and PCLK1
    clock dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

#endif /*USE_USB_CLKSOURCE_CRSHSI48*/

}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
