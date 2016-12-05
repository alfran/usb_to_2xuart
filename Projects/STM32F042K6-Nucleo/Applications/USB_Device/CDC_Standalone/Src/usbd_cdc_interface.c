/**
******************************************************************************
* @file    USB_Device/CDC_Standalone/Src/usbd_cdc_interface.c
* @author  MCD Application Team
* @version V1.5.0
* @date    29-January-2016
* @brief   Source file for USBD CDC interface
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

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
* @{
*/

/** @defgroup USBD_CDC
* @brief usbd core module
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_CDC_LineCodingTypeDef LineCoding =
{
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
};

extern uint8_t UserRxBuffer[2][APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
extern uint8_t UserTxBuffer[2][APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */
uint8_t UART_RxBuffer[2][APP_TX_DATA_SIZE];
extern volatile uint8_t timer_expired;

/* UART handler declaration */
UART_HandleTypeDef UartHandleX;
UART_HandleTypeDef UartHandleY;
/* TIM handler declaration */
TIM_HandleTypeDef    TimHandle;
/* USB handler declaration */
extern USBD_HandleTypeDef  USBD_Device;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init_UARTx     (void);
static int8_t CDC_Itf_DeInit_UARTx   (void);
static int8_t CDC_Itf_Control_UARTx  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive_UARTx  (uint8_t* pbuf, uint32_t *Len);

static int8_t CDC_Itf_Init_UARTy     (void);
static int8_t CDC_Itf_DeInit_UARTy   (void);
static int8_t CDC_Itf_Control_UARTy  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive_UARTy  (uint8_t* pbuf, uint32_t *Len);

static void Error_Handler(void);
static void ComPort_Config(UART_HandleTypeDef *UartHandle);
static void TIM_Config(void);

USBD_CDC_ItfTypeDef USBD_CDC_fops[2] =
{
    {
        CDC_Itf_Init_UARTx,
        CDC_Itf_DeInit_UARTx,
        CDC_Itf_Control_UARTx,
        CDC_Itf_Receive_UARTx
    },
    {
        CDC_Itf_Init_UARTy,
        CDC_Itf_DeInit_UARTy,
        CDC_Itf_Control_UARTy,
        CDC_Itf_Receive_UARTy
    }
};

/* Private functions ---------------------------------------------------------*/

/**
* @brief  CDC_Itf_Init
*         Initializes the CDC media low layer
* @param  None
* @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t CDC_Itf_Init_UARTx()
{
    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* USART configured as follow:
    - Word Length = 8 Bits
    - Stop Bit    = One Stop bit
    - Parity      = No parity
    - BaudRate    = 115200 baud
    - Hardware flow control disabled (RTS and CTS signals) */
    UartHandleX.Instance        = USARTx;
    UartHandleX.Init.BaudRate   = 115200;
    UartHandleX.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandleX.Init.StopBits   = UART_STOPBITS_1;
    UartHandleX.Init.Parity     = UART_PARITY_NONE;
    UartHandleX.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UartHandleX.Init.Mode       = UART_MODE_TX_RX;

    if(HAL_UART_Init(&UartHandleX) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /*##-2- Put UART peripheral in IT reception process ########################*/
    /* Any data received will stored in "UserTxBuffer" buffer  */
    UartHandleX.pRxBuffPtr = (uint8_t*)&UART_RxBuffer[0][0];
    UartHandleX.RxXferSize = APP_TX_DATA_SIZE;
    UartHandleX.ErrorCode = HAL_UART_ERROR_NONE;

    HAL_UART_Receive_DMA(&UartHandleX, (uint8_t*)&UART_RxBuffer[0][0], APP_TX_DATA_SIZE);

    /*##-5- Set Application Buffers ############################################*/
    USBD_CDC_SetTxBuffer(&USBD_Device, UserTxBuffer[0], 0, CDC_IN_EP1);
    USBD_CDC_SetRxBuffer(&USBD_Device, UserRxBuffer[0], CDC_OUT_EP1);

    /*##-3- Configure the TIM Base generation  #################################*/
    TIM_Config();

    /*##-4- Start the TIM Base generation in interrupt mode ####################*/
    /* Start Channel1 */
    if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
    {
        /* Starting Error */
        Error_Handler();
    }

    return (USBD_OK);
}

/**
* @brief  CDC_Itf_Init_UART2
*         Initializes the CDC media low layer
* @param  None
* @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t CDC_Itf_Init_UARTy()
{
    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* USART configured as follow:
    - Word Length = 8 Bits
    - Stop Bit    = One Stop bit
    - Parity      = No parity
    - BaudRate    = 115200 baud
    - Hardware flow control disabled (RTS and CTS signals) */
    UartHandleY.Instance        = USARTy;
    UartHandleY.Init.BaudRate   = 115200;
    UartHandleY.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandleY.Init.StopBits   = UART_STOPBITS_1;
    UartHandleY.Init.Parity     = UART_PARITY_NONE;
    UartHandleY.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UartHandleY.Init.Mode       = UART_MODE_TX_RX;

    if(HAL_UART_Init(&UartHandleY) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /*##-2- Put UART peripheral in IT reception process ########################*/
    /* Any data received will stored in "UserTxBuffer" buffer  */
    UartHandleY.pRxBuffPtr = (uint8_t*)&UART_RxBuffer[1][0];
    UartHandleY.RxXferSize = APP_TX_DATA_SIZE;
    UartHandleY.ErrorCode = HAL_UART_ERROR_NONE;

    HAL_UART_Receive_DMA(&UartHandleY, (uint8_t*)&UART_RxBuffer[1][0], APP_TX_DATA_SIZE);

    /*##-5- Set Application Buffers ############################################*/
    USBD_CDC_SetTxBuffer(&USBD_Device, UserTxBuffer[1], 0, CDC_IN_EP2);
    USBD_CDC_SetRxBuffer(&USBD_Device, UserRxBuffer[1], CDC_OUT_EP2);

    /*##-3- Configure the TIM Base generation  #################################*/
    TIM_Config();

    /*##-4- Start the TIM Base generation in interrupt mode ####################*/
    /* Start Channel1 */
    if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
    {
        /* Starting Error */
        Error_Handler();
    }

    return (USBD_OK);
}

/**
* @brief  CDC_Itf_DeInit
*         DeInitializes the CDC media low layer
* @param  None
* @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t CDC_Itf_DeInit_UARTx(void)
{
    /* DeInitialize the UART peripheral */
    if(HAL_UART_DeInit(&UartHandleX) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
    return (USBD_OK);
}

/**
* @brief  CDC_Itf_DeInit
*         DeInitializes the CDC media low layer
* @param  None
* @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t CDC_Itf_DeInit_UARTy(void)
{
    /* DeInitialize the UART peripheral */
    if(HAL_UART_DeInit(&UartHandleY) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
    return (USBD_OK);
}

/**
* @brief  CDC_Itf_Control
*         Manage the CDC class requests
* @param  Cmd: Command code
* @param  Buf: Buffer containing command data (request parameters)
* @param  Len: Number of data to be sent (in bytes)
* @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t CDC_Itf_Control_UARTx (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
    switch (cmd)
    {
    case CDC_SEND_ENCAPSULATED_COMMAND:
        /* Add your code here */
        break;

    case CDC_GET_ENCAPSULATED_RESPONSE:
        /* Add your code here */
        break;

    case CDC_SET_COMM_FEATURE:
        /* Add your code here */
        break;

    case CDC_GET_COMM_FEATURE:
        /* Add your code here */
        break;

    case CDC_CLEAR_COMM_FEATURE:
        /* Add your code here */
        break;

    case CDC_SET_LINE_CODING:
        LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                                           (pbuf[2] << 16) | (pbuf[3] << 24));
        LineCoding.format     = pbuf[4];
        LineCoding.paritytype = pbuf[5];
        LineCoding.datatype   = pbuf[6];

        /* Set the new configuration */
        ComPort_Config(&UartHandleX);

        break;

    case CDC_GET_LINE_CODING:
        pbuf[0] = (uint8_t)(LineCoding.bitrate);
        pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
        pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
        pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
        pbuf[4] = LineCoding.format;
        pbuf[5] = LineCoding.paritytype;
        pbuf[6] = LineCoding.datatype;

        /* Add your code here */
        break;

    case CDC_SET_CONTROL_LINE_STATE:
        /* Add your code here */
        break;

    case CDC_SEND_BREAK:
        /* Add your code here */
        break;

    default:
        break;
    }

    return (USBD_OK);
}

/**
* @brief  CDC_Itf_Control
*         Manage the CDC class requests
* @param  Cmd: Command code
* @param  Buf: Buffer containing command data (request parameters)
* @param  Len: Number of data to be sent (in bytes)
* @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t CDC_Itf_Control_UARTy (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
    switch (cmd)
    {
    case CDC_SEND_ENCAPSULATED_COMMAND:
        /* Add your code here */
        break;

    case CDC_GET_ENCAPSULATED_RESPONSE:
        /* Add your code here */
        break;

    case CDC_SET_COMM_FEATURE:
        /* Add your code here */
        break;

    case CDC_GET_COMM_FEATURE:
        /* Add your code here */
        break;

    case CDC_CLEAR_COMM_FEATURE:
        /* Add your code here */
        break;

    case CDC_SET_LINE_CODING:
        LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                                           (pbuf[2] << 16) | (pbuf[3] << 24));
        LineCoding.format     = pbuf[4];
        LineCoding.paritytype = pbuf[5];
        LineCoding.datatype   = pbuf[6];

        /* Set the new configuration */
        ComPort_Config(&UartHandleY);
		
				// ----- alfran ----- begin -----
				if (LineCoding.bitrate == 1200) 
					{
								// Reset SAMD21
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
								HAL_Delay(100);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
					}
				// ----- alfran ----- end -----
					
        break;

    case CDC_GET_LINE_CODING:
        pbuf[0] = (uint8_t)(LineCoding.bitrate);
        pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
        pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
        pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
        pbuf[4] = LineCoding.format;
        pbuf[5] = LineCoding.paritytype;
        pbuf[6] = LineCoding.datatype;

        /* Add your code here */
        break;

    case CDC_SET_CONTROL_LINE_STATE:
        /* Add your code here */
        break;

    case CDC_SEND_BREAK:
        /* Add your code here */
        break;

    default:
        break;
    }

    return (USBD_OK);
}

/**
* @brief  TIM period elapsed callback
* @param  htim: TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    timer_expired = 1;
}


/**
* @brief  CDC_Itf_DataRx
*         Data received over USB OUT endpoint are sent over CDC interface
*         through this function.
* @param  Buf: Buffer of data to be transmitted
* @param  Len: Number of data received (in bytes)
* @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t CDC_Itf_Receive_UARTx(uint8_t* Buf, uint32_t *Len)
{
    HAL_UART_Transmit_IT(&UartHandleX, Buf, *Len);
    return (USBD_OK);
}

/**
* @brief  CDC_Itf_DataRx
*         Data received over USB OUT endpoint are sent over CDC interface
*         through this function.
* @param  Buf: Buffer of data to be transmitted
* @param  Len: Number of data received (in bytes)
* @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t CDC_Itf_Receive_UARTy(uint8_t* Buf, uint32_t *Len)
{
    HAL_UART_Transmit_IT(&UartHandleY, Buf, *Len);
    return (USBD_OK);
}

/**
* @brief  Tx Transfer completed callback
* @param  huart: UART handle
* @retval None
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Initiate next USB packet transfer once UART completes transfer (transmitting data over Tx line) */
    if (huart->Instance == USARTx)
    {
        USBD_CDC_ReceivePacket(&USBD_Device, 0);
    }
    else
    {
        USBD_CDC_ReceivePacket(&USBD_Device, 1);
    }
}


/**
* @brief  ComPort_Config
*         Configure the COM Port with the parameters received from host.
* @param  None.
* @retval None.
* @note   When a configuration is not supported, a default value is used.
*/
static void ComPort_Config(UART_HandleTypeDef *UartHandle)
{
    if(HAL_UART_DeInit(UartHandle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /* set the Stop bit */
    switch (LineCoding.format)
    {
    case 0:
        UartHandle->Init.StopBits = UART_STOPBITS_1;
        break;
    case 2:
        UartHandle->Init.StopBits = UART_STOPBITS_2;
        break;
    default :
        UartHandle->Init.StopBits = UART_STOPBITS_1;
        break;
    }

    /* set the parity bit*/
    switch (LineCoding.paritytype)
    {
    case 0:
        UartHandle->Init.Parity = UART_PARITY_NONE;
        break;
    case 1:
        UartHandle->Init.Parity = UART_PARITY_ODD;
        break;
    case 2:
        UartHandle->Init.Parity = UART_PARITY_EVEN;
        break;
    default :
        UartHandle->Init.Parity = UART_PARITY_NONE;
        break;
    }

    /*set the data type : only 8bits and 9bits is supported */
    switch (LineCoding.datatype)
    {
    case 0x07:
        /* With this configuration a parity (Even or Odd) must be set */
        UartHandle->Init.WordLength = UART_WORDLENGTH_8B;
        break;
    case 0x08:
        if(UartHandle->Init.Parity == UART_PARITY_NONE)
        {
            UartHandle->Init.WordLength = UART_WORDLENGTH_8B;
        }
        else
        {
            UartHandle->Init.WordLength = UART_WORDLENGTH_9B;
        }

        break;
    default :
        UartHandle->Init.WordLength = UART_WORDLENGTH_8B;
        break;
    }

    UartHandle->Init.BaudRate = LineCoding.bitrate;
    UartHandle->Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UartHandle->Init.Mode       = UART_MODE_TX_RX;

    if(HAL_UART_Init(UartHandle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /* Start reception: provide the buffer pointer with offset and the buffer size */
    if (UartHandle->Instance == USARTx)
    {
        UartHandleX.pRxBuffPtr = (uint8_t*)&UserTxBuffer[0][0];
        UartHandleX.RxXferSize = APP_TX_DATA_SIZE;
        UartHandleX.ErrorCode = HAL_UART_ERROR_NONE;

        HAL_UART_Receive_DMA(&UartHandleX, (uint8_t*)&UserTxBuffer[0][0], APP_TX_DATA_SIZE);
    }
    else
    {
        UartHandleY.pRxBuffPtr = (uint8_t*)&UserTxBuffer[1][0];
        UartHandleY.RxXferSize = APP_TX_DATA_SIZE;
        UartHandleY.ErrorCode = HAL_UART_ERROR_NONE;

        HAL_UART_Receive_DMA(&UartHandleY, (uint8_t*)&UserTxBuffer[1][0], APP_TX_DATA_SIZE);
    }
}

/**
* @brief  TIM_Config: Configure TIMx timer
* @param  None.
* @retval None.
*/
static void TIM_Config(void)
{
    /* Set TIMx instance */
    TimHandle.Instance = TIMx;

    /* Initialize TIMx peripheral as follows:
         + Period = 10000 - 1
         + Prescaler = (SystemCoreClock/10000) - 1
         + ClockDivision = 0
         + Counter direction = Up
    */
    /* TIMx counter clock equal to 10000 Hz, CDC_POLLING_INTERVAL in ms */
    TimHandle.Init.Period = (10000 / (1000 / CDC_POLLING_INTERVAL)) - 1;
    /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
    TimHandle.Init.Prescaler = (uint32_t)(SystemCoreClock / 10000) - 1;
    TimHandle.Init.ClockDivision = 0;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
}

/**
* @brief  UART error callbacks
* @param  UartHandle: UART handle
* @retval None
*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    /* Transfer error occured in reception and/or transmission process */
    Error_Handler();
}

/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
static void Error_Handler(void)
{
    /* Add your own code here */
}

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

