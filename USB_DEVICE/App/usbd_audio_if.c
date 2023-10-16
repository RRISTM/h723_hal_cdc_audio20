/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_audio_if.c
  * @version        : v1.0_Cube
  * @brief          : Generic media access layer.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usbd_audio_if.h"

/* USER CODE BEGIN INCLUDE */
//#include "uhsdr_board.h"
//#include "audio_driver.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_AUDIO_IF
  * @{
  */

/** @defgroup USBD_AUDIO_IF_Private_TypesDefinitions USBD_AUDIO_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Defines USBD_AUDIO_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Macros USBD_AUDIO_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Variables USBD_AUDIO_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN PRIVATE_VARIABLES */
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Exported_Variables USBD_AUDIO_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceHS;

/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_FunctionPrototypes USBD_AUDIO_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t AUDIO_Init_HS(uint32_t AudioFreq, uint32_t Volume, uint32_t options);
static int8_t AUDIO_DeInit_HS(uint32_t options);
static int8_t AUDIO_AudioCmd_HS(uint8_t* pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_VolumeCtl_HS(uint8_t vol);
static int8_t AUDIO_MuteCtl_HS(uint8_t cmd);
static int8_t AUDIO_PeriodicTC_HS(uint8_t *pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_GetState_HS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
#define AUDIO_STATE_INACTIVE 0
typedef  uint32_t USB_AUDIO_MACHINE_State_t;


static USB_AUDIO_MACHINE_State_t AudioState = AUDIO_STATE_INACTIVE;

typedef enum
{
    USB_DIG_AUDIO_OUT_INIT = 0,
    USB_DIG_AUDIO_OUT_IDLE,
    USB_DIG_ADUIO_OUT_TX,
} USB_AUDIO_State_t;

USB_AUDIO_State_t usb_dig_audio_state = USB_DIG_AUDIO_OUT_INIT;


/**
 * @}
 */

/** @defgroup usbd_audio_out_if_Private_Functions
 * @{
 */

#define USB_AUDIO_OUT_NUM_BUF 16
#define USB_AUDIO_OUT_PKT_SIZE   (AUDIO_OUT_PACKET/2)
#define USB_AUDIO_OUT_BUF_SIZE (USB_AUDIO_OUT_NUM_BUF * USB_AUDIO_OUT_PKT_SIZE)

static volatile int16_t out_buffer[USB_AUDIO_OUT_BUF_SIZE]; //buffer for filtered PCM data from Recv.
static volatile uint16_t out_buffer_tail;
static volatile uint16_t out_buffer_head;
static volatile uint16_t out_buffer_overflow;
static volatile uint16_t out_buffer_underflow;

static void audio_out_put_buffer(int16_t sample)
{

    uint32_t next_head = (out_buffer_head + 1) %USB_AUDIO_OUT_BUF_SIZE;

    if (next_head != out_buffer_head)
    {
        out_buffer[out_buffer_head] = sample;
        out_buffer_head = next_head;
    }
    else
    {
        // ok. We loose data now, should never ever happen, but so what
        // will cause minor distortion if only a few bytes.
        out_buffer_overflow++;
    }
}

uint16_t audio_out_buffer_fill()
{
    uint16_t temp_head = out_buffer_head;
    return ((((temp_head < out_buffer_tail)?USB_AUDIO_OUT_BUF_SIZE:0) + temp_head) - out_buffer_tail);
}

volatile int16_t* audio_out_buffer_next_pkt(uint32_t len)
{
    if (audio_out_buffer_fill() >= len)
    {
        return &out_buffer[out_buffer_tail];
    }
    else
    {

        return NULL;
    }
}
static void audio_out_buffer_pop_pkt(volatile int16_t* ptr, uint32_t len)
{
    if (ptr)
    {
        // there was data and pkt has been used
        // free  the space
        out_buffer_tail = (out_buffer_tail+len)%USB_AUDIO_OUT_BUF_SIZE;
    }
}

/* len is length in  stereo  samples */
void UsbdAudio_FillTxBuffer(AudioSample_t *buffer, uint32_t len)
{
    volatile int16_t *pkt = audio_out_buffer_next_pkt(2*len);

    static uint16_t fill_buffer = 1;
    if (fill_buffer == 0 && pkt)
    {
        for (uint32_t idx = len; idx; idx--)
        {
            // the purpose is to place the USB input exactly as the I2S does
            // which is weird if on F4 and 32 Bit transfers are done. (mixed endian)
            // for all other systems we scale 16bit USB audio to 32bit
            // on 16 bit audio nothing at  all happens here.
//            buffer->l = I2S_Int16_2_AudioSample(*pkt++);
//            buffer->r = I2S_Int16_2_AudioSample(*pkt++);
            buffer++;
        }
        audio_out_buffer_pop_pkt(pkt,2*len);
    }
    else
    {
        if (fill_buffer == 0)
        {
            out_buffer_underflow++;
            fill_buffer = 1;
        }
        if (audio_out_buffer_next_pkt((USB_AUDIO_OUT_BUF_SIZE*2)/3) != NULL)
        {
            fill_buffer = 0;
        }
        // Deliver silence if not enough data is stored in buffer
        for (uint32_t idx = len; idx; idx--)
        {
//            buffer->l = 0;
//            buffer->r = 0;
            buffer++;
        }
    }
}

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops_HS =
{
  AUDIO_Init_HS,
  AUDIO_DeInit_HS,
  AUDIO_AudioCmd_HS,
  AUDIO_VolumeCtl_HS,
  AUDIO_MuteCtl_HS,
  AUDIO_PeriodicTC_HS,
  AUDIO_GetState_HS,
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the AUDIO media low layer over the USB HS IP
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  options: Reserved for future use
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_Init_HS(uint32_t AudioFreq, uint32_t Volume, uint32_t options)
{
  /* USER CODE BEGIN 9 */
  UNUSED(AudioFreq);
  UNUSED(Volume);
  UNUSED(options);
  return (USBD_OK);
  /* USER CODE END 9 */
}

/**
  * @brief  DeInitializes the AUDIO media low layer
  * @param  options: Reserved for future use
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_DeInit_HS(uint32_t options)
{
  /* USER CODE BEGIN 10 */
  UNUSED(options);
  return (USBD_OK);
  /* USER CODE END 10 */
}

/**
  * @brief  Handles AUDIO command.
  * @param  pbuf: Pointer to buffer of data to be sent
  * @param  size: Number of data to be sent (in bytes)
  * @param  cmd: Command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_AudioCmd_HS(uint8_t* pbuf, uint32_t size, uint8_t cmd)
{
  /* USER CODE BEGIN 11 */
  switch(cmd)
  {
    case AUDIO_CMD_START:
    break;

    case AUDIO_CMD_PLAY:
    break;
  }
  UNUSED(pbuf);
  UNUSED(size);
  UNUSED(cmd);
  return (USBD_OK);
  /* USER CODE END 11 */
}

/**
  * @brief  Controls AUDIO Volume.
  * @param  vol: volume level (0..100)
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_VolumeCtl_HS(uint8_t vol)
{
  /* USER CODE BEGIN 12 */
  UNUSED(vol);
  return (USBD_OK);
  /* USER CODE END 12 */
}

/**
  * @brief  Controls AUDIO Mute.
  * @param  cmd: command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_MuteCtl_HS(uint8_t cmd)
{
  /* USER CODE BEGIN 13 */
  UNUSED(cmd);
  return (USBD_OK);
  /* USER CODE END 13 */
}

/**
  * @brief  AUDIO_PeriodicTC_HS
  * @param  cmd: command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_PeriodicTC_HS(uint8_t *pbuf, uint32_t size, uint8_t cmd)
{
  /* USER CODE BEGIN 14 */
  UNUSED(pbuf);
  UNUSED(size);
  UNUSED(cmd);
  return (USBD_OK);
  /* USER CODE END 14 */
}

/**
  * @brief  Gets AUDIO state.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_GetState_HS(void)
{
  /* USER CODE BEGIN 15 */
  return (USBD_OK);
  /* USER CODE END 15 */
}

/**
  * @brief  Manages the DMA full transfer complete event.
  * @retval None
  */
void TransferComplete_CallBack_HS(void)
{
  /* USER CODE BEGIN 16 */
  USBD_AUDIO_Sync(&hUsbDeviceHS, AUDIO_OFFSET_FULL);
  /* USER CODE END 16 */
}

/**
  * @brief  Manages the DMA Half transfer complete event.
  * @retval None
  */
void HalfTransfer_CallBack_HS(void)
{
  /* USER CODE BEGIN 17 */
  USBD_AUDIO_Sync(&hUsbDeviceHS, AUDIO_OFFSET_HALF);
  /* USER CODE END 17 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
