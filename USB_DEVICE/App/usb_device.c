/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device
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

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_audio.h"
#include "usbd_audio_if.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceHS;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */
uint8_t cdc_ep[3]={0x81,0x1,0x82};
uint8_t audio_ep[]={0x03};
/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceHS, &HS_Desc, DEVICE_HS) != USBD_OK)
  {
    Error_Handler();
  }

  if (USBD_CDC_RegisterInterface(&hUsbDeviceHS, &USBD_Interface_fops_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClassComposite(&hUsbDeviceHS, &USBD_CDC,CLASS_TYPE_CDC,cdc_ep) != USBD_OK)
  {
    Error_Handler();
  }

  if (USBD_AUDIO_RegisterInterface(&hUsbDeviceHS, &USBD_AUDIO_fops_HS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClassComposite(&hUsbDeviceHS, &USBD_AUDIO,CLASS_TYPE_AUDIO,audio_ep) != USBD_OK)
  {
    Error_Handler();
  }



  // if (USBD_RegisterClass(&hUsbDeviceHS, &USBD_AUDIO) != USBD_OK)
  // {
  //   Error_Handler();
  // }
  // if (USBD_AUDIO_RegisterInterface(&hUsbDeviceHS, &USBD_AUDIO_fops_HS) != USBD_OK)
  // {
  //   Error_Handler();
  // }
  if (USBD_Start(&hUsbDeviceHS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */
  HAL_PWREx_EnableUSBVoltageDetector();

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

