/**
  ******************************************************************************
  * @file    usbd_audio_if.c
  * @author  MCD Application Team
  * @version V1.2.0RC3
  * @date    6-January-2018
  * @brief   USB Device Audio interface file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
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
#include "usbd_audio.h"
#include "usb_audio_user.h"
#include "audio_sessions_usb.h"
#include "usbd_audio_if.h"
/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static int8_t  AUDIO_USB_Init(USBD_AUDIO_FunctionDescriptionfTypeDef* audio_function, uint32_t private_data);
static int8_t  AUDIO_USB_DeInit(USBD_AUDIO_FunctionDescriptionfTypeDef* audio_function, uint32_t private_data);
static int8_t  AUDIO_USB_GetState(uint32_t private_data);
static int8_t  AUDIO_USB_GetConfigDesc (uint8_t ** pdata, uint16_t * psize, uint32_t private_data);
/* exported  variable ---------------------------------------------------------*/

 USBD_AUDIO_InterfaceCallbacksfTypeDef audio_class_interface =
 {
   .Init = AUDIO_USB_Init,
   .DeInit = AUDIO_USB_DeInit,
   .GetConfigDesc = AUDIO_USB_GetConfigDesc,
   .GetState = AUDIO_USB_GetState,
   .private_data = 0 
 };

 /* private  variables ---------------------------------------------------------*/
 /* list of used sessions */
 
#ifdef USE_USB_AUDIO_PLAYPBACK
  AUDIO_USB_SessionTypedef usb_play_session;
#endif /* USE_USB_AUDIO_PLAYPBACK*/
 
#ifdef USE_USB_AUDIO_RECORDING
  AUDIO_USB_SessionTypedef usb_record_session;
#endif /* USE_USB_AUDIO_RECORDING*/
 /* private  functions ---------------------------------------------------------*/
 
/**
  * @brief  AUDIO_USB_Init
  *         Initializes the interface
  * @param  audio_function: The audio function description
  * @param  private_data:  for future usage
  * @retval status
  */
static int8_t  AUDIO_USB_Init(USBD_AUDIO_FunctionDescriptionfTypeDef* audio_function , uint32_t private_data)
{
  int i=0, j=0;
  uint8_t control_count = 0;

#ifdef USE_USB_AUDIO_PLAYPBACK
   /* Initializes the USB play session */
  AUDIO_Playback_SessionInit(&audio_function->as_interfaces[i], &(audio_function->controls[i]), &control_count, (uint32_t) &usb_play_session);
  i++;
  j += control_count;
#endif /* USE_USB_AUDIO_PLAYPBACK*/
#ifdef USE_USB_AUDIO_RECORDING 
  /* Initializes the USB record session */
  AUDIO_Recording_SessionInit(&audio_function->as_interfaces[i], &(audio_function->controls[j]), &control_count, (uint32_t) &usb_record_session);
  i++;
  j += control_count;
#endif /* USE_USB_AUDIO_RECORDING*/
  audio_function->as_interfaces_count = i;
  audio_function->control_count = j;
#ifdef USE_AUDIO_USB_INTERRUPT
  audio_function->interrupt_ep_num = USB_AUDIO_CONFIG_INTERRUPT_EP_IN;
#endif /* USE_AUDIO_USB_INTERRUPT */
  return 0;
}

/**
  * @brief  AUDIO_USB_DeInit
  *         De-Initializes the interface
  * @param  audio_function: The audio function description
  * @param  private_data:  for future usage
  * @retval status 0 if no error
  */

static int8_t  AUDIO_USB_DeInit(USBD_AUDIO_FunctionDescriptionfTypeDef* audio_function, uint32_t private_data)
{
  int i=0;
  
#ifdef USE_USB_AUDIO_PLAYPBACK
  usb_play_session.SessionDeInit( (uint32_t) &usb_play_session);
  audio_function->as_interfaces[0].alternate = 0;
  i++;
#endif /* USE_USB_AUDIO_PLAYPBACK*/
#ifdef USE_USB_AUDIO_RECORDING
  usb_record_session.SessionDeInit((uint32_t) &usb_record_session);
  audio_function->as_interfaces[i].alternate = 0;
#endif /* USE_USB_AUDIO_RECORDING*/
  
  return 0;
}

/**
  * @brief  AUDIO_USB_GetState          
  *         This function returns the USB Audio state
  * @param  private_data:  for future usage
  * @retval status
  */
static int8_t  AUDIO_USB_GetState(uint32_t private_data)
{
  return 0;
}

/**
  * @brief  AUDIO_USB_GetConfigDesc
  *         Initializes the interface
  * @param  pdata: the returned configuration descriptor 
  * @param  psize:  configuration descriptor length
  * @param  private_data:  for future usage
  * @retval status
  */
static int8_t  AUDIO_USB_GetConfigDesc (uint8_t ** pdata, uint16_t * psize, uint32_t private_data)
{
//   *psize =  USB_AUDIO_GetConfigDescriptor(pdata);
   while(1);
    return 0;
}
#ifdef USE_AUDIO_USB_INTERRUPT
/**
  * @brief  USBD_AUDIO_ExecuteControl
  *         execute external control
  * @param  func: the returned configuration descriptor 
  * @param  control:  configuration descriptor length
  * @param  val:  new value
  * @param  private_data:  for future usage
  * @retval status
  */
int8_t USBD_AUDIO_ExecuteControl( uint8_t func, AUDIO_ControlCommandTypedef control , uint32_t val , uint32_t private_data)
{
#ifdef USE_USB_AUDIO_PLAYPBACK
  if((func&USBD_AUDIO_PLAYBACK)&&((usb_play_session.session.state != AUDIO_SESSION_OFF)&&
                                  (usb_play_session.session.state != AUDIO_SESSION_ERROR)))
  {
   usb_play_session.ExternalControl(control, val, (uint32_t) &usb_play_session);
  }
#endif /*  USE_USB_AUDIO_PLAYPBACK */
#ifdef USE_USB_AUDIO_RECORDING
  if((func&USBD_AUDIO_RECORD)&&((usb_record_session.session.state != AUDIO_SESSION_OFF)&&
                                  (usb_record_session.session.state != AUDIO_SESSION_ERROR)))
  {
   usb_record_session.ExternalControl(control, val, (uint32_t) &usb_record_session);
  }
#endif /*  USE_USB_AUDIO_RECORDING */
  return 0;
}
#endif /* USE_AUDIO_USB_INTERRUPT*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
