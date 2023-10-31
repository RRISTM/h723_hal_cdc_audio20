/**
  ******************************************************************************
  * @file    audio_sessions_usb.h
  * @author  MCD Application Team
  * @version V1.2.0RC3
  * @date    6-January-2018
  * @brief   header file for the audio_session_usb.c file.
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
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AUDIO_SESSIONS_USB_H
#define __AUDIO_SESSIONS_USB_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "audio_node.h"
#include "audio_usb_nodes.h"

/* Exported types ------------------------------------------------------------*/
#ifdef USE_AUDIO_USB_INTERRUPT
typedef enum 
{
  USBD_AUDIO_MUTE_UNMUTE,
  USBD_AUDIO_VOLUME
}AUDIO_ControlCommandTypedef;
#endif /* USE_AUDIO_USB_INTERRUPT*/
typedef struct    AUDIO_USB_StreamingSession
{
  AUDIO_SessionTypeDef session; /* the session structure */
  int8_t               (*SessionDeInit)       (uint32_t /*session handle*/);
#ifdef USE_AUDIO_USB_INTERRUPT
  int8_t               (*ExternalControl)(AUDIO_ControlCommandTypedef /*control*/ , uint32_t /*val*/, uint32_t/*  session_handle*/);
#endif /*USE_AUDIO_USB_INTERRUPT*/
  uint8_t              interface_num; /* interface number for streaming interface */
  uint8_t              alternate; /* alternate number for streaming interface */
  AUDIO_BufferTypeDef  buffer; /* Audio data buffer */
}
AUDIO_USB_SessionTypedef;
 
#ifdef USE_USB_AUDIO_PLAYPBACK
 int8_t  AUDIO_Playback_SessionInit(USBD_AUDIO_AS_InterfaceTypeDef* as_desc,
                                    USBD_AUDIO_ControlTypeDef* controls_desc,
                                    uint8_t* control_count, uint32_t session_handle);
#endif /* USE_USB_AUDIO_PLAYPBACK*/
#ifdef USE_USB_AUDIO_RECORDING
 int8_t  AUDIO_Recording_SessionInit(USBD_AUDIO_AS_InterfaceTypeDef* as_desc,
                                     USBD_AUDIO_ControlTypeDef* controls_desc,
                                     uint8_t* control_count, uint32_t session_handle);
#ifdef  USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO 
 int8_t  AUDIO_Recording_get_Sample_to_add(struct AUDIO_Session* session_handle);
#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO */
#ifdef USE_USB_AUDIO_CLASS_20
#ifdef USE_AUDIO_USB_RECORD_MULTI_FREQUENCES
 int8_t  AUDIO_Recording_SessionSetFrequency(uint32_t freq, uint8_t* as_cnt_to_restart, uint8_t* as_list_to_restart,  uint32_t session_handle);
#endif /*USE_AUDIO_USB_RECORD_MULTI_FREQUENCES*/
#endif /* USE_USB_AUDIO_CLASS_20 */
#endif /* USE_USB_AUDIO_RECORDING*/
#ifdef __cplusplus
}
#endif
#endif  /* __AUDIO_SESSIONS_USB_H */
 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
