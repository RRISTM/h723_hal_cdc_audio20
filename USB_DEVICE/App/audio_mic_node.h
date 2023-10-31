/**
  ******************************************************************************
  * @file    audio_mic_node.h
  * @author  MCD Application Team
  * @version V1.2.0RC3
  * @date    6-January-2018
  * @brief   header file for the audio_mic_node.c file.
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
#ifndef __AUDIO_DEVICES_NODES_H
#define __AUDIO_DEVICES_NODES_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#ifndef USE_AUDIO_DUMMY_MIC
#include "audio_user_devices.h"
#endif /* USE_AUDIO_DUMMY_MIC */
#include  "audio_node.h"
#include "usb_audio_user.h"

/* Exported constants --------------------------------------------------------*/
#ifdef USE_AUDIO_DUMMY_MIC
#define  AUDIO_MicInit AUDIO_DUMMY_MicInit
#else /* USE_AUDIO_DUMMY_MIC */
#define  AUDIO_MicInit AUDIO_USER_MicInit
#endif /* USE_AUDIO_DUMMY_MIC */
/* Exported types ------------------------------------------------------------*/
/* mic node */
#ifdef USE_AUDIO_DUMMY_MIC
typedef struct
{
  int8_t dummy;
}AUDIO_Mic_SpecificTypeDef;
#endif /* USE_AUDIO_DUMMY_MIC */

typedef struct
{
  AUDIO_NodeTypeDef node;
  AUDIO_BufferTypeDef *buf;
  uint16_t packet_length; /* packet maximal length */
  uint8_t volume;
  int8_t  (*MicDeInit)  (uint32_t /*node_handle*/);
  int8_t  (*MicStart)   (AUDIO_BufferTypeDef* /*buffer*/ , uint32_t /*node handle*/);
  int8_t  (*MicStop)    ( uint32_t /*node handle*/);
  int8_t  (*MicChangeFrequence)    ( uint32_t /*node handle*/);
  int8_t  (*MicMute)     (uint16_t /*channel_number*/, uint8_t /*mute */,uint32_t /*node handle*/);
  int8_t  (*MicSetVolume)    ( uint16_t /*channel_number*/, int /*volume_db_256 */, uint32_t /*node handle*/);
  int8_t  (*MicGetVolumeDefaultsValues)    ( int* /*vol_max*/, int* /*vol_min*/,int* /*vol_res*/, uint32_t /*node handle*/);
#ifdef USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO 
  int8_t  (*MicStartReadCount)     (uint32_t /*node handle*/);
  uint16_t  (*MicGetReadCount)    (  uint32_t /*node handle*/);
#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO*/
  AUDIO_Mic_SpecificTypeDef specific;
}
AUDIO_Mic_NodeTypeDef;

/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#ifdef USE_AUDIO_MEMS_MIC

#endif /* USE_AUDIO_MEMS_MIC */
 int8_t  AUDIO_MicInit(AUDIO_DescriptionTypeDef* audio_description, AUDIO_SessionTypeDef* session_handle,  uint32_t node_handle);

 uint32_t AUDIO_GetPacketLength();
uint32_t AUDIO_SendINData(uint8_t* buffer,uint32_t length);
#ifdef __cplusplus
}
#endif
#endif  /* __AUDIO_DEVICES_NODES_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
