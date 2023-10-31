/**
  ******************************************************************************
  * @file    audio_speaker_node.h
  * @author  MCD Application Team
  * @version V1.2.0RC3
  * @date    6-January-2018
  * @brief   header file for the audio_speaker_node.c file.
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
#ifndef __AUDIO_SPEAKER_NODES_H
#define __AUDIO_SPEAKER_NODES_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#ifndef USE_AUDIO_SPEAKER_DUMMY
#include "audio_user_devices.h"
#endif /* USE_AUDIO_SPEAKER_DUMMY */
#include  "audio_node.h"
#include "usb_audio_user.h"

/* Exported constants --------------------------------------------------------*/
#define VOLUME_SPEAKER_RES_DB_256       128     /* 0.5 db 0.5 * 256 = 256*/ 
#define VOLUME_SPEAKER_DEFAULT_DB_256   0       /* 0 db*/ 
#define VOLUME_SPEAKER_MAX_DB_256       1536    /* 6db == 6*256 = 1536*/
#define VOLUME_SPEAKER_MIN_DB_256       -6400   /* -25db == -25*256 = -6400*/
#ifdef USE_AUDIO_TIMER_VOLUME_CTRL
#define SPEAKER_CMD_CHANGE_VOLUME  0x10
#define SPEAKER_CMD_MUTE_UNMUTE    0x20
#define SPEAKER_CMD_MUTE_FIRST     0x40
#endif /* USE_AUDIO_TIMER_VOLUME_CTRL */

#ifdef USE_AUDIO_SPEAKER_DUMMY
#define  AUDIO_SpeakerInit AUDIO_SPEAKER_DUMMY_Init
#else /* USE_AUDIO_SPEAKER_DUMMY */
#define  AUDIO_SpeakerInit AUDIO_SPEAKER_USER_Init
#endif /* USE_AUDIO_SPEAKER_DUMMY */
/* Exported types ------------------------------------------------------------*/
/* mic node */
#ifdef USE_AUDIO_SPEAKER_DUMMY
typedef struct
{
  int8_t dummy;
}AUDIO_Speaker_SpecificTypeDef;
#endif /* USE_AUDIO_SPEAKER_DUMMY */

/* speaker node */
typedef struct
{
  AUDIO_NodeTypeDef      node;            /* the structure of generic node*/
  AUDIO_BufferTypeDef*   buf;             /* the audio data buffer*/
  uint16_t               packet_length;   /* packet maximal length */
#if USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K
  uint16_t               packet_length_max_44_1; /* packet maximal length for frequence 44_1 */
  uint8_t                injection_44_count;     /* used as count to inject 9 packets (44 samples) then 1 packet(45 samples)*/
  uint8_t                injection_45_pos;     /* used as position of  45 samples packet*/
#endif /* USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K*/
  int8_t                (*SpeakerDeInit)  (uint32_t /*node_handle*/);
  int8_t                (*SpeakerStart)   (AUDIO_BufferTypeDef* /*buffer*/, uint32_t /*node handle*/);
  int8_t                (*SpeakerStop)    ( uint32_t /*node handle*/);
  int8_t                (*SpeakerChangeFrequence)    ( uint32_t /*node handle*/);
  int8_t                (*SpeakerMute)    (uint16_t /*channel_number*/, uint8_t /*mute */,uint32_t /*node handle*/);
  int8_t                (*SpeakerSetVolume)    ( uint16_t /*channel_number*/, int /*volume_db_256 */, uint32_t /*node handle*/);
  int8_t                (*SpeakerStartReadCount)     (uint32_t /*node handle*/);
  uint16_t              (*SpeakerGetReadCount)    (  uint32_t /*node handle*/);
  AUDIO_Speaker_SpecificTypeDef specific; /*should be defined by user for user speaker */
}
AUDIO_Speaker_NodeTypeDef;

/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
 int8_t  AUDIO_SpeakerInit(AUDIO_DescriptionTypeDef* audio_description,
                           AUDIO_SessionTypeDef* session_handle,
                           uint32_t node_handle);
#ifdef __cplusplus
}
#endif
#endif  /* __AUDIO_SPEAKER_NODES_H */
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
