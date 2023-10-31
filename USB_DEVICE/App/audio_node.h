/**
  ******************************************************************************
  * @file    audio_node.h
  * @author  MCD Application Team
  * @version V1.2.0RC3
  * @date    6-January-2018
  * @brief   Define audio nodes 
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
#ifndef __AUDIO_NODE_H
#define __AUDIO_NODE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h> 

/* Exported Constantes ------------------------------------------------------------------*/
#define AUDIO_BUFFER_UNDERFLOW_THERSHOLD 0x01
#define AUDIO_BUFFER_OVERFLOW_THERSHOLD  0x02
#define AUDIO_BUFFER_OVERFLOW  0x04
#define AUDIO_BUFFER_UNDERFLOW  0x04
#define AUDIO_BUF_OVERFLOW_THERSHOLD 100
#define AUDIO_BUF_UNDERFLOW_THERSHOLD 100

/* Exported types ------------------------------------------------------------------*/  
typedef struct
{
  uint8_t                   buffer_flags;
  uint8_t*                   data; 
  uint16_t                   rd_ptr;  
  uint16_t                   wr_ptr;
  uint16_t                   size;
}
AUDIO_BufferTypeDef;

/* Node state */
typedef enum 
{
   AUDIO_NODE_OFF,
   AUDIO_NODE_INITIALIZED,
   AUDIO_NODE_STARTED,
   AUDIO_NODE_STOPPED,
   AUDIO_NODE_ERROR
}
 AUDIO_NodeStateTypeDef;

/* description of accepted audio */
typedef struct
{
  uint32_t           frequence; 
  uint8_t            channels_count;  
  uint16_t           channels_map;
  uint16_t           audio_type;
  int                audio_volume_db_256;
  uint8_t            audio_mute;
  uint8_t            audio_res;   
}AUDIO_DescriptionTypeDef;

/* Node type */
typedef enum 
{
  AUDIO_INPUT,
  AUDIO_OUTPUT,
#ifdef USE_USB_AUDIO_CLASS_20
  AUDIO_CLOCK,
#endif /* USE_USB_AUDIO_CLASS_20 */
  AUDIO_CONTROL,
  AUDIO_PROCESSING
}
AUDIO_NodeTypeTypeDef;

/* Definition of audio node */
typedef struct    AUDIO_Node
{
 
  AUDIO_NodeStateTypeDef    state; 
  AUDIO_DescriptionTypeDef* audio_description;
  AUDIO_NodeTypeTypeDef     type;
  struct AUDIO_Session*     session_handle;
  struct AUDIO_Node*        next;
}
AUDIO_NodeTypeDef;

/* Events raised by nodes to session */
typedef enum 
{
  AUDIO_THERSHOLD_REACHED,
  AUDIO_BEGIN_OF_STREAM,
  AUDIO_PACKET_RECEIVED,
  AUDIO_PACKET_PLAYED,
  AUDIO_OVERRUN,
  AUDIO_UNDERRUN,
  AUDIO_OVERRUN_TH_REACHED,
  AUDIO_UNDERRUN_TH_REACHED,
  AUDIO_FREQUENCY_CHANGED
} AUDIO_SessionEventTypeDef;
/* Node state */
typedef enum 
{
   AUDIO_SESSION_OFF,
   AUDIO_SESSION_INITIALIZED,
   AUDIO_SESSION_STARTED,
   AUDIO_SESSION_STOPPED,
   AUDIO_SESSION_ERROR
}
 AUDIO_SessionStateTypeDef;
typedef struct    AUDIO_Session
{
  AUDIO_NodeTypeDef * node_list;
  AUDIO_SessionStateTypeDef state;
  int8_t  (*SessionCallback) (AUDIO_SessionEventTypeDef /* event*/ ,
                              AUDIO_NodeTypeDef* /*node_handle*/,
                              struct    AUDIO_Session* /*session handle*/);
}
AUDIO_SessionTypeDef;

/* Exported macros -----------------------------------------------------------*/ 
#define AUDIO_BUFFER_FREE_SIZE(buff)  (((buff)->wr_ptr>=(buff)->rd_ptr)?(buff)->rd_ptr +(buff)->size -(buff)->wr_ptr : \
                                                                          (buff)->rd_ptr -(buff)->wr_ptr)
#define AUDIO_BUFFER_FILLED_SIZE(buff)  (((buff)->wr_ptr>= (buff)->rd_ptr)?(buff)->wr_ptr -(buff)->rd_ptr : \
(buff)->wr_ptr +(buff)->size -(buff)->rd_ptr)


/* compute one packet size */
#define AUDIO_MS_PACKET_SIZE(freq,channel_count,res_byte) (((uint32_t)((freq) /1000))* (channel_count) * (res_byte)) 
#define AUDIO_MS_MAX_PACKET_SIZE(freq,channel_count,res_byte) AUDIO_MS_PACKET_SIZE(freq+999,channel_count,res_byte)

#ifdef USE_USB_HS
//#define AUDIO_USB_PACKET_SIZE(freq,channel_count,res_byte) (((uint32_t)((freq) /8000))* (channel_count) * (res_byte))
//#define AUDIO_USB_MAX_PACKET_SIZE(freq,channel_count,res_byte) AUDIO_USB_PACKET_SIZE(freq+7999,channel_count,res_byte)
#define AUDIO_USB_PACKET_SIZE(freq,channel_count,res_byte) (((uint32_t)((freq) /1000))* (channel_count) * (res_byte)) 
#define AUDIO_USB_MAX_PACKET_SIZE(freq,channel_count,res_byte) AUDIO_USB_PACKET_SIZE(freq+999,channel_count,res_byte)
#else /* USE_USB_HS */
#endif /* USE_USB_HS */
#define AUDIO_USB_PACKET_SIZE_FROM_AUD_DESC(audio_desc) AUDIO_USB_PACKET_SIZE((audio_desc)->frequence, (audio_desc)->channels_count, (audio_desc)->audio_res)
#define AUDIO_USB_MAX_PACKET_SIZE_FROM_AUD_DESC(audio_desc) AUDIO_USB_MAX_PACKET_SIZE((audio_desc)->frequence, (audio_desc)->channels_count, (audio_desc)->audio_res)
#define AUDIO_MS_PACKET_SIZE_FROM_AUD_DESC(audio_desc) AUDIO_MS_PACKET_SIZE((audio_desc)->frequence, (audio_desc)->channels_count, (audio_desc)->audio_res)
#define AUDIO_MS_MAX_PACKET_SIZE_FROM_AUD_DESC(audio_desc) AUDIO_MS_PACKET_SIZE((audio_desc)->frequence + 999, (audio_desc)->channels_count, (audio_desc)->audio_res)
#define AUDIO_PACKET_SIZE(freq,channel_count,res_byte) (((uint32_t)((freq) /1000))* (channel_count) * (res_byte))
   /* compute 1 sample length */
#define AUDIO_SAMPLE_LENGTH(audio_desc) ( (audio_desc)->channels_count*(audio_desc)->audio_res)

#ifdef __cplusplus
}
#endif
#endif  /* __AUDIO_NODE_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
