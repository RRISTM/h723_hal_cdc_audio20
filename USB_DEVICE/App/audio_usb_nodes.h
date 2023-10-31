/**
  ******************************************************************************
  * @file    audio_usb_nodes.h
  * @author  MCD Application Team
  * @version V1.2.0RC3
  * @date    6-January-2018
  * @brief   header of audio_usb_nodes.c
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
#ifndef __AUDIO_USB_NODES_H
#define __AUDIO_USB_NODES_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_audio.h"
#include  "audio_node.h"
/* Exported constants --------------------------------------------------------*/
#define AUDIO_MAX_SUPPORTED_CHANNEL_COUNT 2    /* we support sterio audio channels */
#define AUDIO_IO_BEGIN_OF_STREAM          0x01 /* Begin of stream flag */
#define AUDIO_IO_BEGIN_OF_READ            0x02
#define AUDIO_IO_RESTART_REQUIRED         0x40 /* Restart of node is required , after frequency changes for exampels */
#define AUDIO_IO_THERSHOLD_REACHED        0x08 /* flag that buffer fill thershold is reached */ 

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    uint16_t thershold; /* after star when received size reach thershold and event is raised to play audio*/
}AUDIO_USB_Input_SpecifcTypeDef;

typedef struct
{
    uint8_t* alt_buff;/* buffer_tosend_when_no_data_prepared*/
#if USB_AUDIO_CONFIG_RECORD_USE_FREQ_44_1_K
    uint8_t packet_44_counter;
#endif /* USB_AUDIO_CONFIG_RECORD_FREQ_44_1_K */
}AUDIO_USB_Output_SpecifcTypeDef;

typedef struct
{
  AUDIO_NodeTypeDef    node; /* generic node structure , must be first field */
  uint8_t 			   flags;/* flags for USB input/output status */
  AUDIO_BufferTypeDef* buf; /* buffer to use */
  uint16_t             max_packet_length; /* the packet to read each time from buffer */
  uint16_t             packet_length; /* the packet normallength */
  int8_t  (*IODeInit) (uint32_t /*node_handle*/);
  int8_t  (*IOStart) (AUDIO_BufferTypeDef* buffer, uint16_t thershold, uint32_t /*node handle*/);
  int8_t  (*IORestart) ( uint32_t /*node handle*/);
  int8_t  (*IOStop) ( uint32_t /*node handle*/);
#ifdef USE_USB_AUDIO_CLASS_20
#if (defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCES)
      int8_t  (*IOChangeFrequency) (uint32_t /*node handle*/);
#endif /*(defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCES)*/
#endif /*USE_USB_AUDIO_CLASS_20*/
  union
  {
    AUDIO_USB_Input_SpecifcTypeDef  input;
    AUDIO_USB_Output_SpecifcTypeDef output;    
  }specific; 
}
AUDIO_USB_IO_NodeTypeDef;

/* control feature types */
typedef struct 
{
  int8_t  (*SetMute)    (uint16_t /*channel_number*/,uint8_t /*mute*/, uint32_t /*private_data*/);
  int8_t  (*SetCurrentVolume)    (uint16_t /*channel_number*/, int /*volume_db_256 */, uint32_t /*private_data*/);
  uint32_t private_data;
} AUDIO_DevicesCommandsTypedef;
/* control feature types */

typedef struct 
{
  int                       max_volume; /* volume max on USB format */
  int                       min_volume; /* volume min on USB format */
  int                       res_volume; /* volume resolution on USB format */
  AUDIO_DescriptionTypeDef* audio_description;/* audio description (frequency , bitdepth , ...) */
} AUDIO_ControlDeviceDefaultsTypedef;

/* feature unit node */

typedef struct
{
  AUDIO_NodeTypeDef node;                       /* generic node structure , must be first field */
  uint8_t unit_id;                              /* UNIT ID for usb audio function description and control*/
  USBD_AUDIO_FeatureControlCallbacksTypeDef usb_control_callbacks;      /* list of callbacks */
  AUDIO_DevicesCommandsTypedef control_cbks;                            /* */
  int8_t  (*CFInit)    (USBD_AUDIO_ControlTypeDef* /*control*/  ,
                        AUDIO_ControlDeviceDefaultsTypedef* /*audio_defaults*/,
                        uint8_t /*unit_id*/,  
                        uint32_t /*node_handle*/);
  int8_t  (*CFDeInit)  (uint32_t /*node_handle*/);
  int8_t  (*CFStart)   (  AUDIO_DevicesCommandsTypedef* /* commands */, uint32_t /*node handle*/);
  int8_t  (*CFStop)    (uint32_t /*node handle*/);
  int8_t  (*CFSetMute)    (uint16_t /*channel*/,uint8_t /*mute*/, uint32_t /* node handle*/);
}
AUDIO_USB_CF_NodeTypeDef;
#ifdef USE_USB_AUDIO_CLASS_20
/* Clock Source Entity */
/* Clock callbacks*/
typedef struct 
{
  int8_t  (*SetFrequency)    (uint32_t /*freq*/, uint8_t* /*as_cnt_to_restart*/ ,uint8_t* /*as_list_to_restart*/ ,uint32_t /*private_data*/);
  uint32_t* clock_freq_list;
  uint16_t clock_freq_count;
  uint32_t private_data;
} AUDIO_DevicesClockCommandsTypedef;

typedef struct
{
  AUDIO_NodeTypeDef node;        /* generic node structure , must be first field */
  uint8_t clock_source_id;      /* Clock_Source ID ID for usb audio function description and control*/

  USBD_AUDIO_ClockSourceCallbacksTypeDef usb_control_callbacks;      /* list of callbacks */
  AUDIO_DevicesClockCommandsTypedef control_cbks;                            /* */
  int8_t  (*CSInit)    (USBD_AUDIO_ControlTypeDef* /*control*/  ,
                        AUDIO_DevicesClockCommandsTypedef* /*clk_cmds*/,
                        uint8_t /*unit_id*/,
                        AUDIO_DescriptionTypeDef* /*audio_description*/,
                        uint32_t /*node_handle*/);
  int8_t  (*CSDeInit)  (uint32_t /*node_handle*/);
  int8_t  (*CSStart)   (uint32_t /*node handle*/);
  int8_t  (*CSStop)    (uint32_t /*node handle*/);
}
AUDIO_USB_ClockSrc_NodeTypeDef;
#endif /* USE_USB_AUDIO_CLASS_20 */
/* Exported macros -----------------------------------------------------------*/ 
#define VOLUME_USB_TO_DB_256(v_db, v_usb) (v_db) = (v_usb <= 0x7FFF)? v_usb:  - (((int)0xFFFF - v_usb)+1)
#define VOLUME_DB_256_TO_USB(v_usb, v_db) (v_usb) = (v_db >= 0)? v_db : ((int)0xFFFF+v_db) +1   
#define AUDIO_MAX_PACKET_WITH_FEEDBACK_LENGTH(audio_desc) AUDIO_USB_MAX_PACKET_SIZE((audio_desc)->frequence + 1, (audio_desc)->channels_count, (audio_desc)->audio_res)

/* Exported functions ------------------------------------------------------- */
#ifdef USE_USB_AUDIO_PLAYPBACK
int8_t  USB_AUDIO_Streaming_Input_Init(USBD_AUDIO_EP_DataTypeDef* data_ep,
                                              AUDIO_DescriptionTypeDef* audio_desc,
                                              AUDIO_SessionTypeDef* session_handle,  uint32_t node_handle);
#endif /* USE_USB_AUDIO_PLAYPBACK*/
#ifdef USE_USB_AUDIO_RECORDING
int8_t  USB_AUDIO_Streaming_Output_Init(USBD_AUDIO_EP_DataTypeDef* data_ep,
                                               AUDIO_DescriptionTypeDef* audio_desc,
                                               AUDIO_SessionTypeDef* session_handle,  uint32_t node_handle);
 int8_t  AUDIO_Recording_get_Sample_to_add(struct AUDIO_Session* session_handle);
 int8_t  AUDIO_Recording_Set_Sample_Written(struct AUDIO_Session* session_handle, uint16_t bytes);
#endif /* USE_USB_AUDIO_RECORDING*/
int8_t USB_AUDIO_Streaming_CF_Init(USBD_AUDIO_ControlTypeDef* usb_control_feature,
                                   AUDIO_ControlDeviceDefaultsTypedef* audio_defaults, uint8_t unit_id,
                                   uint32_t node_handle);
void AUDIO_USB_InitializesDataBuffer(AUDIO_BufferTypeDef* buf, uint32_t buffer_size, 
                                     uint16_t packet_size, uint16_t margin);
/* UAC 2.0 specific functions */
#ifdef USE_USB_AUDIO_CLASS_20
int8_t USB_AUDIO_Streaming_CLK_SRC_Init(USBD_AUDIO_ControlTypeDef* usb_control_feature  ,
                                         AUDIO_DevicesClockCommandsTypedef* clk_cmds,
                                         uint8_t        clock_src_id,
                                         AUDIO_DescriptionTypeDef* audio_description,
                                         uint32_t node_handle);
#endif /* USE_USB_AUDIO_CLASS_20 */

#ifdef __cplusplus
}
#endif

#endif  /* __AUDIO_USB_NODES_H */
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
