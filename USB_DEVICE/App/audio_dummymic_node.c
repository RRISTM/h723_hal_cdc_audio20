
/**
  ******************************************************************************
  * @file    audio_dummymic_node.c
  * @author  MCD Application Team
  * @version V1.2.0RC3
  * @date    6-January-2018
  * @brief   mic node implementation.
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
#include <string.h>
#include "usbd_audio.h"
#include "audio_mic_node.h"
#include "usb_audio_user.h"



/* Private defines -----------------------------------------------------------*/
#define DUMMY_MIC_VOLUME_RES_DB_256     256 /* 1 db 1 * 256 = 256*/ 
#define DUMMY_MIC_VOLUME_MAX_DB_256     8192 /* 32db == 32*256 = 8192*/
#define DUMMY_MIC_VOLUME_MIN_DB_256     -8192 /* -32db == -32*256 = -8192*/


static AUDIO_Mic_NodeTypeDef *current_mic = 0;
uint32_t micStart=0;
/* Private function prototypes -----------------------------------------------*/
/* list of Mic Callbacks */

static int8_t  AUDIO_MicDeInit(uint32_t node_handle);
static int8_t  AUDIO_MicStart(AUDIO_BufferTypeDef* buffer ,  uint32_t node_handle);
static int8_t  AUDIO_MicStop( uint32_t node_handle);
static int8_t  AUDIO_MicChangeFrequence( uint32_t node_handle);
static int8_t  AUDIO_MicMute(uint16_t channel_number,  uint8_t mute , uint32_t node_handle);
static int8_t  AUDIO_MicSetVolume( uint16_t channel_number,  int volume_db_256 ,  uint32_t node_handle);
static int8_t  AUDIO_MicGetVolumeDefaultsValues( int* vol_max, int* vol_min, int* vol_res, uint32_t node_handle);
/* exported functions ---------------------------------------------------------*/

/**
  * @brief  AUDIO_MicInit
  *         Initializes the audio mic node 
  * @param  audio_description: audio parameters
  * @param  session_handle:   session handle
  * @param  node_handle:      mic node handle must be allocated
  * @retval  : 0 if no error
  */
 int8_t  AUDIO_DUMMY_MicInit(AUDIO_DescriptionTypeDef* audio_description,  AUDIO_SessionTypeDef* session_handle,
                       uint32_t node_handle)
{
   AUDIO_Mic_NodeTypeDef* mic;
  
  mic                             = (AUDIO_Mic_NodeTypeDef*)node_handle;
  memset(mic, 0, sizeof(AUDIO_Mic_NodeTypeDef));
  mic->node.type                  = AUDIO_INPUT;
  mic->node.state                 = AUDIO_NODE_INITIALIZED;
  mic->node.session_handle        = session_handle;
  mic->node.audio_description     = audio_description;
  mic->MicDeInit                  = AUDIO_MicDeInit;
  mic->MicStart                   = AUDIO_MicStart;
  mic->MicStop                    = AUDIO_MicStop;
  mic->MicChangeFrequence         = AUDIO_MicChangeFrequence;
  mic->MicMute                    = AUDIO_MicMute;
  mic->MicSetVolume               = AUDIO_MicSetVolume;
  mic->MicGetVolumeDefaultsValues = AUDIO_MicGetVolumeDefaultsValues;
  mic->volume                     = audio_description->audio_volume_db_256;
  mic->packet_length              = AUDIO_MS_PACKET_SIZE_FROM_AUD_DESC(audio_description);
  /* @TO add init fucntion for MIC here */
    current_mic = mic;
  return 0;
}

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  AUDIO_MicDeInit
  *         De-Initializes the audio mic node 
  * @param  node_handle: mic node handle must be initialized
  * @retval 0 if no error
  */
static int8_t  AUDIO_MicDeInit(uint32_t node_handle)
{
  AUDIO_Mic_NodeTypeDef* mic;
  
  mic = (AUDIO_Mic_NodeTypeDef*)node_handle;
  
  if(mic->node.state != AUDIO_NODE_OFF)
  {
    if(mic->node.state == AUDIO_NODE_STARTED)
    {
      AUDIO_MicStop(node_handle);
    }
    mic->node.state = AUDIO_NODE_OFF;
  }
  
    return 0;
}

/**
  * @brief  AUDIO_MicStart
  *         Start the audio mic node 
  * @param  buffer:      
  * @param  node_handle: mic node handle must be initialized
  * @retval  : 0 if no error
  */
static int8_t  AUDIO_MicStart(AUDIO_BufferTypeDef* buffer ,  uint32_t node_handle)
{
  AUDIO_Mic_NodeTypeDef* mic;
  mic = (AUDIO_Mic_NodeTypeDef*)node_handle;

  if(mic->node.state != AUDIO_NODE_STARTED)
  {
    mic->node.state = AUDIO_NODE_STARTED;
    mic->buf        = buffer;
  }
  micStart=1;
    return 0;
}

/**
  * @brief  AUDIO_MicStop
  *         stop the audio mic node 
  * @param  node_handle: mic node handle must be initialized
  * @retval  : 0 if no error
  */
static int8_t  AUDIO_MicStop( uint32_t node_handle)
{
    
  AUDIO_Mic_NodeTypeDef* mic;
  mic=(AUDIO_Mic_NodeTypeDef*)node_handle;

  if(mic->node.state == AUDIO_NODE_STARTED)
  {
    mic->node.state = AUDIO_NODE_STOPPED;
  }
  micStart=0;
    return 0;
}

/**
  * @brief  AUDIO_MicChangeFrequence
  *         change mic frequence 
  * @param  node_handle: mic node handle must be initialized
  * @retval  : 0 if no error
  */
static int8_t  AUDIO_MicChangeFrequence( uint32_t node_handle)
{
    
  AUDIO_Mic_NodeTypeDef* mic;
  
  mic=(AUDIO_Mic_NodeTypeDef*)node_handle;
  if(mic->node.state == AUDIO_NODE_STARTED)
  {
    AUDIO_MicStop(node_handle);
    AUDIO_MicStart(mic->buf, node_handle);
  }
  
    return 0;
}
/**
  * @brief  AUDIO_MicMute
  *         mute  mic 
  * @param  channel_number:   Channel number to mute
  * @param  mute:  1 to mute , 0 to unmute 
  * @param  node_handle:  mic node handle
  * @retval 0 if no error
  */
static int8_t  AUDIO_MicMute(uint16_t channel_number, uint8_t mute, uint32_t node_handle)
{
  /* @TODO check if really mic is muted */
  /* @TO ADD ihere */
  
  return 0;
}

/**
  * @brief  AUDIO_MicMute
  *         set  mic volume 
  * @param  channel_number: channel number to set volume 
  * @param  volume_db_256:  volume value 
  * @param  node_handle:  mic node handle 
  * @retval  : 0 if no error
  */
static int8_t  AUDIO_MicSetVolume( uint16_t channel_number,  int volume_db_256 ,  uint32_t node_handle)
{
  ((AUDIO_Mic_NodeTypeDef*)node_handle)->volume = volume_db_256;
    
  /* @TO ADD set volume function here */
  
  return 0;
}

/**
  * @brief  AUDIO_MicGetVolumeDefaultsValues
  *         get  mic volume mx min & resolution value  in db
  * @param  vol_max:   
  * @param  volume_db_256:
  * @param  vol_min:
  * @param  node_handle:
  * @retval 0 if no error
  */
static int8_t  AUDIO_MicGetVolumeDefaultsValues( int* vol_max, int* vol_min, int* vol_res, uint32_t node_handle)
{
  *vol_max = DUMMY_MIC_VOLUME_RES_DB_256;
  *vol_min = DUMMY_MIC_VOLUME_MIN_DB_256;
  *vol_res = DUMMY_MIC_VOLUME_RES_DB_256;
  return 0;
}

uint32_t AUDIO_GetPacketLength(){
    if ((micStart==1)&&(current_mic->node.state == AUDIO_NODE_STARTED)){
        if ( AUDIO_BUFFER_FREE_SIZE(current_mic->buf)>current_mic->packet_length)
        {
          return current_mic->packet_length;
        }
    }
      return 0;
}

uint32_t AUDIO_SendINData(uint8_t* buffer,uint32_t length){
  if (current_mic->node.state == AUDIO_NODE_STARTED)
  {
	uint16_t wr_distance ;
    wr_distance = AUDIO_BUFFER_FREE_SIZE(current_mic->buf);
    if (wr_distance <= current_mic->packet_length)
    {
      current_mic->node.session_handle->SessionCallback(AUDIO_OVERRUN, (AUDIO_NodeTypeDef *)current_mic,
                                                        current_mic->node.session_handle);
    }
    memcpy((uint16_t*)(current_mic->buf->data+current_mic->buf->wr_ptr),buffer, current_mic->packet_length);
    /* check for overflow */
    current_mic->buf->wr_ptr += current_mic->packet_length;

    if (current_mic->buf->wr_ptr == current_mic->buf->size)
    {
      current_mic->buf->wr_ptr = 0;
    }
  }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
