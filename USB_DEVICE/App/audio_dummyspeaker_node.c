/**
  ******************************************************************************
  * @file    audio_dummyspeaker_node.c
  * @author  MCD Application Team
  * @version V1.2.0RC3
  * @date    6-January-2018
  * @brief   Dummy speaker node implementation.
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
#include "audio_speaker_node.h"
#include "usb_audio_user.h"
/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static int8_t  AUDIO_SpeakerDeInit(uint32_t node_handle);
static int8_t  AUDIO_SpeakerStart(AUDIO_BufferTypeDef* buffer, uint32_t node_handle);
static int8_t  AUDIO_SpeakerStop( uint32_t node_handle);
static int8_t  AUDIO_SpeakerChangeFrequence( uint32_t node_handle);
static int8_t  AUDIO_SpeakerMute( uint16_t channel_number,  uint8_t mute , uint32_t node_handle);
static int8_t  AUDIO_SpeakerSetVolume( uint16_t channel_number,  int volume ,  uint32_t node_handle);
static void    AUDIO_SpeakerInitInjectionsParams( AUDIO_Speaker_NodeTypeDef* speaker);
static uint16_t  AUDIO_SpeakerUpdateBuffer(void);
static int8_t  AUDIO_SpeakerStartReadCount( uint32_t node_handle);
static uint16_t AUDIO_SpeakerGetLastReadCount( uint32_t node_handle);

/* Private typedef -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private variables -----------------------------------------------------------*/
static AUDIO_Speaker_NodeTypeDef *current_speaker = 0;
/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  AUDIO_SpeakerInit
  *         Initializes the audio speaker node 
  * @param  audio_description: audio parameters
  * @param  session_handle:   session handle
  * @param  node_handle:      speaker node handle must be allocated
  * @retval 0 if no error
  */
 int8_t  AUDIO_SpeakerInit(AUDIO_DescriptionTypeDef* audio_description,  AUDIO_SessionTypeDef* session_handle, 
                           uint32_t node_handle)
{
  AUDIO_Speaker_NodeTypeDef* speaker;
  
  speaker = (AUDIO_Speaker_NodeTypeDef*)node_handle;
  memset(speaker, 0, sizeof(AUDIO_Speaker_NodeTypeDef));
  speaker->node.type = AUDIO_OUTPUT;
  speaker->node.state = AUDIO_NODE_INITIALIZED;
  speaker->node.session_handle = session_handle;
  speaker->node.audio_description = audio_description;
  AUDIO_SpeakerInitInjectionsParams( speaker);

  /* set callbacks */
  speaker->SpeakerDeInit = AUDIO_SpeakerDeInit;
  speaker->SpeakerStart = AUDIO_SpeakerStart;
  speaker->SpeakerStop = AUDIO_SpeakerStop;
  speaker->SpeakerChangeFrequence = AUDIO_SpeakerChangeFrequence;
  speaker->SpeakerMute = AUDIO_SpeakerMute;
  speaker->SpeakerSetVolume = AUDIO_SpeakerSetVolume;
  speaker->SpeakerStartReadCount = AUDIO_SpeakerStartReadCount;
  speaker->SpeakerGetReadCount = AUDIO_SpeakerGetLastReadCount;
  current_speaker = speaker;
  return 0;
}

#define BUFFER_OUT_SIZE 280*1024
volatile uint8_t storeBuffer[BUFFER_OUT_SIZE];
uint32_t pointer;

uint32_t AUDIO_GetSpeakerData(uint16_t* data){
    if(current_speaker->node.state == AUDIO_NODE_STARTED){
      uint16_t wr_distance = AUDIO_BUFFER_FILLED_SIZE(current_speaker->buf);
      if(wr_distance>192){
        if(pointer<(BUFFER_OUT_SIZE-192)){
          memcpy(&storeBuffer[pointer],(uint16_t*)(current_speaker->buf->data+current_speaker->buf->rd_ptr),current_speaker->packet_length);
          pointer+=current_speaker->packet_length;
        }
        return AUDIO_SpeakerUpdateBuffer();
      }
    }
    return 0;
}


/**
  * @brief  AUDIO_SpeakerUpdateBuffer
  *         read a packet from the buffer.
  * @param  None
  * @retval None
  */
static uint16_t AUDIO_SpeakerUpdateBuffer(void)
{
  uint16_t wr_distance, read_length = 0;
    
  if((current_speaker)&&(current_speaker->node.state != AUDIO_NODE_OFF))
  {


    /* if speaker was started prepare next data */
    if(current_speaker->node.state == AUDIO_NODE_STARTED)
    {

      /* inform session that a packet is played */
      current_speaker->node.session_handle->SessionCallback(AUDIO_PACKET_PLAYED, (AUDIO_NodeTypeDef*)current_speaker, 
                                                            current_speaker->node.session_handle);
      /* prepare next size to inject */
      read_length = current_speaker->packet_length;
#if USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K
      if(current_speaker->node.audio_description->frequence == USB_AUDIO_CONFIG_FREQ_44_1_K)
      {
        if(current_speaker->injection_44_count < 9)
        {
          current_speaker->injection_44_count++;  
        }
        else
        {
           current_speaker->injection_44_count = 0;
           read_length = current_speaker->packet_length_max_44_1;
        }
      }
#endif /* USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K*/
      wr_distance = AUDIO_BUFFER_FILLED_SIZE(current_speaker->buf);
      if(wr_distance < read_length)
      {
        /** inform session that an underrun is happened */
        current_speaker->node.session_handle->SessionCallback(AUDIO_UNDERRUN, (AUDIO_NodeTypeDef*)current_speaker, 
                                                  current_speaker->node.session_handle);
        read_length = 0;
      }
      else
      {     
        /* update read pointer */
        current_speaker->buf->rd_ptr += read_length;
        if(current_speaker->buf->rd_ptr >= current_speaker->buf->size)
        {
          current_speaker->buf->rd_ptr = current_speaker->buf->rd_ptr - current_speaker->buf->size;
        }
      }
    } /* current_speaker->node.state == AUDIO_NODE_STARTED */
  }
  
  return read_length;
}


/* Private functions ---------------------------------------------------------*/
/**
  * @brief  AUDIO_SpeakerDeInit
  *         De-Initializes the audio speaker node 
  * @param  audio_description: audio parameters
  * @param  node_handle: speaker node handle must be initialized
  * @retval  : 0 if no error
  */
static int8_t  AUDIO_SpeakerDeInit(uint32_t node_handle)
{
  /* @TODO implement function */
  AUDIO_Speaker_NodeTypeDef* speaker;
  
  speaker = (AUDIO_Speaker_NodeTypeDef*)node_handle;
  if(speaker->node.state != AUDIO_NODE_OFF)
  {
    speaker->node.state = AUDIO_NODE_OFF;
  }
  current_speaker = 0;
  return 0;
}

/**
  * @brief  AUDIO_SpeakerStart
  *         Start the audio speaker node 
  * @param  buffer:     buffer to use while node is being started
  * @param  node_handle: speaker node handle must be initialized
  * @retval 0 if no error
  */
static int8_t  AUDIO_SpeakerStart(AUDIO_BufferTypeDef* buffer,  uint32_t node_handle)
{
  AUDIO_Speaker_NodeTypeDef* speaker;

  speaker = (AUDIO_Speaker_NodeTypeDef*)node_handle;
  speaker->buf = buffer;
  AUDIO_SpeakerMute( 0,  speaker->node.audio_description->audio_mute , node_handle);
  AUDIO_SpeakerSetVolume( 0,  speaker->node.audio_description->audio_volume_db_256 , node_handle);
  speaker->node.state = AUDIO_NODE_STARTED;
  return 0;
}

 /**
  * @brief  AUDIO_SpeakerStop
  *         Stop speaker node
  * @param  node_handle: speaker node handle must be Started
  * @retval 0 if no error
  */  
static int8_t  AUDIO_SpeakerStop( uint32_t node_handle)
{
  AUDIO_Speaker_NodeTypeDef* speaker;

  speaker = (AUDIO_Speaker_NodeTypeDef*)node_handle;
  speaker->node.state = AUDIO_NODE_STOPPED;

  return 0;
}

 /**
  * @brief  AUDIO_SpeakerChangeFrequence
  *         change frequency then stop speaker node
  * @param  node_handle: speaker node handle must be Started
  * @retval 0 if no error
  */  
static int8_t  AUDIO_SpeakerChangeFrequence( uint32_t node_handle)
{
  AUDIO_Speaker_NodeTypeDef* speaker;

  speaker = (AUDIO_Speaker_NodeTypeDef*)node_handle;
  speaker->node.state = AUDIO_NODE_STOPPED;
  AUDIO_SpeakerInitInjectionsParams(speaker);
  return 0;
}

 /**
  * @brief  AUDIO_SpeakerInitInjectionsParams
  *         Stop speaker node
  * @param  speaker: speaker node handle must be Started
  * @retval 0 if no error
  */
static void  AUDIO_SpeakerInitInjectionsParams( AUDIO_Speaker_NodeTypeDef* speaker)
{
  speaker->packet_length = AUDIO_MS_PACKET_SIZE_FROM_AUD_DESC(speaker->node.audio_description);
#if USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K
  if(speaker->node.audio_description->frequence == USB_AUDIO_CONFIG_FREQ_44_1_K)
  {
    speaker->packet_length_max_44_1 = speaker->packet_length + AUDIO_SAMPLE_LENGTH(speaker->node.audio_description);
  }
#endif /* USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K*/
 }
 /**
  * @brief  AUDIO_SpeakerMute
  *         set Mute value to speaker
  * @param  channel_number: channel number
* @param  mute: mute value (0 : mute , 1 unmute)
  * @param  node_handle: speaker node handle must be Started
  * @retval  : 0 if no error
  */ 
static int8_t  AUDIO_SpeakerMute( uint16_t channel_number,  uint8_t mute , uint32_t node_handle)
{
  AUDIO_Speaker_NodeTypeDef* speaker;

  speaker = (AUDIO_Speaker_NodeTypeDef*)node_handle;
  speaker->node.audio_description->audio_mute = mute;

  return 0;
}

 /**
  * @brief  AUDIO_SpeakerSetVolume
  *         set Volume value to speaker
  * @param  channel_number: channel number
  * @param  volume_db_256:  volume value in db
  * @param  node_handle:    speaker node handle must be Started
  * @retval 0 if no error
  */ 
static int8_t  AUDIO_SpeakerSetVolume( uint16_t channel_number,  int volume_db_256 ,  uint32_t node_handle)
{
  AUDIO_Speaker_NodeTypeDef* speaker;
  
  speaker = (AUDIO_Speaker_NodeTypeDef*)node_handle;
  speaker->node.audio_description->audio_volume_db_256 = volume_db_256;
  
  return 0;
}
 /**
  * @brief  AUDIO_SpeakerStartReadCount
  *         Start a count to compute read bytes from mic each ms 
  * @param  node_handle: mic node handle must be started
  * @retval  : 0 if no error
  */
static int8_t  AUDIO_SpeakerStartReadCount( uint32_t node_handle)
{
    return 0;    
}
 /**
  * @brief  AUDIO_MicGetLastReadCount
  *         read the count of bytes read in last ms
  * @param  node_handle: mic node handle must be started
  * @retval  :  number of read bytes , 0 if  an error
  */    

static uint16_t  AUDIO_SpeakerGetLastReadCount( uint32_t node_handle)
{
  int read_bytes;
  AUDIO_Speaker_NodeTypeDef* speaker;
  
  speaker = (AUDIO_Speaker_NodeTypeDef*)node_handle;
  read_bytes = AUDIO_SpeakerUpdateBuffer()/(speaker->node.audio_description->audio_res);
    
  return read_bytes;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
