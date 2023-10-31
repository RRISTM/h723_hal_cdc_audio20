/**
  ******************************************************************************
  * @file    audio_usb_playback_session.c
  * @author  MCD Application Team
  * @version V1.2.0RC3
  * @date    6-January-2018
  * @brief   usb audio playback session.
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
#include "audio_speaker_node.h"
#include "audio_sessions_usb.h"
#ifdef USE_USB_AUDIO_PLAYPBACK



/* Private defines -----------------------------------------------------------*/
#define AUDIO_USB_PLAYBACK_ALTERNATE 0x01

/* Private typedef -----------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
#ifdef USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC
#ifdef USE_USB_AUDIO_RECORDING
  extern AUDIO_USB_SessionTypedef usb_record_session;
#endif /* USE_USB_AUDIO_RECORDING*/
#endif /* USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC */
/* Private macros ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Play usb session callbacks */
static int8_t  AUDIO_Playback_SessionStart(AUDIO_USB_SessionTypedef* session);
static int8_t  AUDIO_Playback_SessionStop(AUDIO_USB_SessionTypedef* session);
static int8_t  AUDIO_Playback_SessionDeInit(uint32_t session_handle);
#ifdef USE_AUDIO_USB_INTERRUPT
static int8_t  AUDIO_Playback_SessionExternalControl( AUDIO_ControlCommandTypedef control , uint32_t val, uint32_t session_handle);
#endif /*USE_AUDIO_USB_INTERRUPT*/
static int8_t  AUDIO_Playback_SetAS_Alternate( uint8_t alternate,  uint32_t session_handle);
static int8_t  AUDIO_Playback_GetState(uint32_t session_handle);
static int8_t  AUDIO_Playback_SessionCallback(AUDIO_SessionEventTypeDef  event, 
                                               AUDIO_NodeTypeDef* node_handle, 
                                               struct    AUDIO_Session* session_handle);
#ifdef  USE_AUDIO_PLAYBACK_USB_FEEDBACK
static uint32_t   AUDIO_Playback_GetFeedback( uint32_t session_handle );
static void  AUDIO_USB_Session_Sof_Received(uint32_t session_handle );
#endif  /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */
#ifdef USE_USB_AUDIO_CLASS_20
#ifdef USE_AUDIO_USB_PLAY_MULTI_FREQUENCES
static int8_t  AUDIO_Playback_SessionSetFrequency(uint32_t freq, uint8_t* as_cnt_to_restart, uint8_t* as_list_to_restart,  uint32_t session_handle);
#endif /*USE_AUDIO_USB_PLAY_MULTI_FREQUENCES*/
#endif /* USE_USB_AUDIO_CLASS_20 */

/* Private variables ---------------------------------------------------------*/
#ifdef USE_USB_AUDIO_CLASS_20
#ifdef USE_AUDIO_USB_PLAY_MULTI_FREQUENCES
/* declare table of supprted frequencies */
 uint32_t USB_AUDIO_CONFIG_PLAY_FREQENCIES[USB_AUDIO_CONFIG_PLAY_FREQ_COUNT]=
{
#if USB_AUDIO_CONFIG_PLAY_USE_FREQ_192_K
USB_AUDIO_CONFIG_FREQ_192_K,
#endif /* USB_AUDIO_CONFIG_PLAY_USE_FREQ_192_K */
#if USB_AUDIO_CONFIG_PLAY_USE_FREQ_96_K
USB_AUDIO_CONFIG_FREQ_96_K,
#endif /* USB_AUDIO_CONFIG_PLAY_USE_FREQ_96_K */
#if USB_AUDIO_CONFIG_PLAY_USE_FREQ_48_K
USB_AUDIO_CONFIG_FREQ_48_K,
#endif /*USB_AUDIO_CONFIG_PLAY_USE_FREQ_48_K*/
#if USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K
USB_AUDIO_CONFIG_FREQ_44_1_K,
#endif /*USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K*/
#if USB_AUDIO_CONFIG_PLAY_USE_FREQ_16_K
USB_AUDIO_CONFIG_FREQ_16_K,
#endif /*USB_AUDIO_CONFIG_PLAY_USE_FREQ_16_K*/
#if USB_AUDIO_CONFIG_PLAY_USE_FREQ_8_K
USB_AUDIO_CONFIG_FREQ_8_K,
#endif /*USB_AUDIO_CONFIG_PLAY_USE_FREQ_8_K*/
};
#endif /* USE_AUDIO_USB_PLAY_MULTI_FREQUENCES*/
#endif /* USE_USB_AUDIO_CLASS_20 */
/* list of used nodes */
#ifdef USE_USB_AUDIO_CLASS_20
static AUDIO_USB_ClockSrc_NodeTypeDef streaming_play_clk_source;
#endif /* USE_USB_AUDIO_CLASS_20 */
static AUDIO_USB_IO_NodeTypeDef usb_play_input;
static AUDIO_DescriptionTypeDef play_audio_description;
static AUDIO_USB_CF_NodeTypeDef streaming_feature_control;
static AUDIO_Speaker_NodeTypeDef speaker_output;
#ifdef  USE_AUDIO_PLAYBACK_USB_FEEDBACK
/* Playback synchronization : frequency estimation */
static uint8_t sync_first_time_sof = 0;
static uint32_t sync_estimated_freq = 0;
#endif  /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  AUDIO_Playback_SessionInit
  *         Initializes the play (streaming) session
  * @param  as_desc:  audio streaming callbacks 
  * @param  controls_desc: list of control
  * @param  control_count: list of control count
  * @param  session_handle: session
  * @retval  : 0 if no error
  */
 int8_t  AUDIO_Playback_SessionInit(USBD_AUDIO_AS_InterfaceTypeDef* as_desc,  
                                    USBD_AUDIO_ControlTypeDef* controls_desc,
                                    uint8_t* control_count, uint32_t session_handle)
{
  AUDIO_USB_SessionTypedef *play_session;
  AUDIO_ControlDeviceDefaultsTypedef controller_defaults;
#ifdef USE_USB_AUDIO_CLASS_20
  AUDIO_DevicesClockCommandsTypedef clk_src_cmds;
#endif /* USE_USB_AUDIO_CLASS_20 */
  
   play_session = (AUDIO_USB_SessionTypedef*)session_handle;
   memset( play_session, 0, sizeof(AUDIO_USB_SessionTypedef));
  
   play_session->interface_num = USBD_AUDIO_CONFIG_PLAY_SA_INTERFACE;
   play_session->alternate = 0;
   play_session->SessionDeInit = AUDIO_Playback_SessionDeInit;
#ifdef USE_AUDIO_USB_INTERRUPT
   play_session->ExternalControl = AUDIO_Playback_SessionExternalControl;
#endif /*USE_AUDIO_USB_INTERRUPT*/
   play_session->session.SessionCallback = AUDIO_Playback_SessionCallback;
   play_session->buffer.size = USBD_AUDIO_CONFIG_PLAY_BUFFER_SIZE;
   play_session->buffer.data = malloc( USBD_AUDIO_CONFIG_PLAY_BUFFER_SIZE); 
   if(! play_session->buffer.data)
   {
    Error_Handler();
   }
    /*set audio used option*/
  play_audio_description.audio_res = USBD_AUDIO_CONFIG_PLAY_RES_BYTE;
  play_audio_description.audio_type = USBD_AUDIO_FORMAT_TYPE_PCM; /* PCM*/
  play_audio_description.channels_count = USBD_AUDIO_CONFIG_PLAY_CHANNEL_COUNT;
  play_audio_description.channels_map = USBD_AUDIO_CONFIG_PLAY_CHANNEL_MAP; /* Left and Right */
  play_audio_description.frequence = USB_AUDIO_CONFIG_PLAY_DEF_FREQ;
  play_audio_description.audio_volume_db_256 = VOLUME_SPEAKER_DEFAULT_DB_256;
  play_audio_description.audio_mute = 0;
  *control_count = 0;
 
   /* create usb input node */
  USB_AUDIO_Streaming_Input_Init(&as_desc->data_ep,  &play_audio_description,  &play_session->session,  (uint32_t)&usb_play_input);
   play_session->session.node_list = (AUDIO_NodeTypeDef*)&usb_play_input;
  /* initialize usb feature node */
  controller_defaults.audio_description = &play_audio_description;
    /* @TODO tochanges volumes value to right speaker */
  controller_defaults.max_volume = VOLUME_SPEAKER_MAX_DB_256;
  controller_defaults.min_volume = VOLUME_SPEAKER_MIN_DB_256;
  controller_defaults.res_volume = VOLUME_SPEAKER_RES_DB_256;
  USB_AUDIO_Streaming_CF_Init( controls_desc,  &controller_defaults,  USB_AUDIO_CONFIG_PLAY_UNIT_FEATURE_ID, (uint32_t)&streaming_feature_control);
  (*control_count)++;
  
#ifdef USE_USB_AUDIO_CLASS_20
  clk_src_cmds.private_data = session_handle;
#ifdef USE_AUDIO_USB_PLAY_MULTI_FREQUENCES
  clk_src_cmds.clock_freq_count = USB_AUDIO_CONFIG_PLAY_FREQ_COUNT;
  clk_src_cmds.clock_freq_list = USB_AUDIO_CONFIG_PLAY_FREQENCIES;
  clk_src_cmds.SetFrequency = AUDIO_Playback_SessionSetFrequency;
#else /* USE_AUDIO_USB_PLAY_MULTI_FREQUENCES */
  clk_src_cmds.clock_freq_count = 1;
  clk_src_cmds.clock_freq_list = &play_audio_description.frequence;
    clk_src_cmds.SetFrequency = 0;
#endif /* USE_AUDIO_USB_PLAY_MULTI_FREQUENCES */

  USB_AUDIO_Streaming_CLK_SRC_Init(&(controls_desc[1]), &clk_src_cmds,
                                   USB_AUDIO_CONFIG_PLAY_CLOCK_SOURCE_ID,&play_audio_description,
                                   (uint32_t)&streaming_play_clk_source);
(*control_count)++;
#endif /* USE_USB_AUDIO_CLASS_20 */
  usb_play_input.node.next = (AUDIO_NodeTypeDef*)&streaming_feature_control;
  AUDIO_SpeakerInit(&play_audio_description, &play_session->session, (uint32_t)&speaker_output);
  streaming_feature_control.node.next = (AUDIO_NodeTypeDef*)&speaker_output;

/* initializes synchronization setting */
  
#ifdef USE_AUDIO_PLAYBACK_USB_FEEDBACK
     as_desc->synch_enabled = 1;
     as_desc->synch_ep.ep_num = USB_AUDIO_CONFIG_PLAY_EP_SYNC;
     as_desc->synch_ep.GetFeedback = AUDIO_Playback_GetFeedback;
     as_desc->synch_ep.private_data = (uint32_t) play_session;
     as_desc->SofReceived = AUDIO_USB_Session_Sof_Received;
#endif /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */
  /* set USB AUDIO class callbacks */
  as_desc->interface_num =  play_session->interface_num;
  as_desc->alternate = 0;
  as_desc->max_alternate = AUDIO_USB_PLAYBACK_ALTERNATE;
  as_desc->private_data = session_handle;
  as_desc->SetAS_Alternate = AUDIO_Playback_SetAS_Alternate;
  as_desc->GetState = AUDIO_Playback_GetState;

  /* initialize working buffer */
  uint16_t buffer_margin = (usb_play_input.max_packet_length > usb_play_input.packet_length)?usb_play_input.max_packet_length:0;
  AUDIO_USB_InitializesDataBuffer(&play_session->buffer, USBD_AUDIO_CONFIG_PLAY_BUFFER_SIZE,
                                  AUDIO_MS_PACKET_SIZE_FROM_AUD_DESC(&play_audio_description) , buffer_margin);
  play_session->session.state = AUDIO_SESSION_INITIALIZED;

  return 0;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  AUDIO_Playback_SessionStart
  *         Starts  the play (streaming) session
  * @param  play_session:              
  * @retval  : 0 if no error
  */
static int8_t  AUDIO_Playback_SessionStart(AUDIO_USB_SessionTypedef*  play_session)
{
  if(( play_session->session.state == AUDIO_SESSION_INITIALIZED)
     ||(play_session->session.state == AUDIO_SESSION_STOPPED))
  {
        AUDIO_DevicesCommandsTypedef commands;
    /* start input node */
    usb_play_input.IOStart(& play_session->buffer,   play_session->buffer.size/2,  (uint32_t)&usb_play_input);
    commands.private_data = (uint32_t)&speaker_output;
    commands.SetMute = speaker_output.SpeakerMute;
    commands.SetCurrentVolume = speaker_output.SpeakerSetVolume;
#ifdef USE_USB_AUDIO_CLASS_20
    streaming_play_clk_source.CSStart((uint32_t)&streaming_play_clk_source);
#endif /* USE_USB_AUDIO_CLASS_20 */
    streaming_feature_control.CFStart(&commands,(uint32_t)&streaming_feature_control);
    play_session->session.state = AUDIO_SESSION_STARTED;
  }
  
  return 0;
}

/**
  * @brief  AUDIO_Playback_SessionStop
  *         Stop the play (streaming) session
  * @param  play_session:               
  * @retval 0 if no error
  */
static int8_t  AUDIO_Playback_SessionStop(AUDIO_USB_SessionTypedef*  play_session)
{
  
  if( play_session->session.state == AUDIO_SESSION_STARTED)
  {
    usb_play_input.IOStop((uint32_t)&usb_play_input);
    streaming_feature_control.CFStop((uint32_t)&streaming_feature_control);
#ifdef USE_USB_AUDIO_CLASS_20
    streaming_play_clk_source.CSStop((uint32_t)&streaming_play_clk_source);
#endif /* USE_USB_AUDIO_CLASS_20 */
    speaker_output.SpeakerStop((uint32_t)&speaker_output);
    play_session->session.state = AUDIO_SESSION_STOPPED;
  }
  
  return 0;
}

/**
  * @brief  AUDIO_Playback_SessionDeInit
  *         De-Initializes the play (streaming) session
  * @param  session_handle: session handle
  * @retval 0 if no error
  */
static int8_t  AUDIO_Playback_SessionDeInit(uint32_t session_handle)
{
  AUDIO_USB_SessionTypedef* play_session;
  
   play_session = (AUDIO_USB_SessionTypedef*)session_handle;
  if( play_session->session.state != AUDIO_SESSION_OFF)
  {
    if( play_session->session.state == AUDIO_SESSION_STARTED)
    {
      AUDIO_Playback_SessionStop( play_session);
    }
    speaker_output.SpeakerDeInit((uint32_t)&speaker_output);
    streaming_feature_control.CFDeInit((uint32_t)&streaming_feature_control);
#ifdef USE_USB_AUDIO_CLASS_20
    streaming_play_clk_source.CSDeInit((uint32_t)&streaming_play_clk_source);
#endif /* USE_USB_AUDIO_CLASS_20 */
    usb_play_input.IODeInit((uint32_t)&usb_play_input);
    if( play_session->buffer.data)
    {
      free( play_session->buffer.data);
    }
     play_session->session.state = AUDIO_SESSION_OFF;
  }
  return 0;
}

#ifdef USE_AUDIO_USB_INTERRUPT
/**
  * @brief  AUDIO_Playback_SessionExternalControl
  *         Mute or Unmute 
  * @param  session_handle: session
  * @retval  : 0 if no error
  */
static int8_t  AUDIO_Playback_SessionExternalControl( AUDIO_ControlCommandTypedef control , uint32_t val, uint32_t session_handle)
{
  AUDIO_USB_SessionTypedef *play_session;
  USBD_AUDIO_InterruptTypeDef interrupt;
  
   play_session = (AUDIO_USB_SessionTypedef*)session_handle;
   if( (play_session->session.state != AUDIO_SESSION_OFF)&&(play_session->session.state != AUDIO_SESSION_ERROR))
   {
      switch(control)
    {
      case USBD_AUDIO_MUTE_UNMUTE:
      {
        uint8_t mute;
        
        mute = !play_audio_description.audio_mute;
        streaming_feature_control.CFSetMute(0,mute, (uint32_t) &streaming_feature_control);
        interrupt.type  = USBD_AUDIO_INTERRUPT_INFO_FROM_INTERFACE;
        interrupt.attr = USBD_AUDIO_INTERRUPT_ATTR_CUR;
        interrupt.cs = USBD_AUDIO_FU_MUTE_CONTROL;
        interrupt.cn_mcn = 0;
        interrupt.entity_id = USB_AUDIO_CONFIG_PLAY_UNIT_FEATURE_ID;
        interrupt.ep_if_id = 0;/* Audio control interface 0*/
      interrupt.priority = USBD_AUDIO_NORMAL_PRIORITY;
      }
       break;
    default :
      break;
    }
    USBD_AUDIO_SendInterrupt  (&interrupt);
   }
   else
   {
     return -1;
   }
  return 0;
}
#endif /*USE_AUDIO_USB_INTERRUPT*/
/**
  * @brief  AUDIO_Playback_SessionCallback
  *         session callback for the audio playback
  * @param  event:                      
  * @param  node:                       
  * @param  session_handle: session
  * @retval  : 0 if no error
  */
static int8_t  AUDIO_Playback_SessionCallback(AUDIO_SessionEventTypeDef  event, 
                                               AUDIO_NodeTypeDef* node, 
                                               struct    AUDIO_Session* session_handle)
{
  AUDIO_USB_SessionTypedef * play_session = (AUDIO_USB_SessionTypedef *)session_handle;
  
  switch(event)
  {
  case AUDIO_THERSHOLD_REACHED:
    
    if(node->type  ==  AUDIO_INPUT)
    {
      speaker_output.SpeakerStart(& play_session->buffer, (uint32_t)&speaker_output);
#ifdef  USE_AUDIO_PLAYBACK_USB_FEEDBACK
	  sync_first_time_sof =0;
#endif  /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */
    }
    break;
  case AUDIO_PACKET_RECEIVED:
    
    break;
  case AUDIO_FREQUENCY_CHANGED: 
    {
      /* recompute the buffer size */
     speaker_output.SpeakerChangeFrequence((uint32_t)&speaker_output);
     uint16_t buffer_margin = (usb_play_input.max_packet_length > usb_play_input.packet_length)? usb_play_input.max_packet_length:0;
  AUDIO_USB_InitializesDataBuffer(&play_session->buffer, USBD_AUDIO_CONFIG_PLAY_BUFFER_SIZE,
                                  AUDIO_MS_PACKET_SIZE_FROM_AUD_DESC(&play_audio_description) , buffer_margin);
#ifdef  USE_AUDIO_PLAYBACK_USB_FEEDBACK
     sync_first_time_sof =0;
     sync_estimated_freq = 0;
#endif  /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */   
    break;
    }
  case AUDIO_OVERRUN:
  case AUDIO_UNDERRUN:
    {
     /* restart input and stop output */
     speaker_output.SpeakerStop((uint32_t)&speaker_output);
#ifdef  USE_AUDIO_PLAYBACK_USB_FEEDBACK
     sync_first_time_sof =0;
     sync_estimated_freq = 0;
#endif  /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */ 
     if( play_session->session.state == AUDIO_SESSION_STARTED)
     {
       usb_play_input.IORestart((uint32_t)&usb_play_input);
     }
      break;
    }
  default :
   break;
  }
  return 0;
}


/**
  * @brief  AUDIO_Playback_SetAS_Alternate
  *         set AS interface alternate callback
  * @param  alternate:                      
  * @param  session_handle: session
  * @retval  : 0 if no error
  */
static int8_t  AUDIO_Playback_SetAS_Alternate( uint8_t alternate , uint32_t session_handle)
{
  AUDIO_USB_SessionTypedef * play_session;
  
   play_session = (AUDIO_USB_SessionTypedef*)session_handle;
  if(alternate  ==  0)
  {
    if( play_session->alternate != 0)
    {
       AUDIO_Playback_SessionStop(play_session);
       play_session->alternate = 0;
    }
  }
  else
  {
    if( play_session->alternate  ==  0)
    {
      AUDIO_Playback_SessionStart(play_session);
      play_session->alternate = alternate;
    }
  }
  return 0;
}

/**
  * @brief  AUDIO_Playback_GetState            
  *         return AS interface state
  * @param  session_handle: session
  * @retval 0 if no error
  */
static int8_t  AUDIO_Playback_GetState(uint32_t session_handle)
{  

  return 0;
}

#ifdef  USE_AUDIO_PLAYBACK_USB_FEEDBACK

/**
  * @brief  AUDIO_Playback_GetFeedback
  *         get played sample counter
  * @param  session_handle: session
  * @retval  : 0 if buffer ok , 1 if overrun soon , -1 if underrun soon
  */
static uint32_t   AUDIO_Playback_GetFeedback( uint32_t session_handle )
{
 if((speaker_output.node.state == AUDIO_NODE_STARTED))
  {
    if(sync_estimated_freq)
    {
      return sync_estimated_freq ;
    }
    else
    {
     AUDIO_BufferTypeDef *buffer = &((AUDIO_USB_SessionTypedef*)session_handle)->buffer;
     uint32_t wr_distance;
     
     wr_distance=AUDIO_BUFFER_FREE_SIZE(buffer);
     if(wr_distance <= (buffer->size>>2))
     {
       return play_audio_description.frequence - 1000;
     }
     if( wr_distance >= (buffer->size - (buffer->size>>2)))
     {
       return play_audio_description.frequence + 1000;
     }
    }
  }
 return play_audio_description.frequence;
}

/**
  * @brief  AUDIO_USB_Session_Sof_Received
  *         update the rate of audio
  * @param  session_handle: session
  * @retval  : 
  */

static void  AUDIO_USB_Session_Sof_Received(uint32_t session_handle )
 {
   static uint16_t sof_counter = 0;
#ifdef USE_USB_HS
   static uint8_t micro_sof_counter = 0;
#endif /* USE_USB_HS */
   static uint32_t total_received_sub_samples = 0;
    AUDIO_USB_SessionTypedef *session;
    uint16_t read_samples_per_channel ;
    
  session = (AUDIO_USB_SessionTypedef*)session_handle;
  if( session->session.state == AUDIO_SESSION_STARTED) 
  {
   if(sync_first_time_sof)
   {
#ifdef USE_USB_HS
     if(micro_sof_counter !=7)
     {
       micro_sof_counter++;
     }
     else
     {
#endif /* USE_USB_HS */
        read_samples_per_channel = speaker_output.SpeakerGetReadCount((uint32_t)&speaker_output);
        total_received_sub_samples += read_samples_per_channel;
        if(++sof_counter == 1000)
        {
          sync_estimated_freq =((total_received_sub_samples)>>1); 
          sof_counter =0;
          total_received_sub_samples = 0;
        }
#ifdef USE_USB_HS
        micro_sof_counter = 0;
     }
#endif /* USE_USB_HS */
   }
   else
   {
       speaker_output.SpeakerStartReadCount((uint32_t)&speaker_output);
       sof_counter = 0;
#ifdef USE_USB_HS
       micro_sof_counter = 0;
#endif /* USE_USB_HS */
       total_received_sub_samples = 0;
       sync_first_time_sof = 1;
    }
  }
  else
  {
    sync_first_time_sof = 0;
  }
 }
#endif /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */

#ifdef USE_USB_AUDIO_CLASS_20
#ifdef USE_AUDIO_USB_PLAY_MULTI_FREQUENCES
/**
  * @brief  AUDIO_Playback_SessionSetFrequency
  *         Set frequency
  * @param  session_handle: session
  * @retval  : 
  */                                   
static int8_t  AUDIO_Playback_SessionSetFrequency(uint32_t freq, uint8_t* as_cnt_to_restart, uint8_t* as_list_to_restart,  uint32_t session_handle)
{
       AUDIO_USB_SessionTypedef * play_session = (AUDIO_USB_SessionTypedef *)session_handle;
       
      play_audio_description.frequence = freq;
       /* recompute the buffer size */
      speaker_output.SpeakerChangeFrequence((uint32_t)&speaker_output);
      uint16_t buffer_margin = (usb_play_input.max_packet_length > usb_play_input.packet_length)? usb_play_input.max_packet_length:0;
      AUDIO_USB_InitializesDataBuffer(&play_session->buffer, USBD_AUDIO_CONFIG_PLAY_BUFFER_SIZE,
                                  AUDIO_MS_PACKET_SIZE_FROM_AUD_DESC(&play_audio_description) , buffer_margin);
#ifdef  USE_AUDIO_PLAYBACK_USB_FEEDBACK
     sync_first_time_sof =0;
     sync_estimated_freq = 0;
#endif  /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */  
      usb_play_input.IOChangeFrequency((uint32_t)&usb_play_input);
      *as_cnt_to_restart = 0;
#ifdef USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC
#ifdef USE_USB_AUDIO_RECORDING
 AUDIO_Recording_SessionSetFrequency( freq, as_cnt_to_restart, as_list_to_restart,  (uint32_t) &usb_record_session);
#endif /* USE_USB_AUDIO_RECORDING*/
#endif /* USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC */
      if(play_session->session.state == AUDIO_SESSION_STARTED)
      {
        as_list_to_restart[*as_cnt_to_restart] = play_session->interface_num;
        (*as_cnt_to_restart)++;
      }

  return 0;
}
#endif /* USE_AUDIO_USB_PLAY_MULTI_FREQUENCES */
#endif /* USE_USB_AUDIO_CLASS_20 */
  
#endif /*USE_USB_AUDIO_PLAYPBACK*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
