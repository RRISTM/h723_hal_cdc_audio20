/**
  ******************************************************************************
  * @file    audio_usb_playback_session.c
  * @author  MCD Application Team
  * @version V1.2.0RC3
  * @date    6-January-2018
  * @brief   usb audio recording session implementation.
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
#include "audio_mic_node.h"
#include "audio_sessions_usb.h"
#ifdef USE_USB_AUDIO_RECORDING



/* Private defines -----------------------------------------------------------*/
#define AUDIO_USB_RECORDING_ALTERNATE           0x01
#define DEFAULT_VOLUME_DB_256                   0
#ifdef USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO 
#ifdef USE_USB_FS
#define USB_SOF_COUNT_PER_SECOND 1000
#endif /* USE_USB_FS */
#ifdef USE_USB_HS
#define USB_SOF_COUNT_PER_SECOND 8000
#endif /* USE_USB_HS */
#define AUDIO_SYNC_STARTED                      0x01 /* set to 1 when synchro parameters are ready to use */
#define AUDIO_SYNC_NEEDED                       0x02 /* We need to add or remove some samples */
#define AUDIO_SYNC_STABLE                       0x04 /* computed frequency has a good precision */
#define AUDIO_SYNCHRO_FIRST_VALUE_READ          0x08 /* First time we have to read the remaining bytes in dma buffer , this value is used next time to compute number of transferred bytes from */
#define AUDIO_SYNCHRO_OVERRUN_UNDERR_SOON       0x10 /* Flag to detect if overrun or underrun is soon , then one sample is removed or added to each packet*/
#define AUDIO_SYNCHRO_DRIFT_DETECTED            0x40 /* A small drift is detected*/ 
#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO*/

/* Private typedef -----------------------------------------------------------*/
#ifdef USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO
typedef struct 
{
  uint32_t current_frequency;   /* estimated mic frequency */
  float    sample_step;         /* sample portion to add or remove each ms */
  float    sample_frac_sum;     /* used to compute number of sample to add each ms*/
  int      samples;             /* nuber of sample to add in next ms , when it is negative it means to remove samples*/
  uint8_t  status;              /* status of synchronization*/
  uint32_t mic_estimated_freq;  /* estimated frequency of the mic , updated each second = (number of sample read from min withi 1 second */
  uint32_t read_data_by_second; /* total of sample read in one seconde( increamentad each ms)*/
  uint16_t sof_counter;         /* count of SOF packet reception */
  int      mic_usb_diff;        /* compute the diffrence between : total count of samples read from mic - total count of samples wrtten to USB */
  int8_t   last_write_interval; /* compute time in ms from last USB call (write action ) */
  uint16_t packet_size;         /* packet size */
  int8_t   sample_size;         /* size of 1 sample */
  int      sample_per_s_th;     /* thershold to detect that a small drift is observed */
  uint16_t buffer_fill_max_th;  /* if filled bytes count is more than this thershold an overrun is soon */
  uint16_t buffer_fill_min_th;  /* if filled bytes count is less than this thershold an underrun is soon */
  uint16_t buffer_fill_moy;     /* the center value of filled bytes */
}AUDIO_SynchroParams;
#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO*/




/* Private macros ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Record usb session callbacks */
static int8_t  AUDIO_Recording_SessionStart(AUDIO_USB_SessionTypedef *rec_session);
static int8_t  AUDIO_Recording_SessionStop(AUDIO_USB_SessionTypedef *rec_session);
static int8_t  AUDIO_Recording_SessionDeInit(uint32_t session_handle);
static int8_t  AUDIO_Recording_SetAS_Alternate( uint8_t alternate,  uint32_t session_handle);
static int8_t  AUDIO_Recording_GetState(uint32_t session_handle);
static int8_t  AUDIO_Recording_SessionCallback(AUDIO_SessionEventTypeDef  event, 
                                               AUDIO_NodeTypeDef* node_handle, 
                                               struct    AUDIO_Session* session_handle);
#ifdef USE_AUDIO_USB_INTERRUPT
static int8_t  AUDIO_Recording_SessionExternalControl( AUDIO_ControlCommandTypedef control , uint32_t val, uint32_t session_handle);
#endif /*USE_AUDIO_USB_INTERRUPT*/

#ifdef USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO 
static void  AUDIO_Recording_Sof_Received(uint32_t session_handle );
static void AUDIO_Recording_synchro_init(AUDIO_BufferTypeDef *buf, uint32_t packet_length);
static void  AUDIO_Recording_synchro_update(int wr_distance );
#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO*/

/* Private variables ---------------------------------------------------------*/
#ifdef USE_USB_AUDIO_CLASS_20
#ifdef USE_AUDIO_USB_RECORD_MULTI_FREQUENCES
 uint32_t USB_AUDIO_CONFIG_RECORD_FREQENCIES[USB_AUDIO_CONFIG_RECORD_FREQ_COUNT]=
{
#if USB_AUDIO_CONFIG_RECORD_USE_FREQ_192_K
USB_AUDIO_CONFIG_FREQ_192_K,
#endif /* USB_AUDIO_CONFIG_RECORD_USE_FREQ_192_K */
#if USB_AUDIO_CONFIG_RECORD_USE_FREQ_96_K
USB_AUDIO_CONFIG_FREQ_96_K,
#endif /* USB_AUDIO_CONFIG_RECORD_USE_FREQ_96_K */
#if USB_AUDIO_CONFIG_RECORD_USE_FREQ_48_K
USB_AUDIO_CONFIG_FREQ_48_K,
#endif /*USB_AUDIO_CONFIG_RECORD_USE_FREQ_48_K*/
#if USB_AUDIO_CONFIG_RECORD_USE_FREQ_44_1_K
USB_AUDIO_CONFIG_FREQ_44_1_K,
#endif /*USB_AUDIO_CONFIG_RECORD_USE_FREQ_44_1_K*/
#if USB_AUDIO_CONFIG_RECORD_USE_FREQ_16_K
USB_AUDIO_CONFIG_FREQ_16_K,
#endif /*USB_AUDIO_CONFIG_RECORD_USE_FREQ_16_K*/
#ifdef USB_AUDIO_CONFIG_RECORD_USE_FREQ_8_K
USB_AUDIO_CONFIG_FREQ_8_K,
#endif /*USB_AUDIO_CONFIG_RECORD_USE_FREQ_8_K*/
};
#endif /* USE_AUDIO_USB_PLAY_MULTI_FREQUENCES*/
/* list of used nodes */
#ifndef USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC
static AUDIO_USB_ClockSrc_NodeTypeDef recording_clk_source;
#endif /*USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC*/
#endif /* USE_USB_AUDIO_CLASS_20 */
static AUDIO_USB_IO_NodeTypeDef usb_rec_output;
static AUDIO_DescriptionTypeDef record_audio_description;
static AUDIO_USB_CF_NodeTypeDef recording_feature_control;
static AUDIO_Mic_NodeTypeDef mic_input;
#ifdef USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO 
static  AUDIO_SynchroParams syncp; /* synchro parameters*/
#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO*/

/* exported functions ---------------------------------------------------------*/

/**
  * @brief  AUDIO_Recording_SessionInit
  *         Initializes the play (streaming) session
  * @param  as_desc:  audio streaming callbacks 
  * @param  controls_desc: list of control
  * @param  control_count: list of control count
  * @param  session_handle: session
  * @retval  : 0 if no error
  */
 int8_t  AUDIO_Recording_SessionInit(USBD_AUDIO_AS_InterfaceTypeDef* as_desc,  USBD_AUDIO_ControlTypeDef* controls_desc,
                                     uint8_t* control_count, uint32_t session_handle)
{
  AUDIO_USB_SessionTypedef *rec_session;
  AUDIO_ControlDeviceDefaultsTypedef controller_defaults;
#ifdef USE_USB_AUDIO_CLASS_20
#ifndef USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC
  AUDIO_DevicesClockCommandsTypedef clk_src_cmds;
#endif /* USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC*/
#endif /* USE_USB_AUDIO_CLASS_20 */
  
  rec_session = (AUDIO_USB_SessionTypedef*)session_handle;
  memset(rec_session, 0, sizeof(AUDIO_USB_SessionTypedef));
  rec_session->interface_num = USBD_AUDIO_CONFIG_RECORD_SA_INTERFACE;
  rec_session->alternate = 0;

  rec_session->SessionDeInit = AUDIO_Recording_SessionDeInit;
  #ifdef USE_AUDIO_USB_INTERRUPT
   rec_session->ExternalControl = AUDIO_Recording_SessionExternalControl;
#endif /*USE_AUDIO_USB_INTERRUPT*/
  rec_session->session.SessionCallback = AUDIO_Recording_SessionCallback;
  
  /*set audio used option*/
  record_audio_description.audio_res = USBD_AUDIO_CONFIG_RECORD_RES_BYTE;
  record_audio_description.audio_type = USBD_AUDIO_FORMAT_TYPE_PCM;
  record_audio_description.channels_count = USBD_AUDIO_CONFIG_RECORD_CHANNEL_COUNT;
  record_audio_description.channels_map = USBD_AUDIO_CONFIG_RECORD_CHANNEL_MAP; 
  record_audio_description.frequence = USB_AUDIO_CONFIG_RECORD_DEF_FREQ;
  record_audio_description.audio_mute = 0;
  record_audio_description.audio_volume_db_256 = DEFAULT_VOLUME_DB_256;
  *control_count = 0;
  
  /* create list of node */

  /* create mic node */
  AUDIO_MicInit(&record_audio_description, &rec_session->session, (uint32_t)&mic_input);
  rec_session->session.node_list = (AUDIO_NodeTypeDef*)&mic_input;

  /* create record output */
  USB_AUDIO_Streaming_Output_Init(&as_desc->data_ep,  
                                  &record_audio_description,
                                  &rec_session->session,
                                  (uint32_t)&usb_rec_output);
  
   /* create Feature UNIT */
  controller_defaults.audio_description = &record_audio_description;
  mic_input.MicGetVolumeDefaultsValues(&controller_defaults.max_volume,
                                       &controller_defaults.min_volume,
                                       &controller_defaults.res_volume,
                                       (uint32_t)&mic_input);

  USB_AUDIO_Streaming_CF_Init(controls_desc,  &controller_defaults,
                              USB_AUDIO_CONFIG_RECORD_UNIT_FEATURE_ID,
                              (uint32_t)&recording_feature_control);
 (*control_count)++;
  mic_input.node.next = (AUDIO_NodeTypeDef*)&recording_feature_control;
  recording_feature_control.node.next = (AUDIO_NodeTypeDef*)&usb_rec_output;
#ifndef USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC
#ifdef USE_USB_AUDIO_CLASS_20
  /* clock node init */
  clk_src_cmds.private_data = session_handle;
#ifdef USE_AUDIO_USB_RECORD_MULTI_FREQUENCES
  clk_src_cmds.clock_freq_count = USB_AUDIO_CONFIG_RECORD_FREQ_COUNT;
  clk_src_cmds.clock_freq_list = USB_AUDIO_CONFIG_RECORD_FREQENCIES;
  clk_src_cmds.SetFrequency = AUDIO_Recording_SessionSetFrequency;
#else /*  USE_AUDIO_USB_RECORD_MULTI_FREQUENCES */
  clk_src_cmds.clock_freq_count = 1;
  clk_src_cmds.clock_freq_list = &record_audio_description.frequence;
  clk_src_cmds.SetFrequency = 0;
#endif /*  USE_AUDIO_USB_RECORD_MULTI_FREQUENCES */

  USB_AUDIO_Streaming_CLK_SRC_Init(&(controls_desc[*control_count]), &clk_src_cmds,
                                   USB_AUDIO_CONFIG_RECORD_CLOCK_SOURCE_ID,&record_audio_description,
                                   (uint32_t)&recording_clk_source);
 (*control_count)++;
#endif /*USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC*/
#endif /* USE_USB_AUDIO_CLASS_20 */
  
    /* prepare buffer */
  rec_session->buffer.data = malloc(USBD_AUDIO_CONFIG_RECORD_BUFFER_SIZE);
  if(!rec_session->buffer.data)
  {
    Error_Handler();
  }
  /* @TODO optimize the margin value */
  AUDIO_USB_InitializesDataBuffer(&rec_session->buffer, USBD_AUDIO_CONFIG_RECORD_BUFFER_SIZE,
                                   AUDIO_MS_PACKET_SIZE_FROM_AUD_DESC(&record_audio_description) , AUDIO_MS_PACKET_SIZE_FROM_AUD_DESC(&record_audio_description));
  /* set USB AUDIO class callbacks */
  as_desc->interface_num = rec_session->interface_num;
  as_desc->alternate = 0;
  as_desc->max_alternate = AUDIO_USB_RECORDING_ALTERNATE;
#ifdef  USE_AUDIO_PLAYBACK_USB_FEEDBACK
  as_desc->synch_enabled = 0;
#endif /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */
  as_desc->private_data = session_handle;
  as_desc->SetAS_Alternate = AUDIO_Recording_SetAS_Alternate;
  as_desc->GetState = AUDIO_Recording_GetState;
#ifdef USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO
  as_desc->SofReceived = AUDIO_Recording_Sof_Received;
#else /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO */
  as_desc->SofReceived =  0;
#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO */
  rec_session->session.state = AUDIO_SESSION_INITIALIZED;
  return 0;
}

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  AUDIO_Recording_SessionStart
  *         Start the Recording session
  * @param  rec_session: recording session 
  * @retval 0 if no error
  */
static  int8_t  AUDIO_Recording_SessionStart( AUDIO_USB_SessionTypedef* rec_session)
{
  if(( rec_session->session.state == AUDIO_SESSION_INITIALIZED)
       ||(rec_session->session.state == AUDIO_SESSION_STOPPED))
  {
    AUDIO_DevicesCommandsTypedef commands;
    /* start feature control node */
    commands.private_data = (uint32_t)&mic_input;
    commands.SetCurrentVolume = mic_input.MicSetVolume;
    commands.SetMute = mic_input.MicMute;
    rec_session->buffer.rd_ptr = rec_session->buffer.wr_ptr = 0;
#ifdef USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO
    syncp.status = 0;
#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO */

    /* start the mic */
    mic_input.MicStart(&rec_session->buffer, (uint32_t)&mic_input);
    /* start the feature */
    recording_feature_control.CFStart(&commands, (uint32_t)&recording_feature_control);
#ifdef USE_USB_AUDIO_CLASS_20
#ifndef USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC
    recording_clk_source.CSStart((uint32_t)&recording_clk_source);
#endif /* USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC*/
#endif /* USE_USB_AUDIO_CLASS_20 */
    /* start output node */
    usb_rec_output.IOStart(&rec_session->buffer, 0, (uint32_t)&usb_rec_output);
    rec_session->session.state = AUDIO_SESSION_STARTED; 
  }
  return 0;
}

/**
  * @brief  AUDIO_Recording_SessionStop
  *         stop the recording session
  * @param  rec_session: recording session
  * @retval  : 0 if no error
  */
static int8_t  AUDIO_Recording_SessionStop(AUDIO_USB_SessionTypedef *rec_session)
{
  
  if( rec_session->session.state == AUDIO_SESSION_STARTED)
  {
    usb_rec_output.IOStop((uint32_t)&usb_rec_output);
    recording_feature_control.CFStop((uint32_t)&recording_feature_control);
    mic_input.MicStop((uint32_t)&mic_input);
    rec_session->session.state = AUDIO_SESSION_STOPPED;
  }

  return 0;
}

/**
  * @brief  AUDIO_Recording_SessionDeInit
  *         De-Initialize the recording session
  * @param  session_handle: session handle
  * @retval 0 if no error
  */
static int8_t  AUDIO_Recording_SessionDeInit(uint32_t session_handle)
{
  AUDIO_USB_SessionTypedef *rec_session;
  
  rec_session = (AUDIO_USB_SessionTypedef*)session_handle;
  
  if( rec_session->session.state != AUDIO_SESSION_OFF)
  {
    if( rec_session->session.state == AUDIO_SESSION_STARTED)
    {
      AUDIO_Recording_SessionStop( rec_session);
    }
    mic_input.MicDeInit((uint32_t)&mic_input);
#ifdef USE_USB_AUDIO_CLASS_20
#ifndef USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC
    recording_clk_source.CSDeInit((uint32_t)&recording_clk_source);
#endif /*USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC*/
#endif /* USE_USB_AUDIO_CLASS_20 */
    usb_rec_output.IODeInit((uint32_t)&usb_rec_output);
    recording_feature_control.CFDeInit((uint32_t)&recording_feature_control);
    
    if( rec_session->buffer.data)
    {
      free( rec_session->buffer.data);
    }
    rec_session->session.state = AUDIO_SESSION_OFF;
  }

  return 0;
}
/**
  * @brief  AUDIO_Recording_SessionCallback
  *         recording session callback
  * @param  event: event type
  * @param  node: source of event
  * @param  session_handle: session
  * @retval  : 0 if no error
  */
static int underrun_count = 0;
static int overrun_count = 0;
static int8_t  AUDIO_Recording_SessionCallback(AUDIO_SessionEventTypeDef  event, 
                                               AUDIO_NodeTypeDef* node, 
                                               struct    AUDIO_Session* session_handle)
{
   AUDIO_USB_SessionTypedef *rec_session;
  
  rec_session = (AUDIO_USB_SessionTypedef*)session_handle;
  
  switch(event)
  {
     case AUDIO_FREQUENCY_CHANGED: 
    {
      /* recompute the buffer size */
      mic_input.MicChangeFrequence((uint32_t)&mic_input);
#ifdef USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO
       AUDIO_Recording_synchro_init(&rec_session->buffer, usb_rec_output.packet_length);
#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO */
AUDIO_USB_InitializesDataBuffer(&rec_session->buffer, USBD_AUDIO_CONFIG_RECORD_BUFFER_SIZE,
                                AUDIO_MS_PACKET_SIZE_FROM_AUD_DESC(&record_audio_description) ,
                                AUDIO_MS_MAX_PACKET_SIZE_FROM_AUD_DESC(&record_audio_description));
       break;
    }
    case AUDIO_UNDERRUN :
    case AUDIO_OVERRUN :
    {
      if(event == AUDIO_OVERRUN)
      {
        overrun_count++;
      }
      else
      {
        underrun_count++;
      }
          rec_session->buffer.rd_ptr = rec_session->buffer.wr_ptr = 0;
#ifdef USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO
      AUDIO_Recording_synchro_init(&rec_session->buffer, usb_rec_output.packet_length);
      usb_rec_output.IORestart((uint32_t)&usb_rec_output);
#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO */
    }
    break;
#ifdef USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO
  case AUDIO_PACKET_RECEIVED :
    if(++syncp.last_write_interval == 4)
    {
        /* empty the buffer */
        rec_session->buffer.rd_ptr = rec_session->buffer.wr_ptr = 0;
        syncp.status = 0;
        usb_rec_output.IORestart((uint32_t)&usb_rec_output);
        syncp.last_write_interval = 0;
    }
    break;
  case AUDIO_PACKET_PLAYED:
    syncp.last_write_interval = 0;
    break;
  case AUDIO_BEGIN_OF_STREAM:
    AUDIO_Recording_synchro_init(&rec_session->buffer, usb_rec_output.packet_length);

    break;
#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO */
  default : 
    break;
  }
  return 0;
}

/**
  * @brief  AUDIO_Recording_SetAS_Alternate
  *        SA interface set alternate callback
  * @param  alternate:                  
  * @param  session_handle: session
  * @retval  : 0 if no error
  */
static int8_t  AUDIO_Recording_SetAS_Alternate( uint8_t alternate, uint32_t session_handle )
{
  AUDIO_USB_SessionTypedef *rec_session;
  
  rec_session = (AUDIO_USB_SessionTypedef*)session_handle;
  if(alternate  ==  0)
  {
    if(rec_session->alternate != 0)
    {
      AUDIO_Recording_SessionStop(rec_session);
      rec_session->alternate = 0;
    }
  }
  else
  {
    if(rec_session->alternate  ==  0)
    {
      /* @ADD how to define thershold */
      
      AUDIO_Recording_SessionStart(rec_session);
      rec_session->alternate = alternate;
    }
  }
  return 0;
}

/**
  * @brief  AUDIO_Recording_GetState          
  *         recording SA interface status
  * @param  session_handle: session
  * @retval 0 if no error
  */
static int8_t  AUDIO_Recording_GetState(uint32_t session_handle)
{
  return 0;
}
#ifdef USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO
/**
  * @brief  AUDIO_Recording_Sof_Received
  *         Updates the estimated  microphone frequency by computing read samples each second
  *         In addition, it computes the difference between read sample from microphone count  and written sample to USB count 
  * @param  session_handle: session handle
  * @retval None 
  */
 static void  AUDIO_Recording_Sof_Received(uint32_t session_handle )
 {
    AUDIO_USB_SessionTypedef *rec_session;
    uint16_t read_bytes, wr_distance;
    
  rec_session = (AUDIO_USB_SessionTypedef*)session_handle;
  if(( rec_session->session.state == AUDIO_SESSION_STARTED)&&(  syncp.status & AUDIO_SYNC_STARTED))
  {
   if(syncp.status&AUDIO_SYNCHRO_FIRST_VALUE_READ)
   {

      read_bytes = mic_input.MicGetReadCount((uint32_t)&mic_input);
      syncp.read_data_by_second += read_bytes;
       if(++syncp.sof_counter == USB_SOF_COUNT_PER_SECOND)
      {
        syncp.mic_estimated_freq = syncp.read_data_by_second/syncp.sample_size;/* the new estimated frequency */
        syncp.sof_counter = 0;
        syncp.read_data_by_second = 0;
      }
      
      syncp.mic_usb_diff += read_bytes;
      if(syncp.mic_estimated_freq)
      {
        wr_distance = AUDIO_BUFFER_FILLED_SIZE(&rec_session->buffer);
        AUDIO_Recording_synchro_update(wr_distance);
      }
      else
      {
        if(syncp.mic_usb_diff >= syncp.packet_size*4)
        {
          syncp.samples = syncp.sample_size;
        }
        else
        {
          if(syncp.mic_usb_diff + 4*syncp.packet_size <= 0)
          {
            syncp.samples = -syncp.sample_size;
          }
          else
          if((syncp.mic_usb_diff <= syncp.packet_size)&&
             (syncp.mic_usb_diff + syncp.packet_size >= 0))
          {
            syncp.samples = 0;
          }
        }
      }
   }
    else
    {
      mic_input.MicStartReadCount((uint32_t)&mic_input);
      syncp.status |= AUDIO_SYNCHRO_FIRST_VALUE_READ;
    }
  }
 }


/**
  * @brief  AUDIO_Recording_synchro_init
  *         initializes the synchro structure
  * @param  buf: data buffer
  * @param  packet_length: packet length
  * @retval None
  */
static void AUDIO_Recording_synchro_init(AUDIO_BufferTypeDef *buf, uint32_t packet_length)
{
  syncp.packet_size = packet_length;
  syncp.sample_size = AUDIO_SAMPLE_LENGTH(&record_audio_description);
  syncp.buffer_fill_max_th = buf->size*3/4;
  syncp.buffer_fill_min_th = buf->size/4;
  syncp.buffer_fill_moy = buf->size>>1;
#ifdef USE_USB_HS
  syncp.sample_per_s_th = packet_length<<2;
#else 
  syncp.sample_per_s_th = packet_length>>1;
#endif /* USE_USB_HS */
  syncp.current_frequency = record_audio_description.frequence;
  syncp.mic_estimated_freq = 0;
  syncp.last_write_interval = 0;
  syncp.mic_usb_diff = 0;
  syncp.samples = 0;
  syncp.sof_counter = 0;
  syncp.read_data_by_second = 0;
  syncp.status|= AUDIO_SYNC_NEEDED;
  syncp.status = AUDIO_SYNC_STARTED;
}

/**
  * @brief  AUDIO_Recording_synchro_update
  *         update synchronization parameters
  * @param  wr_distance: buffer filled size
  * @retval None
  */
static void  AUDIO_Recording_synchro_update(int wr_distance)
{
    uint8_t update_synchro = 0;
 
   if((syncp.status&AUDIO_SYNCHRO_OVERRUN_UNDERR_SOON)== 0)
   {
     /* NO SOON OVERRUN OR UNDEURRUN DETECETED*/
     if((wr_distance<syncp.buffer_fill_max_th) && (wr_distance>syncp.buffer_fill_min_th))
     {
       /* In this block no risk of buffer overflow or underflow*/
       if((syncp.mic_usb_diff < syncp.sample_per_s_th)
          &&(syncp.mic_usb_diff > (-syncp.sample_per_s_th)))
       {
         /* the sample rate drift is less than limits */
          if((syncp.status&AUDIO_SYNCHRO_DRIFT_DETECTED) == 0)
          {
           if( syncp.mic_estimated_freq != syncp.current_frequency)
           {
             update_synchro = 1;/* frequency value is changed than compute the new frequency */
           }
          }
          else
          {
            /* in last ms a drift was detected then check if this drift is eliminated */
            if(((syncp.mic_usb_diff<=0 )
                 &&( syncp.mic_estimated_freq < syncp.current_frequency))||
               ( (syncp.mic_usb_diff>=0 )
                  &&( syncp.mic_estimated_freq > syncp.current_frequency)))
            {
               update_synchro = 1;
            }
          }
       }
       else
       {
        if(((syncp.mic_usb_diff>0) && ( syncp.mic_estimated_freq > syncp.current_frequency))||
           ((syncp.mic_usb_diff<0) && ( syncp.mic_estimated_freq < syncp.current_frequency)))
        {
           update_synchro = 1;
        }
        else
        {
          if(syncp.mic_usb_diff > 0)
          {
            syncp.current_frequency++;
            syncp.sample_step += (record_audio_description.frequence < syncp.current_frequency)? (float)syncp.sample_size/USB_SOF_COUNT_PER_SECOND:(float)-syncp.sample_size/USB_SOF_COUNT_PER_SECOND;
          }
           if(syncp.mic_usb_diff < 0)
          {
            syncp.current_frequency--;
            syncp.sample_step += (record_audio_description.frequence < syncp.current_frequency)? (float)-syncp.sample_size/USB_SOF_COUNT_PER_SECOND:(float)syncp.sample_size/USB_SOF_COUNT_PER_SECOND;
          }
          syncp.sample_frac_sum = syncp.sample_size;
          syncp.status|= AUDIO_SYNCHRO_DRIFT_DETECTED;
        }
       }
     }
     else
     {
       syncp.samples = (wr_distance>=syncp.buffer_fill_max_th)? syncp.sample_size:-syncp.sample_size;
       syncp.status|= AUDIO_SYNCHRO_OVERRUN_UNDERR_SOON;
     }
   }
   else
   {
     if(((syncp.samples>0)&&(wr_distance>=syncp.buffer_fill_moy))||
        ((syncp.samples<0)&&(wr_distance<=syncp.buffer_fill_moy)))
     {
       update_synchro = 1;
       syncp.status &= ~AUDIO_SYNCHRO_OVERRUN_UNDERR_SOON;
     }
   }
   
   if((syncp.status&AUDIO_SYNCHRO_OVERRUN_UNDERR_SOON) == 0)
   {
     if(update_synchro)
     {
       syncp.current_frequency = syncp.mic_estimated_freq;
       if(record_audio_description.frequence > syncp.current_frequency)
       {
         syncp.sample_step = (float)((record_audio_description.frequence - syncp.current_frequency))*syncp.sample_size/USB_SOF_COUNT_PER_SECOND;
       }
       else
       {
         syncp.sample_step = (float)((syncp.current_frequency- record_audio_description.frequence))*syncp.sample_size/USB_SOF_COUNT_PER_SECOND;
       }
       syncp.status = (AUDIO_SYNC_NEEDED|AUDIO_SYNC_STARTED|AUDIO_SYNCHRO_FIRST_VALUE_READ);
       syncp.sample_frac_sum = 0;
       syncp.samples = 0;
       syncp.mic_usb_diff = 0;
    }
    
    if(syncp.sample_step)
    {
       syncp.sample_frac_sum+=syncp.sample_step;
       if(syncp.sample_frac_sum>syncp.sample_size)
       {
         syncp.samples = (record_audio_description.frequence < syncp.current_frequency)?syncp.sample_size:-syncp.sample_size;
         syncp.sample_frac_sum -= syncp.sample_size;
       }
       else
       {
         syncp.samples = 0;
       }
     }
   }
}

#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO */

#ifdef  USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO 
/**
  * @brief  AUDIO_Recording_get_Sample_to_add
  *         get sample count to add or remove
  * @param  session_handle: data buffer
  * @retval sample to add(positive value) or remove (negative value)
  */
int8_t  AUDIO_Recording_get_Sample_to_add(struct  AUDIO_Session* session_handle)
{
   if(syncp.status&AUDIO_SYNC_STARTED)
   {
     return syncp.samples;
   }
   return 0;
}
/**
  * @brief  AUDIO_Recording_Set_Sample_Written
  *         set last packet written bytes
  * @param  session_handle: session handles
  * @retval bytes : written or readen
  */
 int8_t  AUDIO_Recording_Set_Sample_Written(struct AUDIO_Session* session_handle, uint16_t bytes)
{
   if(syncp.status&AUDIO_SYNC_STARTED)
   {
     syncp.mic_usb_diff -= bytes;
   }
   return 0;
}
#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO */
#ifdef USE_USB_AUDIO_CLASS_20
#ifdef USE_AUDIO_USB_RECORD_MULTI_FREQUENCES
/**
  * @brief  AUDIO_Recording_SessionSetFrequency
  *         Set frequency
  * @param  session_handle: session
  * @retval  : 
  */                                   
 int8_t  AUDIO_Recording_SessionSetFrequency(uint32_t freq, uint8_t* as_cnt_to_restart, uint8_t* as_list_to_restart,  uint32_t session_handle)
{
  AUDIO_USB_SessionTypedef * rec_session = (AUDIO_USB_SessionTypedef *)session_handle;
  record_audio_description.frequence = freq;
  /* recompute the buffer size */
  mic_input.MicChangeFrequence((uint32_t)&mic_input);
#ifdef USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO
  AUDIO_Recording_synchro_init(&rec_session->buffer, usb_rec_output.packet_length);
#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO */
AUDIO_USB_InitializesDataBuffer(&rec_session->buffer, USBD_AUDIO_CONFIG_RECORD_BUFFER_SIZE,
                                AUDIO_MS_PACKET_SIZE_FROM_AUD_DESC(&record_audio_description) ,
                                AUDIO_MS_MAX_PACKET_SIZE_FROM_AUD_DESC(&record_audio_description));
  usb_rec_output.IOChangeFrequency((uint32_t)&usb_rec_output);
  *as_cnt_to_restart = 1;
  as_list_to_restart[0] = rec_session->interface_num;
  return 0;
}
#endif /* USE_AUDIO_USB_RECORD_MULTI_FREQUENCES */
#endif /* USE_USB_AUDIO_CLASS_20 */
#ifdef USE_AUDIO_USB_INTERRUPT
/**
  * @brief  AUDIO_Recording_SessionExternalControl
  *         Mute or Unmute 
  * @param  session_handle: session
  * @retval  : 0 if no error
  */
static int8_t  AUDIO_Recording_SessionExternalControl( AUDIO_ControlCommandTypedef control , uint32_t val, uint32_t session_handle)
{
  AUDIO_USB_SessionTypedef *rec_session;
  USBD_AUDIO_InterruptTypeDef interrupt;
  
   rec_session = (AUDIO_USB_SessionTypedef*)session_handle;
   if( (rec_session->session.state != AUDIO_SESSION_OFF)&&(rec_session->session.state != AUDIO_SESSION_ERROR))
   {
      switch(control)
    {
      case USBD_AUDIO_MUTE_UNMUTE:
      {
        uint8_t mute;
        
        mute = !record_audio_description.audio_mute;
        recording_feature_control.CFSetMute(0,mute, (uint32_t) &recording_feature_control);
        interrupt.type  = USBD_AUDIO_INTERRUPT_INFO_FROM_INTERFACE;
        interrupt.attr = USBD_AUDIO_INTERRUPT_ATTR_CUR;
        interrupt.cs = USBD_AUDIO_FU_MUTE_CONTROL;
        interrupt.cn_mcn = 0;
        interrupt.entity_id = USB_AUDIO_CONFIG_RECORD_UNIT_FEATURE_ID;
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
#endif /* USE_USB_AUDIO_RECORDING*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
