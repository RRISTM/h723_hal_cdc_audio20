/**
  ******************************************************************************
  * @file    audio_usb_nodes.c
  * @author  MCD Application Team
  * @version V1.2.0RC3
  * @date    6-January-2018
  * @brief   Usb input output and Feature unit implementation.
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
#include "usb_audio_user.h"
#include "audio_usb_nodes.h"

/* External variables --------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#ifdef USE_USB_AUDIO_CLASS_10
#if (defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCES)
#if USBD_SUPPORT_AUDIO_MULTI_FREQUENCES
#define USE_AUDIO_USB_MULTI_FREQUENCES 1
#else /* USBD_SUPPORT_AUDIO_MULTI_FREQUENCES */
#error "USBD_SUPPORT_AUDIO_MULTI_FREQUENCES must be defined to support multi-frequencies"
#endif /* USBD_SUPPORT_AUDIO_MULTI_FREQUENCES */
#endif /*(defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCES) */
#endif /* USE_USB_AUDIO_CLASS_10 */

//#define DEBUG_USB_NODES 1  /* uncomment to debug USB input for playback */
#ifdef DEBUG_USB_NODES
#define USB_INPUT_NODE_DEBUG_BUFFER_SIZE 1000
/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  uint32_t time;
  uint16_t write;
  uint16_t read;
  uint16_t error;
} AUDIO_USB_Input_Buffer_StatsTypeDef;
#endif /* DEBUG_USB_NODES */

/* Private function prototypes -----------------------------------------------*/
static int8_t     USB_AUDIO_Streaming_IO_DeInit(uint32_t node_handle);
static int8_t     USB_AUDIO_Streaming_IO_Start( AUDIO_BufferTypeDef* buffer, uint16_t thershold ,uint32_t node_handle);
static int8_t     USB_AUDIO_Streaming_IO_Stop( uint32_t node_handle);
static uint16_t   USB_AUDIO_Streaming_IO_GetMaxPacketLength(uint32_t node_handle);
#ifdef USE_USB_AUDIO_CLASS_10
static int8_t     USB_AUDIO_Streaming_IO_GetState(uint32_t node_handle);
#endif /*USE_USB_AUDIO_CLASS_10*/
static int8_t     USB_AUDIO_Streaming_IO_Restart( uint32_t node_handle);
#ifdef USE_USB_AUDIO_PLAYPBACK
static int8_t     USB_AUDIO_Streaming_Input_DataReceived( uint16_t data_len,uint32_t node_handle);
static uint8_t*   USB_AUDIO_Streaming_Input_GetBuffer(uint32_t node_handle, uint16_t* max_packet_length);
#endif /* USE_USB_AUDIO_PLAYPBACK*/
#ifdef USE_USB_AUDIO_RECORDING
static uint8_t*   USB_AUDIO_Streaming_Output_GetBuffer(uint32_t node_handle, uint16_t* max_packet_length);
#endif /* USE_USB_AUDIO_RECORDING*/

#ifdef USE_USB_AUDIO_CLASS_20
#ifdef USE_AUDIO_USB_PLAY_MULTI_FREQUENCES
static int8_t  USB_AUDIO_Streaming_Input_SetCurFrequence(uint32_t node_handle);
#endif /*USE_AUDIO_USB_PLAY_MULTI_FREQUENCES*/

#ifdef USE_AUDIO_USB_RECORD_MULTI_FREQUENCES
static int8_t  USB_AUDIO_Streaming_Output_SetCurFrequence(uint32_t node_handle);
#endif /*USE_AUDIO_USB_RECORD_MULTI_FREQUENCES*/
#endif /* USE_USB_AUDIO_CLASS_20 */

static int8_t USB_AUDIO_Streaming_CF_DInit(uint32_t node_handle);
static int8_t USB_AUDIO_Streaming_CF_Start(AUDIO_DevicesCommandsTypedef* commands, uint32_t node_handle);
static int8_t USB_AUDIO_Streaming_CF_Stop( uint32_t node_handle);
static int8_t USB_AUDIO_Streaming_CF_GetMute(uint16_t channel,uint8_t* mute, uint32_t node_handle);
static int8_t USB_AUDIO_Streaming_CF_SetMute(uint16_t channel,uint8_t mute, uint32_t node_handle);
static int8_t USB_AUDIO_Streaming_CF_SetCurVolume(uint16_t channel, uint16_t volume, uint32_t node_handle);
static int8_t USB_AUDIO_Streaming_CF_GetCurVolume(uint16_t channel, uint16_t* volume, uint32_t node_handle);
#ifdef USE_USB_AUDIO_CLASS_10
static int8_t USB_AUDIO_Streaming_CF_GetStatus(uint32_t node_handle);
#endif /*USE_USB_AUDIO_CLASS_10*/
#ifdef USE_USB_AUDIO_CLASS_20
static int8_t USB_AUDIO_Streaming_CLK_SRC_Start( uint32_t node_handle);
static int8_t USB_AUDIO_Streaming_CLK_SRC_Stop( uint32_t node_handle);
#if (defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCES)
static int8_t USB_AUDIO_Streaming_CLK_SRC_SetCurFrequency(uint32_t frequency, uint8_t* as_cnt_to_restart, uint8_t* as_list_to_restart, uint32_t node_handle);
#endif /*(defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCES)*/
static int8_t USB_AUDIO_Streaming_CLK_SRC_GetCurFrequency(uint32_t* frequency, uint32_t node_handle);
static int8_t USB_AUDIO_Streaming_CLK_SRC_GetFrequenciesList(uint32_t** frequencies,uint8_t* freq_count, uint32_t node_handle);
static int8_t USB_AUDIO_Streaming_CLK_SRC_GetIsValid(uint8_t *is_valid, uint32_t node_handle);
#if (defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCES)
static uint32_t  USB_AUDIO_Streaming_GetBestFrequence(uint32_t freq,  uint32_t* freq_table, int freq_count);
#endif /* (defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCES) */
#endif /* USE_USB_AUDIO_CLASS_20 */
#ifdef USE_USB_AUDIO_CLASS_10
#ifdef USE_AUDIO_USB_MULTI_FREQUENCES  
static int8_t  USB_AUDIO_Streaming_IO_GetCurFrequence(uint32_t* freq, uint32_t node_handle);
static int8_t  USB_AUDIO_Streaming_IO_SetCurFrequence(uint32_t freq,uint8_t*  restart_req , uint32_t node_handle);
static uint32_t  USB_AUDIO_Streaming_GetBestFrequence(uint32_t freq,  uint32_t* freq_table, int freq_count);
#endif /*USE_AUDIO_USB_MULTI_FREQUENCES*/

#ifdef USE_AUDIO_USB_PLAY_MULTI_FREQUENCES
/* declare table of supprted frequencies */
 uint32_t USB_AUDIO_CONFIG_PLAY_FREQENCIES[USB_AUDIO_CONFIG_PLAY_FREQ_COUNT]=
{
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
#ifdef USE_AUDIO_USB_RECORD_MULTI_FREQUENCES
 uint32_t USB_AUDIO_CONFIG_RECORD_FREQENCIES[USB_AUDIO_CONFIG_RECORD_FREQ_COUNT]=
{
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
#endif /* USE_USB_AUDIO_CLASS_10 */

#ifdef DEBUG_USB_NODES
static AUDIO_USB_Input_Buffer_StatsTypeDef stats_buffer [USB_INPUT_NODE_DEBUG_BUFFER_SIZE];
static int stats_count=0;
extern __IO uint32_t uwTick;
#endif /* DEBUG_USB_NODES */

/* Private functions ---------------------------------------------------------*/
#ifdef USE_USB_AUDIO_PLAYPBACK
/**
  * @brief  USB_AUDIO_Streaming_Input_Init
  *         Initializes the AUDIO usb input node
  * @param  data_ep:            Data endpoint description , used to set endpoint callbacks and description
  * @param  audio_desc:         audio description
  * @param  session_handle:     the mother session handle
  * @param  node_handle:        the node handle, node must be allocated
  * @retval 0 if error
  */
 int8_t  USB_AUDIO_Streaming_Input_Init(USBD_AUDIO_EP_DataTypeDef* data_ep,
                                              AUDIO_DescriptionTypeDef* audio_desc,
                                              AUDIO_SessionTypeDef* session_handle,  uint32_t node_handle)
{
  AUDIO_USB_IO_NodeTypeDef * input_node;
  
  input_node = (AUDIO_USB_IO_NodeTypeDef *)node_handle;
  input_node->node.audio_description = audio_desc;
  input_node->node.session_handle = session_handle;
  input_node->flags = 0;
  input_node->node.state = AUDIO_NODE_INITIALIZED;
  input_node->node.type = AUDIO_INPUT;
  /* set the node  callback wich are called by session */
  input_node->IODeInit = USB_AUDIO_Streaming_IO_DeInit;
  input_node->IOStart = USB_AUDIO_Streaming_IO_Start;
  input_node->IORestart = USB_AUDIO_Streaming_IO_Restart;
  input_node->IOStop = USB_AUDIO_Streaming_IO_Stop;
  input_node->packet_length = AUDIO_USB_PACKET_SIZE_FROM_AUD_DESC(audio_desc);
  /* compute the packet_max_length */
  #ifdef  USE_AUDIO_PLAYBACK_USB_FEEDBACK
  input_node->max_packet_length = AUDIO_MAX_PACKET_WITH_FEEDBACK_LENGTH(audio_desc);
  #else
    input_node->max_packet_length = AUDIO_USB_MAX_PACKET_SIZE_FROM_AUD_DESC(audio_desc);
  #endif /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */
  /* set data end point callbacks to be called by USB class */
  data_ep->ep_num = USBD_AUDIO_CONFIG_PLAY_EP_OUT;
  data_ep->control_name_map = 0;
  data_ep->control_selector_map = 0;
  data_ep->private_data = node_handle;
  data_ep->DataReceived = USB_AUDIO_Streaming_Input_DataReceived;
  data_ep->GetBuffer = USB_AUDIO_Streaming_Input_GetBuffer;
  data_ep->GetMaxPacketLength = USB_AUDIO_Streaming_IO_GetMaxPacketLength;
#ifdef USE_USB_AUDIO_CLASS_10
  data_ep->GetState = USB_AUDIO_Streaming_IO_GetState;
#ifdef USE_AUDIO_USB_PLAY_MULTI_FREQUENCES
  data_ep->control_selector_map = USBD_AUDIO_CONTROL_EP_SAMPL_FREQ;
  data_ep->control_cbk.GetCurFrequence = USB_AUDIO_Streaming_IO_GetCurFrequence;
  data_ep->control_cbk.SetCurFrequence = USB_AUDIO_Streaming_IO_SetCurFrequence;
  data_ep->control_cbk.MaxFrequence = USB_AUDIO_CONFIG_PLAY_FREQENCIES[0];
  data_ep->control_cbk.MinFrequence = USB_AUDIO_CONFIG_PLAY_FREQENCIES[USB_AUDIO_CONFIG_PLAY_FREQ_COUNT-1];
  data_ep->control_cbk.ResFrequence = 1; 
#endif /* USE_AUDIO_USB_PLAY_MULTI_FREQUENCES */
#endif /* USE_USB_AUDIO_CLASS_10 */
#ifdef USE_USB_AUDIO_CLASS_20
#ifdef USE_AUDIO_USB_PLAY_MULTI_FREQUENCES
   input_node->IOChangeFrequency = USB_AUDIO_Streaming_Input_SetCurFrequence;
#endif /* USE_USB_AUDIO_CLASS_20 */
#endif /* USE_AUDIO_USB_PLAY_MULTI_FREQUENCES */
  return 0;
}
#endif /* USE_USB_AUDIO_PLAYPBACK*/

#ifdef USE_USB_AUDIO_RECORDING
/**
  * @brief  USB_AUDIO_Streaming_Output_Init
  *         Initializes adio output node
  * @param  data_ep: Data endpoint description , used to set endpoint callbacks and description
  * @param  audio_desc: audio description
  * @param  session_handle: the mother session handle
  * @param  node_handle: the node handle, node must be allocated
  * @retval  0 for no error
  */
 int8_t  USB_AUDIO_Streaming_Output_Init(USBD_AUDIO_EP_DataTypeDef* data_ep,
                                         AUDIO_DescriptionTypeDef* audio_desc,
                                         AUDIO_SessionTypeDef* session_handle,  uint32_t node_handle)
{
  AUDIO_USB_IO_NodeTypeDef * output_node;
  output_node=(AUDIO_USB_IO_NodeTypeDef *)node_handle;
  
  output_node->node.audio_description = audio_desc;
  output_node->node.session_handle = session_handle;
  output_node->node.state = AUDIO_NODE_INITIALIZED;
  output_node->node.type = AUDIO_OUTPUT;
#ifdef  USE_AUDIO_RECORDING_USB_NO_REMOVE
  output_node->max_packet_length = AUDIO_MAX_PACKET_WITH_FEEDBACK_LENGTH(audio_desc);
#else /*USE_AUDIO_RECORDING_USB_NO_REMOVE */
  output_node->max_packet_length = AUDIO_USB_MAX_PACKET_SIZE_FROM_AUD_DESC(audio_desc);
#endif /*USE_AUDIO_RECORDING_USB_NO_REMOVE*/
  output_node->packet_length = AUDIO_USB_PACKET_SIZE_FROM_AUD_DESC(audio_desc);
  output_node->specific.output.alt_buff = (uint8_t *) malloc(output_node->max_packet_length);
  if(output_node->specific.output.alt_buff)
  {
    memset(output_node->specific.output.alt_buff, 0, output_node->max_packet_length);
  }
  else
  {
    Error_Handler();
  }
  output_node->IODeInit = USB_AUDIO_Streaming_IO_DeInit;
  output_node->IOStart = USB_AUDIO_Streaming_IO_Start;
  output_node->IOStop = USB_AUDIO_Streaming_IO_Stop;
  output_node->IORestart = USB_AUDIO_Streaming_IO_Restart;
  /* set data end point callbacks */
  data_ep->ep_num = USB_AUDIO_CONFIG_RECORD_EP_IN;
  data_ep->control_name_map = 0;
  data_ep->control_selector_map = 0;
  data_ep->private_data = node_handle;
  data_ep->DataReceived = 0;
  data_ep->GetBuffer = USB_AUDIO_Streaming_Output_GetBuffer;
  data_ep->GetMaxPacketLength = USB_AUDIO_Streaming_IO_GetMaxPacketLength;
#ifdef USE_USB_AUDIO_CLASS_10
  data_ep->GetState = USB_AUDIO_Streaming_IO_GetState;

#ifdef USE_AUDIO_USB_RECORD_MULTI_FREQUENCES
  data_ep->control_selector_map = USBD_AUDIO_CONTROL_EP_SAMPL_FREQ;
  data_ep->control_cbk.GetCurFrequence = USB_AUDIO_Streaming_IO_GetCurFrequence;
  data_ep->control_cbk.SetCurFrequence = USB_AUDIO_Streaming_IO_SetCurFrequence;
  data_ep->control_cbk.MaxFrequence = USB_AUDIO_CONFIG_RECORD_FREQENCIES[0];
  data_ep->control_cbk.MinFrequence = USB_AUDIO_CONFIG_RECORD_FREQENCIES[USB_AUDIO_CONFIG_RECORD_FREQ_COUNT-1];
  data_ep->control_cbk.ResFrequence = 1; 
#endif /* USE_AUDIO_USB_RECORD_MULTI_FREQUENCES */
#endif /* USE_USB_AUDIO_CLASS_10 */
 #ifdef USE_USB_AUDIO_CLASS_20
#ifdef USE_AUDIO_USB_RECORD_MULTI_FREQUENCES
   output_node->IOChangeFrequency = USB_AUDIO_Streaming_Output_SetCurFrequence;
#endif /* USE_USB_AUDIO_CLASS_20 */
#endif /* USE_AUDIO_USB_RECORD_MULTI_FREQUENCES */
  return 0;
}

#endif /* USE_USB_AUDIO_RECORDING*/
/**
  * @brief  USB_AUDIO_Streaming_IO_DeInit
  *         De-Initializes the AUDIO usb input node
  * @param  node_handle: the node handle, node must be allocated
  * @retval  0 for no error
  */
 static int8_t  USB_AUDIO_Streaming_IO_DeInit(uint32_t node_handle)
{
  ((AUDIO_USB_IO_NodeTypeDef *)node_handle)->node.state = AUDIO_NODE_OFF;
  
  return 0;
}


/**
  * @brief  USB_AUDIO_Streaming_IO_Start
  *         Start Usb input  or ouput node
  * @param  buffer:             buffer which is used while node is being started
  * @param  thershold:          buffer thershold for input node
  * @param  node_handle:        the node handle, node must be initialized
  * @retval 0 for no error
  */
static int8_t  USB_AUDIO_Streaming_IO_Start( AUDIO_BufferTypeDef* buffer, uint16_t thershold ,uint32_t node_handle)
{
  AUDIO_USB_IO_NodeTypeDef * io_node;
   
   io_node = (AUDIO_USB_IO_NodeTypeDef *)node_handle;
   
   if((io_node->node.state == AUDIO_NODE_INITIALIZED ) ||(io_node->node.state == AUDIO_NODE_STOPPED))
   {
       io_node->node.state = AUDIO_NODE_STARTED;
       io_node->buf = buffer;
       io_node->buf->rd_ptr = io_node->buf->wr_ptr=0;
       
       io_node->flags = 0;
       if(io_node->node.type == AUDIO_INPUT)
       {
         io_node->specific.input.thershold = thershold;
       }
       else
       {
#if USB_AUDIO_CONFIG_RECORD_USE_FREQ_44_1_K
          if(io_node->node.audio_description->frequence == USB_AUDIO_CONFIG_FREQ_44_1_K)
          {
            io_node->specific.output.packet_44_counter = 0;
          }
#endif /* USB_AUDIO_CONFIG_RECORD_USE_FREQ_44_1_K */
       }
   }
   return 0;
}

/**
  * @brief  USB_AUDIO_Streaming_IO_Stop
  *         Stops Usb input or output node
  * @param  node_handle: the node handle, node must be initialized
  * @retval  0 for no error
  */
static int8_t  USB_AUDIO_Streaming_IO_Stop( uint32_t node_handle)
{
  AUDIO_USB_IO_NodeTypeDef * io_node;
   
   io_node = (AUDIO_USB_IO_NodeTypeDef *)node_handle;
   io_node->node.state = AUDIO_NODE_STOPPED;
  return 0;
}
/**
  * @brief  USB_AUDIO_Streaming_IO_Restart
  *         restart buffer
  * @param  node_handle: the node handle, node must be started
  * @retval  0 for no error
  */
static int8_t  USB_AUDIO_Streaming_IO_Restart( uint32_t node_handle)
{
  AUDIO_USB_IO_NodeTypeDef * io_node;
   
   io_node = (AUDIO_USB_IO_NodeTypeDef *)node_handle;
   if(io_node->node.state == AUDIO_NODE_STARTED)
   {
     io_node->flags = AUDIO_IO_RESTART_REQUIRED;
     return 0;
   }
   return 0;
}

#ifdef USE_USB_AUDIO_PLAYPBACK
/**
  * @brief  USB_AUDIO_Streaming_Input_DataReceived
  *         callback called by USB class when new data is received
  * @param  data_len:           Data length
  * @param  node_handle:        the input node handle, node must be initialized  and stardted
  * @retval  0 for no error
  */
static int8_t  USB_AUDIO_Streaming_Input_DataReceived( uint16_t data_len, uint32_t node_handle)
 {
   AUDIO_USB_IO_NodeTypeDef * input_node;
   AUDIO_BufferTypeDef *buf;
   uint16_t wr_distance;
   
   input_node = (AUDIO_USB_IO_NodeTypeDef *)node_handle;
   if(input_node->node.state == AUDIO_NODE_STARTED)
   {
     /* @TODO add overrun detection */
     if(input_node->flags&AUDIO_IO_RESTART_REQUIRED)
     {
       input_node->flags = 0;
       input_node->buf->rd_ptr=input_node->buf->wr_ptr = 0;
       return 0;
     }
     buf=input_node->buf;

     buf->wr_ptr += data_len;/* increment buffer */

     if((input_node->flags&AUDIO_IO_BEGIN_OF_STREAM) == 0)
     {
       
       input_node->node.session_handle->SessionCallback(AUDIO_BEGIN_OF_STREAM,(AUDIO_NodeTypeDef*)input_node,
                                                        input_node->node.session_handle);
       input_node->flags |= AUDIO_IO_BEGIN_OF_STREAM;
       
     }
     else
     {
#ifdef USE_AUDIO_PLAYBACK_USB_FEEDBACK
       /* check if data is in margin, then copy it */
        if(buf->wr_ptr > buf->size)
        {
          buf->wr_ptr -= buf->size;
          memcpy(buf->data, buf->data+buf->size, buf->wr_ptr);
        }
#else /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */
#if USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K
        if(input_node->node.audio_description->frequence ==   USB_AUDIO_CONFIG_FREQ_44_1_K)
        {
          if(buf->wr_ptr > buf->size)
          {
            buf->wr_ptr -= buf->size;
            memcpy(buf->data, buf->data+buf->size, buf->wr_ptr);
          }
        }
#endif /* USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K*/
        if(buf->wr_ptr > buf->size)
        {
          Error_Handler();
        }
#endif /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */
        
       
      wr_distance = AUDIO_BUFFER_FILLED_SIZE(buf); 
      if(buf->wr_ptr == buf->size)
      {
        buf->wr_ptr = 0;
      }
      if(((input_node->flags&AUDIO_IO_THERSHOLD_REACHED) == 0)&&
          (wr_distance >= input_node->specific.input.thershold))
      {
         input_node->node.session_handle->SessionCallback(AUDIO_THERSHOLD_REACHED, (AUDIO_NodeTypeDef*)input_node,
                                                         input_node->node.session_handle); 
          input_node->flags |= AUDIO_IO_THERSHOLD_REACHED ;
       }
       else
       {
        input_node->node.session_handle->SessionCallback(AUDIO_PACKET_RECEIVED, (AUDIO_NodeTypeDef*)input_node,
                                                         input_node->node.session_handle); 
       }
     }
    }
   else
   {
     Error_Handler();
   }
   return 0;
 }

/**
  * @brief  USB_AUDIO_Streaming_Input_GetBuffer
  *         callback called by USB class to get working buffer in order to receive next packet           
  * @param  node_handle:        the input node handle, node must be initialized and started
  * @param  max_packet_length:  max data length to be received
  * @retval  0 for no error                          
  */
static uint8_t* USB_AUDIO_Streaming_Input_GetBuffer(uint32_t node_handle, uint16_t* max_packet_length)
{
  AUDIO_USB_IO_NodeTypeDef* input_node;
  uint16_t wr_distance;
  
  input_node = (AUDIO_USB_IO_NodeTypeDef *)node_handle;
#ifdef DEBUG_USB_NODES
  stats_buffer[stats_count].read = input_node->buf->rd_ptr;
  stats_buffer[stats_count].write = input_node->buf->wr_ptr;
  stats_buffer[stats_count].time = uwTick;
  
  stats_count++;
  if(stats_count == USB_INPUT_NODE_DEBUG_BUFFER_SIZE)
  {
    stats_count=0;
  }
#endif /*DEBUG_USB_NODES*/
  *max_packet_length = input_node->max_packet_length;
  if( input_node->node.state == AUDIO_NODE_STARTED)
  {
    /* check for overflow */
    wr_distance  = AUDIO_BUFFER_FREE_SIZE(input_node->buf);
    
    if(wr_distance < input_node->max_packet_length)
    {
      input_node->node.session_handle->SessionCallback(AUDIO_OVERRUN, (AUDIO_NodeTypeDef*)input_node,
                                                       input_node->node.session_handle);
    }
    
    if(input_node->flags&AUDIO_IO_RESTART_REQUIRED)
    {
     input_node->flags = 0;
     input_node->buf->rd_ptr = input_node->buf->wr_ptr = 0;
    }
    return input_node->buf->data+input_node->buf->wr_ptr;
  }
  else
  {
    Error_Handler();
    return 0; /* return statement non reachable */
  }
}


#endif /* USE_USB_AUDIO_PLAYPBACK*/
#ifdef USE_USB_AUDIO_RECORDING
/**
  * @brief  USB_AUDIO_Streaming_Output_GetBuffer
  *         callback called by USB class to get working buffer to receive next packet   
  * @param  node_handle:        the output node handle, node must be initialized and stardted
  * @param  packet_length:      max data length to send         
  * @retval  0 if no error     
  */
static uint8_t* USB_AUDIO_Streaming_Output_GetBuffer(uint32_t node_handle,uint16_t* packet_length)
{

   AUDIO_USB_IO_NodeTypeDef *output_node;
   uint16_t wr_distance;
   AUDIO_BufferTypeDef *buf;
   uint8_t* packet_data;
#ifdef USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO 
   int8_t sample_add_remove;
#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO */

   output_node = (AUDIO_USB_IO_NodeTypeDef *)node_handle;

   if(output_node->node.state == AUDIO_NODE_STARTED)
   {
     if(output_node->flags&AUDIO_IO_RESTART_REQUIRED)
     {
       output_node->flags = 0;
       output_node->buf->rd_ptr = 0;
#if USB_AUDIO_CONFIG_RECORD_USE_FREQ_44_1_K
      if(output_node->node.audio_description->frequence == USB_AUDIO_CONFIG_FREQ_44_1_K)
      {
        output_node->specific.output.packet_44_counter = 0;
      }
#endif /* USB_AUDIO_CONFIG_RECORD_USE_FREQ_44_1_K */
       return output_node->specific.output.alt_buff;
     }
#ifdef USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO 
      output_node->node.session_handle->SessionCallback(AUDIO_PACKET_PLAYED, (AUDIO_NodeTypeDef*)output_node,
                                                        output_node->node.session_handle);
#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO */
#if USB_AUDIO_CONFIG_RECORD_USE_FREQ_44_1_K
    if(output_node->node.audio_description->frequence == USB_AUDIO_CONFIG_FREQ_44_1_K)
    {
#ifdef USE_USB_HS
        if(output_node->specific.output.packet_44_counter == 79)
#else /* USE_USB_HS */
      if(output_node->specific.output.packet_44_counter == 9)
#endif /* USE_USB_HS */
      {
        *packet_length = output_node->max_packet_length;
        output_node->specific.output.packet_44_counter = 0;
      }
      else
      {
        output_node->specific.output.packet_44_counter ++;
        *packet_length = output_node->packet_length;
      }
    }
    else
    {
      *packet_length = output_node->packet_length;
    }
#else /* USB_AUDIO_CONFIG_RECORD_USE_FREQ_44_1_K */
    *packet_length = output_node->packet_length;
#endif /* USB_AUDIO_CONFIG_RECORD_USE_FREQ_44_1_K */
    
     buf = output_node->buf;
      /* @TODO add underrun detection */
     if(!(output_node->flags&AUDIO_IO_BEGIN_OF_STREAM))
     { 
     if(buf->wr_ptr < (buf->size>>1)) /* first thershold is a half of buffer */
      {
        /* buffer is not ready  */
        return output_node->specific.output.alt_buff;
      }
       output_node->node.session_handle->SessionCallback(AUDIO_BEGIN_OF_STREAM, (AUDIO_NodeTypeDef*)output_node,
                                                        output_node->node.session_handle);
       output_node->flags |= AUDIO_IO_BEGIN_OF_STREAM;
     }
       /* Check for underrun */
      wr_distance = AUDIO_BUFFER_FILLED_SIZE(buf);       
      if(wr_distance < *packet_length)
      {
       output_node->node.session_handle->SessionCallback(AUDIO_UNDERRUN, (AUDIO_NodeTypeDef*)output_node,
                                                        output_node->node.session_handle);
        return output_node->specific.output.alt_buff;
      }
      else
      {
#ifdef USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO
      sample_add_remove = AUDIO_Recording_get_Sample_to_add(output_node->node.session_handle);
#ifdef  USE_AUDIO_RECORDING_USB_NO_REMOVE
      *packet_length += sample_add_remove;
      if(*packet_length > output_node->max_packet_length)
      {
        *packet_length = output_node->max_packet_length;
      }
      AUDIO_Recording_Set_Sample_Written(output_node->node.session_handle, *packet_length);
#else /*USE_AUDIO_RECORDING_USB_NO_REMOVE */
        if(sample_add_remove<0)
        {
            *packet_length += sample_add_remove;
            
        }
        else
        {
          if(sample_add_remove>0)
          {
            buf->rd_ptr += sample_add_remove;
            if(buf->rd_ptr == buf->size)
            {
              buf->rd_ptr = 0;
            }
          }
        }
        AUDIO_Recording_Set_Sample_Written(output_node->node.session_handle, *packet_length+sample_add_remove);
#endif /*USE_AUDIO_RECORDING_USB_NO_REMOVE*/
#endif /* USE_AUDIO_RECORDING_USB_IMPLECIT_SYNCHRO */
         /* increment read pointer */
        packet_data = buf->data + buf->rd_ptr;
        buf->rd_ptr += *packet_length;
        
        /* manage the packet not aligned */
        if(buf->rd_ptr > buf->size)
        {
          buf->rd_ptr -= buf->size;
          memcpy(buf->data+buf->size, buf->data, buf->rd_ptr);
          if(buf->rd_ptr+buf->size >=USBD_AUDIO_CONFIG_RECORD_BUFFER_SIZE)
          {
            Error_Handler();
          }
         }
          
        if(buf->rd_ptr == buf->size)
        {
          buf->rd_ptr = 0;
        }
      }
     return (packet_data);
   }
   else
   {
     /*Should not happen */
     Error_Handler();
     return 0; /* return statement not reachable */
   }
 
}
#endif /* USE_USB_AUDIO_RECORDING*/

/**
  * @brief  USB_AUDIO_Streaming_IO_GetMaxPacketLength
  *         return max packet length 
  * @param  node_handle: the input node handle, node must be initialized
  * @retval  max packet length
*/
static uint16_t  USB_AUDIO_Streaming_IO_GetMaxPacketLength(uint32_t node_handle)
{
  
  return ((AUDIO_USB_IO_NodeTypeDef *)node_handle)->max_packet_length;
}

#ifdef USE_USB_AUDIO_CLASS_10
#ifdef USE_AUDIO_USB_MULTI_FREQUENCES 
/**
  * @brief  USB_AUDIO_Streaming_IO_GetCurFrequence
  *         return cuerrent frequence
  * @param  node_handle: the usb io node handle, node must be initialized
  * @param  freq:                   
  * @retval  0 if no error 
*/
static int8_t  USB_AUDIO_Streaming_IO_GetCurFrequence(uint32_t* freq, uint32_t node_handle)
{
 *freq = ((AUDIO_USB_IO_NodeTypeDef *)node_handle) ->node.audio_description->frequence;
 return 0;
}

/**
  * @brief  USB_AUDIO_Streaming_IO_SetCurFrequence
  *         set  current frequence, if frequence not supported , set nearst frequence
  * @param  freq:                 
  * @param  restart_req:           
  * @param  node_handle: the usb io node handle, node must be initialized
  * @retval  0 if no error 
*/
static int8_t  USB_AUDIO_Streaming_IO_SetCurFrequence(uint32_t freq, uint8_t* restart_req, uint32_t node_handle)
{
  AUDIO_USB_IO_NodeTypeDef *usb_io_node=(AUDIO_USB_IO_NodeTypeDef *)node_handle;
  AUDIO_DescriptionTypeDef* aud;
  uint32_t best_freq;
 /* search nearst supported frequence */
 aud = usb_io_node->node.audio_description;
 if(aud->frequence != freq)
 {
 #ifdef USE_AUDIO_USB_PLAY_MULTI_FREQUENCES
 if(usb_io_node->node.type == AUDIO_INPUT)
 {
   
    best_freq = USB_AUDIO_Streaming_GetBestFrequence(freq,USB_AUDIO_CONFIG_PLAY_FREQENCIES,USB_AUDIO_CONFIG_PLAY_FREQ_COUNT);
  if(aud->frequence == best_freq)
  {
    *restart_req = 0;
    return 0;
  }
  else
  {
    aud->frequence = best_freq;
  }
#ifdef  USE_AUDIO_PLAYBACK_USB_FEEDBACK
  usb_io_node->max_packet_length = AUDIO_MAX_PACKET_WITH_FEEDBACK_LENGTH(aud);
#else
    usb_io_node->max_packet_length = AUDIO_USB_MAX_PACKET_SIZE_FROM_AUD_DESC(aud);
#endif /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */
 }
 #endif /* USE_AUDIO_USB_PLAY_MULTI_FREQUENCES*/
 
#ifdef USE_AUDIO_USB_RECORD_MULTI_FREQUENCES
 if(usb_io_node->node.type == AUDIO_OUTPUT)
 {
   best_freq = USB_AUDIO_Streaming_GetBestFrequence(freq,
                                                         USB_AUDIO_CONFIG_RECORD_FREQENCIES,
                                                         USB_AUDIO_CONFIG_RECORD_FREQ_COUNT);
  if(aud->frequence == best_freq)
  {
    *restart_req = 1;
    return 0;
  }
  else
  {
    aud->frequence = best_freq;
  }
#ifdef  USE_AUDIO_RECORDING_USB_NO_REMOVE
  usb_io_node->max_packet_length = AUDIO_MAX_PACKET_WITH_FEEDBACK_LENGTH(aud);
#else /*USE_AUDIO_RECORDING_USB_NO_REMOVE */
   usb_io_node->max_packet_length = AUDIO_USB_MAX_PACKET_SIZE_FROM_AUD_DESC(aud);
#endif /*USE_AUDIO_RECORDING_USB_NO_REMOVE*/
   /* reallocat alternate buffer */
  free(usb_io_node->specific.output.alt_buff);
  usb_io_node->specific.output.alt_buff = (uint8_t *) malloc(usb_io_node->max_packet_length);
   if(usb_io_node->specific.output.alt_buff)
   {
     memset(usb_io_node->specific.output.alt_buff, 0, usb_io_node->max_packet_length);
   }
   else
   {
     Error_Handler();
   }
 }
#endif /* USE_AUDIO_USB_RECORD_MULTI_FREQUENCES*/
  usb_io_node->packet_length = AUDIO_USB_PACKET_SIZE_FROM_AUD_DESC(aud);
  usb_io_node->node.session_handle->SessionCallback(AUDIO_FREQUENCY_CHANGED,(AUDIO_NodeTypeDef*)usb_io_node,
                                                        usb_io_node->node.session_handle);
 *restart_req = 1;
 }
 else
 {
   *restart_req = 0;
#ifdef USE_USB_AUDIO_RECORDING
    if(usb_io_node->node.type == AUDIO_OUTPUT)
    {
        *restart_req = 1;
    }
#endif /* USE_USB_AUDIO_RECORDING */
 }
 return 0;
}
#endif /*USE_AUDIO_USB_MULTI_FREQUENCES*/
#endif /* USE_USB_AUDIO_CLASS_10 */


#ifdef USE_USB_AUDIO_CLASS_20
#ifdef USE_AUDIO_USB_PLAY_MULTI_FREQUENCES
/**
  * @brief  USB_AUDIO_Streaming_Input_SetCurFrequence
  *         set  current frequence, if frequence not supported , set nearst frequence
  * @param  freq:                 
  * @param  restart_req:           
  * @param  node_handle: the usb io node handle, node must be initialized
  * @retval  0 if no error 
*/
static int8_t  USB_AUDIO_Streaming_Input_SetCurFrequence(uint32_t node_handle)
{
  AUDIO_USB_IO_NodeTypeDef *input=(AUDIO_USB_IO_NodeTypeDef *)node_handle;
 
#ifdef  USE_AUDIO_PLAYBACK_USB_FEEDBACK
  input->max_packet_length = AUDIO_MAX_PACKET_WITH_FEEDBACK_LENGTH(input->node.audio_description);
#else
    input->max_packet_length = AUDIO_USB_MAX_PACKET_SIZE_FROM_AUD_DESC(input->node.audio_description);
#endif /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */
  input->packet_length = AUDIO_USB_PACKET_SIZE_FROM_AUD_DESC(input->node.audio_description);
  return 0;
}
#endif /*USE_AUDIO_USB_PLAY_MULTI_FREQUENCES*/

#ifdef USE_AUDIO_USB_RECORD_MULTI_FREQUENCES
/**
  * @brief  USB_AUDIO_Streaming_Output_SetCurFrequence
  *         set  current frequence, if frequence not supported , set nearst frequence
  * @param  freq:                 
  * @param  restart_req:           
  * @param  node_handle: the usb output node handle, node must be initialized
  * @retval  0 if no error 
*/
static int8_t  USB_AUDIO_Streaming_Output_SetCurFrequence(uint32_t node_handle)
{
  AUDIO_USB_IO_NodeTypeDef *output=(AUDIO_USB_IO_NodeTypeDef *)node_handle;
 
#ifdef  USE_AUDIO_RECORDING_USB_NO_REMOVE
  output->max_packet_length = AUDIO_MAX_PACKET_WITH_FEEDBACK_LENGTH(output->node.audio_description);
#else /*USE_AUDIO_RECORDING_USB_NO_REMOVE */
   output->max_packet_length = AUDIO_USB_MAX_PACKET_SIZE_FROM_AUD_DESC(output->node.audio_description);
#endif /*USE_AUDIO_RECORDING_USB_NO_REMOVE*/
   /* reallocat alternate buffer */
  free(output->specific.output.alt_buff);
  output->specific.output.alt_buff = (uint8_t *) malloc(output->max_packet_length);
  if(output->specific.output.alt_buff)
  {
    memset(output->specific.output.alt_buff, 0, output->max_packet_length);
  }
  else
  {
    Error_Handler();
  }
  output->packet_length = AUDIO_USB_PACKET_SIZE_FROM_AUD_DESC(output->node.audio_description);

 return 0;
}
#endif /*USE_AUDIO_USB_RECORD_MULTI_FREQUENCES*/
#endif /* USE_USB_AUDIO_CLASS_20 */

#if (defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCES)
/**
  * @brief  USB_AUDIO_Streaming_GetBestFrequence
  *        get nearst supprted frequence
  * @param  freq               
  * @param  freq_table          
  * @param  freq_count          
  * @retval  0 if no error 
*/
static uint32_t  USB_AUDIO_Streaming_GetBestFrequence(uint32_t freq,  uint32_t* freq_table,  int freq_count)
{
  
  if(freq >= freq_table[0])
 {
   return freq_table[0];
 }
 else
 {
    if(freq <= freq_table[freq_count-1])
   {
     return freq_table[freq_count-1];
   }
   else
   {
     for(int i = 1; i<freq_count; i++)
     {
       if(freq >= freq_table[i])
       {
         return ((freq_table[i-1] - freq )<= ( freq - freq_table[i]))?
           freq_table[i-1] : freq_table[i];
       }
     }
   }
 }
 
return 0; 
}
#endif /* (defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCES) */

#ifdef USE_USB_AUDIO_CLASS_10
/**
  * @brief  USB_AUDIO_Streaming_IO_GetState
  *         return data ep   state   
  * @param  node_handle: the input node handle, node must be initialized
  * @retval  max packet length
*/
static int8_t  USB_AUDIO_Streaming_IO_GetState(uint32_t node_handle)
{
  return 0;
}
#endif /* USE_USB_AUDIO_CLASS_10 */

/**
  * @brief  USB_AUDIO_Streaming_CF_Init
  *         Initializes control feature node
  * @param  usb_control_feature:        supported controls
  * @param  audio_defaults:             audio defaults setting
  * @param  unit_id:                    usb unit id
  * @param  node_handle:                the node handle, node must be allocated
  * @retval  0 for no error
  */
 int8_t USB_AUDIO_Streaming_CF_Init(USBD_AUDIO_ControlTypeDef* usb_control_feature,
                                   AUDIO_ControlDeviceDefaultsTypedef* audio_defaults, uint8_t unit_id,
                                   uint32_t node_handle)
{
  AUDIO_USB_CF_NodeTypeDef * cf;
 
  cf = (AUDIO_USB_CF_NodeTypeDef*)node_handle;  
  memset(cf,0,sizeof(AUDIO_USB_CF_NodeTypeDef));
  cf->node.state = AUDIO_NODE_INITIALIZED;
  cf->node.type = AUDIO_CONTROL;
  cf->unit_id = unit_id;
  
  cf->CFInit = USB_AUDIO_Streaming_CF_Init;
  cf->CFDeInit = USB_AUDIO_Streaming_CF_DInit;
  cf->CFStart = USB_AUDIO_Streaming_CF_Start;
  cf->CFStop = USB_AUDIO_Streaming_CF_Stop;
  cf->CFSetMute = USB_AUDIO_Streaming_CF_SetMute;
#ifdef USE_USB_AUDIO_CLASS_10
  cf->usb_control_callbacks.GetStatus = USB_AUDIO_Streaming_CF_GetStatus;
#endif /*USE_USB_AUDIO_CLASS_10*/
  cf->usb_control_callbacks.GetMute = USB_AUDIO_Streaming_CF_GetMute;
  cf->usb_control_callbacks.SetMute = USB_AUDIO_Streaming_CF_SetMute;
  cf->usb_control_callbacks.GetCurVolume = USB_AUDIO_Streaming_CF_GetCurVolume;
  cf->usb_control_callbacks.SetCurVolume = USB_AUDIO_Streaming_CF_SetCurVolume;
  VOLUME_DB_256_TO_USB(cf->usb_control_callbacks.MaxVolume, audio_defaults->max_volume);
  VOLUME_DB_256_TO_USB(cf->usb_control_callbacks.MinVolume, audio_defaults->min_volume);
  cf->usb_control_callbacks.ResVolume = audio_defaults->res_volume;
  cf->node.audio_description=audio_defaults->audio_description;
  
  /* @TODO fill next request map and selector */
  usb_control_feature->id = unit_id;
  usb_control_feature->control_req_map = 0;  
  usb_control_feature->control_selector_map = USBD_AUDIO_FU_MUTE_CONTROL|USBD_AUDIO_FU_VOLUME_CONTROL;
  usb_control_feature->type = USBD_AUDIO_CS_AC_SUBTYPE_FEATURE_UNIT;
  usb_control_feature->Callbacks.feature_control = &cf->usb_control_callbacks;
  usb_control_feature->private_data = node_handle;
  return 0;
}


/**
  * @brief  USB_AUDIO_Streaming_CF_DInit              
  *         De-Initialize control feature node
  * @param  node_handle: the node handle, node must be Initialized
  * @retval  0 for no error
  */
static int8_t USB_AUDIO_Streaming_CF_DInit(uint32_t node_handle)
{
  ((AUDIO_USB_CF_NodeTypeDef*)node_handle)->node.state = AUDIO_NODE_OFF;
  return 0;
}

/**
  * @brief  USB_AUDIO_Streaming_CF_Start
  *         start control feature node
  * @param  commands:                   
  * @param  node_handle: the node handle, node must be allocated
  * @retval  0 for no error
  */
static int8_t USB_AUDIO_Streaming_CF_Start(AUDIO_DevicesCommandsTypedef* commands, uint32_t node_handle)
{
  AUDIO_USB_CF_NodeTypeDef *cf;
  
  cf = (AUDIO_USB_CF_NodeTypeDef*)node_handle;
  cf->control_cbks = *commands;
  cf->node.state = AUDIO_NODE_STARTED;
  if(cf->control_cbks.SetCurrentVolume)
  {
    cf->control_cbks.SetCurrentVolume(0, 
                                      cf->node.audio_description->audio_volume_db_256,
                                      cf->control_cbks.private_data);
  }
  return 0;
}

/**
  * @brief  USB_AUDIO_Streaming_CF_Stop
  *         stop control feature node
  * @param  node_handle: the node handle, node must be started
  * @retval  0 for no error
  */
static int8_t USB_AUDIO_Streaming_CF_Stop( uint32_t node_handle)
{
  /* @TODO develop feature */
  AUDIO_USB_CF_NodeTypeDef * cf;
  
  cf = (AUDIO_USB_CF_NodeTypeDef*)node_handle;
  cf->node.state = AUDIO_NODE_STOPPED;
  return 0;
}

/**
  * @brief  USB_AUDIO_Streaming_CF_GetMute
  *         get mute value
  * @param  channel: channel number , 0 for master channel
  * @param  mute: returned mute value
  * @param  node_handle: the Feature node handle, node must be initialized
  * @retval  0 for no error
  */
static int8_t USB_AUDIO_Streaming_CF_GetMute(uint16_t channel, uint8_t* mute, uint32_t node_handle)
{
  /**@TODO add channel management  */
  *mute = ((AUDIO_USB_CF_NodeTypeDef*)node_handle)->node.audio_description->audio_mute; 
  return 0; 
}

/**
  * @brief  USB_AUDIO_Streaming_CF_SetMute
  *         set mute value
  * @param  channel: channel number , 0 for master channel
  * @param  mute:  mute value
  * @param  node_handle: the Feature node handle, node must be initialized
  * @retval  0 for no error
  */
static int8_t USB_AUDIO_Streaming_CF_SetMute(uint16_t channel, uint8_t mute, uint32_t node_handle)
{
  AUDIO_USB_CF_NodeTypeDef * cf;
  
  cf = (AUDIO_USB_CF_NodeTypeDef*)node_handle;
  /**@TODO add channel management  */
  
  cf->node.audio_description->audio_mute = mute;
  if((cf->node.state == AUDIO_NODE_STARTED)&&(cf->control_cbks.SetMute))
  {
      cf->control_cbks.SetMute(channel, mute, cf->control_cbks.private_data);
  }
  return 0;
}

/**
  * @brief  USB_AUDIO_Streaming_CF_GetCurVolume
  *         get current volume  value
  * @param  channel:            channel number , 0 for master channel
  * @param  volume:             returned volume value
  * @param  node_handle:        the Feature node handle, node must be initialized
  * @retval  0 for no error
  */
static int8_t USB_AUDIO_Streaming_CF_GetCurVolume(uint16_t channel, uint16_t* volume, uint32_t node_handle)
{
  /**@TODO add channel management  */
  VOLUME_DB_256_TO_USB(*volume, ((AUDIO_NodeTypeDef*)node_handle)->audio_description->audio_volume_db_256);
  return 0; 
}

/**
  * @brief  USB_AUDIO_Streaming_CF_SetCurVolume
  *         set current volume  value
  * @param  channel:            channel number , 0 for master channel
  * @param  volume:             volume value
  * @param  node_handle:        the Feature node handle, node must be initialized
  * @retval  0 for no error
  */
static int8_t USB_AUDIO_Streaming_CF_SetCurVolume(uint16_t channel, uint16_t volume, uint32_t node_handle)
{
  AUDIO_USB_CF_NodeTypeDef* cf;
  
  cf = (AUDIO_USB_CF_NodeTypeDef*)node_handle;
  /**@TODO add channel management  */
  
  VOLUME_USB_TO_DB_256(cf->node.audio_description->audio_volume_db_256, volume);
  if((cf->node.state == AUDIO_NODE_STARTED)&&(cf->control_cbks.SetCurrentVolume))
  {
    cf->control_cbks.SetCurrentVolume(channel, 
                                      cf->node.audio_description->audio_volume_db_256,
                                      cf->control_cbks.private_data);
  }
  return 0;
}

#ifdef USE_USB_AUDIO_CLASS_10
/**
  * @brief  USB_AUDIO_Streaming_CF_GetStatus          
  *         get Feature uint status
  * @param  node_handle:        the Feature node handle, node must be initialized      
  * @retval 0 for no error
  */
static int8_t  USB_AUDIO_Streaming_CF_GetStatus( uint32_t node_handle )
{
  return 0;
}
#endif /* USE_USB_AUDIO_CLASS_10 */

#ifdef USE_USB_AUDIO_CLASS_20
/**
  * @brief  USB_AUDIO_Streaming_CLK_SRC_Init
  *         Initializes clock source node
  * @param  usb_control_feature:        supported controls
  * @param  clk_cmds:             Specific clock source commands
  * @param  clock_src_id:                    usb unit id
  * @param  node_handle:                the node handle, node must be allocated
  * @retval  0 for no error
  */
 int8_t USB_AUDIO_Streaming_CLK_SRC_Init(USBD_AUDIO_ControlTypeDef* usb_control_feature  ,
                                         AUDIO_DevicesClockCommandsTypedef* clk_cmds,
                                         uint8_t        clock_src_id,
                                         AUDIO_DescriptionTypeDef* audio_description,
                                         uint32_t node_handle)
{
  AUDIO_USB_ClockSrc_NodeTypeDef * clk;
 
  clk = (AUDIO_USB_ClockSrc_NodeTypeDef*)node_handle;  
  memset(clk,0,sizeof(AUDIO_USB_ClockSrc_NodeTypeDef));
  clk->node.state = AUDIO_NODE_INITIALIZED;
  clk->node.type = AUDIO_CLOCK;
  clk->clock_source_id = clock_src_id;
  
  clk->CSInit = USB_AUDIO_Streaming_CLK_SRC_Init;
  clk->CSDeInit = USB_AUDIO_Streaming_CF_DInit;
  clk->CSStart = USB_AUDIO_Streaming_CLK_SRC_Start;
  clk->CSStop = USB_AUDIO_Streaming_CLK_SRC_Stop;
 

  clk->usb_control_callbacks.GetCurFrequency = USB_AUDIO_Streaming_CLK_SRC_GetCurFrequency;
  clk->usb_control_callbacks.GetFrequencyList = USB_AUDIO_Streaming_CLK_SRC_GetFrequenciesList;
  clk->usb_control_callbacks.IsValid = USB_AUDIO_Streaming_CLK_SRC_GetIsValid;
#if (defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCES)
     clk->usb_control_callbacks.SetCurFrequency = (clk_cmds->SetFrequency)? USB_AUDIO_Streaming_CLK_SRC_SetCurFrequency : 0;
#else /*(defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCES)*/
     clk->usb_control_callbacks.SetCurFrequency = 0;
#endif /*(defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCES)*/
  clk->control_cbks = *clk_cmds;
  clk->node.audio_description = audio_description;
  
  /* @TODO fill next request map and selector */
  usb_control_feature->id = clock_src_id;
  usb_control_feature->control_req_map = 0;  
  usb_control_feature->control_selector_map = USBD_AUDIO_CS_CLOCK_VALID_CONTROL|USBD_AUDIO_CS_SAM_FREQ_CONTROL;
  usb_control_feature->type = USBD_AUDIO_CS_AC_SUBTYPE_CLOCK_SOURCE;
  usb_control_feature->Callbacks.clk_src_control = &clk->usb_control_callbacks;
  usb_control_feature->private_data = node_handle;
  return 0;
}




/**
  * @brief  USB_AUDIO_Streaming_CLK_SRC_Start
  *         start Clock source  node                  
  * @param  node_handle: the node handle, node must be allocated
  * @retval  0 for no error
  */
static int8_t USB_AUDIO_Streaming_CLK_SRC_Start(uint32_t node_handle)
{
  AUDIO_USB_ClockSrc_NodeTypeDef *clk;
  
  clk = (AUDIO_USB_ClockSrc_NodeTypeDef*)node_handle;
  clk->node.state = AUDIO_NODE_STARTED;
  return 0;
}

/**
  * @brief  USB_AUDIO_Streaming_CLK_SRC_Stop
  *         stop Clock source  node
  * @param  node_handle: the node handle, node must be started
  * @retval  0 for no error
  */
static int8_t USB_AUDIO_Streaming_CLK_SRC_Stop( uint32_t node_handle)
{
  AUDIO_USB_ClockSrc_NodeTypeDef *clk;
  
  clk = (AUDIO_USB_ClockSrc_NodeTypeDef*)node_handle;
  clk->node.state = AUDIO_NODE_STOPPED;
  return 0;
}
#if (defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCES)
/**
  * @brief  USB_AUDIO_Streaming_CLK_SRC_SetCurFrequency
  *         set currentfrequency 
  * @param  frequency: Freq to set
  * @param  restart_req: Check if restart alternate function is needed
  * @param  node_handle: the Clock source node handle, node must be initialized
  * @retval  0 for no error
  */
static int8_t USB_AUDIO_Streaming_CLK_SRC_SetCurFrequency(uint32_t frequency, uint8_t* as_cnt_to_restart, uint8_t* as_list_to_restart, uint32_t node_handle)
{
  AUDIO_USB_ClockSrc_NodeTypeDef *clk;
  uint32_t best_freq;
  
  clk = (AUDIO_USB_ClockSrc_NodeTypeDef*)node_handle;
  
    
      best_freq = USB_AUDIO_Streaming_GetBestFrequence(frequency, clk->control_cbks.clock_freq_list,clk->control_cbks.clock_freq_count);
  if(clk->node.audio_description->frequence == best_freq)
  {
        *as_cnt_to_restart = 0;
         return 0;
  }
  else
  {
    clk->control_cbks.SetFrequency(frequency, as_cnt_to_restart, as_list_to_restart,  clk->control_cbks.private_data);
  }
  return 0;
}
#endif /*(defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCES)*/
/**
  * @brief  USB_AUDIO_Streaming_CLK_SRC_GetCurFrequency
  *         get current frequency  value
  * @param  frequency:             returned frequency value
  * @param  node_handle:        the the Clock source node handle, node must be initialized
  * @retval  0 for no error
  */
static int8_t USB_AUDIO_Streaming_CLK_SRC_GetCurFrequency(uint32_t* frequency, uint32_t node_handle)
{
  AUDIO_USB_ClockSrc_NodeTypeDef *clk;
  
  clk = (AUDIO_USB_ClockSrc_NodeTypeDef*)node_handle;
  *frequency = clk->node.audio_description->frequence;
  return 0; 
}

/**
  * @brief  USB_AUDIO_Streaming_CLK_SRC_GetFrequenciesList
  *         return supported frequencies list 
  * @param  frequencies:            returned frequencies list
  * @param  max_supported_count:     frequencies list size
  * @param  node_handle:        the Clock source node handle, node must be initialized
  * @retval  0 for no error
  */
static int8_t USB_AUDIO_Streaming_CLK_SRC_GetFrequenciesList(uint32_t** frequencies,uint8_t* freq_count, uint32_t node_handle)
{
  AUDIO_USB_ClockSrc_NodeTypeDef *clk;
  
  clk = (AUDIO_USB_ClockSrc_NodeTypeDef*)node_handle;
  *frequencies = clk->control_cbks.clock_freq_list;
  *freq_count = clk->control_cbks.clock_freq_count;
  return 0;
}
/**
  * @brief  USB_AUDIO_Streaming_CLK_SRC_GetIsValid
  *         check if clock is valid  
  * @param  is_valid:             returned isvalid value
  * @param  node_handle:        the Clock source node handle, node must be initialized
  * @retval  0 for no error
  */
static int8_t USB_AUDIO_Streaming_CLK_SRC_GetIsValid(uint8_t *is_valid, uint32_t node_handle)
{
  *is_valid = 1;
  return 0;
}
#endif /* USE_USB_AUDIO_CLASS_20 */


/**
  * @brief  AUDIO_USB_InitializesDataBuffer
  *         compute the  buffer size and add margin if needed
  * @param  buf: session               
  * @param  buffer_size:                
  * @param  packet_size:                
  * @param  margin:                    
  * @retval 0 if no error
  */
  void AUDIO_USB_InitializesDataBuffer(AUDIO_BufferTypeDef* buf, 
                                       uint32_t buffer_size, 
                                       uint16_t packet_size, uint16_t margin)
 {
    buf->size = ((int)((buffer_size - margin )
                       / packet_size)) * packet_size; 
    buf->rd_ptr = buf->wr_ptr = 0;
 }
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
