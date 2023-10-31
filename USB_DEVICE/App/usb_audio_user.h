/**
  ******************************************************************************
  * @file    usb_audio_user.h
  * @author  MCD Application Team
  * @version V1.2.0RC3
  * @date    6-January-2018
  * @brief   USB audio application configuration.
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
#ifndef __USB_AUDIO_USER_H
#define __USB_AUDIO_USER_H

#ifdef __cplusplus
 extern "C" {
#endif
/* includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "audio_node.h"
/* Exported constants --------------------------------------------------------*/
/* list of frequencies*/
#define USB_AUDIO_CONFIG_FREQ_192_K  192000
#define USB_AUDIO_CONFIG_FREQ_96_K   96000
#define USB_AUDIO_CONFIG_FREQ_48_K   48000 
#define USB_AUDIO_CONFIG_FREQ_44_1_K 44100
#define USB_AUDIO_CONFIG_FREQ_16_K   16000
#define USB_AUDIO_CONFIG_FREQ_8_K    8000 

#define USB_IRQ_PREPRIO 3
#ifdef USE_USB_FS_INTO_HS
#define USB_FIFO_WORD_SIZE  320
#else /* USE_USB_FS_INTO_HS */
#ifdef USE_USB_FS
#define USB_FIFO_WORD_SIZE  320
#else  /*  USE_USB_FS */
#define USB_FIFO_WORD_SIZE  1000
#endif  /*  USE_USB_FS */
#endif  /* USE_USB_FS_INTO_HS */

#ifdef USE_USB_AUDIO_PLAYPBACK
/*play session : list of terminal and unit id for audio function */
/* must be greater than the highest interface number(to avoid request destination confusion */
#define USB_AUDIO_CONFIG_PLAY_TERMINAL_INPUT_ID       0x12
#define USB_AUDIO_CONFIG_PLAY_UNIT_FEATURE_ID         0x16
#define USB_AUDIO_CONFIG_PLAY_TERMINAL_OUTPUT_ID      0x14
#define USB_AUDIO_CONFIG_PLAY_CLOCK_SOURCE_ID         0x18

#define USBD_AUDIO_CONFIG_PLAY_CHANNEL_COUNT          0x02 /* channels Left dn right */
#define USBD_AUDIO_CONFIG_PLAY_CHANNEL_MAP            0x03 /* channels Left dn right */

#ifdef USE_AUDIO_PLAYPBACK_24_BIT
#define USBD_AUDIO_CONFIG_PLAY_RES_BIT                0x18 /* 24 bit per sample */
#define USBD_AUDIO_CONFIG_PLAY_RES_BYTE               0x03 /* 3 bytes */
#else /*  USE_AUDIO_PLAYPBACK_24_BIT  */
#define USBD_AUDIO_CONFIG_PLAY_RES_BIT                0x10 /* 16 bit per sample */
#define USBD_AUDIO_CONFIG_PLAY_RES_BYTE               0x02 /* 2 bytes */
#endif /*  USE_AUDIO_PLAYPBACK_24_BIT  */
   
#ifdef USE_AUDIO_USB_PLAY_MULTI_FREQUENCES
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_192_K           0 /* to set by user  1 : to use , 0 to not support*/
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_96_K           1 /* to set by user  1 : to use , 0 to not support*/
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_48_K           1 /* to set by user  1 : to use , 0 to not support*/
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K         1 /* to set by user  1 : to use , 0 to not support*/
#if USB_AUDIO_CONFIG_PLAY_USE_FREQ_192_K
#define USB_AUDIO_CONFIG_PLAY_FREQ_MAX   USB_AUDIO_CONFIG_FREQ_192_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_96_K
#define USB_AUDIO_CONFIG_PLAY_FREQ_MAX   USB_AUDIO_CONFIG_FREQ_96_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_48_K
#define USB_AUDIO_CONFIG_PLAY_FREQ_MAX   USB_AUDIO_CONFIG_FREQ_48_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K
#define USB_AUDIO_CONFIG_PLAY_FREQ_MAX   USB_AUDIO_CONFIG_FREQ_44_1_K
#else
#error "Playback frequency is missed"
#endif 
#if USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K
#define USB_AUDIO_CONFIG_PLAY_FREQ_MIN   USB_AUDIO_CONFIG_FREQ_44_1_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_48_K
#define USB_AUDIO_CONFIG_PLAY_FREQ_MIN   USB_AUDIO_CONFIG_FREQ_48_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_96_K
#define USB_AUDIO_CONFIG_PLAY_FREQ_MIN   USB_AUDIO_CONFIG_FREQ_96_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_192_K
#define USB_AUDIO_CONFIG_PLAY_FREQ_MIN   USB_AUDIO_CONFIG_FREQ_192_K
#endif 

#define USB_AUDIO_CONFIG_PLAY_FREQ_COUNT              (USB_AUDIO_CONFIG_PLAY_USE_FREQ_192_K + USB_AUDIO_CONFIG_PLAY_USE_FREQ_96_K +\
                                                       USB_AUDIO_CONFIG_PLAY_USE_FREQ_48_K + USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K)
#define USB_AUDIO_CONFIG_PLAY_DEF_FREQ                USB_AUDIO_CONFIG_PLAY_FREQ_MAX


#else /* USE_AUDIO_USB_PLAY_MULTI_FREQUENCES */
#define USB_AUDIO_CONFIG_PLAY_FREQ_COUNT  1
#define USB_AUDIO_CONFIG_PLAY_FREQ_MAX USB_AUDIO_CONFIG_PLAY_DEF_FREQ
#define USB_AUDIO_CONFIG_PLAY_DEF_FREQ  USB_AUDIO_CONFIG_FREQ_48_K //change the freqeuncy for audio
#if (USB_AUDIO_CONFIG_PLAY_DEF_FREQ == USB_AUDIO_CONFIG_FREQ_44_1_K)
#define  USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K     1
#endif /* (USB_AUDIO_CONFIG_PLAY_DEF_FREQ == USB_AUDIO_CONFIG_FREQ_44_1_K)*/
#endif /* USE_AUDIO_USB_PLAY_MULTI_FREQUENCES*/
   
#define  USBD_AUDIO_CONFIG_PLAY_BUFFER_SIZE (1024 * 10)  
#else /* USE_AUDIO_PLAYPBACK */
#ifndef  USE_USB_AUDIO_RECORDING
#error "USE_USB_AUDIO_RECORDING or(and) USE_AUDIO_PLAYPBACK must be defined"
#endif /* USE_USB_AUDIO_RECORDING*/
#endif /*USE_AUDIO_PLAYPBACK*/


#ifdef  USE_USB_AUDIO_RECORDING   
/*record session : list of terminal and unit id for audio function */
/* must be greater than the highest interface number(to avoid request destination confusion */
#define USB_AUDIO_CONFIG_RECORD_TERMINAL_INPUT_ID     0x011
#define USB_AUDIO_CONFIG_RECORD_UNIT_FEATURE_ID       0x015
#define USB_AUDIO_CONFIG_RECORD_TERMINAL_OUTPUT_ID    0x013
#ifdef USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC
#ifdef USE_USB_AUDIO_PLAYPBACK 
#define USB_AUDIO_CONFIG_RECORD_CLOCK_SOURCE_ID       USB_AUDIO_CONFIG_PLAY_CLOCK_SOURCE_ID
#else  /* USE_USB_AUDIO_PLAYPBACK */
#error "to support USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC activate playback , in the\
        standalone recording deactivate  USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC flag"
#endif /* USE_USB_AUDIO_PLAYPBACK */ 
#else  /* USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC */
#define USB_AUDIO_CONFIG_RECORD_CLOCK_SOURCE_ID       0x019
#endif /* USE_AUDIO_PLAYBACK_RECORDING_SHARED_CLOCK_SRC */

#define  USBD_AUDIO_CONFIG_RECORD_BUFFER_SIZE         (1024 * 2) 
  
/*record session : audio description */
#define USBD_AUDIO_CONFIG_RECORD_CHANNEL_COUNT        0x02 /* channels Left dn right */
#define USBD_AUDIO_CONFIG_RECORD_CHANNEL_MAP          0x03 /* channels Left dn right */

#ifdef USE_AUDIO_RECORDING_24_BIT
#define USBD_AUDIO_CONFIG_RECORD_RES_BIT              0x18 /* 24 bit per sample */
#define USBD_AUDIO_CONFIG_RECORD_RES_BYTE             0x03 /* 3 bytes */
#else
#define USBD_AUDIO_CONFIG_RECORD_RES_BIT              0x10 /* 16 bit per sample */
#define USBD_AUDIO_CONFIG_RECORD_RES_BYTE             0x02 /* 2 bytes */
#endif

  
#ifdef USE_AUDIO_USB_RECORD_MULTI_FREQUENCES
#define USB_AUDIO_CONFIG_RECORD_USE_FREQ_192_K       0  /* to set by user  1 : to use , 0 to not support*/
#define USB_AUDIO_CONFIG_RECORD_USE_FREQ_96_K        0  /* to set by user  1 : to use , 0 to not support*/
#define USB_AUDIO_CONFIG_RECORD_USE_FREQ_48_K        1  /* to set by user  1 : to use , 0 to not support*/
#define USB_AUDIO_CONFIG_RECORD_USE_FREQ_44_1_K      0  /* to set by user  1 : to use , 0 to not support*/
#define USB_AUDIO_CONFIG_RECORD_USE_FREQ_16_K        1  /* to set by user  1 : to use , 0 to not support*/
#if USB_AUDIO_CONFIG_RECORD_USE_FREQ_192_K
#define USB_AUDIO_CONFIG_RECORD_FREQ_MAX   USB_AUDIO_CONFIG_FREQ_192_K
#elif USB_AUDIO_CONFIG_RECORD_USE_FREQ_96_K
#define USB_AUDIO_CONFIG_RECORD_FREQ_MAX   USB_AUDIO_CONFIG_FREQ_96_K
#elif USB_AUDIO_CONFIG_RECORD_USE_FREQ_48_K
#define USB_AUDIO_CONFIG_RECORD_FREQ_MAX   USB_AUDIO_CONFIG_FREQ_48_K
#elif USB_AUDIO_CONFIG_RECORD_USE_FREQ_44_1_K
#define USB_AUDIO_CONFIG_RECORD_FREQ_MAX   USB_AUDIO_CONFIG_FREQ_44_1_K
#else
#error "recording frequency is missed"
#endif 
#if USB_AUDIO_CONFIG_RECORD_USE_FREQ_44_1_K
#define USB_AUDIO_CONFIG_RECORD_FREQ_MIN   USB_AUDIO_CONFIG_FREQ_44_1_K
#elif USB_AUDIO_CONFIG_RECORD_USE_FREQ_48_K
#define USB_AUDIO_CONFIG_RECORD_FREQ_MIN   USB_AUDIO_CONFIG_FREQ_48_K
#elif USB_AUDIO_CONFIG_RECORD_USE_FREQ_96_K
#define USB_AUDIO_CONFIG_RECORD_FREQ_MIN   USB_AUDIO_CONFIG_FREQ_96_K
#elif USB_AUDIO_CONFIG_RECORD_USE_FREQ_192_K
#define USB_AUDIO_CONFIG_RECORD_FREQ_MIN   USB_AUDIO_CONFIG_FREQ_192_K
#endif 
#define USB_AUDIO_CONFIG_RECORD_FREQ_COUNT           (USB_AUDIO_CONFIG_RECORD_USE_FREQ_192_K + USB_AUDIO_CONFIG_RECORD_USE_FREQ_96_K + \
                                                      USB_AUDIO_CONFIG_RECORD_USE_FREQ_48_K + \
                                                     USB_AUDIO_CONFIG_RECORD_USE_FREQ_44_1_K + USB_AUDIO_CONFIG_RECORD_USE_FREQ_16_K)
#define USB_AUDIO_CONFIG_RECORD_DEF_FREQ             USB_AUDIO_CONFIG_RECORD_FREQ_MAX

#else /*USE_AUDIO_USB_RECORD_MULTI_FREQUENCES*/
#define USB_AUDIO_CONFIG_RECORD_FREQ_COUNT           1 /* 1 frequence */
#define USB_AUDIO_CONFIG_RECORD_FREQ_MAX             USB_AUDIO_CONFIG_RECORD_DEF_FREQ
#define USB_AUDIO_CONFIG_RECORD_DEF_FREQ             USB_AUDIO_CONFIG_FREQ_48_K /* to set by user */
#if (USB_AUDIO_CONFIG_RECORD_DEF_FREQ == USB_AUDIO_CONFIG_FREQ_44_1_K)
#define  USB_AUDIO_CONFIG_RECORD_USE_FREQ_44_1_K     1
#endif /* (USB_AUDIO_CONFIG_RECORD_DEF_FREQ == USB_AUDIO_CONFIG_FREQ_44_1_K)*/
#endif /*USE_AUDIO_USB_RECORD_MULTI_FREQUENCES*/
#endif /* USE_USB_AUDIO_RECORDING*/
   
/* defining the max packet length*/
#ifdef USE_USB_AUDIO_PLAYPBACK
#ifdef  USE_AUDIO_PLAYBACK_USB_FEEDBACK
#define USBD_AUDIO_CONFIG_PLAY_MAX_PACKET_SIZE ((uint16_t)(AUDIO_USB_MAX_PACKET_SIZE((USB_AUDIO_CONFIG_PLAY_FREQ_MAX+ 1),\
      USBD_AUDIO_CONFIG_PLAY_CHANNEL_COUNT,\
      USBD_AUDIO_CONFIG_PLAY_RES_BYTE)))
#else /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */
#define USBD_AUDIO_CONFIG_PLAY_MAX_PACKET_SIZE ((uint16_t)(AUDIO_USB_MAX_PACKET_SIZE((USB_AUDIO_CONFIG_PLAY_FREQ_MAX),\
      USBD_AUDIO_CONFIG_PLAY_CHANNEL_COUNT,\
      USBD_AUDIO_CONFIG_PLAY_RES_BYTE)))
#endif /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */
#endif /* USE_USB_AUDIO_PLAYPBACK */

#ifdef USE_USB_AUDIO_RECORDING
#ifdef  USE_AUDIO_RECORDING_USB_NO_REMOVE
#define USBD_AUDIO_CONFIG_RECORD_MAX_PACKET_SIZE ((uint16_t)(AUDIO_USB_MAX_PACKET_SIZE((USB_AUDIO_CONFIG_RECORD_FREQ_MAX+1),\
      USBD_AUDIO_CONFIG_RECORD_CHANNEL_COUNT,\
      USBD_AUDIO_CONFIG_RECORD_RES_BYTE)))
#else /*USE_AUDIO_RECORDING_USB_NO_REMOVE */
#define USBD_AUDIO_CONFIG_RECORD_MAX_PACKET_SIZE ((uint16_t)(AUDIO_USB_MAX_PACKET_SIZE(USB_AUDIO_CONFIG_RECORD_FREQ_MAX,\
      USBD_AUDIO_CONFIG_RECORD_CHANNEL_COUNT,\
      USBD_AUDIO_CONFIG_RECORD_RES_BYTE)))
#endif /*USE_AUDIO_RECORDING_USB_NO_REMOVE*/
#endif /*USE_USB_AUDIO_RECORDING*/
/* endpoint& streaming interface numbers definitions*/
#ifdef USE_USB_AUDIO_PLAYPBACK
#define USBD_AUDIO_CONFIG_PLAY_SA_INTERFACE              0x03 /* AUDIO STREAMING INTERFACE NUMBER FOR PLAY SESSION */
#define USBD_AUDIO_CONFIG_PLAY_EP_OUT                    0x03
#ifdef USE_AUDIO_PLAYBACK_USB_FEEDBACK   
#define USB_AUDIO_CONFIG_PLAY_EP_SYNC                    0x85
#ifdef USE_USB_AUDIO_CLASS_10
#define USB_AUDIO_CONFIG_PLAY_FEEDBACK_REFRESH           0x07 /* refresh every 32(2^5) ms */
#endif /* USE_USB_AUDIO_CLASS_10 */
#ifdef USE_USB_AUDIO_CLASS_20
#define USB_AUDIO_CONFIG_PLAY_FEEDBACK_REFRESH           1 /* refresh every 32(2^5) microframe/ms */
#endif /* USE_USB_AUDIO_CLASS_20 */
#endif /* USE_AUDIO_PLAYBACK_USB_FEEDBACK  */
#ifdef USE_USB_AUDIO_RECORDING
#define USBD_AUDIO_CONFIG_RECORD_SA_INTERFACE            0x04 /* AUDIO STREAMING INTERFACE NUMBER FOR RECORD SESSION */
#define USB_AUDIO_CONFIG_RECORD_EP_IN                    0x83
#if USE_AUDIO_USB_INTERRUPT
#define USB_AUDIO_CONFIG_INTERRUPT_EP_IN                 0x84
#endif /* USE_AUDIO_USB_INTERRUPT */
#else /* USE_USB_AUDIO_RECORDING */
#if USE_AUDIO_USB_INTERRUPT
#define USB_AUDIO_CONFIG_INTERRUPT_EP_IN                 0x82
#endif /* USE_AUDIO_USB_INTERRUPT */
#endif /* USE_USB_AUDIO_RECORDING */
#else /* USE_USB_AUDIO_PLAYPBACK */ 
#define USBD_AUDIO_CONFIG_RECORD_SA_INTERFACE            0x04 /* AUDIO STREAMING INTERFACE NUMBER FOR RECORD SESSION */
#define USB_AUDIO_CONFIG_RECORD_EP_IN                    0x83
#if USE_AUDIO_USB_INTERRUPT
#define USB_AUDIO_CONFIG_INTERRUPT_EP_IN                 0x82
#endif /* USE_AUDIO_USB_INTERRUPT */
#endif /* USE_USB_AUDIO_PLAYPBACK */

#if USE_AUDIO_USB_INTERRUPT
#define USB_AUDIO_CONFIG_INTERRUPT_EP_REFRESH        100 /*  */
#endif /* USE_AUDIO_USB_INTERRUPT */

/* Exported types ------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
   uint16_t USB_AUDIO_GetConfigDescriptor(uint8_t **desc);
   void Error_Handler(void);
#ifdef __cplusplus
}
#endif

#endif /* __USB_AUDIO_USER_H */
 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
