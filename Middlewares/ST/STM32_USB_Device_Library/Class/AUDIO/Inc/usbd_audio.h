/**
  ******************************************************************************
  * @file    usbd_audio.h
  * @author  MCD Application Team
  * @version V1.2.0RC3
  * @date    6-January-2018
  * @brief   header file for the usbd_audio.c file, it is new implementation of USB audio class 2.0 which 
  * supports more features.
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
#ifndef __USB_AUDIO_H
#define __USB_AUDIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup USBD_AUDIO
  * @brief This file is the Header file for usbd_audio.c
  * @{
  */ 


/** @defgroup USBD_AUDIO_Exported_Defines
  * @{
  */ 

#define USB_DEVICE_CLASS_AUDIO                        0x01U
#define AUDIO_SUBCLASS_AUDIOCONTROL                   0x01U

#define AUDIO_OUT_EP                0x03
#define AUDIO_IN_EP                 0x83
#define AUDIO_CTRL_IF               0x02
#define AUDIO_OUT_IF                0x03
#define AUDIO_IN_IF                 0x04
#define AUDIO_TOTAL_IF_NUM          0x03


#define USBD_AUDIO_ADC_BCD                                           0x0200
#define USBD_AUDIO_AUDIO_FUNCTION                                    0x01   
#define USBD_AUDIO_FUNCTION_SUBCLASS_UNDEFINED                       0x00
#define USBD_AUDIO_CLASS_CODE                                        0x01
/* Audio Interface Subclass Codes */
#define USBD_AUDIO_INTERFACE_SUBCLASS_AUDIOCONTROL                   0x01
#define USBD_AUDIO_INTERFACE_SUBCLASS_AUDIOSTREAMING                 0x02
#define USBD_AUDIO_INTERFACE_SUBCLASS_MIDISTREAMING                  0x03
/* Audio Interface Protocol Codes  */
#define USBD_AUDIO_INTERFACE_PROTOCOL_UNDEFINED                      0x00
#define USBD_AUDIO_INTERFACE_PROTOCOL_IP_VERSION_02_00               0x20
   
 
/* Table A-2: Audio Data Format Type I Bit Allocations */
#define USBD_AUDIO_FORMAT_TYPE_PCM                                   0x0001
/* Table A-1: Format Type Codes */
#define USBD_AUDIO_FORMAT_TYPE_I                                     0x01
#define USBD_AUDIO_FORMAT_TYPE_II                                    0x02
#define USBD_AUDIO_FORMAT_TYPE_III                                   0x03

/* Audio Descriptor Types */
#define USBD_AUDIO_DESC_TYPE_CS_DEVICE                               0x21
#define USBD_AUDIO_DESC_TYPE_CS_INTERFACE                            0x24
#define USBD_AUDIO_DESC_TYPE_CS_ENDPOINT                             0x25
#define USBD_AUDIO_DESC_TYPE_INTERFACE_ASSOC                         0x0B /* Interface association descriptor */


 /* audio specific descriptor size */
#define USBD_AUDIO_STANDARD_INTERFACE_DESC_SIZE                      0x09
#define USBD_AUDIO_INTERFACE_ASSOC_DESC_SIZE                         0x08
#define USB_AUDIO_DESC_SIZ                                           0x09
#define USBD_AUDIO_STANDARD_ENDPOINT_DESC_SIZE                       0x07
#define USBD_AUDIO_INTERRUPT_ENDPOINT_DESC_SIZE                      0x07
#define USBD_AUDIO_SPECIFIC_DATA_ENDPOINT_DESC_SIZE                  0x08
#define USBD_AUDIO_INPUT_TERMINAL_DESC_SIZE                          0x11
#define USBD_AUDIO_FEATURE_UNIT_DESC_SIZE(CH_NB)                    (0x06+(((CH_NB)+1)<<2))
#define USBD_AUDIO_OUTPUT_TERMINAL_DESC_SIZE                         0x0C
#define USBD_AUDIO_AS_CS_INTERFACE_DESC_SIZE                         0x10
#define USBD_AUDIO_AC_CS_INTERFACE_DESC_SIZE                         0x09
#define USBD_AUDIO_CLOCK_SOURCE_DESC_SIZE                            0x08
#define USBD_USBD_AUDIO_FORMAT_TYPE_I_DESC_SIZE                      0x06

#if USE_AUDIO_USB_INTERRUPT   
#define USBD_AUDIO_INTERRUPT_DATA_MESSAGE_SIZE                        6    
#endif /* USE_AUDIO_USB_INTERRUPT */
/* Feature Unit Descriptor bmaControls */
#define USBD_AUDIO_FU_CONTROL_MUTE                                   0x0003 /* D0..1 = 1*/
#define USBD_AUDIO_FU_CONTROL_VOLUME                                 0x000C /* D2..3 = 1*/
/* Audio Class-Specific Endpoint Descriptor Subtypes*/
#define USBD_AUDIO_SPECIFIC_EP_DESC_SUBTYPE_GENERAL                  0x01 /* EP_GENERAL */

#define USBD_EP_ATTR_ISOC_NOSYNC                                     0x00 /* attribute no synchro */
#define USBD_EP_ATTR_ISOC_ASYNC                                      0x04 /* attribute synchro by feedback  */
#define USBD_EP_ATTR_ISOC_ADAPT                                      0x08 /* attribute synchro adaptative  */
#define USBD_EP_ATTR_ISOC_SYNC                                       0x0C /* attribute synchro synchronous  */
#define USBD_EP_ATTR_USAGE_FEEDBACK                                  0x10 /* attribute synchro by feedback  */
   
    /* Clock Source Control Selectors */
#define USBD_AUDIO_CS_CONTROL_UNDEFINED                              0x00
#define USBD_AUDIO_CS_SAM_FREQ_CONTROL                               0x01
#define USBD_AUDIO_CS_CLOCK_VALID_CONTROL                            0x02
    /* Clock Selector Control Selectors */
#define USBD_AUDIO_CX_CONTROL_UNDEFINED                              0x00
#define USBD_AUDIO_CX_CLOCK_SELECTOR_CONTROL                         0x01
    /* Clock Multiplier Control Selectors */
#define USBD_AUDIO_CM_CONTROL_UNDEFINED                              0x00
#define USBD_AUDIO_CM_NUMERATOR_CONTROL                              0x01
#define USBD_AUDIO_CM_DENOMINATOR_CONTROL                            0x02 
  /* Feature Unit Control Selectors */
#define USBD_AUDIO_FU_MUTE_CONTROL                                    0x01 
#define USBD_AUDIO_FU_VOLUME_CONTROL                                  0x02 

  /* AudioStreaming Interface Control Selectors */
#define USBD_AUDIO_AS_CONTROL_UNDEFINED                               0x00
#define USBD_AUDIO_AS_ACT_ALT_SETTING_CONTROL                         0x01    
#define USBD_AUDIO_AS_VAL_ALT_SETTINGS_CONTROL                        0x02 
#define USBD_AUDIO_AS_AUDIO_DATA_FORMAT_CONTROL                       0x03  
/* configuration of current implementation of audio class */
#define USBD_AUDIO_AS_INTERFACE_COUNT                                 2
#define USBD_AUDIO_MAX_IN_EP                                          5
#define USBD_AUDIO_MAX_OUT_EP                                         5
#define USBD_AUDIO_MAX_AS_INTERFACE                                   2
#define USBD_AUDIO_EP_MAX_CONTROL                                     3
#define USBD_AUDIO_CONFIG_CONTROL_UNIT_COUNT                          4 /*2 feature unit and 2 clock*/
#define USBD_AUDIO_FEATURE_MAX_CONTROL                                2  
#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK
#ifdef USE_USB_FS
#define AUDIO_FEEDBACK_EP_PACKET_SIZE                                 0x03
#else /* USE_USB_HS*/
#define AUDIO_FEEDBACK_EP_PACKET_SIZE                                 0x04
#endif
#endif /*USBD_SUPPORT_AUDIO_OUT_FEEDBACK */


/**
  * @}
  */ 


/** @defgroup USBD_AUDIO_Exported_TypesDefinitions
  * @{
  */
/* Audio Control Interface Descriptor Subtypes */
typedef enum 
{
  USBD_AUDIO_CS_AC_SUBTYPE_UNDEFINED                               = 0x00,
  USBD_AUDIO_CS_AC_SUBTYPE_HEADER                                  = 0x01,
  USBD_AUDIO_CS_AC_SUBTYPE_INPUT_TERMINAL                          = 0x02,
  USBD_AUDIO_CS_AC_SUBTYPE_OUTPUT_TERMINAL                         = 0x03,
  USBD_AUDIO_CS_AC_SUBTYPE_MIXER_UNIT                              = 0x04,
  USBD_AUDIO_CS_AC_SUBTYPE_SELECTOR_UNIT                           = 0x05,
  USBD_AUDIO_CS_AC_SUBTYPE_FEATURE_UNIT                            = 0x06,
  USBD_AUDIO_CS_AC_SUBTYPE_EFFECT_UNIT                             = 0x07,
  USBD_AUDIO_CS_AC_SUBTYPE_PROCESSING_UNIT                         = 0x08,
  USBD_AUDIO_CS_AC_SUBTYPE_EXTENSION_UNIT                          = 0x09,
  USBD_AUDIO_CS_AC_SUBTYPE_CLOCK_SOURCE                            = 0x0A,
  USBD_AUDIO_CS_AC_SUBTYPE_CLOCK_SELECTOR                          = 0x0B,
  USBD_AUDIO_CS_AC_SUBTYPE_CLOCK_MULTIPLIER                        = 0x0C,
  USBD_AUDIO_CS_AC_SUBTYPE_SAMPLE_RATE_CONVERTER                   = 0x0D
}USBD_AUDIO_SpecificACInterfaceDescSubtypeTypeDef;

/* Audio Function Category Codes */
typedef enum 
{
  USBD_AUDIO_CATEGORY_UNDEFINED                                    = 0x00,
  USBD_AUDIO_CATEGORY_DESKTOP_SPEAKER                              = 0x01,
  USBD_AUDIO_CATEGORY_HOME_THEATER                                 = 0x02,
  USBD_AUDIO_CATEGORY_MICROPHONE                                   = 0x03,
  USBD_AUDIO_CATEGORY_HEADSET                                      = 0x04,
  USBD_AUDIO_CATEGORY_TELEPHONE                                    = 0x05,
  USBD_AUDIO_CATEGORY_CONVERTER                                    = 0x06,
  USBD_AUDIO_CATEGORY_VOICE_SOUND_RECORDER                         = 0x07,
  USBD_AUDIO_CATEGORY_I_O_BOX                                      = 0x08,
  USBD_AUDIO_CATEGORY_MUSICAL_INSTRUMENT                           = 0x09,
  USBD_AUDIO_CATEGORY_PRO_AUDIO                                    = 0x0A,
  USBD_AUDIO_CATEGORY_AUDIO_VIDEO                                  = 0x0B,
  USBD_AUDIO_CATEGORY_CONTROL_PANEL                                = 0x0C,
  USBD_AUDIO_CATEGORY_OTHER                                        = 0x0D
}USBD_AUDIO_FunctionCategoryTypeDef;

/* Audio Streaming Interface Descriptor Subtypes */
typedef enum 
{
  USBD_AUDIO_CS_SUBTYPE_AS_UNDEFINED                               = 0x00,
  USBD_AUDIO_CS_SUBTYPE_AS_GENERAL                                 = 0x01,
  USBD_AUDIO_CS_SUBTYPE_AS_FORMAT_TYPE                             = 0x02,
  USBD_AUDIO_CS_SUBTYPE_AS_ENCODER                                 = 0x03,
  USBD_AUDIO_CS_SUBTYPE_AS_DECODER                                 = 0x04,
}USBD_AUDIO_SpecificASInterfaceDescSubtypeTypeDef;
typedef enum 
{
  USBD_AUDIO_CS_REQ_UNDEFINED                                      = 0x00,
  USBD_AUDIO_CS_REQ_CUR                                            = 0x01,
  USBD_AUDIO_CS_REQ_RANGE                                          = 0x02,
  USBD_AUDIO_CS_REQ_MEM                                            = 0x03
}USBD_AUDIO_SpecificReqCodeTypeDef;


typedef enum
{
  USBD_AUDIO_TERMINAL_IO_USB_UNDEFINED                             = 0x0100 ,
  USBD_AUDIO_TERMINAL_IO_USB_STREAMING                             = 0x0101 ,
  USBD_AUDIO_TERMINAL_IO_USB_VENDOR_SPECIFIC                       = 0x01FF ,
  USBD_AUDIO_TERMINAL_I_UNDEFINED                                  = 0x0200 ,
  USBD_AUDIO_TERMINAL_I_MICROPHONE                                 = 0x0201 ,
  USBD_AUDIO_TERMINAL_I_DESKTOP_MICROPHONE                         = 0x0202 ,
  USBD_AUDIO_TERMINAL_O_UNDEFINED                                  = 0x0300 ,
  USBD_AUDIO_TERMINAL_O_SPEAKER                                    = 0x0301 ,
  USBD_AUDIO_TERMINAL_O_HEADPHONES                                 = 0x0302 
}USBD_AUDIOTerminalTypeDef;

typedef enum 
{
  USBD_AUDIO_INTERRUPT_INFO_VENDOR_SPECIFIC                        = 0x01,
  USBD_AUDIO_INTERRUPT_INFO_FROM_INTERFACE                         = 0x00,
  USBD_AUDIO_INTERRUPT_INFO_FROM_EP                                = 0x02
}USBD_AUDIO_InterruptSourceTypeDef;
typedef enum 
{
  USBD_AUDIO_INTERRUPT_ATTR_CUR                                    = USBD_AUDIO_CS_REQ_CUR,
  USBD_AUDIO_INTERRUPT_ATTR_RANGE                                  = USBD_AUDIO_CS_REQ_RANGE,
  USBD_AUDIO_INTERRUPT_ATTR_MEM                                    = USBD_AUDIO_CS_REQ_MEM
}USBD_AUDIO_InterruptATTRTypeDef; 
/* The feature Unit callbacks */
typedef struct
{
   int8_t  (*GetMute)    (uint16_t /*channel*/,uint8_t* /*mute*/, uint32_t /* privatedata*/);
   int8_t  (*SetMute)    (uint16_t /*channel*/,uint8_t /*mute*/, uint32_t /* privatedata*/);
   int8_t  (*SetCurVolume)    (uint16_t /*channel*/,uint16_t /*volume*/, uint32_t /* privatedata*/);
   int8_t  (*GetCurVolume)    (uint16_t /*channel*/,uint16_t* /*volume*/, uint32_t /* privatedata*/);
   uint16_t MaxVolume; 
   uint16_t MinVolume;
   uint16_t ResVolume;
   int8_t  (*GetStatus)     (uint32_t /*privatedata*/);
}USBD_AUDIO_FeatureControlCallbacksTypeDef;

/* The Clock Source callbacks */
typedef struct
{
   int8_t  (*IsValid)    (uint8_t* /*is_valid*/, uint32_t /* privatedata*/);
   int8_t  (*SetCurFrequency) (uint32_t /*freq*/,uint8_t* /*as_cnt_to_restart*/ ,uint8_t* /*as_list_to_restart*/, uint32_t /* privatedata*/);
   int8_t  (*GetCurFrequency) (uint32_t* /* freq*/, uint32_t /* privatedata*/);
   int8_t  (*GetFrequencyList) ( uint32_t** /*freq_list*/,uint8_t* /* max_supported_count*/, uint32_t /* privatedata*/);
}USBD_AUDIO_ClockSourceCallbacksTypeDef;

/* the Unit callbacks , used when a control is called (Get_Cur, Set Cur ....) */
typedef union
{
   USBD_AUDIO_FeatureControlCallbacksTypeDef* feature_control;
   USBD_AUDIO_ClockSourceCallbacksTypeDef* clk_src_control; 
}USBD_AUDIO_ControlCallbacksTypeDef;
/** Audio Unit  supported cmd and related callbacks */


/* Next strucure define an audio Unit/Clock */
typedef struct
{
    uint8_t id; /* Unit/Clock Id */
    USBD_AUDIO_SpecificACInterfaceDescSubtypeTypeDef type; /* type of Unit */
    uint16_t control_req_map; /* a map of requests GET_CUR, GET_MAX, ....*/
    uint16_t control_selector_map; /* List of supported control , for example Mute and Volume */
    USBD_AUDIO_ControlCallbacksTypeDef Callbacks; /* list of callbacks */
    uint32_t  private_data; /* used as the last arguement of each callback */
}USBD_AUDIO_ControlTypeDef;

/* Structure Define a data endpoint and it's callbacks */
 typedef struct 
 {
   uint8_t ep_num; 
   uint16_t control_name_map; /* a bitmap of supported commands : get_cur|set_cur ...*/
   uint16_t control_selector_map; /* a bitmap of supported controls : freq, pitsh ... */
   uint8_t* buf;
   uint16_t length;
   int8_t  (*DataReceived)     ( uint16_t/* data_len*/,uint32_t/* privatedata*/); /* called for OUT EP when data is received */
   uint8_t*  (*GetBuffer)    (uint32_t /* privatedata*/, uint16_t* packet_length); /* called for IN and OUt  EP to get working buffer */
   uint16_t  (*GetMaxPacketLength)    (uint32_t /*privatedata*/); /* Called beforre openeing the EP to get Max Size length */
   int8_t  (*GetState)     (uint32_t/*privatedata*/);
   uint32_t  private_data;/* used as the last arguement of each callback */
 }  USBD_AUDIO_EP_DataTypeDef;
 
 
#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK  
 /* Structure Define a feedback endpoint and it's callbacks */
 typedef struct 
 {
   uint8_t  ep_num; /* endpoint number */
   uint8_t feedback_data[AUDIO_FEEDBACK_EP_PACKET_SIZE]; /* buffer used to send feedback */
   uint32_t      (*GetFeedback)     (  uint32_t/* privatedata*/); /* return  count of played sample  since last ResetRate */
   uint32_t private_data;
 }  USBD_AUDIO_EP_SynchTypeDef;
#endif /*USBD_SUPPORT_AUDIO_OUT_FEEDBACK */
 
 
/* Strucure define Audio streaming interface */
typedef struct USBD_AUDIO_AS_Interface
{
    uint8_t interface_num; /* audio streaming interface num */
    uint8_t max_alternate; /* audio streaming interface most greate  alternate num */
    uint8_t alternate;/* audio streaming interface current  alternate  */
    USBD_AUDIO_EP_DataTypeDef data_ep; /* audio streaming interface main data EP  */
#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK     
    uint8_t synch_enabled;
    USBD_AUDIO_EP_SynchTypeDef synch_ep; /* synchro ep description */
#endif /*USBD_SUPPORT_AUDIO_OUT_FEEDBACK */
    void  (*SofReceived)     ( uint32_t/*privatedata*/);
    int8_t  (*SetAS_Alternate)     ( uint8_t/*alternate*/,uint32_t/*privatedata*/);
    int8_t  (*GetState)     (uint32_t/*privatedata*/);
    uint32_t  private_data; /* used as the last arguement of each callback */      
}USBD_AUDIO_AS_InterfaceTypeDef;

/* Structure define the whole audio function will be initialized by application*/
typedef struct
{
  uint8_t control_count; /* the count  of Unit controls */
  uint8_t as_interfaces_count;/* the count  of audio streaming interface */
  USBD_AUDIO_ControlTypeDef controls[USBD_AUDIO_CONFIG_CONTROL_UNIT_COUNT]; /* list of Unit control */
  USBD_AUDIO_AS_InterfaceTypeDef as_interfaces[USBD_AUDIO_AS_INTERFACE_COUNT];/* the list  of audio streaming interface */
#if USBD_AUDIO_SUPPORT_INTERRUPT   
  uint8_t interrupt_ep_num;
#endif /* USBD_AUDIO_SUPPORT_INTERRUPT  */
}USBD_AUDIO_FunctionDescriptionfTypeDef;

/* Structure define audio interface */
typedef struct
{
    int8_t  (*Init)         (USBD_AUDIO_FunctionDescriptionfTypeDef* /* as_desc*/ , uint32_t /*privatedata*/);
    int8_t  (*DeInit)       (USBD_AUDIO_FunctionDescriptionfTypeDef* /* as_desc*/,uint32_t /*privatedata*/);
    int8_t  (*GetConfigDesc) (uint8_t ** /*pdata*/, uint16_t * /*psize*/, uint32_t /*private_data*/);
    int8_t  (*GetState)     (uint32_t privatedata);
    uint32_t private_data;  
}USBD_AUDIO_InterfaceCallbacksfTypeDef;
 
#if USBD_AUDIO_SUPPORT_INTERRUPT

  typedef enum 
 {
  USBD_AUDIO_NOT_USED_PRIORITY                          = 0x00,
  USBD_AUDIO_HIGH_PRIORITY                              = 0x02,
  USBD_AUDIO_NORMAL_PRIORITY                            = 0x04,
  USBD_AUDIO_LOW_PRIORITY                               = 0x08

 }  USBD_AUDIO_InterruptPriorityTypeDef;
/* Interrupt Data Message */
 typedef struct 
 {
   USBD_AUDIO_InterruptSourceTypeDef  type; /* interrupt type */
   USBD_AUDIO_InterruptATTRTypeDef  attr; /* The attribute that caused the interrupt*/
   uint8_t cs; /* control selector */
   uint8_t cn_mcn; /* Channel Number*/
   uint8_t entity_id; /* Entity ID or zero .*/
   uint8_t ep_if_id; /*  Interface or Endpoint.*/
   USBD_AUDIO_InterruptPriorityTypeDef priority;
 }  USBD_AUDIO_InterruptTypeDef;
#endif /* USBD_AUDIO_SUPPORT_INTERRUPT  */
/**
  * @}
  */ 



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 
 /* to use in configuration descriptor */
#define AUDIO_2_L2_CUR_VAL_TO_DATA(vcur, bytes)      do{\
                                                                (bytes)[0]= (uint8_t)(vcur);\
                                                                (bytes)[1]= (uint8_t)(((vcur) >> 8));\
                                               }while(0);
#define AUDIO_2_L3_CUR_VAL_TO_DATA(vcur, bytes)      do{\
                                                                (bytes)[0]= (uint8_t)(vcur);\
                                                                (bytes)[1]= (uint8_t)(((vcur) >> 8));\
                                                                (bytes)[2]= (uint8_t)(((vcur) >> 16));\
                                                                (bytes)[3]= (uint8_t)(((vcur) >> 24));\
                                               }while(0);
#define AUDIO_2_L2_RNG_VAL_TO_DATA(vmax, vmin, vres , bytes)      do{\
                                                                (bytes)[0]= (uint8_t)(vmax);\
                                                                (bytes)[1]= (uint8_t)(((vmax) >> 8));\
                                                                (bytes)[2]= (uint8_t)(vmin);\
                                                                (bytes)[3]= (uint8_t)(((vmin) >> 8));\
                                                                (bytes)[4]= (uint8_t)(vres);\
                                                                (bytes)[5]= (uint8_t)(((vres) >> 8));\
                                               }while(0);
#define AUDIO_2_L3_RNG_VAL_TO_DATA(vmax, vmin, vres , bytes)      do{\
                                                                (bytes)[0]= (uint8_t)(vmax);\
                                                                (bytes)[1]= (uint8_t)(((vmax) >> 8));\
                                                                (bytes)[2]= (uint8_t)(((vmax) >> 16));\
                                                                (bytes)[3]= (uint8_t)(((vmax) >> 24));\
                                                                (bytes)[4]= (uint8_t)(vmin);\
                                                                (bytes)[5]= (uint8_t)(((vmin) >> 8));\
                                                                (bytes)[6]= (uint8_t)(((vmin) >> 16));\
                                                                (bytes)[7]= (uint8_t)(((vmin) >> 24));\
                                                                (bytes)[8]= (uint8_t)(vres);\
                                                                (bytes)[9]= (uint8_t)(((vres) >> 8));\
                                                                (bytes)[10]= (uint8_t)(((vres) >> 16));\
                                                                (bytes)[11]= (uint8_t)(((vres) >> 24));\
                                               }while(0);


#define AUDIO_2_L2_CUR_DATA_TO_VAL(bytes)       (((uint16_t)((bytes)[1]))<<8)|(((uint16_t)((bytes)[0])))
#define AUDIO_2_L3_CUR_DATA_TO_VAL(bytes)      (uint32_t)((((uint32_t)((bytes)[3]))<<24)|(((uint32_t)((bytes)[2]))<<16)| (((uint32_t)((bytes)[1]))<<8)| (((uint32_t)((bytes)[0]))))


/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 

extern USBD_ClassTypeDef  USBD_AUDIO;
#define USBD_AUDIO_CLASS    &USBD_AUDIO
/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */ 
uint8_t  USBD_AUDIO_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                        USBD_AUDIO_InterfaceCallbacksfTypeDef *aifc);
#if USBD_AUDIO_SUPPORT_INTERRUPT
uint8_t  USBD_AUDIO_SendInterrupt  (USBD_AUDIO_InterruptTypeDef *interrupt);
#endif /* USBD_AUDIO_SUPPORT_INTERRUPT  */

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif  /* __USB_AUDIO_H */
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
