/**
  ******************************************************************************
  * @file    usbd_audio.c
  * @author  MCD Application Team
  * @version V1.2.0RC3
  * @date    6-January-2018
  * @brief   This file provides the Audio core functions.
  *
  * @verbatim
  *      
  *          ===================================================================      
  *                                AUDIO Class  Description
  *          ===================================================================
 *           This driver manages the Audio Class 2.0 following the "USB Device Class Definition for
  *           Audio Devices V2.0 Mai 31, 2006".
  *           It is new implementation of USB audio class which supports more features.
  *           This driver implements the following aspects of the specification:
  *             - Standard AC Interface Descriptor management
  *             - 2 Audio Streaming Interface (with single channel, PCM, Stereo mode)
  *           
  *
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
#include "usbd_ctlreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_AUDIO 
  * @brief usbd core module
  * @{
  */ 
#if USBD_AUDIO_SUPPORT_INTERRUPT
#define USBD_AUDIO_INTERRUPT_TABLE_SIZE               10
#endif /* USBD_AUDIO_SUPPORT_INTERRUPT */
/** @defgroup USBD_AUDIO_Private_TypesDefinitions
  * @{
  */ 
   typedef enum
{
  USBD_AUDIO_DATA_EP,
  USBD_AUDIO_FEEDBACK_EP,
  USBD_AUDIO_INTERRUPT_EP
}USBD_AUDIO_EpUsageTypeDef;
  
    /* Structure define ep:  description and state */
    typedef struct
{
  union
  {
    USBD_AUDIO_EP_DataTypeDef* data_ep;
#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK  
    USBD_AUDIO_EP_SynchTypeDef* sync_ep;
#endif /* USBD_SUPPORT_AUDIO_OUT_FEEDBACK */ 
  }ep_description;
  USBD_AUDIO_EpUsageTypeDef ep_usage;
  uint8_t open; /* 0 closed , 1 open */
  uint16_t max_packet_length; /* the max packet length */
  uint16_t tx_rx_soffn;
#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK 
#endif /*USBD_SUPPORT_AUDIO_OUT_FEEDBACK */
}USBD_AUDIO_EPTypeDef;

/* Structure define audio class data */
typedef struct 
{
  USBD_AUDIO_FunctionDescriptionfTypeDef aud_function; /* description of audio function */
  USBD_AUDIO_EPTypeDef ep_in[USBD_AUDIO_MAX_IN_EP]; /*  list of IN EP */
  USBD_AUDIO_EPTypeDef ep_out[USBD_AUDIO_MAX_OUT_EP]; /*  list of OUT EP */ 

  /* Strcture used for control handeling */
  struct
  {
    union
    {
      USBD_AUDIO_ControlTypeDef *controller; /* related Control Unit */
    } entity;
    uint8_t request_target;
    uint8_t data[USB_MAX_EP0_SIZE];  /* buffer to receive request value or send response */
    uint32_t len; /* used length of data buffer */
    uint16_t  wValue;/* wValue of request which is specific for each control*/
    uint8_t  req;/* the request type specific for each unit*/
  }last_control;
#if USBD_AUDIO_SUPPORT_INTERRUPT
  USBD_AUDIO_InterruptTypeDef interrupts[USBD_AUDIO_INTERRUPT_TABLE_SIZE];
  uint8_t is_ep_busy;
  uint8_t interrupt_message[USBD_AUDIO_INTERRUPT_DATA_MESSAGE_SIZE+4];
#endif /* USBD_AUDIO_SUPPORT_INTERRUPT */
}USBD_AUDIO_HandleTypeDef;

/**
  * @}
  */ 


/** @defgroup USBD_AUDIO_Private_Defines
  * @{
  */
#if USBD_AUDIO_SUPPORT_INTERRUPT
#define USBD_AUDIO_INTERRUPT_MESSAGE_BINFO_OFFSET 0
#define USBD_AUDIO_INTERRUPT_MESSAGE_BATTRIBUTE_OFFSET 1
#define USBD_AUDIO_INTERRUPT_MESSAGE_WVALUE_OFFSET 2
#define USBD_AUDIO_INTERRUPT_MESSAGE_WINDEX_OFFSET 4
#endif /*USBD_AUDIO_SUPPORT_INTERRUPT*/
#define AUDIO_UNIT_CONTROL_REQUEST 0x01
#define AUDIO_EP_REQUEST 0x02
#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK 
#define USBD_AUDIO_SOF_COUNT_FEEDBACK_BITS 7
#define USBD_AUDIO_SOF_COUNT_FEEDBACK (1 << USBD_AUDIO_SOF_COUNT_FEEDBACK_BITS)
#endif /*USBD_SUPPORT_AUDIO_OUT_FEEDBACK */

/**
  * @}
  */ 

/** @defgroup USBD_AUDIO_Private_Macros
  * @{
  */ 

                                         
/**
  * @}
  */ 




/** @defgroup USBD_AUDIO_Private_FunctionPrototypes
  * @{
  */

static uint8_t  USBD_AUDIO_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx);

static uint8_t  USBD_AUDIO_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx);

static uint8_t  USBD_AUDIO_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req);

static uint8_t  *USBD_AUDIO_GetCfgDesc (uint16_t *length);

static uint8_t  *USBD_AUDIO_GetDeviceQualifierDesc (uint16_t *length);

static uint8_t  USBD_AUDIO_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_AUDIO_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_AUDIO_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_AUDIO_EP0_TxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_AUDIO_SOF (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_AUDIO_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_AUDIO_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t AUDIO_REQ(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK
#ifdef USE_USB_FS
#define get_usb_speed_rate  get_usb_full_speed_rate
static  uint32_t get_usb_full_speed_rate(unsigned int rate, unsigned char * buf);
#else /* USE_USB_HS*/
#define get_usb_speed_rate  get_usb_high_speed_rate
static  uint32_t get_usb_high_speed_rate(unsigned int rate, unsigned char * buf);
#endif
#endif /*USBD_SUPPORT_AUDIO_OUT_FEEDBACK */
#if USBD_AUDIO_SUPPORT_INTERRUPT
static uint8_t  USBD_AUDIO_TransmitInterrupt(void);
#endif /*USBD_AUDIO_SUPPORT_INTERRUPT*/
static uint8_t  USBD_AUDIO_SetInterfaceAlternate(USBD_HandleTypeDef *pdev,uint8_t as_interface_num,uint8_t new_alt);

/**
  * @}
  */ 

/** @defgroup USBD_AUDIO_Private_Variables
  * @{
  */ 

USBD_ClassTypeDef  USBD_AUDIO = 
{
  USBD_AUDIO_Init,
  USBD_AUDIO_DeInit,
  USBD_AUDIO_Setup,
  USBD_AUDIO_EP0_TxReady,  
  USBD_AUDIO_EP0_RxReady,
  USBD_AUDIO_DataIn,
  USBD_AUDIO_DataOut,
  USBD_AUDIO_SOF,
  USBD_AUDIO_IsoINIncomplete,
  USBD_AUDIO_IsoOutIncomplete,
  #ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#endif
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc, 
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetDeviceQualifierDesc,
};
/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END=
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

static uint8_t *USBD_AUDIO_CfgDesc=0;
static uint16_t USBD_AUDIO_CfgDescSize=0;
#if USBD_AUDIO_SUPPORT_INTERRUPT
  USBD_HandleTypeDef   *pdev_audio = 0;
#endif /* USBD_AUDIO_SUPPORT_INTERRUPT */

/**
  * @}
  */ 

/** @defgroup USBD_AUDIO_Private_Functions
  * @{
  */ 

/**
  * @brief  USBD_AUDIO_Init
  *         Initialize the AUDIO interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index , not used
  * @retval status
  */
static uint8_t  USBD_AUDIO_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
  /* Allocate Audio structure */
  USBD_AUDIO_HandleTypeDef   *haudio;
  USBD_AUDIO_InterfaceCallbacksfTypeDef * aud_if_cbks;
  
  haudio = USBD_malloc(sizeof (USBD_AUDIO_HandleTypeDef));
  if(haudio == NULL)
  {
    return USBD_FAIL; 
  }
  else
  {
    memset(haudio, 0, sizeof(USBD_AUDIO_HandleTypeDef));
    pdev->pClassDataCmsit[pdev->classId] = (void *)haudio;
    pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];
    aud_if_cbks = (USBD_AUDIO_InterfaceCallbacksfTypeDef *)pdev->pUserData[pdev->classId];
    /* Initialize the Audio output Hardware layer */
    if (aud_if_cbks->Init(&haudio->aud_function,aud_if_cbks->private_data)!= USBD_OK)
    {
      USBD_free(pdev->pClassData);
      pdev->pClassData = 0;
      return USBD_FAIL;
    }
  }

  return USBD_OK;
}

static uint8_t AUDIOOutEpAdd = AUDIO_OUT_EP;
/**
  * @brief  USBD_AUDIO_Init
  *         DeInitialize the AUDIO layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index, not used 
  * @retval status
  */
static uint8_t  USBD_AUDIO_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
    USBD_AUDIO_HandleTypeDef   *haudio;
    USBD_AUDIO_InterfaceCallbacksfTypeDef * aud_if_cbks;
    
#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  AUDIOOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_ISOC, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];
    aud_if_cbks =  (USBD_AUDIO_InterfaceCallbacksfTypeDef *)pdev->pUserData;
    
    /* Close open EP */
    for(int i=1;i < USBD_AUDIO_MAX_IN_EP; i++)
    {
      if(haudio->ep_in[i].open)
      {
        USBD_LL_CloseEP(pdev, i|0x80);
      }
    }

    for(int i=1;i < USBD_AUDIO_MAX_OUT_EP; i++)
    {
      if(haudio->ep_out[i].open)
      {
        USBD_LL_CloseEP(pdev, i);
      }
    }
  /* DeInit  physical Interface components */
  if(haudio != NULL)
  {
   aud_if_cbks->DeInit(&haudio->aud_function,aud_if_cbks->private_data);
    USBD_free(haudio);
    pdev->pClassDataCmsit[pdev->classId]  = NULL;
    
  }
  
  return USBD_OK;
}


/**
  * @brief  USBD_AUDIO_SetInterfaceAlternate
  *         Set the Alternate interface of a streaming interface
  * @param  pdev: device instance
  * @param  as_interface_num: audio streaming interface number
  * @param  new_alt: new alternate number
  * @retval status
  */
static uint8_t  USBD_AUDIO_SetInterfaceAlternate(USBD_HandleTypeDef *pdev,uint8_t as_interface_num,uint8_t new_alt)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  USBD_AUDIO_AS_InterfaceTypeDef* pas_interface;
  USBD_AUDIO_EPTypeDef * ep;
  
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];
  pas_interface = &haudio->aud_function.as_interfaces[as_interface_num];
  ep = (pas_interface->data_ep.ep_num&0x80)?&haudio->ep_in[pas_interface->data_ep.ep_num&0x0F]:
                                            &haudio->ep_out[pas_interface->data_ep.ep_num];
  
  
  /* close old alternate interface */
  if(new_alt==0)
  {
    /* close all opned ep */
    if (pas_interface->alternate!=0)
    {
        /* @TODO : Close related End Points */
      if(ep->open)
      {
        USBD_LL_CloseEP(pdev, ep->ep_description.data_ep->ep_num);
        ep->open=0;
      }
#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK  
      if(pas_interface->synch_enabled)
      {
        /* close synch ep */
          ep=&haudio->ep_in[pas_interface->synch_ep.ep_num&0x0F];
          if(ep->open)
          {
            USBD_LL_CloseEP(pdev, ep->ep_description.sync_ep->ep_num);
            ep->open = 0;
          }
      }
#endif /*USBD_SUPPORT_AUDIO_OUT_FEEDBACK */
    }
    pas_interface->SetAS_Alternate(new_alt,pas_interface->private_data);
    pas_interface->alternate=0;
  }
  /* start new  alternate interface */
  else
  {
    /* prepare EP */
    ep->ep_description.data_ep=&pas_interface->data_ep;
    
    /* open the data ep */
    pas_interface->SetAS_Alternate(new_alt,pas_interface->private_data);
    pas_interface->alternate=new_alt;
    ep->max_packet_length=ep->ep_description.data_ep->GetMaxPacketLength(ep->ep_description.data_ep->private_data);
    /* open data end point */
    USBD_LL_OpenEP(pdev,
                 ep->ep_description.data_ep->ep_num,
                 USBD_EP_TYPE_ISOC,
                 ep->max_packet_length);             
     ep->open = 1;
     ep->ep_usage = USBD_AUDIO_DATA_EP;
     /* get usb working buffer */
    ep->ep_description.data_ep->buf= ep->ep_description.data_ep->GetBuffer(ep->ep_description.data_ep->private_data,
                                                                           &ep->ep_description.data_ep->length);        
    
    if(ep->ep_description.data_ep->ep_num&0x80)  /* IN EP */
    {
      USBD_LL_FlushEP(pdev, ep->ep_description.data_ep->ep_num);
      ep->tx_rx_soffn = USB_SOF_NUMBER();
      USBD_LL_Transmit(pdev, 
                        ep->ep_description.data_ep->ep_num,
                        ep->ep_description.data_ep->buf,
                        ep->ep_description.data_ep->length);
    }
    else/* OUT EP */
    {
#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK 
        uint32_t rate;
#endif /*USBD_SUPPORT_AUDIO_OUT_FEEDBACK */  
    /* Prepare Out endpoint to receive 1st packet */ 
    USBD_LL_PrepareReceive(pdev,
                           ep->ep_description.data_ep->ep_num,
                           ep->ep_description.data_ep->buf,                        
                           ep->max_packet_length); 
#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK 
    if(pas_interface->synch_enabled)
      {
           USBD_AUDIO_EP_SynchTypeDef* sync_ep; /* synchro ep description */
           ep = &haudio->ep_in[pas_interface->synch_ep.ep_num&0x0F];
           sync_ep = &pas_interface->synch_ep;
           ep->ep_description.sync_ep = sync_ep;
           ep->max_packet_length = AUDIO_FEEDBACK_EP_PACKET_SIZE;
           ep->ep_usage = USBD_AUDIO_FEEDBACK_EP;
           /* open synchro ep */
           USBD_LL_OpenEP(pdev, sync_ep->ep_num,
                 USBD_EP_TYPE_ISOC, ep->max_packet_length);             
            ep->open = 1;
            rate = sync_ep->GetFeedback(sync_ep->private_data);
            get_usb_speed_rate(rate,sync_ep->feedback_data);
            ep->tx_rx_soffn = USB_SOF_NUMBER();
            USBD_LL_Transmit(pdev, sync_ep->ep_num,
                             sync_ep->feedback_data, ep->max_packet_length);
      }
#endif /*USBD_SUPPORT_AUDIO_OUT_FEEDBACK */   

    }
  }
  return USBD_OK;
}
#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK
#ifdef USE_USB_FS
/**
  * @brief   get_usb_full_speed_rate
  *         Set feedback value from rate 
  * @param  rate: 
  * @param  buf: 
  * @retval 
  */
static  uint32_t get_usb_full_speed_rate(unsigned int rate, unsigned char * buf)
{
        uint32_t freq =  ((rate << 13) + 62) / 125;
        buf[0] =    freq>> 2;
        buf[1] =    freq>> 10;
        buf[2] =    freq>> 18;
return freq;
 }
#else /* USE_USB_FS */
/*
 * convert a sampling rate into USB high speed format (fs/8000 in Q16.16)
 * this will overflow at approx 4 MHz
 */
static  uint32_t get_usb_high_speed_rate(unsigned int rate, unsigned char * buf)
{
        uint32_t freq =  ((rate << 10) + 62) / 125;
        buf[0] =    freq;
        buf[1] =    freq>> 8;
        buf[2] =    freq>> 16;
        buf[3] =    freq>> 24;
return freq;
}
#endif /* USE_USB_FS */
#endif /*USBD_SUPPORT_AUDIO_OUT_FEEDBACK */ 

/**
  * @brief  USBD_AUDIO_Setup
  *         Handle the AUDIO specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
uint32_t setCount=0;
//if(setCount==1){
//	  __NOP();
//}else{
//	  setCount++;
//}
static uint8_t  USBD_AUDIO_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  uint16_t len;
  uint8_t *pbuf;
  uint8_t ret = USBD_OK;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :  
    if((req->bmRequest & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_INTERFACE)
    {
      switch (req->bRequest)
      {
      case USBD_AUDIO_CS_REQ_CUR:
      case USBD_AUDIO_CS_REQ_RANGE:
           AUDIO_REQ(pdev, req);
        break;
        
      default:
        USBD_CtlError (pdev, req);
        ret = USBD_FAIL; 
      }
    }
    else
    {
     USBD_CtlError (pdev, req);
        ret = USBD_FAIL;
    }
    break;
    
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR:      
      if( (req->wValue >> 8) == USBD_AUDIO_DESC_TYPE_CS_DEVICE)
      {
        pbuf = USBD_AUDIO_CfgDesc + 18;
        len = MIN(USB_AUDIO_DESC_SIZ , req->wLength);
        
        
        USBD_CtlSendData (pdev, 
                          pbuf,
                          len);
      }
      break;
      
    case USB_REQ_GET_INTERFACE :
      {
        for(int i=0;i<haudio->aud_function.as_interfaces_count;i++)
        {
            if((uint8_t)(req->wIndex)==haudio->aud_function.as_interfaces[i].interface_num)
            {
              USBD_CtlSendData (pdev,
                        (uint8_t *)&(haudio->aud_function.as_interfaces[i].alternate),
                        1);
              return USBD_OK;
            }
        }
        USBD_CtlError (pdev, req);
        ret = USBD_FAIL; 
      }
      break;
      
    case USB_REQ_SET_INTERFACE :
      {

        for(int i=0;i<haudio->aud_function.as_interfaces_count;i++)
        {
            if((uint8_t)(req->wIndex)==haudio->aud_function.as_interfaces[i].interface_num)
            {
              if((uint8_t)(req->wValue)==haudio->aud_function.as_interfaces[i].alternate)
              {
                /* Nothing to do*/
                return USBD_OK;
              }
              else
              {               
                /*Alternate is changed*/
                return USBD_AUDIO_SetInterfaceAlternate(pdev,i,(uint8_t)(req->wValue));
              }
            }
        } 

        
        if(((uint8_t)(req->wIndex) ==0)&&((uint8_t)(req->wValue))==0)
        {
          /* Audio Control Control interface, only alternate zero is accepted  */     
                return USBD_OK;
        }
          /* Call the error management function (command will be nacked */
          USBD_CtlError (pdev, req);
          ret = USBD_FAIL; 
      } 
      break;      
      
    default:
      USBD_CtlError (pdev, req);
      ret = USBD_FAIL;     
    }
  }
  return ret;
}

/**
  * @brief  USBD_AUDIO_GetCfgDesc 
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_AUDIO_GetCfgDesc (uint16_t *length)
{
  *length = USBD_AUDIO_CfgDescSize;
  return USBD_AUDIO_CfgDesc;
}

/**
  * @brief  USBD_AUDIO_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_AUDIO_DataIn (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{
  USBD_AUDIO_EPTypeDef * ep;

   ep = &((USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId])->ep_in[epnum&0x7F];
   if(ep->open)
   {
     switch(ep->ep_usage)
     {
     case USBD_AUDIO_DATA_EP : 
       {
        ep->ep_description.data_ep->buf = ep->ep_description.data_ep->GetBuffer(ep->ep_description.data_ep->private_data,
                                                                                  &ep->ep_description.data_ep->length);
          ep->tx_rx_soffn = USB_SOF_NUMBER();
          USBD_LL_Transmit(pdev, 
                      epnum|0x80,
                      ep->ep_description.data_ep->buf,
                      ep->ep_description.data_ep->length);
          break;
        }
#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK 
     case USBD_AUDIO_FEEDBACK_EP : 
       {
         uint32_t rate; 
         USBD_AUDIO_EP_SynchTypeDef* sync_ep=ep->ep_description.sync_ep;
         rate = sync_ep->GetFeedback(sync_ep->private_data);
         get_usb_speed_rate(rate,sync_ep->feedback_data);
         ep->tx_rx_soffn = USB_SOF_NUMBER();
         USBD_LL_Transmit(pdev, 
              epnum|0x80,
              sync_ep->feedback_data,
              AUDIO_FEEDBACK_EP_PACKET_SIZE);
            break;
        }
#endif /*USBD_SUPPORT_AUDIO_OUT_FEEDBACK */
     
#if USBD_AUDIO_SUPPORT_INTERRUPT
     case USBD_AUDIO_INTERRUPT_EP : 
        {
          ((USBD_AUDIO_HandleTypeDef*) pdev->pClassData)->is_ep_busy = 0;
          USBD_AUDIO_TransmitInterrupt();
          break;
        }
#endif /*USBD_AUDIO_SUPPORT_INTERRUPT */ 
     default :
        USBD_error_handler();
     }
   }
   else
   {
    USBD_error_handler();
   }
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_AUDIO_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  
 
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId]; 
  if(haudio->last_control.req == 0x00)
  {
    /* @TODO Manage this error */
    return USBD_OK;
  }
  if(haudio->last_control.request_target == AUDIO_UNIT_CONTROL_REQUEST)
  {
    USBD_AUDIO_ControlTypeDef *ctl;
     uint16_t selector;
    ctl=haudio->last_control.entity.controller;
    selector = HIBYTE(haudio->last_control.wValue);
    switch(ctl->type)
    {
    case USBD_AUDIO_CS_AC_SUBTYPE_FEATURE_UNIT:
         {
            USBD_AUDIO_FeatureControlCallbacksTypeDef* feature_control = ctl->Callbacks.feature_control;
          switch(selector)
          {
                  case USBD_AUDIO_FU_MUTE_CONTROL:
                    {
                      /* @TODO treat multi channel case and error when req! of GetCur*/  
                      if(feature_control->SetMute)
                      {
                        feature_control->SetMute(LOBYTE(haudio->last_control.wValue),
                                                                haudio->last_control.data[0], ctl->private_data);
                      }
                      break;
                    }
                  case USBD_AUDIO_FU_VOLUME_CONTROL:
                     {
                   
                        /* @TODO check the len uses cases and control req->wLength*/

                       switch(haudio->last_control.req)
                        {
                        case USBD_AUDIO_CS_REQ_CUR:
                              if(feature_control->SetCurVolume)
                              {
                                  feature_control->SetCurVolume(LOBYTE(haudio->last_control.wValue),
                                                                               AUDIO_2_L3_CUR_DATA_TO_VAL(haudio->last_control.data),
                                                                               ctl->private_data);
                              }
                              break;
                        default :
                              
                                USBD_error_handler();
                        }
                         break;
                       }
                  
                default :
                 
                          USBD_error_handler();
                }
          break;
         }
    case USBD_AUDIO_CS_AC_SUBTYPE_CLOCK_SOURCE:
         {
            uint8_t as_cnt_to_restart;
            uint8_t as_list_to_restart[USBD_AUDIO_AS_INTERFACE_COUNT];
            uint16_t selector = HIBYTE(haudio->last_control.wValue);
            uint32_t freq;
            USBD_AUDIO_ClockSourceCallbacksTypeDef* clk_control = ctl->Callbacks.clk_src_control;
            if((selector == USBD_AUDIO_CS_SAM_FREQ_CONTROL)&&(haudio->last_control.req == USBD_AUDIO_CS_REQ_CUR)&&clk_control->SetCurFrequency)
            {
              freq = AUDIO_2_L3_CUR_DATA_TO_VAL(haudio->last_control.data);
               clk_control->SetCurFrequency(freq ,&as_cnt_to_restart ,as_list_to_restart, ctl->private_data);
               for(int i=0, j=0; j<as_cnt_to_restart; j++)
               {
                 i = as_list_to_restart[j];
/* update sampling rate for syenchronization EP */
#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK
                 if(haudio->aud_function.as_interfaces[i].synch_enabled)
                 {
                    uint32_t rate; 
                    rate = haudio->aud_function.as_interfaces[i].synch_ep.GetFeedback(
                           haudio->aud_function.as_interfaces[i].synch_ep.private_data);
                    get_usb_speed_rate(rate,haudio->aud_function.as_interfaces[i].synch_ep.feedback_data);
                 }
#endif /*USBD_SUPPORT_AUDIO_OUT_FEEDBACK */
                 if(haudio->aud_function.as_interfaces[i].alternate != 0)
                 {
                  int alt = haudio->aud_function.as_interfaces[i].alternate;
                  USBD_AUDIO_SetInterfaceAlternate(pdev, i, 0);
                  USBD_AUDIO_SetInterfaceAlternate(pdev, i, alt);
                 }
               }
          }
            else
            {
               USBD_error_handler();
            }
             break;
         }     
  default : /* switch(ctl->type)*/
            USBD_error_handler();
                             
    }
  }
  return USBD_OK;
}
/**
  * @brief  USBD_AUDIO_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_AUDIO_EP0_TxReady (USBD_HandleTypeDef *pdev)
{
  /* Only OUT control data are processed */
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_AUDIO_SOF (USBD_HandleTypeDef *pdev)
{
    USBD_AUDIO_HandleTypeDef   *haudio;
  
 
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId]; 
  
  for(int i=0;i<haudio->aud_function.as_interfaces_count;i++)
  {
      if(haudio->aud_function.as_interfaces[i].alternate!=0)
      {
        if(haudio->aud_function.as_interfaces[i].SofReceived)
          
        {
          haudio->aud_function.as_interfaces[i].SofReceived(haudio->aud_function.as_interfaces[i].private_data);
        }
      }
  }
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_AUDIO_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
 USBD_AUDIO_EPTypeDef   *ep;
 USBD_AUDIO_HandleTypeDef   *haudio;
 uint16_t current_sof;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];
 /* @TODO check if the feedback is responsible of event */
  for(int i = 1; i<USBD_AUDIO_MAX_IN_EP; i++)
  {
    ep = &haudio->ep_in[i];
    current_sof = USB_SOF_NUMBER();
    if((ep->open)&&
       ((ep->ep_usage == USBD_AUDIO_FEEDBACK_EP)||(ep->ep_usage == USBD_AUDIO_DATA_EP))
         /*&& IS_ISO_IN_INCOMPLETE_EP(i,current_sof, ep->tx_rx_soffn)*/)
    {
    if(i==3){

    	__NOP();
    }
      epnum = i|0x80;
      USB_CLEAR_INCOMPLETE_IN_EP(epnum);
      USBD_LL_FlushEP(pdev, epnum);
      ep->tx_rx_soffn = USB_SOF_NUMBER();
#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK  
      if(ep->ep_usage == USBD_AUDIO_FEEDBACK_EP)
      {
        USBD_LL_Transmit(pdev, 
                         epnum,
                         ep->ep_description.sync_ep->feedback_data,
                         ep->max_packet_length);
        continue;
      }
      else
#endif /*USBD_SUPPORT_AUDIO_OUT_FEEDBACK */
      if(ep->ep_usage == USBD_AUDIO_DATA_EP)
      {
          USBD_LL_Transmit(pdev, 
                      epnum,
                      ep->ep_description.data_ep->buf,
                      ep->ep_description.data_ep->length);
      }
    }
  }
  return 0;
}
/**
  * @brief  USBD_AUDIO_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_AUDIO_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{

  return USBD_OK;
}
/**
  * @brief  USBD_AUDIO_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */

static uint8_t  USBD_AUDIO_DataOut (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{
  
  USBD_AUDIO_EPTypeDef * ep;
  uint8_t *pbuf ;
  uint16_t packet_length;


  ep=&((USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId])->ep_out[epnum];
  if(ep->open)
  {
    /* get received length */
    packet_length = USBD_LL_GetRxDataSize(pdev, epnum);
    /* inform user about data reception  */
    ep->ep_description.data_ep->DataReceived(packet_length,ep->ep_description.data_ep->private_data);
     
    /* get buffer to receive new packet */  
    pbuf=  ep->ep_description.data_ep->GetBuffer(ep->ep_description.data_ep->private_data,&packet_length);                               
    /* Prepare Out endpoint to receive next audio packet */
     USBD_LL_PrepareReceive(pdev,
                            epnum,
                            pbuf,
                            packet_length);
    }
    else
    {
      USBD_error_handler();
    }
    
  
    return USBD_OK;
}

/**
  * @brief  AUDIO_REQ
  *         Handles the Control requests.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static uint8_t AUDIO_REQ(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{  
  USBD_AUDIO_HandleTypeDef   *haudio;
  USBD_AUDIO_ControlTypeDef * ctl = 0;
  uint8_t unit_id,control_selector;
 
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];
  
  /* reset last command */
  haudio->last_control.req = 0x00;
  
  /* get the Unit Id */
  unit_id = HIBYTE(req->wIndex);
  
  for (int i = 0;i < haudio->aud_function.control_count; i++)
  {
    if(unit_id == haudio->aud_function.controls[i].id)
    {
      ctl = &haudio->aud_function.controls[i];
      break;
    }     
  }
   control_selector = HIBYTE(req->wValue);
  if(!ctl)
  {
    /* control not supported */
    USBD_CtlError (pdev, req);
    return  USBD_FAIL; 
  }
  
  if((ctl->control_selector_map & control_selector) == 0)
  {
    /* control not supported */
    USBD_CtlError (pdev, req);
      return  USBD_FAIL; 
  }
  
  if(!(req->bmRequest&0x80))
  {
    /* set request */
    /* @TODO check the length */
     haudio->last_control.wValue  = req->wValue;
     haudio->last_control.entity.controller= ctl;
     haudio->last_control.request_target = AUDIO_UNIT_CONTROL_REQUEST;
     haudio->last_control.len = req->wLength;
     haudio->last_control.req = req->bRequest;
     USBD_CtlPrepareRx (pdev,
                        haudio->last_control.data,                                  
                       req->wLength);
      return USBD_OK;   
  }
  
  switch(ctl->type)
  {
    case USBD_AUDIO_CS_AC_SUBTYPE_FEATURE_UNIT:
         {
           USBD_AUDIO_FeatureControlCallbacksTypeDef* feature_control = ctl->Callbacks.feature_control;
          switch(control_selector)
          {
                  case USBD_AUDIO_FU_MUTE_CONTROL:
                    {
                      /* @TODO treat multi channel case and error when req! of GetCur*/
                      
                      haudio->last_control.data[0] = 0;
                      haudio->last_control.len = 1;
                      if(feature_control->GetMute)
                      {
                        feature_control->GetMute(LOBYTE(req->wValue),
                                                                &haudio->last_control.data[0], ctl->private_data);
                      }

                      break;
                     }
                  case USBD_AUDIO_FU_VOLUME_CONTROL:
                     {
                        /* set request */
                        /* @TODO check the len uses cases and control req->wLength*/
                                                     
                        switch(req->bRequest)
                        {
                        case USBD_AUDIO_CS_REQ_CUR:
                              if(feature_control->GetCurVolume)
                              {
                                  uint16_t cur_vol;
                                  feature_control->GetCurVolume(LOBYTE(req->wValue),
                                                                (uint16_t*)&cur_vol, ctl->private_data);
                                  AUDIO_2_L2_CUR_VAL_TO_DATA(cur_vol, haudio->last_control.data);
                                  haudio->last_control.len = 2;
                              }
                              else
                              {
                                /* return error*/
                                  USBD_error_handler();
                              }
                              
                              break;
                          
                        case USBD_AUDIO_CS_REQ_RANGE:
                          haudio->last_control.data[0] = 1 ; /* one subrange */
                          haudio->last_control.data[1] = 0 ;
                          AUDIO_2_L2_RNG_VAL_TO_DATA( feature_control->MinVolume,feature_control->ResVolume,
                                                     feature_control->MaxVolume, haudio->last_control.data + 2);
                          haudio->last_control.len = 8;
                          break;
                        default :
                                USBD_error_handler();
                        }
                         break;
                       }
                  
                default :
                          USBD_error_handler();
                }
          break;
         }
  case USBD_AUDIO_CS_AC_SUBTYPE_CLOCK_SOURCE:
    {
      USBD_AUDIO_ClockSourceCallbacksTypeDef* clk_control = ctl->Callbacks.clk_src_control;
      switch(control_selector)
      {
              case USBD_AUDIO_CS_SAM_FREQ_CONTROL:
                {
                  if(req->bRequest  == USBD_AUDIO_CS_REQ_CUR)
                  {
                    /* Get Cur frequency */
                    if(clk_control->GetCurFrequency)
                    {
                      uint32_t freq;
                      
                      clk_control->GetCurFrequency(&freq, ctl->private_data);
                      AUDIO_2_L3_CUR_VAL_TO_DATA(freq , haudio->last_control.data);
                      haudio->last_control.len = 4;
                    }
                    else
                    {
                      /* @TODO return an error */
                       USBD_error_handler();
                    }
                  }
                  else
                  {
                    /* Get Range frequency */

                    if(clk_control->GetFrequencyList)
                    {
                       uint32_t* freq_list;
                       uint8_t  freq_count;
                       uint8_t* control_data = haudio->last_control.data;
                      
                      clk_control->GetFrequencyList(&freq_list, &freq_count, ctl->private_data);
                      if(freq_count > 0)
                      {
                        *(control_data++) = freq_count;
                        *(control_data++) = 0;

                        for(int i= 0; i < freq_count; i++)
                        {
                           AUDIO_2_L3_RNG_VAL_TO_DATA(freq_list[i] ,freq_list[i], 0, control_data );
                           control_data+=12;
                        }
                         haudio->last_control.len = control_data -  haudio->last_control.data;
                      }
                      else
                      {
                        /* @TODO return an error */
                         USBD_error_handler();
                      }
                    }
                    else
                    {
                       /* @TODO return an error */
                       USBD_error_handler();
                    }
                    
                  }
                  break;
                 }
                case USBD_AUDIO_CS_CLOCK_VALID_CONTROL:
                {
                  if(req->bRequest  == USBD_AUDIO_CS_REQ_CUR)
                  {
                    /* Get Cur frequency */
                    if(clk_control->IsValid)
                    {
                      clk_control->IsValid(&haudio->last_control.data[0], ctl->private_data);
                      haudio->last_control.len = 1;
                    }
                    else
                    {
                      /* @TODO return an error */
                       USBD_error_handler();
                    }
                  }
                  else
                  {
                       /* @TODO return an error */
                     USBD_error_handler();
                  }
                  break;
                    
                }
                default :
                      USBD_error_handler();  
            } 


                  break;
                 }
            default :
                      USBD_error_handler();
    }
  /* @TODO check next blolck */
    if(haudio->last_control.len> req->wLength)
    {
      haudio->last_control.len = req->wLength;
    }
  
    USBD_CtlSendData (pdev, haudio->last_control.data,haudio->last_control.len);
  return USBD_OK;
}

/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_AUDIO_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_AUDIO_DeviceQualifierDesc);
  return USBD_AUDIO_DeviceQualifierDesc;
}

/**
* @brief  USBD_AUDIO_RegisterInterface
* @param  fops: Audio interface callback
* @retval status
*/
uint8_t  USBD_AUDIO_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                        USBD_AUDIO_InterfaceCallbacksfTypeDef *aifc)
{
  if(aifc != NULL)
  {
    pdev->pUserData[pdev->classId] = aifc;
//    aifc->GetConfigDesc(&USBD_AUDIO_CfgDesc, &USBD_AUDIO_CfgDescSize, aifc->private_data);
    
  }
  return 0;
}

#if USBD_AUDIO_SUPPORT_INTERRUPT
/**
* @brief  USBD_AUDIO_TransmitInterrupt
* @param  void
* @retval status
*/
static uint8_t  USBD_AUDIO_TransmitInterrupt()
{
    USBD_AUDIO_HandleTypeDef   *haudio;
  int pos = USBD_AUDIO_INTERRUPT_TABLE_SIZE;
  
  if(pdev_audio&&pdev_audio->pClassData)
  {
    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

    if(haudio->is_ep_busy == 0)
    {
  
      for( int i = 0; i < USBD_AUDIO_INTERRUPT_TABLE_SIZE; i++)
      {
        if((haudio->interrupts[i].priority != USBD_AUDIO_NOT_USED_PRIORITY)
            &&((pos == USBD_AUDIO_INTERRUPT_TABLE_SIZE)||
                (haudio->interrupts[i].priority<haudio->interrupts[pos].priority))) 
        {
          pos = i;
        }
      }
      if(pos<USBD_AUDIO_INTERRUPT_TABLE_SIZE)
      {
        haudio->is_ep_busy = 1;
        haudio->interrupt_message[USBD_AUDIO_INTERRUPT_MESSAGE_BINFO_OFFSET]= haudio->interrupts[pos].type;
        haudio->interrupt_message[USBD_AUDIO_INTERRUPT_MESSAGE_BATTRIBUTE_OFFSET]= haudio->interrupts[pos].attr;
        haudio->interrupt_message[USBD_AUDIO_INTERRUPT_MESSAGE_WVALUE_OFFSET]= haudio->interrupts[pos].cn_mcn;
        haudio->interrupt_message[USBD_AUDIO_INTERRUPT_MESSAGE_WVALUE_OFFSET+1]= haudio->interrupts[pos].cs;
        haudio->interrupt_message[USBD_AUDIO_INTERRUPT_MESSAGE_WINDEX_OFFSET+1]=  haudio->interrupts[pos].entity_id;
        haudio->interrupt_message[USBD_AUDIO_INTERRUPT_MESSAGE_WINDEX_OFFSET]=haudio->interrupts[pos].ep_if_id;
        haudio->interrupts[pos].priority = USBD_AUDIO_NOT_USED_PRIORITY;
            USBD_LL_Transmit(pdev_audio, haudio->aud_function.interrupt_ep_num,
                            haudio->interrupt_message, USBD_AUDIO_INTERRUPT_DATA_MESSAGE_SIZE);
        return 0;
      }
    }
  }
  return 1;
}

/*
* @brief  USBD_AUDIO_SendInterrupt
* @param  interrupt
* @retval status
*/
uint8_t  USBD_AUDIO_SendInterrupt  (USBD_AUDIO_InterruptTypeDef *interrupt)
{
  int pos = USBD_AUDIO_INTERRUPT_TABLE_SIZE;
  USBD_AUDIO_HandleTypeDef   *haudio;
  
  if(pdev_audio&&pdev->pClassDataCmsit[pdev->classId])
  {
    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

    for( int i = 0; i < USBD_AUDIO_INTERRUPT_TABLE_SIZE; i++)
    {
      if((haudio->interrupts[i].priority == USBD_AUDIO_NOT_USED_PRIORITY)&&(pos == USBD_AUDIO_INTERRUPT_TABLE_SIZE))
      {
        pos = i;
      }
      else
       if(haudio->interrupts[i].priority != USBD_AUDIO_NOT_USED_PRIORITY)
       {
         if(interrupt->type == haudio->interrupts[i].type &&
            interrupt->attr == haudio->interrupts[i].attr &&
            interrupt->ep_if_id == haudio->interrupts[i].ep_if_id &&
            interrupt->entity_id == haudio->interrupts[i].entity_id &&
            interrupt->cs == haudio->interrupts[i].cs &&
            interrupt->cn_mcn == haudio->interrupts[i].cn_mcn)
         {
           return 0;
         }
       }
      
    }
    if(pos < USBD_AUDIO_INTERRUPT_TABLE_SIZE)
    {
      haudio->interrupts[pos].type = interrupt->type;
      haudio->interrupts[pos].attr = interrupt->attr;
      haudio->interrupts[pos].ep_if_id = interrupt->ep_if_id;
      haudio->interrupts[pos].entity_id = interrupt->entity_id;
      haudio->interrupts[pos].cs = interrupt->cs;
      haudio->interrupts[pos].cn_mcn = interrupt->cn_mcn;
      haudio->interrupts[pos].priority = interrupt->priority;
    }
    else
    {
      return 2;
    }
    
    if(haudio->is_ep_busy == 0)
    {
      USBD_AUDIO_TransmitInterrupt();
    }
    return 0;
  }
  else
  {
    return 1;
  }
}
#endif /* USBD_AUDIO_SUPPORT_INTERRUPT  */
/**
  * @}
  */ 


/**
  * @}
  */ 


/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
