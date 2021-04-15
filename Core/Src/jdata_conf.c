/**
  ******************************************************************************
  * @file    LibJPEG/LibJPEG_Encoding/Src/jdata_conf.c
  * @author  MCD Application Team
  * @brief   this file contains the libJPEG abstruction defines for the memory
  *          management routines and IO File operations
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "jdata_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

size_t read_file (JFILE  *file, uint8_t *buf, uint32_t sizeofbuf)
{
    
  memcpy (buf, file->buffer + file->index, sizeofbuf); 
  return sizeofbuf; 
}

size_t write_file (JFILE  *file, uint8_t *buf, uint32_t sizeofbuf)
{
  memcpy (file->buffer + file->index, buf , sizeofbuf); 
  file ->index += sizeofbuf;
  return sizeofbuf; 
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
