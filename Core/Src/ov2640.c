/**
  ******************************************************************************
  * @file
  * @author
  * @version
  * @date
  * @brief   OV2640
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ov2640.h"

/** @addtogroup DCMI_Camera
  * @{
  */

static OV2640_TypeDef *_OV2640;

/* QQVGA 160x120 */
const char OV2640_QQVGA[][2] =
    {
        0xff,
        0x00,
        0x2c,
        0xff,
        0x2e,
        0xdf,
        0xff,
        0x01,
        0x3c,
        0x32,
        0x11,
        0x00,
        0x09,
        0x02,
        0x03,
        0xcf,
        0x04,
        0x08,
        0x13,
        0xe5,
        0x14,
        0x48,
        0x2c,
        0x0c,
        0x33,
        0x78,
        0x3a,
        0x33,
        0x3b,
        0xfb,
        0x3e,
        0x00,
        0x43,
        0x11,
        0x16,
        0x10,
        0x39,
        0x02,
        0x35,
        0x88,
        0x22,
        0x0a,
        0x37,
        0x40,
        0x23,
        0x00,
        0x34,
        0xa0,
        0x36,
        0x1a,
        0x06,
        0x02,
        0x07,
        0xc0,
        0x0d,
        0xb7,
        0x0e,
        0x01,
        0x4c,
        0x00,
        0x4a,
        0x81,
        0x21,
        0x99,
        0x24,
        0x3a,
        0x25,
        0x32,
        0x26,
        0x82,
        0x5c,
        0x00,
        0x63,
        0x00,
        0x5d,
        0x55,
        0x5e,
        0x7d,
        0x5f,
        0x7d,
        0x60,
        0x55,
        0x61,
        0x70,
        0x62,
        0x80,
        0x7c,
        0x05,
        0x20,
        0x80,
        0x28,
        0x30,
        0x6c,
        0x00,
        0x6d,
        0x80,
        0x6e,
        0x00,
        0x70,
        0x02,
        0x71,
        0x96,
        0x73,
        0xe1,
        0x3d,
        0x34,
        0x5a,
        0x57,
        0x4f,
        0xbb,
        0x50,
        0x9c,
        0x0f,
        0x43,
        0xff,
        0x00,
        0xe5,
        0x7f,
        0xf9,
        0xc0,
        0x41,
        0x24,
        0xe0,
        0x14,
        0x76,
        0xff,
        0x33,
        0xa0,
        0x42,
        0x20,
        0x43,
        0x18,
        0x4c,
        0x00,
        0x87,
        0xd0,
        0x88,
        0x3f,
        0xd7,
        0x03,
        0xd9,
        0x10,
        0xd3,
        0x82,
        0xc8,
        0x08,
        0xc9,
        0x80,
        0x7c,
        0x00,
        0x7d,
        0x02,
        0x7c,
        0x03,
        0x7d,
        0x48,
        0x7d,
        0x48,
        0x7c,
        0x08,
        0x7d,
        0x20,
        0x7d,
        0x10,
        0x7d,
        0x0e,
        0x90,
        0x00,
        0x91,
        0x0e,
        0x91,
        0x1a,
        0x91,
        0x31,
        0x91,
        0x5a,
        0x91,
        0x69,
        0x91,
        0x75,
        0x91,
        0x7e,
        0x91,
        0x88,
        0x91,
        0x8f,
        0x91,
        0x96,
        0x91,
        0xa3,
        0x91,
        0xaf,
        0x91,
        0xc4,
        0x91,
        0xd7,
        0x91,
        0xe8,
        0x91,
        0x20,
        0x92,
        0x00,
        0x93,
        0x06,
        0x93,
        0xe3,
        0x93,
        0x05,
        0x93,
        0x05,
        0x93,
        0x00,
        0x93,
        0x00,
        0x93,
        0x00,
        0x93,
        0x00,
        0x93,
        0x00,
        0x93,
        0x00,
        0x93,
        0x00,
        0x93,
        0x00,
        0x93,
        0x00,
        0x96,
        0x00,
        0x97,
        0x08,
        0x97,
        0x19,
        0x97,
        0x02,
        0x97,
        0x0c,
        0x97,
        0x24,
        0x97,
        0x30,
        0x97,
        0x28,
        0x97,
        0x26,
        0x97,
        0x02,
        0x97,
        0x98,
        0x97,
        0x80,
        0x97,
        0x00,
        0x97,
        0x00,
        0xc3,
        0xed,
        0xa4,
        0x00,
        0xa8,
        0x00,
        0xbf,
        0x00,
        0xba,
        0xf0,
        0xbc,
        0x64,
        0xbb,
        0x02,
        0xb6,
        0x3d,
        0xb8,
        0x57,
        0xb7,
        0x38,
        0xb9,
        0x4e,
        0xb3,
        0xe8,
        0xb4,
        0xe1,
        0xb5,
        0x66,
        0xb0,
        0x67,
        0xb1,
        0x5e,
        0xb2,
        0x04,
        0xc7,
        0x00,
        0xc6,
        0x51,
        0xc5,
        0x11,
        0xc4,
        0x9c,
        0xcf,
        0x02,
        0xa6,
        0x00,
        0xa7,
        0xe0,
        0xa7,
        0x10,
        0xa7,
        0x1e,
        0xa7,
        0x21,
        0xa7,
        0x00,
        0xa7,
        0x28,
        0xa7,
        0xd0,
        0xa7,
        0x10,
        0xa7,
        0x16,
        0xa7,
        0x21,
        0xa7,
        0x00,
        0xa7,
        0x28,
        0xa7,
        0xd0,
        0xa7,
        0x10,
        0xa7,
        0x17,
        0xa7,
        0x21,
        0xa7,
        0x00,
        0xa7,
        0x28,
        0xc0,
        0xc8,
        0xc1,
        0x96,
        0x86,
        0x1d,
        0x50,
        0x00,
        0x51,
        0x90,
        0x52,
        0x18,
        0x53,
        0x00,
        0x54,
        0x00,
        0x55,
        0x88,
        0x57,
        0x00,
        0x5a,
        0x90,
        0x5b,
        0x18,
        0x5c,
        0x05,
        0xc3,
        0xef,
        0x7f,
        0x00,
        0xda,
        0x00,
        0xe5,
        0x1f,
        0xe1,
        0x67,
        0xe0,
        0x00,
        0xdd,
        0xff,
        0x05,
        0x00,
        0xff,
        0x01,
        0xff,
        0x01,
        0x12,
        0x00,
        0x17,
        0x11,
        0x18,
        0x75,
        0x19,
        0x01,
        0x1a,
        0x97,
        0x32,
        0x36,
        0x4f,
        0xbb,
        0x6d,
        0x80,
        0x3d,
        0x34,
        0x39,
        0x02,
        0x35,
        0x88,
        0x22,
        0x0a,
        0x37,
        0x40,
        0x23,
        0x00,
        0x34,
        0xa0,
        0x36,
        0x1a,
        0x06,
        0x02,
        0x07,
        0xc0,
        0x0d,
        0xb7,
        0x0e,
        0x01,
        0x4c,
        0x00,
        0xff,
        0x00,
        0xe0,
        0x04,
        0x8c,
        0x00,
        0x87,
        0xd0,
        0xe0,
        0x00,
        0xff,
        0x00,
        0xe0,
        0x14,
        0xe1,
        0x77,
        0xe5,
        0x1f,
        0xd7,
        0x03,
        0xda,
        0x10,
        0xe0,
        0x00,
        0xff,
        0x00,
        0xe0,
        0x04,
        0xc0,
        0xc8,
        0xc1,
        0x96,
        0x86,
        0x1d,
        0x50,
        0x00,
        0x51,
        0x90,
        0x52,
        0x2c,
        0x53,
        0x00,
        0x54,
        0x00,
        0x55,
        0x88,
        0x57,
        0x00,
        0x5a,
        0x90,
        0x5b,
        0x2c,
        0x5c,
        0x05,
        0xe0,
        0x00,
        0xd3,
        0x04,
        0xff,
        0x00,
        0xc3,
        0xef,
        0xa6,
        0x00,
        0xa7,
        0xdd,
        0xa7,
        0x78,
        0xa7,
        0x7e,
        0xa7,
        0x24,
        0xa7,
        0x00,
        0xa7,
        0x25,
        0xa6,
        0x06,
        0xa7,
        0x20,
        0xa7,
        0x58,
        0xa7,
        0x73,
        0xa7,
        0x34,
        0xa7,
        0x00,
        0xa7,
        0x25,
        0xa6,
        0x0c,
        0xa7,
        0x28,
        0xa7,
        0x58,
        0xa7,
        0x6d,
        0xa7,
        0x34,
        0xa7,
        0x00,
        0xa7,
        0x25,
        0xff,
        0x00,
        0xe0,
        0x04,
        0xe1,
        0x67,
        0xe5,
        0x1f,
        0xd7,
        0x01,
        0xda,
        0x08,
        0xda,
        0x09,
        0xe0,
        0x00,
        0x98,
        0x00,
        0x99,
        0x00,
        0xff,
        0x01,
        0x04,
        0x28,
        0xff,
        0x01,
        0x12,
        0x40,
        0x17,
        0x11,
        0x18,
        0x43,
        0x19,
        0x00,
        0x1a,
        0x4b,
        0x32,
        0x09,
        0x4f,
        0xca,
        0x50,
        0xa8,
        0x5a,
        0x23,
        0x6d,
        0x00,
        0x39,
        0x12,
        0x35,
        0xda,
        0x22,
        0x1a,
        0x37,
        0xc3,
        0x23,
        0x00,
        0x34,
        0xc0,
        0x36,
        0x1a,
        0x06,
        0x88,
        0x07,
        0xc0,
        0x0d,
        0x87,
        0x0e,
        0x41,
        0x4c,
        0x00,
        0xff,
        0x00,
        0xe0,
        0x04,
        0xc0,
        0x64,
        0xc1,
        0x4b,
        0x86,
        0x35,
        0x50,
        0x92,
        0x51,
        0xc8,
        0x52,
        0x96,
        0x53,
        0x00,
        0x54,
        0x00,
        0x55,
        0x00,
        0x57,
        0x00,
        0x5a,
        0x28,
        0x5b,
        0x1e,
        0x5c,
        0x00,
        0xe0,
        0x00,
        0xff,
        0x01,
        0x11,
        0x00,
        0x3d,
        0x38,
        0x2d,
        0x00,
        0x50,
        0x65,
        0xff,
        0x00,
        0xd3,
        0x04,
        0x7c,
        0x00,
        0x7d,
        0x04,
        0x7c,
        0x09,
        0x7d,
        0x28,
        0x7d,
        0x00,
};

const unsigned char OV2640_UXGA[][2] =
    {
        0xff, 0x00,
        0x2c, 0xff,
        0x2e, 0xdf,
        0xff, 0x01,
        0x3c, 0x32,
        0x11, 0x00,
        0x09, 0x02,
        0x04, 0xd0, //|0x80,
        0x13, 0xe5,
        0x14, 0x48,
        0x2c, 0x0c,
        0x33, 0x78,
        0x3a, 0x33,
        0x3b, 0xfB,
        0x3e, 0x00,
        0x43, 0x11,
        0x16, 0x10,
        0x4a, 0x81,
        0x21, 0x99,
        0x24, 0x40,
        0x25, 0x38,
        0x26, 0x82,
        0x5c, 0x00,
        0x63, 0x00,
        0x46, 0x3f,
        0x0c, 0x3c,
        0x61, 0x70,
        0x62, 0x80,
        0x7c, 0x05,
        0x20, 0x80,
        0x28, 0x30,
        0x6c, 0x00,
        0x6d, 0x80,
        0x6e, 0x00,
        0x70, 0x02,
        0x71, 0x94,
        0x73, 0xc1,
        0x3d, 0x34,
        0x5a, 0x57,
        0x12, 0x00,
        0x11, 0x00,
        0x17, 0x11,
        0x18, 0x75,
        0x19, 0x01,
        0x1a, 0x97,
        0x32, 0x36,
        0x03, 0x0f,
        0x37, 0x40,
        0x4f, 0xbb,
        0x50, 0x9c,
        0x5a, 0x57,
        0x6d, 0x80,
        0x6d, 0x38,
        0x39, 0x02,
        0x35, 0x88,
        0x22, 0x0a,
        0x37, 0x40,
        0x23, 0x00,
        0x34, 0xa0,
        0x36, 0x1a,
        0x06, 0x02,
        0x07, 0xc0,
        0x0d, 0xb7,
        0x0e, 0x01,
        0x4c, 0x00,
        0xff, 0x00,
        0xe5, 0x7f,
        0xf9, 0xc0,
        0x41, 0x24,
        0xe0, 0x14,
        0x76, 0xff,
        0x33, 0xa0,
        0x42, 0x20,
        0x43, 0x18,
        0x4c, 0x00,
        0x87, 0xd0,
        0x88, 0x3f,
        0xd7, 0x03,
        0xd9, 0x10,
        0xd3, 0x82,
        0xc8, 0x08,
        0xc9, 0x80,
        0x7d, 0x00,
        0x7c, 0x03,
        0x7d, 0x48,
        0x7c, 0x08,
        0x7d, 0x20,
        0x7d, 0x10,
        0x7d, 0x0e,
        0x90, 0x00,
        0x91, 0x0e,
        0x91, 0x1a,
        0x91, 0x31,
        0x91, 0x5a,
        0x91, 0x69,
        0x91, 0x75,
        0x91, 0x7e,
        0x91, 0x88,
        0x91, 0x8f,
        0x91, 0x96,
        0x91, 0xa3,
        0x91, 0xaf,
        0x91, 0xc4,
        0x91, 0xd7,
        0x91, 0xe8,
        0x91, 0x20,
        0x92, 0x00,
        0x93, 0x06,
        0x93, 0xe3,
        0x93, 0x02,
        0x93, 0x02,
        0x93, 0x00,
        0x93, 0x04,
        0x93, 0x00,
        0x93, 0x03,
        0x93, 0x00,
        0x93, 0x00,
        0x93, 0x00,
        0x93, 0x00,
        0x93, 0x00,
        0x93, 0x00,
        0x93, 0x00,
        0x96, 0x00,
        0x97, 0x08,
        0x97, 0x19,
        0x97, 0x02,
        0x97, 0x0c,
        0x97, 0x24,
        0x97, 0x30,
        0x97, 0x28,
        0x97, 0x26,
        0x97, 0x02,
        0x97, 0x98,
        0x97, 0x80,
        0x97, 0x00,
        0x97, 0x00,
        0xc3, 0xef,
        0xff, 0x00,
        0xba, 0xdc,
        0xbb, 0x08,
        0xb6, 0x24,
        0xb8, 0x33,
        0xb7, 0x20,
        0xb9, 0x30,
        0xb3, 0xb4,
        0xb4, 0xca,
        0xb5, 0x43,
        0xb0, 0x5c,
        0xb1, 0x4f,
        0xb2, 0x06,
        0xc7, 0x00,
        0xc6, 0x51,
        0xc5, 0x11,
        0xc4, 0x9c,
        0xbf, 0x00,
        0xbc, 0x64,
        0xa6, 0x00,
        0xa7, 0x1e,
        0xa7, 0x6b,
        0xa7, 0x47,
        0xa7, 0x33,
        0xa7, 0x00,
        0xa7, 0x23,
        0xa7, 0x2e,
        0xa7, 0x85,
        0xa7, 0x42,
        0xa7, 0x33,
        0xa7, 0x00,
        0xa7, 0x23,
        0xa7, 0x1b,
        0xa7, 0x74,
        0xa7, 0x42,
        0xa7, 0x33,
        0xa7, 0x00,
        0xa7, 0x23,
        0xc0, 0xc8,
        0xc1, 0x96,
        0x8c, 0x00,
        0x86, 0x3d,
        0x50, 0x92,
        0x51, 0x90,
        0x52, 0x2c,
        0x53, 0x00,
        0x54, 0x00,
        0x55, 0x88,
        0x5a, 0x50,
        0x5b, 0x3c,
        0x5c, 0x00,
        0xd3, 0x04,
        0x7f, 0x00,
        0xda, 0x00,
        0xe5, 0x1f,
        0xe1, 0x67,
        0xe0, 0x00,
        0xdd, 0x7f,
        0x05, 0x00,
        0xff, 0x00,
        0xe0, 0x04,
        0xc0, 0xc8,
        0xc1, 0x96,
        0x86, 0x3d,
        0x50, 0x92,
        0x51, 0x90,
        0x52, 0x2c,
        0x53, 0x00,
        0x54, 0x00,
        0x55, 0x88,
        0x57, 0x00,
        0x5a, 0x50,
        0x5b, 0x3c,
        0x5c, 0x00,
        0xd3, 0x04,
        0xe0, 0x00,
        0xFF, 0x00,
        0x05, 0x00,
        0xDA, 0x08,
        0xda, 0x09,
        0x98, 0x00,
        0x99, 0x00,
        0x00, 0x00,

        0xff, 0x01,
        0x11, 0x00,
        //  0x12, 0x10,
        //  0x2a, 0x00,
        //  0x2b, 0x00,
        //  0x46, 0x87,
        //  0x47, 0x00,
        //  0x3d, 0x33,
        //  0xff, 0x00,
        //  0xe0, 0x04,
        //  0xc0, 0xc8,
        //  0xc1, 0x96,
        //  0x86, 0x35,
        //  0x50, 0x80,
        //  0x51, 0x90,
        //  0x52, 0x2c,
        //  0x53, 0x00,
        //  0x54, 0x00,
        //  0x55, 0x88,
        //  0x57, 0x00,
        //  0x5a, 0x78,//480
        //  0x5b, 0x44,//272
        //  0x5c, 0x00,
        //  0xd3, 0x04,
        //  0xe0, 0x00,
};

const unsigned char OV2640_JPEG_INIT[][2] =
    {
        0xff,
        0x00,
        0x2c,
        0xff,
        0x2e,
        0xdf,
        0xff,
        0x01,
        0x3c,
        0x32,
        0x11,
        0x30,
        0x09,
        0x02,
        0x04,
        0x28,
        0x13,
        0xe5,
        0x14,
        0x48,
        0x2c,
        0x0c,
        0x33,
        0x78,
        0x3a,
        0x33,
        0x3b,
        0xfB,
        0x3e,
        0x00,
        0x43,
        0x11,
        0x16,
        0x10,
        0x39,
        0x92,
        0x35,
        0xda,
        0x22,
        0x1a,
        0x37,
        0xc3,
        0x23,
        0x00,
        0x34,
        0xc0,
        0x36,
        0x1a,
        0x06,
        0x88,
        0x07,
        0xc0,
        0x0d,
        0x87,
        0x0e,
        0x41,
        0x4c,
        0x00,
        0x48,
        0x00,
        0x5B,
        0x00,
        0x42,
        0x03,
        0x4a,
        0x81,
        0x21,
        0x99,
        0x24,
        0x40,
        0x25,
        0x38,
        0x26,
        0x82,
        0x5c,
        0x00,
        0x63,
        0x00,
        0x61,
        0x70,
        0x62,
        0x80,
        0x7c,
        0x05,
        0x20,
        0x80,
        0x28,
        0x30,
        0x6c,
        0x00,
        0x6d,
        0x80,
        0x6e,
        0x00,
        0x70,
        0x02,
        0x71,
        0x94,
        0x73,
        0xc1,
        0x12,
        0x40,
        0x17,
        0x11,
        0x18,
        0x43,
        0x19,
        0x00,
        0x1a,
        0x4b,
        0x32,
        0x09,
        0x37,
        0xc0,
        0x4f,
        0x60,
        0x50,
        0xa8,
        0x6d,
        0x00,
        0x3d,
        0x38,
        0x46,
        0x3f,
        0x4f,
        0x60,
        0x0c,
        0x3c,
        0xff,
        0x00,
        0xe5,
        0x7f,
        0xf9,
        0xc0,
        0x41,
        0x24,
        0xe0,
        0x14,
        0x76,
        0xff,
        0x33,
        0xa0,
        0x42,
        0x20,
        0x43,
        0x18,
        0x4c,
        0x00,
        0x87,
        0xd5,
        0x88,
        0x3f,
        0xd7,
        0x03,
        0xd9,
        0x10,
        0xd3,
        0x82,
        0xc8,
        0x08,
        0xc9,
        0x80,
        0x7c,
        0x00,
        0x7d,
        0x00,
        0x7c,
        0x03,
        0x7d,
        0x48,
        0x7d,
        0x48,
        0x7c,
        0x08,
        0x7d,
        0x20,
        0x7d,
        0x10,
        0x7d,
        0x0e,
        0x90,
        0x00,
        0x91,
        0x0e,
        0x91,
        0x1a,
        0x91,
        0x31,
        0x91,
        0x5a,
        0x91,
        0x69,
        0x91,
        0x75,
        0x91,
        0x7e,
        0x91,
        0x88,
        0x91,
        0x8f,
        0x91,
        0x96,
        0x91,
        0xa3,
        0x91,
        0xaf,
        0x91,
        0xc4,
        0x91,
        0xd7,
        0x91,
        0xe8,
        0x91,
        0x20,
        0x92,
        0x00,
        0x93,
        0x06,
        0x93,
        0xe3,
        0x93,
        0x05,
        0x93,
        0x05,
        0x93,
        0x00,
        0x93,
        0x04,
        0x93,
        0x00,
        0x93,
        0x00,
        0x93,
        0x00,
        0x93,
        0x00,
        0x93,
        0x00,
        0x93,
        0x00,
        0x93,
        0x00,
        0x96,
        0x00,
        0x97,
        0x08,
        0x97,
        0x19,
        0x97,
        0x02,
        0x97,
        0x0c,
        0x97,
        0x24,
        0x97,
        0x30,
        0x97,
        0x28,
        0x97,
        0x26,
        0x97,
        0x02,
        0x97,
        0x98,
        0x97,
        0x80,
        0x97,
        0x00,
        0x97,
        0x00,
        0xc3,
        0xed,
        0xa4,
        0x00,
        0xa8,
        0x00,
        0xc5,
        0x11,
        0xc6,
        0x51,
        0xbf,
        0x80,
        0xc7,
        0x10,
        0xb6,
        0x66,
        0xb8,
        0xA5,
        0xb7,
        0x64,
        0xb9,
        0x7C,
        0xb3,
        0xaf,
        0xb4,
        0x97,
        0xb5,
        0xFF,
        0xb0,
        0xC5,
        0xb1,
        0x94,
        0xb2,
        0x0f,
        0xc4,
        0x5c,
        0xc0,
        0x64,
        0xc1,
        0x4B,
        0x8c,
        0x00,
        0x86,
        0x3D,
        0x50,
        0x00,
        0x51,
        0xC8,
        0x52,
        0x96,
        0x53,
        0x00,
        0x54,
        0x00,
        0x55,
        0x00,
        0x5a,
        0xC8,
        0x5b,
        0x96,
        0x5c,
        0x00,
        0xd3,
        0x7f,
        0xc3,
        0xed,
        0x7f,
        0x00,
        0xda,
        0x00,
        0xe5,
        0x1f,
        0xe1,
        0x67,
        0xe0,
        0x00,
        0xdd,
        0x7f,
        0x05,
        0x00,

        0x12,
        0x40,
        0xd3,
        0x7f,
        0xc0,
        0x16,
        0xC1,
        0x12,
        0x8c,
        0x00,
        0x86,
        0x3d,
        0x50,
        0x00,
        0x51,
        0x2C,
        0x52,
        0x24,
        0x53,
        0x00,
        0x54,
        0x00,
        0x55,
        0x00,
        0x5A,
        0x2c,
        0x5b,
        0x24,
        0x5c,
        0x00,
};

const unsigned char OV2640_YUV422[][2] =
    {
        0xFF,
        0x00,
        0x05,
        0x00,
        0xDA,
        0x10,
        0xD7,
        0x03,
        0xDF,
        0x00,
        0x33,
        0x80,
        0x3C,
        0x40,
        0xe1,
        0x77,
        0x00,
        0x00,
};

const unsigned char OV2640_JPEG[][2] =
    {
        0xe0,
        0x14,
        0xe1,
        0x77,
        0xe5,
        0x1f,
        0xd7,
        0x03,
        0xda,
        0x10,
        0xe0,
        0x00,
        0xFF,
        0x01,
        0x04,
        0x08,
};

/* JPG 160x120 */
const unsigned char OV2640_160x120_JPEG[][2] =
    {
        0xff,
        0x01,
        0x12,
        0x40,
        0x17,
        0x11,
        0x18,
        0x43,
        0x19,
        0x00,
        0x1a,
        0x4b,
        0x32,
        0x09,
        0x4f,
        0xca,
        0x50,
        0xa8,
        0x5a,
        0x23,
        0x6d,
        0x00,
        0x39,
        0x12,
        0x35,
        0xda,
        0x22,
        0x1a,
        0x37,
        0xc3,
        0x23,
        0x00,
        0x34,
        0xc0,
        0x36,
        0x1a,
        0x06,
        0x88,
        0x07,
        0xc0,
        0x0d,
        0x87,
        0x0e,
        0x41,
        0x4c,
        0x00,
        0xff,
        0x00,
        0xe0,
        0x04,
        0xc0,
        0x64,
        0xc1,
        0x4b,
        0x86,
        0x35,
        0x50,
        0x92,
        0x51,
        0xc8,
        0x52,
        0x96,
        0x53,
        0x00,
        0x54,
        0x00,
        0x55,
        0x00,
        0x57,
        0x00,
        0x5a,
        0x28,
        0x5b,
        0x1e,
        0x5c,
        0x00,
        0xe0,
        0x00,
};

/* JPG, 0x176x144 */
const unsigned char OV2640_176x144_JPEG[][2] =
    {
        0xff,
        0x01,
        0x12,
        0x40,
        0x17,
        0x11,
        0x18,
        0x43,
        0x19,
        0x00,
        0x1a,
        0x4b,
        0x32,
        0x09,
        0x4f,
        0xca,
        0x50,
        0xa8,
        0x5a,
        0x23,
        0x6d,
        0x00,
        0x39,
        0x12,
        0x35,
        0xda,
        0x22,
        0x1a,
        0x37,
        0xc3,
        0x23,
        0x00,
        0x34,
        0xc0,
        0x36,
        0x1a,
        0x06,
        0x88,
        0x07,
        0xc0,
        0x0d,
        0x87,
        0x0e,
        0x41,
        0x4c,
        0x00,
        0xff,
        0x00,
        0xe0,
        0x04,
        0xc0,
        0x64,
        0xc1,
        0x4b,
        0x86,
        0x35,
        0x50,
        0x92,
        0x51,
        0xc8,
        0x52,
        0x96,
        0x53,
        0x00,
        0x54,
        0x00,
        0x55,
        0x00,
        0x57,
        0x00,
        0x5a,
        0x2c,
        0x5b,
        0x24,
        0x5c,
        0x00,
        0xe0,
        0x00,
};

/* JPG 320x240 */
const unsigned char OV2640_320x240_JPEG[][2] =
    {
        0xff,
        0x01,
        0x12,
        0x40,
        0x17,
        0x11,
        0x18,
        0x43,
        0x19,
        0x00,
        0x1a,
        0x4b,
        0x32,
        0x09,
        0x4f,
        0xca,
        0x50,
        0xa8,
        0x5a,
        0x23,
        0x6d,
        0x00,
        0x39,
        0x12,
        0x35,
        0xda,
        0x22,
        0x1a,
        0x37,
        0xc3,
        0x23,
        0x00,
        0x34,
        0xc0,
        0x36,
        0x1a,
        0x06,
        0x88,
        0x07,
        0xc0,
        0x0d,
        0x87,
        0x0e,
        0x41,
        0x4c,
        0x00,
        0xff,
        0x00,
        0xe0,
        0x04,
        0xc0,
        0x64,
        0xc1,
        0x4b,
        0x86,
        0x35,
        0x50,
        0x89,
        0x51,
        0xc8,
        0x52,
        0x96,
        0x53,
        0x00,
        0x54,
        0x00,
        0x55,
        0x00,
        0x57,
        0x00,
        0x5a,
        0x50,
        0x5b,
        0x3c,
        0x5c,
        0x00,
        0xe0,
        0x00,
};

/* JPG 352x288 */
const unsigned char OV2640_352x288_JPEG[][2] =
    {
        0xff,
        0x01,
        0x12,
        0x40,
        0x17,
        0x11,
        0x18,
        0x43,
        0x19,
        0x00,
        0x1a,
        0x4b,
        0x32,
        0x09,
        0x4f,
        0xca,
        0x50,
        0xa8,
        0x5a,
        0x23,
        0x6d,
        0x00,
        0x39,
        0x12,
        0x35,
        0xda,
        0x22,
        0x1a,
        0x37,
        0xc3,
        0x23,
        0x00,
        0x34,
        0xc0,
        0x36,
        0x1a,
        0x06,
        0x88,
        0x07,
        0xc0,
        0x0d,
        0x87,
        0x0e,
        0x41,
        0x4c,
        0x00,
        0xff,
        0x00,
        0xe0,
        0x04,
        0xc0,
        0x64,
        0xc1,
        0x4b,
        0x86,
        0x35,
        0x50,
        0x89,
        0x51,
        0xc8,
        0x52,
        0x96,
        0x53,
        0x00,
        0x54,
        0x00,
        0x55,
        0x00,
        0x57,
        0x00,
        0x5a,
        0x58,
        0x5b,
        0x48,
        0x5c,
        0x00,
        0xe0,
        0x00,
};

void OV2640_Init(OV2640_TypeDef *OV2640)
{
  _OV2640 = OV2640;
}

/**
  * @brief  Resets the OV2640 camera.
  * @param  None
  * @retval None
  */
void OV2640_Reset(void)
{
  /*OV2640*/
  OV2640_WriteReg(OV2640_DSP_RA_DLMT, 0x01);
  OV2640_WriteReg(OV2640_SENSOR_COM7, 0x80);
}

/**
  * @brief
  * @param  OV2640ID:
  * @retval None
  */
void OV2640_ReadID(OV2640_IDTypeDef *OV2640ID)
{

  OV2640_WriteReg(OV2640_DSP_RA_DLMT, 0x01);

  OV2640ID->Manufacturer_ID1 = OV2640_ReadReg(OV2640_SENSOR_MIDH);
  OV2640ID->Manufacturer_ID2 = OV2640_ReadReg(OV2640_SENSOR_MIDL);
  OV2640ID->PIDH = OV2640_ReadReg(OV2640_SENSOR_PIDH);
  OV2640ID->PIDL = OV2640_ReadReg(OV2640_SENSOR_PIDL);
}

/**
  * @brief
  * @param
  * @retval 0
  */
u8 OV2640_OutSize_Set(u16 width, u16 height)
{
  u16 outh;
  u16 outw;
  u8 temp;
  if (width % 4)
    return 1;
  if (height % 4)
    return 2;
  outw = width / 4;
  outh = height / 4;
  OV2640_WriteReg(0XFF, 0X00);
  OV2640_WriteReg(0XE0, 0X04);
  OV2640_WriteReg(0X50, outw & 0X00);
  OV2640_WriteReg(0X5A, outw & 0XFF);
  OV2640_WriteReg(0X5B, outh & 0XFF);
  temp = (outw >> 8) & 0X03;
  temp |= (outh >> 6) & 0X04;
  OV2640_WriteReg(0X5C, temp);
  OV2640_WriteReg(0XE0, 0X00);
  return 0;
}

/**
  * @brief  Configures the OV2640 camera in QQVGA mode.
  * @param  None
  * @retval None
  */
void OV2640_QQVGAConfig(void)
{
  uint32_t i;

  OV2640_Reset();
  Delay(200);

  /* Initialize OV2640 */
  for (i = 0; i < (sizeof(OV2640_QQVGA) / 2); i++)
  {
    OV2640_WriteReg(OV2640_QQVGA[i][0], OV2640_QQVGA[i][1]);
    Delay(2);
  }
}

/**
  * @brief
  * @param  None
  * @retval None
  */
void OV2640_UXGAConfig(void)
{
  uint32_t i;

  OV2640_Reset();

  for (i = 0; i < (sizeof(OV2640_UXGA) / 2); i++)
  {
    OV2640_WriteReg(OV2640_UXGA[i][0], OV2640_UXGA[i][1]);
  }

  OV2640_OutSize_Set(_OV2640->frame->width, _OV2640->frame->height);
}

/**
  * @brief  Configures the OV2640 camera in JPEG mode.
  * @param  JPEGImageSize: JPEG image size
  * @retval None
  */
void OV2640_JPEGConfig(ImageFormat_TypeDef ImageFormat)
{
  uint32_t i;

  OV2640_Reset();
  HAL_Delay(200);

  /* Initialize OV2640 */
  for (i = 0; i < (sizeof(OV2640_JPEG_INIT) / 2); i++)
  {
    OV2640_WriteReg(OV2640_JPEG_INIT[i][0], OV2640_JPEG_INIT[i][1]);
  }

  /* Set to output YUV422 */
  for (i = 0; i < (sizeof(OV2640_YUV422) / 2); i++)
  {
    OV2640_WriteReg(OV2640_YUV422[i][0], OV2640_YUV422[i][1]);
  }

  OV2640_WriteReg(0xff, 0x01);
  OV2640_WriteReg(0x15, 0x00);

  /* Set to output JPEG */
  for (i = 0; i < (sizeof(OV2640_JPEG) / 2); i++)
  {
    OV2640_WriteReg(OV2640_JPEG[i][0], OV2640_JPEG[i][1]);
  }

  HAL_Delay(100);

  switch (ImageFormat)
  {
  case JPEG_160x120:
  {
    for (i = 0; i < (sizeof(OV2640_160x120_JPEG) / 2); i++)
    {
      OV2640_WriteReg(OV2640_160x120_JPEG[i][0], OV2640_160x120_JPEG[i][1]);
    }
    break;
  }
  case JPEG_176x144:
  {
    for (i = 0; i < (sizeof(OV2640_176x144_JPEG) / 2); i++)
    {
      OV2640_WriteReg(OV2640_176x144_JPEG[i][0], OV2640_176x144_JPEG[i][1]);
    }
    break;
  }
  case JPEG_320x240:
  {
    for (i = 0; i < (sizeof(OV2640_320x240_JPEG) / 2); i++)
    {
      OV2640_WriteReg(OV2640_320x240_JPEG[i][0], OV2640_320x240_JPEG[i][1]);
    }
    break;
  }
  case JPEG_352x288:
  {
    for (i = 0; i < (sizeof(OV2640_352x288_JPEG) / 2); i++)
    {
      OV2640_WriteReg(OV2640_352x288_JPEG[i][0], OV2640_352x288_JPEG[i][1]);
    }
    break;
  }
  default:
  {
    for (i = 0; i < (sizeof(OV2640_160x120_JPEG) / 2); i++)
    {
      OV2640_WriteReg(OV2640_160x120_JPEG[i][0], OV2640_160x120_JPEG[i][1]);
    }
    break;
  }
  }
}

/**
  * @brief
  * @param
  *         0x00 Auto
  *         0x01 Sunny
  *         0x02 Cloudy
  *         0x03 Office
  *         0x04 Home

  * @retval None
  */
void OV2640_LightMode(uint8_t mode)
{
  switch (mode)
  {

  case 0: //Auto
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0xc7, 0x00); //AWB on
    break;

  case 1: //Sunny
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0xc7, 0x40); //AWB off
    OV2640_WriteReg(0xcc, 0x5e);
    OV2640_WriteReg(0xcd, 0x41);
    OV2640_WriteReg(0xce, 0x54);

    break;

  case 2: //Cloudy
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0xc7, 0x40); //AWB off
    OV2640_WriteReg(0xcc, 0x65);
    OV2640_WriteReg(0xcd, 0x41);
    OV2640_WriteReg(0xce, 0x4f);
    break;

  case 3: //Office
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0xc7, 0x40); //AWB off
    OV2640_WriteReg(0xcc, 0x52);
    OV2640_WriteReg(0xcd, 0x41);
    OV2640_WriteReg(0xce, 0x66);
    break;

  case 4: //Home
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0xc7, 0x40); //AWB off
    OV2640_WriteReg(0xcc, 0x42);
    OV2640_WriteReg(0xcd, 0x3f);
    OV2640_WriteReg(0xce, 0x71);
    break;
  }
}

/**
  * @brief
  * @param
  *         0x00 Antique
  *         0x01 Bluish
  *         0x02 Greenish
  *         0x03 Reddish
  *         0x04 B&W
  *         0x05 Negative
  *         0x06 B&W negative
  *         0x07 Normal

  * @retval None
  */
void OV2640_SpecialEffects(uint8_t mode)
{
  switch (mode)
  {
  case 0:
    // Antique
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x18);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0x40);
    OV2640_WriteReg(0x7d, 0xa6);
    break;

  case 1:
    //Bluish
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x18);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0xa0);
    OV2640_WriteReg(0x7d, 0x40);

    break;

  case 2:
    //Greenish
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x18);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0x40);
    OV2640_WriteReg(0x7d, 0x40);
    break;

  case 3:
    // Reddish
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x18);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0x40);
    OV2640_WriteReg(0x7d, 0xc0);
    break;

  case 4:
    // B&W
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x18);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0x80);
    OV2640_WriteReg(0x7d, 0x80);
    break;

  case 5:
    //Negative
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x40);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0x80);
    OV2640_WriteReg(0x7d, 0x80);

    break;

  case 6:
    //B&W negative
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x58);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0x80);
    OV2640_WriteReg(0x7d, 0x80);

    break;

  case 7:
    //Normal
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x00);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0x80);
    OV2640_WriteReg(0x7d, 0x80);

    break;
  }
}

/**
  * @brief  Configures the OV2640 camera brightness.
  * @param  Brightness: Brightness value, where Brightness can be:
  *         0x40 for Brightness +2,
  *         0x30 for Brightness +1,
  *         0x20 for Brightness 0,
  *         0x10 for Brightness -1,
  *         0x00 for Brightness -2,
  * @retval None
  */
void OV2640_BrightnessConfig(uint8_t Brightness)
{
  OV2640_WriteReg(0xff, 0x00);
  OV2640_WriteReg(0x7c, 0x00);
  OV2640_WriteReg(0x7d, 0x04);
  OV2640_WriteReg(0x7c, 0x09);
  OV2640_WriteReg(0x7d, Brightness);
  OV2640_WriteReg(0x7d, 0x00);
}

/**
  * @brief  Configures the OV2640 camera Black and white mode.
  * @param  BlackWhite: BlackWhite value, where BlackWhite can be:
  *         0x18 for B&W,
  *         0x40 for Negative,
  *         0x58 for B&W negative,
  *         0x00 for Normal,
  * @retval None
  */
void OV2640_BandWConfig(uint8_t BlackWhite)
{
  OV2640_WriteReg(0xff, 0x00);
  OV2640_WriteReg(0x7c, 0x00);
  OV2640_WriteReg(0x7d, BlackWhite);
  OV2640_WriteReg(0x7c, 0x05);
  OV2640_WriteReg(0x7d, 0x80);
  OV2640_WriteReg(0x7d, 0x80);
}

/**
  * @brief  Configures the OV2640 camera color effects.
  * @param  value1: Color effects value1
  * @param  value2: Color effects value2
  *         where value1 and value2 can be:
  *         value1 = 0x40, value2 = 0xa6 for Antique,
  *         value1 = 0xa0, value2 = 0x40 for Bluish,
  *         value1 = 0x40, value2 = 0x40 for Greenish,
  *         value1 = 0x40, value2 = 0xc0 for Reddish,
  * @retval None
  */
void OV2640_ColorEffectsConfig(uint8_t value1, uint8_t value2)
{
  OV2640_WriteReg(0xff, 0x00);
  OV2640_WriteReg(0x7c, 0x00);
  OV2640_WriteReg(0x7d, 0x18);
  OV2640_WriteReg(0x7c, 0x05);
  OV2640_WriteReg(0x7d, value1);
  OV2640_WriteReg(0x7d, value2);
}

/**
  * @brief  Configures the OV2640 camera contrast.
  * @param  value1: Contrast value1
  * @param  value2: Contrast value2
  *         where value1 and value2 can be:
  *         value1 = 0x28, value2 = 0x0c for Contrast +2,
  *         value1 = 0x24, value2 = 0x16 for Contrast +1,
  *         value1 = 0x20, value2 = 0x20 for Contrast 0,
  *         value1 = 0x1c, value2 = 0x2a for Contrast -1,
  *         value1 = 0x18, value2 = 0x34 for Contrast -2,
  * @retval None
  */
void OV2640_ContrastConfig(uint8_t value1, uint8_t value2)
{
  OV2640_WriteReg(0xff, 0x00);
  OV2640_WriteReg(0x7c, 0x00);
  OV2640_WriteReg(0x7d, 0x04);
  OV2640_WriteReg(0x7c, 0x07);
  OV2640_WriteReg(0x7d, 0x20);
  OV2640_WriteReg(0x7d, value1);
  OV2640_WriteReg(0x7d, value2);
  OV2640_WriteReg(0x7d, 0x06);
}

void OV2640_Start(void)
{
	OV2640_DMA_Config(_OV2640->frame->buffer, _OV2640->frame->length);
}

/**
  * @brief
  * @param
  * @param
  */
void OV2640_DMA_Config(uint8_t *DMA_Memory0BaseAddr, uint32_t DMA_BufferSize)
{

  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_DMA_Init(_OV2640->dma);

  __HAL_LINKDMA(_OV2640->dcmi, DMA_Handle, *(_OV2640->dma));

  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

  HAL_DCMI_Start_DMA(_OV2640->dcmi, DCMI_MODE_CONTINUOUS, (uint32_t)DMA_Memory0BaseAddr, DMA_BufferSize);
}

/**
  * @brief
  * @param  None
  * @retval None
  */
void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	_OV2640->fps++;
	_OV2640->time_show = 1;

	//osSemaphoreAcquire(_OV2640->frame->sem_showHandle, wait);
	LOG("\n\rfps:%d\n\r", _OV2640->fps);
}


/**
  * @brief  Manages error callback by re-initializing I2C.
  * @param  Addr: I2C Address
  * @retval None
  */
static void I2Cx_Error(void)
{

  HAL_I2C_DeInit(_OV2640->i2c);
  if (HAL_I2C_GetState(_OV2640->i2c) == HAL_I2C_STATE_RESET)
    {

      __HAL_RCC_I2C4_FORCE_RESET();
      __HAL_RCC_I2C4_RELEASE_RESET();

      HAL_I2C_Init(_OV2640->i2c);

      HAL_I2CEx_AnalogFilter_Config(_OV2640->i2c, I2C_ANALOGFILTER_ENABLE);
    }
    HAL_Delay(100);
}

/**
  * @brief
  * @param  ַ
  * @param
  * @retval
  */
uint8_t OV2640_WriteReg(uint16_t Addr, uint8_t Data)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(_OV2640->i2c, OV2640_DEVICE_ADDRESS, (uint16_t)Addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&Data, 1, 1000);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Re-Initiaize the I2C Bus */
    I2Cx_Error();
  }
  return status;
}

/**
  * @brief
  * @param
  */
uint8_t OV2640_ReadReg(uint16_t Addr)
{
  uint8_t Data = 0;

  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(_OV2640->i2c, OV2640_DEVICE_ADDRESS, (uint16_t)Addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&Data, 1, 1000);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* I2C error occurred */
    I2Cx_Error();
  }
  /* return the read data */
  return Data;
}
