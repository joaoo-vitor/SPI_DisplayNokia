/*
 * imagem.h
 *
 *  Created on: 26 de jun de 2019
 *      Author: ASUS
 */

#ifndef IMAGEM_H_
#define IMAGEM_H_



unsigned char const liber_bmp2[] = {
0X3F,0X01,0X01,0X01,0X01,0X01,0X00,0X00,0X80,0XC0,0XE0,0XF0,0XF8,0X7C,0X3E,0X9C,
0XC8,0XE0,0XE0,0XC0,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0XE0,0X20,0X20,0X20,0X00,
0X00,0XE0,0X00,0X00,0X00,0XE0,0X00,0X00,0XE0,0XC0,0X00,0X00,0XE0,0X00,0X00,0XE0,
0X20,0X20,0X40,0X80,0X00,0X00,0X00,0XC0,0X20,0XC0,0X00,0X00,0X00,0XC0,0X20,0X20,
0X20,0X40,0X00,0X00,0X08,0XC4,0X28,0XC4,0X00,0X00,0X00,0XC0,0X20,0X20,0X20,0X20,
0XC0,0X01,0X01,0X01,0X80,0XC0,0XE0,0XF0,0XF8,0X7C,0X3E,0X9F,0XCF,0XE7,0XF3,0XF9,
0XFC,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFC,0XF8,0XF0,0XE0,0XCF,
0X01,0X01,0X00,0XC0,0X00,0X07,0X08,0X08,0X08,0XC7,0X00,0X00,0XCF,0X40,0X41,0X46,
0X8F,0X00,0X00,0XCF,0X48,0X48,0X44,0X43,0X00,0X08,0XC7,0X42,0X42,0X42,0X87,0X08,
0X00,0X07,0X98,0X48,0X88,0X04,0X00,0X08,0X47,0X42,0XC2,0X42,0X47,0X08,0X00,0X87,
0X48,0X48,0X48,0X48,0X87,0X00,0X00,0X00,0X00,0X01,0X03,0X07,0X0F,0X1F,0X3E,0X7C,
0XF9,0XF3,0X67,0X0F,0X9F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X7F,0X3F,0X1F,0X0F,
0X07,0X03,0X01,0X00,0X00,0X00,0X00,0X1F,0X10,0X10,0X10,0X10,0X00,0X1F,0X00,0X00,
0X1F,0X12,0X12,0X12,0X0F,0X00,0X00,0X1F,0X12,0X12,0X12,0X12,0X00,0X00,0X1F,0X02,
0X02,0X06,0X19,0X00,0X10,0X0E,0X05,0X04,0X05,0X0E,0X10,0X00,0X00,0X00,0X1F,0X00,
0X00,0X00,0X00,0X0F,0X10,0X10,0X10,0X10,0X0F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X02,0X07,0X0F,0X1F,0X1F,0X0F,0XC7,0X23,0X21,0X20,
0X40,0X00,0X00,0X80,0X00,0X00,0X80,0X00,0X00,0X80,0X80,0X80,0X00,0X00,0X80,0X80,
0X80,0X00,0X00,0X80,0X80,0X00,0X00,0X00,0X00,0X20,0X20,0XE0,0X20,0X20,0X00,0X00,
0X00,0XA0,0X90,0X00,0X00,0X00,0X00,0X80,0X80,0X00,0X00,0X80,0X80,0X80,0X00,0X00,
0X00,0XA0,0X00,0X00,0X00,0X80,0X80,0X00,0X00,0X00,0X80,0X80,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X80,0X80,0X80,0X80,0X80,0X80,0X00,0X00,0X80,0X00,0X00,0X00,
0X07,0X08,0X08,0X88,0X84,0X80,0X80,0X87,0X88,0X08,0X8F,0X80,0X80,0X8F,0X80,0X80,
0X80,0X09,0X8A,0X8A,0X84,0X80,0X87,0X88,0X08,0X07,0X00,0X00,0X00,0XA0,0X80,0XAF,
0X00,0X00,0X00,0X00,0X87,0X0A,0X0A,0X0B,0X00,0X00,0X87,0X08,0X08,0X85,0X00,0X0F,
0X00,0X00,0X8F,0X80,0X80,0X0F,0X00,0X00,0X07,0X08,0X08,0X85,0X00,0X07,0X08,0X08,
0X07,0X00,0X00,0X00,0XF8,0X80,0X00,0X00,0XFF,0X88,0X88,0X88,0X88,0X88,0X00,0X00,
0XFF,0X80,0X80,0X80,0X80,0X80,0X00,0XFF,0X88,0X88,0X88,0X88,0X88,0X00,0X00,0X00,
0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X08,0X08,0X08,0X18,0X68,0X87,0X00,0X00,0X3E,
0X41,0X80,0X80,0X80,0X41,0X3E,0X00,0X00,0XFF,0X01,0X06,0X08,0X30,0X40,0XFF,0X00,
0X00,0XFF,0X00,0X00,0X3E,0X41,0X80,0X80,0X80,0X41,0X22,0X00,0XC0,0X38,0X17,0X10,
0X17,0X38,0XC0,0X00,0X00,0X80,0X80,0X80
};
unsigned char const liber_bmp3[] = {
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X80,0XC0,0XE0,
0XF0,0XF8,0XFC,0XFE,0XFF,0X7F,0X3F,0X1F,0X8E,0XC6,0XE0,0XF0,0XF8,0XF0,0XE0,0XC0,
0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X40,0X60,0X70,0X78,0X38,0X18,0X48,0X60,0X70,0X78,0X78,
0X38,0X18,0X48,0X60,0X70,0X78,0X78,0X78,0X38,0X18,0X48,0X60,0X70,0X78,0X78,0X7A,
0X79,0X7A,0X3B,0X13,0X0B,0X43,0X63,0X71,0X78,0X38,0X18,0X4A,0X63,0X73,0X7B,0X7B,
0X7B,0X7B,0X7B,0X7B,0X7B,0X7B,0X7A,0X78,0X78,0XF0,0XE0,0XC0,0X80,0X00,0X00,0X20,
0X30,0X38,0X2C,0X36,0X1B,0X0D,0X0D,0X0B,0X12,0X24,0X28,0X30,0X20,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X10,0X18,0X1C,0X1E,0X13,0X91,0XD1,0XF1,0XF1,0X71,0X31,0X11,0X01,0X01,0X81,0X41,
0X21,0X11,0X91,0XD1,0XD1,0XDF,0XEF,0XF0,0XFF,0XFF,0X3F,0X1F,0X07,0X03,0X01,0X61,
0X71,0X59,0X49,0X49,0X49,0X49,0XC9,0XB1,0X81,0X81,0X81,0XC1,0XF2,0X3C,0X01,0XCF,
0X67,0X33,0X1A,0X0C,0X06,0XC3,0X61,0X31,0X09,0X09,0X09,0X09,0X09,0XF1,0X01,0X03,
0X03,0X02,0X06,0XFC,0XF8,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X18,0X3C,0X26,0X23,0X21,0X20,0X20,0X10,0X3C,
0X3E,0X3D,0X38,0X30,0X22,0X07,0X0F,0X1F,0X3F,0X3F,0X27,0X23,0X21,0X20,0X30,0X38,
0X3C,0X0E,0X03,0X03,0X37,0X3E,0X20,0X20,0X20,0X20,0X21,0X21,0X27,0X3F,0X3F,0X38,
0X00,0X00,0X1F,0X3F,0X38,0X30,0X30,0X30,0X30,0X33,0X36,0X34,0X34,0X34,0X36,0X32,
0X33,0X31,0X38,0X18,0X1C,0X0C,0X06,0X03,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X1F,0X15,0X11,0X00,0X1F,0X10,0X10,0X00,0X1F,0X15,0X11,0X00,0X01,0X1F,
0X01,0X01,0X00,0X1F,0X05,0X0D,0X10,0X00,0X0E,0X11,0X11,0X4A,0XC0,0XDF,0XC0,0XC0,
0XDE,0X80,0X1F,0X00,0X4E,0XD1,0XD0,0XC0,0XDE,0XC5,0X45,0X1C,0X00,0X98,0XDC,0XDE,
0XDF,0XDF,0XDF,0XDF,0XDF,0X5F,0X1F,0X0F,0X07,0X13,0X19,0X1C,0X1E,0X0F,0X07,0X13,
0X19,0X1C,0X1E,0X0F,0X07,0X13,0X19,0X1C,0X0E,0X07,0X03,0X01,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X01,0X03,0X07,0X0F,0X1F,0X3F,0X3E,0X1C,0X88,0XC1,0XE3,0XF1,0XF8,0XFC,0XFE,
0XFF,0X7F,0X3F,0X1F,0X0F,0X07,0X03,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00
};
/*
unsigned char const aero[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x02, 0x06, 0x04, 0x48, 0xb8, 0x50, 0xb8, 0x28, 0x50, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x3a, 0x78, 0xf9, 0xfa, 0xf4, 0x0c, 0x36, 0xff, 0xfe, 0xfc, 0xf8, 0xf8, 0xf8, 0xf0, 0xe0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x40, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x60, 0x38, 0xd8, 0x08, 0x04, 0x04, 0x04, 0x70, 0x72, 0x32, 0x32, 0x02, 0x72, 0x43, 0x02, 0x03, 0x3f, 0x3b, 0x03, 0x03, 0x03, 0x03, 0x66, 0x86, 0x04, 0x1c, 0x18, 0x70, 0xe0, 0xc0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x38, 0x30, 0x6f, 0x1f, 0x3e, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xbf, 0x9f, 0x44, 0x48, 0x30, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0xc0, 0x1c, 0x04, 0x02, 0x02, 0x02, 0x1e, 0x3a,
  0x00, 0x00, 0x3c, 0x3e, 0x63, 0xe0, 0xf8, 0xfc, 0xee, 0xa1, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x03, 0x2c, 0x78, 0x10, 0x10, 0x00, 0x03, 0x07, 0x07, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0xf8, 0xfb, 0xfc, 0xc0, 0x00, 0x00, 0x00, 0x03, 0x07, 0x1e, 0xfc, 0xf0, 0x0c, 0x4f, 0x3f, 0xf0, 0x7f, 0x23, 0x63, 0x67, 0xc7, 0xcf, 0x8f, 0x8f, 0x9f, 0x9f, 0xfe, 0xfe, 0x03, 0x03, 0xff, 0xff, 0x03, 0x03, 0xfe, 0xfe, 0xfe, 0x03, 0x0f, 0x9f, 0xff, 0xee, 0x86, 0x83, 0x03, 0x03, 0x03, 0x06,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x03, 0x83, 0x83, 0x81, 0xc1, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x06, 0x00, 0x00, 0x80, 0xc0, 0xf0, 0x7f, 0x6f, 0x40, 0x7e, 0x7f, 0x71, 0xf3, 0x66, 0x46, 0x4c, 0x6c, 0x7c, 0x78, 0x79, 0x4f, 0x47, 0x7f, 0x7f, 0xc0, 0xc0, 0xff, 0xcf, 0x80, 0xc0, 0xff, 0x7f, 0xff, 0xfe, 0xff, 0xff, 0xff, 0x03, 0x02, 0x01, 0x01, 0x03, 0xfe, 0x38,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x0e, 0x1e, 0x3b, 0x31, 0xf1, 0xf9, 0xfc, 0x9c, 0x0e, 0x06, 0x06, 0x02, 0x01, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x40, 0x80, 0x80, 0xc0, 0x7e, 0xbe, 0xc0, 0xc0, 0x60, 0x30, 0x18, 0x0e, 0x06, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06
};
*/
#endif /* IMAGEM_H_ */
