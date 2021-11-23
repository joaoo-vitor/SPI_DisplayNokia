#include "stm32f4xx_hal.h"
typedef enum {B_FREE=0,B_BUSY} BufStatus_t;

typedef struct{
	uint8_t *dado;
	BufStatus_t estado;
	uint16_t ocupacao;
}BufferCompartilhado_t;

typedef enum{
	LCD_BLOCK, LCD_IT, LCD_DMA
}LCD_ModeTypeDef;

typedef struct
{
	SPI_HandleTypeDef *hspi; //!Handle da SPI utilizada

	LCD_ModeTypeDef modo; //! Modo de operação do SPI
	/*
	 * (modo de op. pode ser bloqueante, por interrupção ou por dma)
	 */

	GPIO_TypeDef* CS_Port; //!Porta do pino CS
	uint16_t CS_Pin; //!Pino do CS

	GPIO_TypeDef* DC_Port; //!Porta do pino CS
	uint16_t DC_Pin; //!Pino do CS

	void (*TxCpltCallback)(SPI_HandleTypeDef *hspi);
	/*
	 * usuário pode apontar para a função de callback que quiser
	 * no Tx
	 */
}LCD_HandleTypeDef;

#define LCD_CS 	12 		//CE
#define LCD_RST 10 		//RST
/*
 * MO E SCK são pinos da SPI
 */
#define LCD_DC	14 		//DO
#define PORT	GPIOB	//GPIO onde está o display
#define LEFT 0
#define RIGHT 9999
#define CENTER 9998

#define false 0
#define true 1

#define TAM_MAX_STRING 100 //num. máximo de caract. para mandar string (arbitrário)
#define TAM_TELA ((uint16_t) 504)
#define TAM_BUFF  ((uint16_t) 504)
#define LARG_CHAR 6 //largura de bytes de um caract

#define LCD_DATA 1 //informação	a ser mandada
#define LCD_COMMAND 0 //comando a ser mandado

#define X_ADDR_MASK 0x7f   //mask de 7 bits
#define Y_ADDR_MASK 0x07   //mask de 3 bits

#define PCD8544_SETYADDR 0x40
#define PCD8544_SETXADDR 0x80

//==================DEFINES DO DISPLAY====================
//os seguintes defines foram retirados do seguinte repositório do GitHub:
//https://github.com/adafruit/Adafruit-PCD8544-Nokia-5110-LCD-library/blob/master/Adafruit_PCD8544.h
#define LCD5110_POWERDOWN 0x04 ///< Function set, Power down mode
#define LCD5110_ENTRYMODE 0x02 ///< Function set, Entry mode
#define LCD5110_EXTENDEDINSTRUCTION                                            \
		0x01 ///< Function set, Extended instruction set control

#define LCD5110_DISPLAYBLANK 0x0    ///< Display control, blank
#define LCD5110_DISPLAYNORMAL 0x4   ///< Display control, normal mode
#define LCD5110_DISPLAYALLON 0x1    ///< Display control, all segments on
#define LCD5110_DISPLAYINVERTED 0x5 ///< Display control, inverse mode

#define LCD5110_FUNCTIONSET 0x20 ///< Basic instruction set
#define LCD5110_DISPLAYCONTROL                                                 \
		0x08 ///< Basic instruction set - Set display configuration
#define LCD5110_SETYADDR                                                       \
		0x40 ///< Basic instruction set - Set Y address of RAM, 0 <= Y <= 5
#define LCD5110_SETXADDR                                                       \
		0x80 ///< Basic instruction set - Set X address of RAM, 0 <= X <= 83

#define LCD5110_SETTEMP                                                        \
		0x04 ///< Extended instruction set - Set temperature coefficient
#define LCD5110_SETBIAS 0x10 ///< Extended instruction set - Set bias system
#define LCD5110_SETVOP                                                         \
		0x80 ///< Extended instruction set - Write Vop to register

#define LCD_CONTRASTE 0x40
#define LCD_TEMPERATURA 0x02
#define LCD_VALOR_BIAS 0x13

#define SPI_TIMEOUT 30000

void LCD5110_init(LCD_HandleTypeDef *hlcd5110);

void LCD5110_set_callback (void *callback(SPI_HandleTypeDef *hspi));

HAL_StatusTypeDef LCD_write_bloque(BufferCompartilhado_t *b, uint8_t mode);

HAL_StatusTypeDef LCD_write_IT(BufferCompartilhado_t *b, uint8_t mode);

HAL_StatusTypeDef LCD_write_DMA(BufferCompartilhado_t *b, uint8_t mode);

void LCD_drawchar(char c, uint8_t *dat);

void LCD_drawchar_inv(char c, uint8_t *dat);

HAL_StatusTypeDef LCD5110_write_char(unsigned char c, uint8_t invert);

HAL_StatusTypeDef LCD5110_clear(void);

HAL_StatusTypeDef LCD5110_write_block(uint8_t *img, uint16_t tam);

HAL_StatusTypeDef LCD5110_set_XY(uint8_t x, uint8_t y);

HAL_StatusTypeDef LCD5110_write_string(char *s);



