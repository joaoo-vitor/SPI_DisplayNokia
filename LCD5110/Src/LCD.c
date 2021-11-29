/*
 * @file lcd.c
 * @author João Vitor de Souza
 *
 * @brief BIBLIOTECA LCD NOKIA5110 HAL E SPI, com possibilidade
 * de escolha de modo de operação (DMA, IT, MODO BLOQUEANTE), e
 * também a escolha do spi usado e os pinos para as funções CS
 * e DC.
 */
#include "stm32f4xx.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_gpio.h"

#include "LCD.h"
#include "font.h"

static LCD_HandleTypeDef *lcd; //!handler display
static uint8_t buffer[TAM_BUFF]; //!espaço de memória auxiliar

static BufferCompartilhado_t buf;

static BufferCompartilhado_t *buff_atual;

//ponteiro para função LCD_Write
static HAL_StatusTypeDef (*LCD_write)(BufferCompartilhado_t *b, uint8_t mode);

/*array para limpar tela
 * é preenchida no INIT
 * não é const para poder receber assign
 */
uint8_t telaLimpa[TAM_TELA];


/**
  * @brief  Manda informação por SPI no modo bloqueante
  * @param  Struct do buffer com dado a ser mandado
  * @param  Modo da mensagem (dado ou comando?)
  * @retval HAL status
  */
HAL_StatusTypeDef LCD_write_bloque(BufferCompartilhado_t *b, uint8_t mode) //manda UM BLOCO (de qqr tamanho) PARA O DISPLAY
{
	HAL_StatusTypeDef status;

	//Ativa (desliga) CS
	HAL_GPIO_WritePin(lcd->CS_Port, lcd->CS_Pin, 0);

	//Seleção dados/comando
	HAL_GPIO_WritePin(lcd->DC_Port, lcd->DC_Pin, mode);

	status = HAL_SPI_Transmit(lcd->hspi, b->dado, b->ocupacao, SPI_TIMEOUT);

	//Fim da transf.
	HAL_GPIO_WritePin(lcd->CS_Port, lcd->CS_Pin, 1);

	b->estado=B_FREE;
	b->dado = buffer;
	return status;

}

/**
  * @brief  Manda informação por SPI no modo interrupção
  * @param  Struct do buffer com dado a ser mandado
  * @param  Modo da mensagem (dado ou comando?)
  * @retval HAL status
  */
HAL_StatusTypeDef LCD_write_IT(BufferCompartilhado_t *b, uint8_t mode) //manda UM BLOCO (de qqr tamanho) PARA O DISPLAY
{
	HAL_StatusTypeDef status;

	if (lcd->hspi->State != HAL_SPI_STATE_READY)
		return HAL_BUSY;
	buff_atual = b;

	//Ativa (desliga) CS (chipselect)
	HAL_GPIO_WritePin(lcd->CS_Port, lcd->CS_Pin, 0);
	//Seleção dados/comando
	HAL_GPIO_WritePin(lcd->DC_Port, lcd->DC_Pin, mode);

	//envia dado
	status = HAL_SPI_Transmit_IT(lcd->hspi, b->dado, b->ocupacao);

	return status;

}

/**
  * @brief  Manda informação por SPI no modo DMA
  * @param  Struct do buffer com dado a ser mandado
  * @param  Modo da mensagem (dado ou comando?)
  * @retval HAL status
  */
HAL_StatusTypeDef LCD_write_DMA(BufferCompartilhado_t *b, uint8_t mode) //manda UM BLOCO (de qqr tamanho) PARA O DISPLAY
{
	HAL_StatusTypeDef status;

	if (lcd->hspi->State != HAL_SPI_STATE_READY)
		return HAL_BUSY;
	buff_atual = b;

	//Ativa (desliga) CS (chipselect)
	HAL_GPIO_WritePin(lcd->CS_Port, lcd->CS_Pin, 0);
	//Seleção dados/comando
	HAL_GPIO_WritePin(lcd->DC_Port, lcd->DC_Pin, mode);

	//envia dado
	status = HAL_SPI_Transmit_DMA(lcd->hspi, b->dado, b->ocupacao);

	return status;

}

/**
  * @brief  Inicializa o display com as configurações básicas e modo de operação SPI
  * @param  Handler do display em questão
  */
void LCD5110_init(LCD_HandleTypeDef *hlcd5110) {
	lcd = hlcd5110;
	//preenche com zeros array para clearScream
	for (int i = 0; i < TAM_TELA; i++) {
		telaLimpa[i] = 0;
	}

	buf.dado = buffer;
	buf.ocupacao = 0;
	buf.estado = B_FREE;

	//!Aponta a função write para a operação específica
	switch(lcd->modo){
	default:
		LCD_write = LCD_write_bloque;
		break;
	case LCD_IT:
		LCD_write = LCD_write_IT;
		break;
	case LCD_DMA:
		LCD_write = LCD_write_DMA;
		break;
	}

	//garante que a função de callback comece nula (segurança)
	lcd->TxCpltCallback=NULL;

	HAL_GPIO_WritePin(lcd->DC_Port, lcd->DC_Pin, 1);
	HAL_GPIO_WritePin(lcd->CS_Port, lcd->CS_Pin, 1);

	HAL_GPIO_WritePin(lcd->RST_Port, lcd->RST_Pin, 0);
	HAL_Delay(10);
	HAL_GPIO_WritePin(lcd->RST_Port, lcd->RST_Pin, 1);

	while(LCD5110_clear()!=HAL_OK);
	buffer[0] = LCD5110_FUNCTIONSET | LCD5110_EXTENDEDINSTRUCTION; //x21
	buffer[1] = LCD5110_SETVOP | LCD_CONTRASTE; //C0
	buffer[2] = LCD5110_SETTEMP | LCD_TEMPERATURA; //x06
	buffer[3] = LCD5110_SETBIAS | LCD_VALOR_BIAS; //x13
	buffer[4] = LCD5110_FUNCTIONSET; //x20
	buffer[5] = (LCD5110_DISPLAYCONTROL | LCD5110_DISPLAYNORMAL); //x0C

	while (buf.estado == B_BUSY);//garanto que o clear ja terminou
	buf.ocupacao=6;
	buf.estado=B_BUSY;
	LCD_write(&buf, LCD_COMMAND);
	while (buf.estado == B_BUSY); //garanto que terminei a config ao final
}

/**
  * @brief  Desenha o caractere e bota em endereço
  * @param  Caractere a ser desenhado
  * @param  Endereço onde o desenho será hospedado
  */
void LCD_drawchar(char c, uint8_t *dat) //desenha o char e hospeda em dat
{
	uint8_t i; //indice do desenho

	c = c - ' ';

	for (i = 0; i < 6; i++) {
		*dat = font6_8[c][i];
		dat++;

	}
}

/**
  * @brief  Desenha o caractere INVERTIDO e bota em endereço
  * @param  Caractere a ser desenhado
  * @param  Endereço onde o desenho será hospedado
  */
void LCD_drawchar_inv(char c, uint8_t *dat) //desenha o char invertido
{
	uint8_t i; //indice do desenho

	c = c - ' ';

	for (i = 0; i < 6; i++) {
		*dat = ~font6_8[c][i];
		dat++;

	}
}

/**
  * @brief  Escreve caracter no display nokia LCD5110
  * @param  Caractere a ser escrito
  * @param  É invertido? (1- Sim, 0- Não)
  * @retval HAL status
  */
HAL_StatusTypeDef LCD5110_write_char(unsigned char c, uint8_t invert) {
	HAL_StatusTypeDef status;
	uint8_t *caract; //onde sera hospedado o desenho do caract.]=

	if (buf.estado == B_BUSY) //se buffer estiver ocupado, retorna estado
		return HAL_BUSY;

	buf.estado = B_BUSY; //se não der o return, vira ocupado
	caract = buf.dado;

	/*
	 * invert diz se vai inverter o caract, c é o caract
	 */
	if (invert)
		LCD_drawchar_inv(c, caract);
	else
		LCD_drawchar(c, caract);

	buf.ocupacao = LARG_CHAR; //atualiza o tamanho do buffer
	status = LCD_write(&buf, LCD_DATA);
	return status;
}

/**
  * @brief  Escreve string no display nokia LCD5110
  * @param  Endereço da string a ser desenhada
  * @retval HAL status
  */
HAL_StatusTypeDef LCD5110_write_string(char *s) {

	uint16_t tam = 0;
	uint8_t *c;
	/*Ponteiro para onde ficará hospedada a data (desenho do
	 display (buff).
	 */
	HAL_StatusTypeDef status;
	if (buf.estado == B_BUSY) //se buffer estiver ocupado, retorna estado
		return HAL_BUSY;
	buf.estado = B_BUSY; //se não der o return, virá ocupado

	c = buf.dado;
	while (*s != '\0') {
		LCD_drawchar(*s, c);
		/*Desloca buffer nos dois lugares
		 */
		s++;  //proxima letra da string
		c += LARG_CHAR; //proximo desenho de char no strf
		tam += LARG_CHAR; //cada letra soma 6 bytes
	}
	buf.ocupacao = tam;
	status = LCD_write(&buf, LCD_DATA);
	return status; //se buffer n estiver ocupado, retorna estado
}

/**
  * @brief  Limpa tela do display nokia 5110
  * @retval HAL status
  */
HAL_StatusTypeDef LCD5110_clear()
{
	HAL_StatusTypeDef status;

	if(buf.estado==B_BUSY)
		return HAL_BUSY;

	buf.estado=B_BUSY;
	buf.dado=telaLimpa;
	buf.ocupacao=TAM_BUFF;

	status = LCD_write(&buf,LCD_DATA);
			return status;
}

/**
  * @brief  Escreve um bloco (imagem) no display
  * @param  Endereço da imagem a ser desenhada
  * @param  Tamanho da imagem
  * @retval HAL status
  */
HAL_StatusTypeDef LCD5110_write_block(uint8_t *img, uint16_t tam)
{
	HAL_StatusTypeDef status;

	if(buf.estado==B_BUSY) //se buffer estiver ocupado, retorna estado
		return HAL_BUSY;

	buf.estado=B_BUSY; //se não der o return, virá ocupado
	buf.dado=img;
	buf.ocupacao=tam; //atualiza o tamanho do buffer

	status=LCD_write(&buf,LCD_DATA);
	return status; //se buffer n estiver ocupado, retorna estado
}

/**
  * @brief  "Seta" a posição de escrita no display nokia 5110
  * @param  Posição horizontal (x)
  * @param  Posição horizontal (y)
  * @retval HAL status
  */
HAL_StatusTypeDef LCD5110_set_XY(uint8_t x, uint8_t y) {
	HAL_StatusTypeDef status;

	if (buf.estado == B_BUSY)
		return HAL_BUSY;

	buf.estado = B_BUSY;
	x *= LARG_CHAR;
	x &= X_ADDR_MASK;
	y &= Y_ADDR_MASK;

	buf.dado[0] = PCD8544_SETYADDR | y;
	buf.dado[1] = PCD8544_SETXADDR | x;
	buf.ocupacao = 2;

	status = LCD_write(&buf, LCD_COMMAND);
	return status;
}

/**
  * @brief  Callback da biblioteca que deve ser chamado no callback do "main.c"
  * @param  Handler do spi para callback
  */
void LCD5110_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == lcd->hspi->Instance) //se for o SPI usado no lcd
			{
		//Fim da transf.
		HAL_GPIO_WritePin(lcd->CS_Port, lcd->CS_Pin, 1);
		buff_atual->estado = B_FREE;
		buf.dado = buffer;
	}

}
