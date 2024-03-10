/*
 * MC21605H6W-SPTLY-V2.c
 *
 *  Created on: Feb 16, 2024
 *      Author: Bachir Dekdouk
 *  Driver for LCD - MIDAS - MC21605H6W-SPTLY-V2
 *
 */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

#include "main.h"
#include "MC21605H6W-SPTLY-V2.h"


SPI_HandleTypeDef hspi1;


void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

void init_lcd(void) {

	// Initial delay
	HAL_Delay(40);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	uint8_t data =  0x30;
	HAL_SPI_Transmit(&hspi1, &data, 1, 10000);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    write_cmd(0x20);
    HAL_Delay(37e-3);

	write_cmd(0x20);
	HAL_Delay(37e-3);

	write_cmd(0x0C);
	HAL_Delay(37e-3);

	write_cmd(0x01);
	HAL_Delay(1520e-3);

	write_cmd(0x06);
	HAL_Delay(37e-3);

	write_cmd(0x28);
	HAL_Delay(37e-3);

}

//Write 4bits to the LCD
void write_4bit(int nibble, int mode) {

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	uint8_t data=  (nibble | ENABLE | mode);
    HAL_SPI_Transmit(&hspi1, &data, 1, 10000);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    HAL_Delay(1e-3);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    data = (nibble & ~ENABLE);
	HAL_SPI_Transmit(&hspi1, &data, 1, 10000);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);


}

//Write a command to the LCD
void write_cmd(int data) {

    int hi_n, lo_n;

	hi_n = (data & 0xF0);
    lo_n = ((data << 4) & 0xF0);

    write_4bit(hi_n, COMMAND_MODE);
    write_4bit(lo_n, COMMAND_MODE);
}

//Write data to the LCD
void write_data(char c) {
    int hi_n;
    int lo_n;

    hi_n = (c & 0xF0);
    lo_n = ((c << 4) & 0xF0);

    write_4bit(hi_n, DATA_MODE);
    write_4bit(lo_n, DATA_MODE);
}

//Set cursor position
void set_cursor(int column, int row) {
    int addr;

    addr = (row * LINE_LENGTH) + column;
    addr |= TOT_LENGTH;
    write_cmd(addr);
}

//Print strings to the LCD
void print_lcd(const char *string) {
    while(*string){
        write_data(*string++);
    }
}
