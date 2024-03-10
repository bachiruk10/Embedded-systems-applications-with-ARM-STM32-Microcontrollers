/*
 * MC21605H6W-SPTLY-V2.h
 *
 *  Created on: Feb 16, 2024
 *      Author: MFIBADEK
 */

#ifndef INC_MC21605H6W_SPTLY_V2_H_
#define INC_MC21605H6W_SPTLY_V2_H_


#define ENABLE 0x08
#define DATA_MODE 0x04
#define COMMAND_MODE 0x00

#define LINE_LENGTH 0x40
#define TOT_LENGTH 0x80


extern void MX_SPI1_Init(void);
extern void write_4bit(int nibble, int mode);
extern void write_cmd(int data) ;
extern void init_lcd(void);
extern void write_data(char c);
extern void set_cursor(int column, int row);
extern void print_lcd(const char *string) ;




#endif /* INC_MC21605H6W_SPTLY_V2_H_ */
