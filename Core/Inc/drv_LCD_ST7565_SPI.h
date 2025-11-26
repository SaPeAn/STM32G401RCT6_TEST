/******************************************************************************/
//      
//    LCD display
//    Controller - ST7565
//    
//    sofrware  hardware SPI
//    
/******************************************************************************/
#ifndef DRV_LCDST7565_H
#define	DRV_LCDST7565_H
#include "common.h"


void LCD_init(void);
void LCD_writebyte(uint8);
void LCD_senddata(const uint8*, uint8);
void LCD_setpagecolumn(uint8, uint8);
void LCD_printsmb8x5(const unsigned char, uint8, uint8);
void LCD_bufupload_buferase(void);
void LCD_erasestring(uint8, uint8, uint8);
uint8 LCD_printstr8x5(const uint8*, uint8, uint8);
void LCD_printclockanddate(uint8, uint8);

void LCD_printbatlevel(uint8, uint8, uint8);
void LCD_printbrightnes(uint8, uint8);
void LCD_printmenucoursor(uint8, uint8);
void LCD_printbutselhint(uint8, uint8, uint8);
void LCD_printvertline(uint8, uint8, uint8);
void LCD_printhorline(uint8, uint8, uint8);
void LCD_printweekday(uint8, uint8, uint8);
void LCD_printmonth(uint8, uint8, uint8);

void LCDbuf_upload(void);
void LCDbuf_erase(void);
uint8 LCDbuf_writestring(uint8*, uint8, uint8);

#endif	/* DRV_LCDST7565_H */

