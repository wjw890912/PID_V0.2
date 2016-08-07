


#ifndef __DS12864_H
#define __DS12864_H

void LCD_init(void);
void LCD_draw_clr(void);//清屏
void LCD_Setaddress(unsigned char x,unsigned char y);//设置显示地址
void lcd_main(void);
void Display_Task(uint32_t Time);//刷新
void Set_Vrms2Display(int Vrms);
void Set_Irms2Display(int Irms);
void Set_Watt2Display(int Watt);
void Set_Watthr2Display(int Watthr);
void LCD_write_com_clr(unsigned char com);
#endif


