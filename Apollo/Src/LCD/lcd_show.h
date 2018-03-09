#ifndef _LCD_SHOW_H
#define _LCD_SHOW_H
#include "lcd.h"
#include "AutoPilot.h"

void lcdshow(char*s);

void lcdshowpilotstate(PilotState state);

void lcdshowcmd(CmdType cmd);
void lcdshowenginemode(TypeEngineMode mode);
void lcdshowdrivermode(TypeDriverMode mode);
void lcdshowtanklevel(uint8_t level);
void lcdshowbatteryvolt(uint16_t voltage);
void lcdshowinmdata(INM_Data data);
void lcdshowimudata(IMU_Data data);
void lcdshowBLEdata(uint8_t isdoing,uint32_t ptnum,float x,float y);

#endif	

