#include "lcd_show.h"
#include "CommonAlg.h"

#define	LCD_DEBUG_MSG_X	10
#define 	LCD_DEBUG_MSG_Y	420
#define	LCD_DEBUG_MSG_WIDTH	240
#define	LCD_DEBUG_MSG_HEIGHT	48

void lcdshow(char*s)
{
	LCD_Fill(LCD_DEBUG_MSG_X,LCD_DEBUG_MSG_Y,LCD_DEBUG_MSG_X+LCD_DEBUG_MSG_WIDTH,LCD_DEBUG_MSG_Y+LCD_DEBUG_MSG_HEIGHT,WHITE);
	LCD_ShowString(LCD_DEBUG_MSG_X,LCD_DEBUG_MSG_Y,LCD_DEBUG_MSG_WIDTH,LCD_DEBUG_MSG_HEIGHT,16,(u8*)s); 
}

#define	LCD_PILOTSTATE_X	10
#define	LCD_PILOTSTATE_Y	10
#define	LCD_PILOTSTATE_WIDTH	200
#define	LCD_PILOTSTATE_HEIGHT	16
void lcdshowpilotstate(PilotState state)
{
	char ss[50];
	switch(state)
	{
		case PILOT_STATE_AUTO:
			sprintf(ss,"PILOT_STATE_AUTO\n");
			break;
		case PILOT_STATE_BLE_TRANSFER:
			sprintf(ss,"PILOT_STATE_BLE_TRANSFER\n");
			break;
		case PILOT_STATE_EMERGENCY:
			sprintf(ss,"PILOT_STATE_EMERGENCY\n");
			break;
		case PILOT_STATE_IDLE:
			sprintf(ss,"PILOT_STATE_IDLE\n");
			break;
		case PILOT_STATE_INIT:
			sprintf(ss,"PILOT_STATE_INIT\n");
			break;
		case PILOT_STATE_MANUAL_WORK:
			sprintf(ss,"PILOT_STATE_MANUAL_WORK\n");
			break;
		case PILOT_STATE_SUPPLY:
			sprintf(ss,"PILOT_STATE_SUPPLY\n");
			break;
		case PILOT_STATE_TRANSITION:
			sprintf(ss,"PILOT_STATE_TRANSITION\n");
			break;
		default:
			sprintf(ss,"PILOT_STATE: Unknown!\n");
			break;
	}

	LCD_Fill(LCD_PILOTSTATE_X,LCD_PILOTSTATE_Y,LCD_PILOTSTATE_X+LCD_PILOTSTATE_WIDTH,LCD_PILOTSTATE_Y+LCD_PILOTSTATE_HEIGHT,WHITE);
	LCD_ShowString(LCD_PILOTSTATE_X,LCD_PILOTSTATE_Y,LCD_PILOTSTATE_WIDTH,LCD_PILOTSTATE_HEIGHT,16,(u8*)ss);
}


#define	LCD_CMD_X	10
#define	LCD_CMD_Y	30
#define	LCD_CMD_WIDTH	200
#define	LCD_CMD_HEIGHT	16
void lcdshowcmd(CmdType cmd)
{
	char ss[50];
	switch(cmd)
	{
		case CMD_AUTO:
			sprintf(ss,"CMD_AUTO\n");
			break;
		case CMD_MANUAL:
			sprintf(ss,"CMD_MANUAL\n");
			break;
		case CMD_HEARTBEAT:
			sprintf(ss,"CMD_HEARTBEAT\n");
			break;
		case CMD_BLE_END:
			sprintf(ss,"CMD_BLE_END\n");
			break;
		case CMD_STOP:
			sprintf(ss,"CMD_STOP\n");
			break;
		case CMD_SUPPLY:
			sprintf(ss,"CMD_SUPPLY\n");
			break;
		case CMD_TRANSITION:
			sprintf(ss,"CMD_TRANSITION\n");
			break;
		case CMD_BLE_ABORT:
			sprintf(ss,"CMD_BLE_ABORT\n");
			break;
		case CMD_BLE_START:
			sprintf(ss,"CMD_BLE_START\n");
			break;
		case CMD_NONE:
			sprintf(ss,"CMD_NONE\n");
			break;
		default:
			sprintf(ss,"CMD:Unkown!\n");
			break;
	}

	LCD_Fill(LCD_CMD_X,LCD_CMD_Y,LCD_CMD_X+LCD_CMD_WIDTH,LCD_CMD_Y+LCD_CMD_HEIGHT,WHITE);
	LCD_ShowString(LCD_CMD_X,LCD_CMD_Y,LCD_CMD_WIDTH,LCD_CMD_HEIGHT,16,(u8*)ss);
}

#define	LCD_ENGINE_X	10
#define	LCD_ENGINE_Y	50
#define	LCD_ENGINE_WIDTH	200
#define	LCD_ENGINE_HEIGHT	16
void lcdshowenginemode(TypeEngineMode mode)
{
	char ss[50];
	switch(mode)
	{
		case ENGINE_MODE_START:
			sprintf(ss,"Engine Sate:Start!\n");
			break;
		case ENGINE_MODE_STOP:
			sprintf(ss,"Engine Sate:Stop!\n");
			break;
		default:
			sprintf(ss,"Engine Sate:Unknown!\n");
			break;
	}

	LCD_Fill(LCD_ENGINE_X,LCD_ENGINE_Y,LCD_ENGINE_X+LCD_ENGINE_WIDTH,LCD_ENGINE_Y+LCD_ENGINE_HEIGHT,WHITE);
	LCD_ShowString(LCD_ENGINE_X,LCD_ENGINE_Y,LCD_ENGINE_WIDTH,LCD_ENGINE_HEIGHT,16,(u8*)ss);
}

#define	LCD_DRIVER_X	10
#define	LCD_DRIVER_Y	70
#define	LCD_DRIVER_WIDTH	200
#define	LCD_DRIVER_HEIGHT	16
void lcdshowdrivermode(TypeDriverMode mode)
{
	char ss[50];
	switch(mode)
	{
		case DRIVER_MODE_AUTO:
			sprintf(ss,"Driver Sate:Auto!\n");
			break;
		case DRIVER_MODE_MANUAL:
			sprintf(ss,"Driver Sate:Manual!\n");
			break;
		case DRIVER_MODE_EMERGENCY:
			sprintf(ss,"Driver Sate:Emergency!\n");
			break;
		default:
			sprintf(ss,"Driver Sate:Unknown!\n");
			break;
	}

	LCD_Fill(LCD_DRIVER_X,LCD_DRIVER_Y,LCD_DRIVER_X+LCD_DRIVER_WIDTH,LCD_DRIVER_Y+LCD_DRIVER_HEIGHT,WHITE);
	LCD_ShowString(LCD_DRIVER_X,LCD_DRIVER_Y,LCD_DRIVER_WIDTH,LCD_DRIVER_HEIGHT,16,(u8*)ss);
}

#define	LCD_TANK_X	10
#define	LCD_TANK_Y	90
#define	LCD_TANK_WIDTH	200
#define	LCD_TANK_HEIGHT	16
void lcdshowtanklevel(uint8_t level)
{
	char ss[50];
	sprintf(ss,"Tank Level:%.1f cm\n",((float)level/250.0f*500.0f));
	
	LCD_Fill(LCD_TANK_X,LCD_TANK_Y,LCD_TANK_X+LCD_TANK_WIDTH,LCD_TANK_Y+LCD_TANK_HEIGHT,WHITE);
	LCD_ShowString(LCD_TANK_X,LCD_TANK_Y,LCD_TANK_WIDTH,LCD_TANK_HEIGHT,16,(u8*)ss);
}

#define	LCD_VOLT_X	10
#define	LCD_VOLT_Y	110
#define	LCD_VOLT_WIDTH	200
#define	LCD_VOLT_HEIGHT	16
void lcdshowbatteryvolt(uint16_t voltage)
{
	char ss[50];
	sprintf(ss,"Baterry Volt:%.1f V\n",(float)voltage/100.0f);
	
	LCD_Fill(LCD_VOLT_X,LCD_VOLT_Y,LCD_VOLT_X+LCD_VOLT_WIDTH,LCD_VOLT_Y+LCD_VOLT_HEIGHT,WHITE);
	LCD_ShowString(LCD_VOLT_X,LCD_VOLT_Y,LCD_VOLT_WIDTH,LCD_VOLT_HEIGHT,16,(u8*)ss);
}

#define	LCD_INMDATA_X	10
#define	LCD_INMDATA_Y	130
#define	LCD_INMDATA_WIDTH	240
#define	LCD_INMDATA_HEIGHT	64
void lcdshowinmdata(INM_Data data)
{
	char ss[100];
	
	float posex,posey,poseyaw;
	cvtINMData2Pose(data,&posex,&posey,&poseyaw);
	sprintf(ss,"INM data:poseX=%.2fm,PoseY=%.2fm,PoseYaw=%.1fdeg,rol=%.1f,pitch=%.1f, gps time:%dms\n",
	posex,posey,(float)RAD2DEG(poseyaw),(float)RAD2DEG(data.roll),(float)RAD2DEG(data.pitch),data.gps_ms);
		
	LCD_Fill(LCD_INMDATA_X,LCD_INMDATA_Y,LCD_INMDATA_X+LCD_INMDATA_WIDTH,LCD_INMDATA_Y+LCD_INMDATA_HEIGHT,WHITE);
	LCD_ShowString(LCD_INMDATA_X,LCD_INMDATA_Y,LCD_INMDATA_WIDTH,LCD_INMDATA_HEIGHT,16,(u8*)ss);
}

