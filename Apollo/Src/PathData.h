#ifndef _PATHDATA_H
#define _PATHDATA_H

#include "sdram.h"

#define		METER_PER_LATITUDE	(111130.0*180.0/3.1415926)   //1 rad γ�ȶ�Ӧ�ľ����ϵ������m

extern float gBaseLongitude;
extern float gBaseLatitude;
extern float gBaseAltitude;

typedef struct 
{
	float startPt[2];//pre path point
	float aPt[2];//arc center point or next line point ,depends on whether deltaPhi equal to zero or not
	float deltaPhi;
}PathPoint;

#define PATH_PT_SIZE	1000000
extern PathPoint gPathPoints[PATH_PT_SIZE] __attribute__((at(Bank5_SDRAM_ADDR)));

extern int gValidPathPtNum;//
extern uint16_t gCurPathId;

void initPathPointsData(void);
void addPathPoint(PathPoint pt);
uint32_t getPathPointNum(void);
uint8_t getCurPathPoint(PathPoint *ppt);
void setCurPathPointId(uint32_t id);
PathPoint getPathPoint(uint32_t ppt_id);
uint16_t getCurPathPointId(void);
uint8_t isPathDataFileExist(void);
void setPathDataFileExist(void);
void moveCurPpt2Next(void);

void cvtGpsPt2Xy(float gpsdataLon,float gpsdataLat,float* xyzdataX,float* xyzdataY);//����������ϵ

#endif
