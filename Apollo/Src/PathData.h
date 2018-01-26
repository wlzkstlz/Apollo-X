#ifndef _PATHDATA_H
#define _PATHDATA_H

#include "sdram.h"

#define		METER_PER_LATITUDE	(111130.0*180.0/3.1415926)   //1 rad 纬度对应的经线上地面距离m

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
extern int gCurPathId;

void initPathPointsData(void);
void addPathPoint(PathPoint pt);
uint8_t getCurPathPoint(PathPoint *ppt);
uint8_t isPathDataFileExist(void);
void setPathDataFileExist(void);
void moveCurPpt2Next(void);

void cvtGpsPt2Xy(float gpsdataLon,float gpsdataLat,float* xyzdataX,float* xyzdataY);//东北上坐标系

#endif
