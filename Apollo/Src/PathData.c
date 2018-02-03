#include "PathData.h"
#include "math.h"
#include "CommonAlg.h"
#include "string.h"

double gBaseLongitude=(113.895098/180.0*ALG_PI);
double gBaseLatitude=(22.959097/180.0*ALG_PI);

PathPoint gPathPoints[PATH_PT_SIZE];

int gValidPathPtNum=0;
uint16_t gCurPathId=1;

uint8_t gPathDataFileExist=0;

void initPathPointsData(void)
{
//	for(int i=0;i<PATH_PT_SIZE;i++)
//	{
//		gPathPoints[i].startPt[0]=i;
//	}
	gValidPathPtNum=0;
	gCurPathId=1;
	gPathDataFileExist=0;
}


void addPathPoint(PathPoint pt)
{
	if(gValidPathPtNum<PATH_PT_SIZE)
	{
		gPathPoints[gValidPathPtNum++]=pt;
	}
}

uint32_t getPathPointNum(void)
{
	return gValidPathPtNum;
}

uint8_t getCurPathPoint(PathPoint *ppt)
{
	if(gCurPathId>=gValidPathPtNum||gCurPathId==0)
		return 0;
	
	(*ppt)=gPathPoints[gCurPathId];
	return 1;
}

void setCurPathPointId(uint32_t id)
{
	gCurPathId=id;
}

PathPoint getPathPoint(uint32_t ppt_id)
{
	return gPathPoints[ppt_id%PATH_PT_SIZE];
}

uint8_t isPathDataFileExist(void)
{
	return gPathDataFileExist;
}

void setPathDataFileExist(void)
{
	gPathDataFileExist=1;
}

void moveCurPpt2Next(void)
{
	gCurPathId++;
}


void updateBaseLocationByFile(void)
{
	uint8_t *ptr=(uint8_t *)&gPathPoints[0];
	memcpy(&gBaseLongitude,ptr,8);
	ptr+=8;
	memcpy(&gBaseLatitude,ptr,8);
}


void cvtGpsPt2Xy(double gpsdataLon,double gpsdataLat,float* xyzdataX,float* xyzdataY)
{
	(*xyzdataY)=gpsdataLat-gBaseLatitude;
	(*xyzdataY)=METER_PER_LATITUDE*(*xyzdataY);
	
	(*xyzdataX)=gpsdataLon-gBaseLongitude;
	(*xyzdataX)=cos(gBaseLatitude)*METER_PER_LATITUDE*(*xyzdataX);
}

void cvtXyPt2Gps(float xyzdataX,float xyzdataY,double* gpsdataLon,double* gpsdataLat)
{
	(*gpsdataLat)=(double)xyzdataY/METER_PER_LATITUDE+gBaseLatitude;
	(*gpsdataLon)=(double)xyzdataX/METER_PER_LATITUDE/cos(gBaseLatitude)+gBaseLongitude;
}


