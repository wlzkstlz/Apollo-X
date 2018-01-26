#include "PathData.h"
#include "math.h"
#include "CommonAlg.h"

float gBaseLongitude=(113.895098/180.0*ALG_PI);
float gBaseLatitude=(22.959097/180.0*ALG_PI);
float gBaseAltitude=0;

PathPoint gPathPoints[PATH_PT_SIZE];

int gValidPathPtNum=0;
uint16_t gCurPathId=0;

uint8_t gPathDataFileExist=0;

void initPathPointsData(void)
{
//	for(int i=0;i<PATH_PT_SIZE;i++)
//	{
//		gPathPoints[i].startPt[0]=i;
//	}
	gValidPathPtNum=0;
	gCurPathId=0;
	gPathDataFileExist=0;
}


void addPathPoint(PathPoint pt)
{
	if(gValidPathPtNum<PATH_PT_SIZE)
	{
		gPathPoints[gValidPathPtNum]=pt;
		gValidPathPtNum++;
	}
}

uint32_t getPathPointNum(void)
{
	return gValidPathPtNum;
}

uint8_t getCurPathPoint(PathPoint *ppt)
{
	if(gCurPathId>=gValidPathPtNum)
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

uint16_t getCurPathPointId(void)
{
	return gCurPathId;
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


void cvtGpsPt2Xy(float gpsdataLon,float gpsdataLat,float* xyzdataX,float* xyzdataY)
{
	(*xyzdataY)=gpsdataLat-gBaseLatitude;
	(*xyzdataY)=METER_PER_LATITUDE*(*xyzdataY);
	
	(*xyzdataX)=gpsdataLon-gBaseLongitude;
	(*xyzdataX)=cos(gBaseLatitude)*METER_PER_LATITUDE*(*xyzdataX);
}


