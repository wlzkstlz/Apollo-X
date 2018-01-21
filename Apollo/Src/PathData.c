#include "PathData.h"
#include "math.h"

float gBaseLongitude=0;
float gBaseLatitude=0;
float gBaseAltitude=0;

PathPoint gPathPoints[PATH_PT_SIZE];

int gValidPathPtNum;
int gCurPathId;

void initPathPointsData(void)
{
	SDRAM_Init();                   //≥ı ºªØSDRAM
	for(int i=0;i<PATH_PT_SIZE;i++)
	{
		gPathPoints[i].startPt[0]=i;
	}
	
	gValidPathPtNum=0;
	gCurPathId=0;
}


void addPathPoint(PathPoint pt)
{
	if(gValidPathPtNum<PATH_PT_SIZE)
	{
		gPathPoints[gValidPathPtNum]=pt;
		gValidPathPtNum++;
	}
}


void cvtGpsPt2Xyz(float* gpsdataLon,float* gpsdataLat,float* gpsdataAlt,float* xyzdataX,float* xyzdataY,float* xyzdataZ)
{
	(*xyzdataZ)=(*gpsdataAlt)-gBaseAltitude;
	
	(*xyzdataY)=(*gpsdataLat)-gBaseLatitude;
	(*xyzdataY)=METER_PER_LATITUDE*(*xyzdataY);
	
	(*xyzdataX)=(*gpsdataLon)-gBaseLongitude;
	(*xyzdataX)=cos(gBaseLatitude)*METER_PER_LATITUDE*(*xyzdataX);
}
