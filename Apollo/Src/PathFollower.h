#ifndef _PATHFOLLOWER_H
#define _PATHFOLLOWER_H

#include "CommonAlg.h"
#include "PathData.h"

#define EPSINON		0.00001



#define		PF_CAR_WIDTH	0.652

/*
����ָ�����ٶ�velocity��·����path����ǰλ�ˣ�����������ٶȺͿ��ƽ��ٶȣ�
*/

typedef enum
{
	CRS_CONTINUE,
	CRS_REACHED,
	CRS_XERR,
	CRS_YERR,
	CRS_PHIERR
}CRS;

CRS runController(float vel,PathPoint path,float curX,float curY,float curPhi,float *Vc,float *Wc);

#endif


