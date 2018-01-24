#ifndef _PATHFOLLOWER_H
#define _PATHFOLLOWER_H

#include "CommonAlg.h"
#include "PathData.h"

#define EPSINON		0.00001

#define MAX_Y_ERR	0.5

#define		PF_CAR_WIDTH	0.652

/*
输入指令线速度velocity，路径点path，当前位姿，输出控制线速度和控制角速度；
*/

typedef enum
{
	CRS_CONTINUE,
	CRS_REACHED,
	CRS_YERR
}CRS;

CRS runController(float vel,PathPoint path,float curX,float curY,float curPhi,float *Vc,float *Wc);

#endif


