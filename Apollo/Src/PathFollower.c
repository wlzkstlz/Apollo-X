#include "PathFollower.h"
#include "stdlib.h"
#include "math.h"

#define		PF_CONTROL_PARA_S			1.0
#define		PF_CONTROL_PARA_KX			(PF_CONTROL_PARA_S*4.2)//	35.0//25.0//35.0//18.0//10
#define		PF_CONTROL_PARA_KTHETA		(PF_CONTROL_PARA_S*3.0)//1.5
#define		PF_CONTROL_TURN_W			0.5

#define MAX_X_ERR	2.0
#define MAX_Y_ERR	0.5

CRS runController(float vel, PathPoint path, float curX, float curY, float curPhi, float *Vc, float *Wc)
{
	float yErr = 0.0f, phiErr = 0.0f;
	float w = 0;
	//1 区分线段or圆弧
	if (abs(path.deltaPhi) < EPSINON)//线段
	{
		float cvtangel = GetThetaFromVector(path.aPt[0] - path.startPt[0], path.aPt[1] - path.startPt[1]);
		//Point2f curpt=Point2f(curX,curY),start = path.startPt, end = path.aPt;
		float curpt_x=curX,curpt_y=curY,start_x=path.startPt[0],start_y=path.startPt[1],end_x=path.aPt[0],end_y=path.aPt[1];
		RotateTheta4Point(&start_x,&start_y, -cvtangel);
		RotateTheta4Point(&end_x,&end_y, -cvtangel);
		RotateTheta4Point(&curpt_x,&curpt_y, -cvtangel);

		if (curpt_x >= end_x)//2 判断是否到达终点
			return CRS_REACHED;
		
		if(curpt_x-start_x<-MAX_X_ERR)//2.1落后起点太多，同样视为轨迹偏差太大
			return CRS_XERR;

		//3 计算横向偏差和航向偏差
		yErr = end_y - curpt_y;
		phiErr = cvtangel - curPhi;
	}
	else//圆弧
	{
		float angel1 = GetThetaFromVector(path.startPt[0] - path.aPt[0], path.startPt[1] - path.aPt[1]);
		float angel2 = GetThetaFromVector(curX - path.aPt[0], curY - path.aPt[1]);
		float delta_angel = angel2 - angel1;
		while (delta_angel < -ALG_PI) { delta_angel += ALG_2_PI; }
		while (delta_angel > ALG_PI) { delta_angel -= ALG_2_PI; }
		if ((path.deltaPhi > 0 && delta_angel >= path.deltaPhi) || (path.deltaPhi < 0 && delta_angel <= path.deltaPhi))
			return CRS_REACHED;
		
		float xerr=sqrt(pow(path.startPt[0]-curX,2)+pow(path.startPt[1]-curY,2));
		if(xerr>MAX_X_ERR&&(delta_angel*path.deltaPhi<0))
			return CRS_XERR;

		float R = sqrt(pow(path.startPt[0]-path.aPt[0],2)+pow(path.startPt[1]-path.aPt[1],2));
		w = vel / R;
		float dist= sqrt(pow(curX - path.aPt[0], 2) + pow(curY - path.aPt[1], 2));
		float angel3 = 0.0f;
		if (path.deltaPhi > 0)
		{
			yErr = dist - R;
			angel3 = angel2 + 0.5*ALG_PI;
		}
		else
		{
			yErr = R - dist;
			angel3 = angel2 - 0.5*ALG_PI;
			w = -w;
		}
		phiErr = angel3 - curPhi;
	}

	if (abs(yErr) > MAX_Y_ERR)
		return CRS_YERR;
	
	
	
	
	
	
	while (phiErr > ALG_PI ) { phiErr -= ALG_PI * 2; }
	while (phiErr < -ALG_PI ) { phiErr += ALG_PI * 2; }

	static uint8_t isTurning = 0;
	if ((!isTurning)&&abs(phiErr) > DEG2RAD(60))
	{
		(*Vc) = 0;
		(*Wc) = phiErr > 0 ? PF_CONTROL_TURN_W : (-PF_CONTROL_TURN_W);
		isTurning = 1;
	}
	else if (isTurning&&abs(phiErr) > DEG2RAD(10))
	{
		(*Vc) = 0;
		(*Wc) = phiErr > 0 ? PF_CONTROL_TURN_W : (-PF_CONTROL_TURN_W);
	}
	else
	{
		isTurning = 0;
		(*Vc) = vel*cos(phiErr);
		(*Wc) = w + PF_CONTROL_PARA_KX*vel*yErr + PF_CONTROL_PARA_KTHETA*sin(phiErr);
	}

	return CRS_CONTINUE;
}
