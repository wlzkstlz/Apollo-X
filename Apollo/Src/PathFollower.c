#include "PathFollower.h"
#include "stdlib.h"
#include "math.h"

#define		PF_CONTROL_PARA_S			1.0
#define		PF_CONTROL_PARA_KX			(PF_CONTROL_PARA_S*4.2)//	35.0//25.0//35.0//18.0//10
#define		PF_CONTROL_PARA_KTHETA		(PF_CONTROL_PARA_S*3.0)//1.5

CRS runController(float vel, PathPoint path, float curX, float curY, float curPhi, float *Vc, float *Wc)
{
	float yErr = 0.0f, phiErr = 0.0f;
	float w = 0;
	//1 �����߶�orԲ��
	if (abs(path.deltaPhi) < EPSINON)//�߶�
	{
		float cvtangel = GetThetaFromVector(path.aPt[0] - path.startPt[0], path.aPt[1] - path.startPt[1]);
		//Point2f curpt=Point2f(curX,curY),start = path.startPt, end = path.aPt;
		float curpt_x=curX,curpt_y=curY,start_x=path.startPt[0],start_y=path.startPt[1],end_x=path.aPt[0],end_y=path.aPt[1];
		RotateTheta4Point(&start_x,&start_y, -cvtangel);
		RotateTheta4Point(&end_x,&end_y, -cvtangel);
		RotateTheta4Point(&curpt_x,&curpt_y, -cvtangel);

		if (curpt_x >= end_x)//2 �ж��Ƿ񵽴��յ�
			return CRS_REACHED;

		//3 �������ƫ��ͺ���ƫ��
		yErr = end_y - curpt_y;
		phiErr = cvtangel - curPhi;
	}
	else//Բ��
	{
		float angel1 = GetThetaFromVector(path.startPt[0] - path.aPt[0], path.startPt[1] - path.aPt[1]);
		float angel2 = GetThetaFromVector(curX - path.aPt[0], curY - path.aPt[1]);
		float delta_angel = angel2 - angel1;
		while (delta_angel < -ALG_PI) { delta_angel += ALG_2_PI; }
		while (delta_angel > ALG_PI) { delta_angel -= ALG_2_PI; }
		if ((path.deltaPhi > 0 && delta_angel >= path.deltaPhi) || (path.deltaPhi < 0 && delta_angel <= path.deltaPhi))
			return CRS_REACHED;

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

	*Vc = vel*cos(phiErr);
	*Wc = w + PF_CONTROL_PARA_KX*vel*yErr + PF_CONTROL_PARA_KTHETA*sin(phiErr);


	return CRS_CONTINUE;
}
