#include "CommonAlg.h"
#include "math.h"

double GetThetaFromVector(float x,float y)
{
	double theta;
	if (x==0)
	{
		theta=y>0?0.5*ALG_PI:1.5*ALG_PI;
	}
	else if (y==0)
	{
		theta=x>0?0:ALG_PI;
	}
	else if (x>0)
	{
		theta=atan(y/x);
		theta=theta>=0?theta:(theta+2.0*ALG_PI);
	}
	else
	{
		theta=atan(y/x);
		theta=theta+ALG_PI;
	}
	return theta;
}
void RotateTheta4Point(float *x,float *y,double theta)
{
	float x_temp=(*x);
	float y_temp=(*y);
	(*x)=x_temp*cos(theta)-y_temp*sin(theta);
	(*y)=x_temp*sin(theta)+y_temp*cos(theta);
}
