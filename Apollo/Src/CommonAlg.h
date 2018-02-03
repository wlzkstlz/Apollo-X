#ifndef _COMMONALG_H
#define _COMMONALG_H

#define ALG_PI	3.1415926
#define ALG_2_PI	(2.0*ALG_PI)

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29577951)
#endif

/*将平面坐标系下向量坐标转换成向量与X轴的夹角，弧度制，[0,2*PI)*/
double GetThetaFromVector(float x,float y);

/*将平面直角坐标系下的点pt逆时针旋转theta角，弧度制，右手法则*/
void RotateTheta4Point(float *x,float *y,double theta);

#endif

