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

/*��ƽ������ϵ����������ת����������X��ļнǣ������ƣ�[0,2*PI)*/
double GetThetaFromVector(float x,float y);

/*��ƽ��ֱ������ϵ�µĵ�pt��ʱ����תtheta�ǣ������ƣ����ַ���*/
void RotateTheta4Point(float *x,float *y,double theta);

#endif

