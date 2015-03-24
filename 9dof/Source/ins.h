#ifndef _INS_H
#define _INS_H

#define int16_t int

void init_dcm(float* init_angle,float* d);
void DCM(int flag,float* dd,float* W,float* cbn,float* ff,float* hmc,float T,float* angle,float* q_error);
void compass(float* f,float* MA,float* m,float* MT);
void mpu_calibration();
void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void UART1_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);

#endif