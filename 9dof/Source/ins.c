/*包含姿态初始化、四元数初始化、姿态更新、陀螺启动零位标定以及通信协议，版权属于开发者asha*/
#include "MPU6050.h" 
#include "ins.h"
#include "math.h"
#include <acc_comp.h> 
#define int16_t int
unsigned int  insdv_flag=1;
unsigned int  inss_flag=1;
#define q0nosize 0.01
#define q1nosize 0.01
#define q2nosize 0.01
#define q3nosize 0.01
#define axnosize 0.03
#define aynosize 0.03
#define aznosize 0.03
#define gxnosize 0.09
#define gynosize 0.09
#define gznosize 0.09
#define axerror 0.01
#define ayerror 0.01
#define azerror 0.01
#define gxerror 0.09
#define gyerror 0.09
#define gzerror 0.09
#define mxnosize 0.02
#define mynosize 0.02
#define mznosize 0.02
float q_error[7]={0,0,0,0,0,0,0};//四元数、陀螺零位EKF滤波使用参数
float MN=0;        
float ME=0;          
float MU=0;   
float flag_m=0;
float myaw_offset=0.0;
float mpitch_offset=0;
float mroll_offset=0;
float g=9.82;
float pi=3.1416;
float DX,DY,DZ;
float* ACCk1[3]={0}; 
float* cbnkk[9]={0};
float* cbnkkf[9]={0};

float* acc_off[3]={0};
float* ACCk2[3]={0};
float wf[3]={0};
float wf1[3]={0};
float wf2[3]={0};
float cbn[9]={0};
float q_b[4]={0};
float q_errortemp0=0;
float q_errortemp1=0;
float q_errortemp2=0;
float* W[3]={0};
float* init_angle[3];
float* angle[3];
float model_q;
float TT[3][3];
float init_T11,init_T12,init_T13,init_T21,init_T22,init_T23,init_T31,init_T32,init_T33;
float d[4];
float dd[4];
float hx,hy,hz;
float* m[3];
float* MT[3];
float* f[3];
float* MA[3];
/*磁椭球到圆球矩阵*/
float compas_c[9]={0.93508599962364,-0.34178767895328,-0.09378355838627,0.34355263802937,0.93912905768505,0.00286320007361,0.08709625830611,-0.03489692719034,0.99558849243136};//磁场本身偏角
float compas_b[3]={0};
float magk[3]={1,1,1};

float insdv[3]={0};
float aggg[3]={0};

float mpusumx=0;
float mpusumy=0;
float mpusumz=0;
float accsumx=0;
float accsumy=0;
float accsumz=0;
float insv[3]={0};
float inss[3]={0};
float insg[3]={0}; 
float insdv_init[3]={0};
float inss_init[3]={0};
float wx_a=0;
float wy_a=0;
float wz_a=0;

extern int flag;
extern float seq[4];
extern float mpuoffsetx;
extern float mpuoffsety;
extern float mpuoffsetz;
extern float mpugsetx;
extern float mpugsety;
extern float mpugsetz;
extern float mpugkx;
extern float mpugky;
extern float mpugkz;
extern  float mpukx;
extern  float mpuky;
extern  float mpukz;

float PP[16];
int   gyrodatasss[3]={0};
int   accggg[3]={0};
int gryoo[3]={0};
int flagzuhe=1;
/*下面是误差四元数使用的变量*/
float kf_t=0.02;
float xk[4]={0};
float FQ[16]={0};
float RA[36]={0};
float RM[3]={0};
float ZAK[6]={0};
float ZMK[3]={0};
float xkerror[4]={0};
float PM[36]={0};
float PPA[16]={0};
float GE=0;
float GN=0;
float GU=1;
int count_newmoxing=0;
int flag_count_newmoxing=0;
int yuce_only=0;
/*电子罗盘程序*/
/*电子罗盘程序*/
void compass(float* f,float* MA,float* m,float* MT)

{

float mpitch,hx,hy=0;
float phi=0;

MT[0]=f[0];
MT[1]=f[1];
MT[2]=f[2];

mpitch=-asin(MT[0]);
m[1]=mpitch;
m[2]=atan2(MT[1],MT[2]);

/*根据惯性技术书上，应该是这个样子,网上得坐标轴定义和一般得惯性定义不一致,本程序中选取的是Z Y X旋转顺序，下面的补偿和这个顺序很相关*/
/*Mn=cnb*Mb */
hx=MA[0]*cos(m[1])+MA[1]*sin(m[2])*sin(m[1])+MA[2]*cos(m[2])*sin(m[1]);
hy=MA[1]*cos(m[2])-MA[2]*sin(m[2]);
hz=-MA[0]*sin(m[1])+MA[1]*sin(m[2])*cos(m[1])+MA[2]*cos(m[2])*cos(m[1]);


/*判断航向角度的象限*/
if(hx==0&&hy<0)
{
m[0]=pi;
}
if(hx==0&&hy>0)
{
m[0]=0;
}
if(hy==0&&hx<0)
{
m[0]=1.5*pi;
}
if(hy==0&&hx>0)
{
m[0]=0.5*pi;
}
if(hx<0&&hy>0)
{
m[0]=1.5*pi-atan(hy/hx);
}
if(hx<0&&hy<0)
{
m[0]=1.5*pi-atan(hy/hx);
}
if(hx>0&&hy<0)
{
m[0]=0.5*pi-atan(hy/hx);
}
if(hx>0&&hy>0)
{
m[0]=0.5*pi-atan(hy/hx);
}
/*Me=Cnb1*Hb1*/
/*将0-360的范围变为+-180范围，使得满足三角函数的取值*/
if(m[0]>pi)

{
phi=m[0]-2*pi;
}
else
{
phi=m[0];
}
/*这个地方请参考《9DOF算法解释》，很重要的一个地方*/
ME=cos(phi)*hx-sin(phi)*hy;
MN=sin(phi)*hx+cos(phi)*hy;
MU=hz;


//printf("%.3f %.3f %.3f %.3f\r\n",ME,MN,MU,phi*57.3);
}

/*欧拉角度转四元数代码，旋转次序是Z Y X   q=(cos(0.5roll)-isin(0.5roll)(cos(0.5pitch)-jsin(0.5pitch))(cos(0.5yaw)-jsin(0.5yaw)))*/
void init_dcm(float* init_angle,float* d)
{
float init_yaw,init_pitch,init_roll=0;
float cosroll,sinroll,cospitch,sinpitch,cosyaw,sinyaw;
init_yaw=init_angle[0];
init_pitch=init_angle[1];
init_roll=init_angle[2];
if(init_yaw>pi)

{
init_yaw=init_yaw-2*pi;
}
cosroll=cos(0.5*init_roll);
sinroll=sin(0.5*init_roll);

cospitch=cos(0.5*init_pitch);
sinpitch=sin(0.5*init_pitch);

cosyaw=cos(0.5*init_yaw);
sinyaw=sin(0.5*init_yaw);
/*Z Y X旋转*/
d[0]=cosroll*cospitch*cosyaw+sinroll*sinpitch*sinyaw;
d[1]=sinroll*cospitch*cosyaw-cosroll*sinpitch*sinyaw;
d[2]=cosroll*sinpitch*cosyaw+sinroll*cospitch*sinyaw;
d[3]=cosroll*cospitch*sinyaw-sinroll*sinpitch*cosyaw;
}
/*姿态跟新算法，采用比卡的3阶近似*/
void DCM(int flag,float* dd,float* W,float* cbn,float* ff,float* hmc,float T,float* angle,float* q_error)

{


float q0,q1,q2,q3;
float temp;
int i=0;
float dtemp[16]={0};
float pitch=0;
float FA[16]={0};
float HA[24]={0};
float q0temp=0;
float q1temp=0;
float q2temp=0;
float q3temp=0;
float g_temp=0;
/*下面这个flag_count_nwemoxing 和本程序相关性不大，是我的AHRS产品版本中的标志位，请大家忽略*/
if(flag_count_newmoxing==0)
{
count_newmoxing++;
}
else
{
count_newmoxing=count_newmoxing;
}

if(count_newmoxing>0)
{
flag_count_newmoxing=1;
}
else
{
flag_count_newmoxing=0;
}

/*准备添加二子样圆锥优化 注：经过发现效果不大，绝对不加入2子样*/
DX=(W[0])*T*0.01745;
DY=(W[1])*T*0.01745;
DZ=(W[2])*T*0.01745;
temp=(DX*DX+DY*DY+DZ*DZ);
g_temp=sqrt(ff[0]*ff[0]+ff[1]*ff[1]+ff[2]*ff[2]);
if(flag==1)/*flag=1的时候说明是初始化刚完成，这个时候不进行四元数更新，下个计算周期开始进入四元数更新*/
{
d[0]=dd[0];
d[1]=dd[1];
d[2]=dd[2];
d[3]=dd[3];
}
else
{
/*下面是毕卡算法的3阶展开，如果追求更高的精度可以使用毕卡的完全形式，这里dtemp也是状态矩阵，详细参考《9DOF算法解释》*/
dtemp[0]=1*(1-0.125*temp);
dtemp[1]=-(0.5-temp*0.0208)*DX;
dtemp[2]=-(0.5-temp*0.0208)*DY;
dtemp[3]=-(0.5-temp*0.0208)*DZ;

dtemp[4]=(0.5-temp*0.0208)*DX;
dtemp[5]=1*(1-0.125*temp);
dtemp[6]=(0.5-temp*0.0208)*DZ;
dtemp[7]=-(0.5-temp*0.0208)*DY;

dtemp[8]=(0.5-temp*0.0208)*DY;
dtemp[9]=-(0.5-temp*0.0208)*DZ;
dtemp[10]=1*(1-0.125*temp);
dtemp[11]=(0.5-temp*0.0208)*DX;

dtemp[12]=(0.5-temp*0.0208)*DZ;
dtemp[13]=(0.5-temp*0.0208)*DY;
dtemp[14]=-(0.5-temp*0.0208)*DX;
dtemp[15]=1*(1-0.125*temp);

q0=dtemp[0]*d[0]+dtemp[1]*d[1]+dtemp[2]*d[2]+dtemp[3]*d[3];
q1=dtemp[4]*d[0]+dtemp[5]*d[1]+dtemp[6]*d[2]+dtemp[7]*d[3];
q2=dtemp[8]*d[0]+dtemp[9]*d[1]+dtemp[10]*d[2]+dtemp[11]*d[3];
q3=dtemp[12]*d[0]+dtemp[13]*d[1]+dtemp[14]*d[2]+dtemp[15]*d[3];
/*四元数归一化，这个地方在书上提到4次归一化一次，经过我的实验，归一化间隔有影响，因此每次计算都归一化一次*/
model_q=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
d[0]=q0/model_q;
d[1]=q1/model_q;
d[2]=q2/model_q;
d[3]=q3/model_q;
}

q0=d[0];
q1=d[1];
q2=d[2];
q3=d[3];

/*姿态转换矩阵*/

TT[0][0]=q0*q0+q1*q1-q2*q2-q3*q3;
TT[0][1]=2*(q1*q2-q0*q3);

TT[0][2]=2*(q1*q3+q0*q2);
TT[1][0]=2*(q1*q2+q0*q3);
TT[1][1]=q0*q0-q1*q1+q2*q2-q3*q3;

TT[1][2]=2*(q2*q3-q0*q1);
TT[2][0]=2*(q1*q3-q0*q2);
TT[2][1]=2*(q2*q3+q0*q1);

TT[2][2]=q0*q0-q1*q1-q2*q2+q3*q3;
/**/
cbn[0]=TT[0][0];
cbn[1]=TT[0][1];
cbn[2]=TT[0][2];
cbn[3]=TT[1][0];
cbn[4]=TT[1][1];
cbn[5]=TT[1][2];
cbn[6]=TT[2][0];
cbn[7]=TT[2][1];
cbn[8]=TT[2][2];

for(i=0;i<16;i++)
{
FA[i]=dtemp[i];
}
/*观测矩阵，已经进行了EKF化 详细参考《9DOF算法解释》*/
HA[0]=2*d[0]*GE+2*d[3]*GN-2*d[2]*GU;
HA[1]=2*d[1]*GE+2*d[2]*GN+2*d[3]*GU;
HA[2]=-2*d[2]*GE+2*d[1]*GN-2*d[0]*GU;
HA[3]=-2*d[3]*GE+2*d[0]*GN+2*d[1]*GU;

HA[4]=-2*d[3]*GE+2*d[0]*GN+2*d[1]*GU;
HA[5]=2*d[2]*GE-2*d[1]*GN+2*d[0]*GU;
HA[6]=2*d[1]*GE+2*d[2]*GN+2*d[3]*GU;
HA[7]=-2*d[0]*GE-2*d[3]*GN+2*d[2]*GU;


HA[8]=2*d[2]*GE-2*d[1]*GN+2*d[0]*GU;
HA[9]=2*d[3]*GE-2*d[0]*GN-2*d[1]*GU;
HA[10]=2*d[0]*GE+2*d[3]*GN-2*d[2]*GU;
HA[11]=2*d[1]*GE+2*d[2]*GN+2*d[3]*GU;

HA[12]=2*d[0]*ME+2*d[3]*MN-2*d[2]*MU;
HA[13]=2*d[1]*ME+2*d[2]*MN+2*d[3]*MU;
HA[14]=-2*d[2]*ME+2*d[1]*MN-2*d[0]*MU;
HA[15]=-2*d[3]*ME+2*d[0]*MN+2*d[1]*MU;

HA[16]=-2*d[3]*ME+2*d[0]*MN+2*d[1]*MU;
HA[17]=2*d[2]*ME-2*d[1]*MN+2*d[0]*MU;
HA[18]=2*d[1]*ME+2*d[2]*MN+2*d[3]*MU;
HA[19]=-2*d[0]*ME-2*d[3]*MN+2*d[2]*MU;


HA[20]=2*d[2]*ME-2*d[1]*MN+2*d[0]*MU;
HA[21]=2*d[3]*ME-2*d[0]*MN-2*d[1]*MU;
HA[22]=2*d[0]*ME+2*d[3]*MN-2*d[2]*MU;
HA[23]=2*d[1]*ME+2*d[2]*MN+2*d[3]*MU;

ZAK[0]=(ff[0]-cbn[0]*GE-cbn[3]*GN-cbn[6]*GU);
ZAK[1]=(ff[1]-cbn[1]*GE-cbn[4]*GN-cbn[7]*GU);
ZAK[2]=(ff[2]-cbn[2]*GE-cbn[5]*GN-cbn[8]*GU);
ZAK[3]=(hmc[0]-cbn[0]*ME-cbn[3]*MN-cbn[6]*MU);
ZAK[4]=(hmc[1]-cbn[1]*ME-cbn[4]*MN-cbn[7]*MU);
ZAK[5]=(hmc[2]-cbn[2]*ME-cbn[5]*MN-cbn[8]*MU);

for(i=0;i<4;i++)
{
xkerror[i]=0;//反馈式卡尔曼滤波归零
xk[i]=0;
}
if(g_temp>1.3||g_temp<0.7)/*抗震优化*/
{
yuce_only=1;
}
if(flag_count_newmoxing==1)
{
acc_filterUpdate_UD(FA,ZAK,HA,xk,RA,xkerror);//使用acc_comp7.c里面的程序
}
else
{
}
yuce_only=0;
for(i=0;i<4;i++)
{
xk[i]=xk[i]+xkerror[i];
}

/*进行四元数补偿*/
q0temp=q0+xk[0];
q1temp=q1+xk[1];
q2temp=q2+xk[2];
q3temp=q3+xk[3];
/*存储四元数，为下次四元数计算提供数据*/
q0=q0temp;
q1=q1temp;
q2=q2temp;
q3=q3temp;
/*再次归一化*/
model_q=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);//四元数模
d[0]=q0/model_q;
d[1]=q1/model_q;
d[2]=q2/model_q;
d[3]=q3/model_q;
q0=d[0];
q1=d[1];
q2=d[2];
q3=d[3];

/*变为姿态矩阵形式*/

TT[0][0]=q0*q0+q1*q1-q2*q2-q3*q3;
TT[0][1]=2*(q1*q2-q0*q3);

TT[0][2]=2*(q1*q3+q0*q2);
TT[1][0]=2*(q1*q2+q0*q3);
TT[1][1]=q0*q0-q1*q1+q2*q2-q3*q3;

TT[1][2]=2*(q2*q3-q0*q1);
TT[2][0]=2*(q1*q3-q0*q2);
TT[2][1]=2*(q2*q3+q0*q1);

TT[2][2]=q0*q0-q1*q1-q2*q2+q3*q3;//pitch为俯仰角，roll为滚转，yaw为航向


cbn[0]=TT[0][0];
cbn[1]=TT[0][1];
cbn[2]=TT[0][2];
cbn[3]=TT[1][0];
cbn[4]=TT[1][1];
cbn[5]=TT[1][2];
cbn[6]=TT[2][0];
cbn[7]=TT[2][1];
cbn[8]=TT[2][2];

/*将四元数变为欧拉角*/
pitch=asin(-TT[2][0]);
angle[1]=pitch;
angle[2]=atan2((cbn[7]),(cbn[8]));
angle[0]=atan2(cbn[3],cbn[0]);
}
//请注意上面的角度都是rad格式，请自己转换成°
/*系统开机时候得陀螺零位自标定，并且对R P Q矩阵进行初始化*/
void mpu_calibration()
{
        

  int gryx[16]={0};
  int gryy[16]={0};
  int gryz[16]={0};
  float mpusumx=0;
  float mpusumy=0;
  float mpusumz=0;
  
  float mpuxx[100]={0};
  float mpuyy[100]={0};
  float mpuzz[100]={0};
  float mpusumxx=0;
  float mpusumyy=0;
  float mpusumzz=0;
  int i,j=0;
  //printf("%.3f %.3f %.3f\r\n",1.0,2.0,3.0);
  for(i=0;i<100;i++)

  {      mpusumx=0;
         mpusumy=0;
         mpusumz=0;
         for(j=0;j<16;j++)
         {
         MPU6050ReadGyro(gyrodatasss);
         gryoo[0]= gyrodatasss[0];
         gryoo[1]= gyrodatasss[1];
         gryoo[2]= gyrodatasss[2];
         gryx[j]=gryoo[0];
         gryy[j]=gryoo[1];
         gryz[j]=gryoo[2];
         mpusumx+=gryx[j];
         mpusumy+=gryy[j];
         mpusumz+=gryz[j];
         }
  mpuxx[i]=mpusumx*0.0625;
  mpuyy[i]=mpusumy*0.0625;
  mpuzz[i]=mpusumz*0.0625;
  mpusumxx+=mpuxx[i];
  mpusumyy+=mpuyy[i];
  mpusumzz+=mpuzz[i];
  }
/*求和取均值*/
  mpusumxx*=0.01;
  mpusumyy*=0.01;
  mpusumzz*=0.01;
  mpuoffsetx=mpusumxx;
  mpuoffsety=mpusumyy;
  mpuoffsetz=mpusumzz;

  


  /*FQ PP赋值，具体方法请参考《卡尔曼算法解释》，不过也可以通过试验的手段对这些数据进行初始化*/
  FQ[0]=0.25*gxnosize*gxnosize*kf_t*kf_t;
  FQ[5]=0.25*gynosize*gynosize*kf_t*kf_t;
  FQ[10]=0.25*gznosize*gznosize*kf_t*kf_t;
  FQ[15]=0.25*gznosize*gznosize*kf_t*kf_t;
  
  
  PP[0]=q0nosize*q0nosize;
  PP[5]=q1nosize*q1nosize;
  PP[10]=q2nosize*q2nosize;
  PP[15]=q3nosize*q3nosize;

  RA[0]=axerror*axerror;
  RA[7]=ayerror*ayerror;
  RA[14]=azerror*azerror;
  RA[21]=mxnosize*mxnosize;
  RA[28]=mynosize*mynosize;
  RA[35]=mznosize*mznosize;

 
}



////通信协议输出部分
void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
 	unsigned int temp=0xaF+2;
	char ctemp;
	USART_Send(0xa5);
	USART_Send(0x5a);
	USART_Send(14+2);
	USART_Send(0xA1);

	if(yaw<0)
          yaw=32768-yaw;
      
	ctemp=yaw>>8;
	USART_Send(ctemp);
	temp+=ctemp;
	ctemp=yaw;
	USART_Send(ctemp);
	temp+=ctemp;

	if(pitch<0)
        pitch=32768-pitch;
	ctemp=pitch>>8;
	USART_Send(ctemp);
	temp+=ctemp;
	ctemp=pitch;
	USART_Send(ctemp);
	temp+=ctemp;

	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	USART_Send(ctemp);
	temp+=ctemp;
	ctemp=roll;
	USART_Send(ctemp);
	temp+=ctemp;

	if(alt<0)
        alt=32768-alt;
	ctemp=alt>>8;
	USART_Send(ctemp);
	temp+=ctemp;
	ctemp=alt;
	USART_Send(ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	USART_Send(ctemp);
	temp+=ctemp;
	ctemp=tempr;
	USART_Send(ctemp);
	temp+=ctemp;

	if(press<0)
        press=32768-press;
	ctemp=press>>8;
	USART_Send(ctemp);
	temp+=ctemp;
	ctemp=press;
	USART_Send(ctemp);
	temp+=ctemp;

	USART_Send(temp%256);
        //putchar(0x09);
	USART_Send(0xaa);
}


void UART1_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	USART_Send(0xa5);
	USART_Send(0x5a);
	USART_Send(14+8);
	USART_Send(0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	USART_Send(ctemp);
	temp+=ctemp;
	ctemp=ax;
	USART_Send(ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	USART_Send(ctemp);
	temp+=ctemp;
	ctemp=ay;
	USART_Send(ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	USART_Send(ctemp);
	temp+=ctemp;
	ctemp=az;
	USART_Send(ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	USART_Send(ctemp);
	temp+=ctemp;
	ctemp=gx;
	USART_Send(ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	USART_Send(ctemp);
	temp+=ctemp;
	ctemp=gy;
	USART_Send(ctemp);
	temp+=ctemp;
//-------------------------
	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	USART_Send(ctemp);
	temp+=ctemp;
	ctemp=gz;
	USART_Send(ctemp);
	temp+=ctemp;

	if(hx<0)hx=32768-hx;
	ctemp=hx>>8;
	USART_Send(ctemp);
	temp+=ctemp;
	ctemp=hx;
	USART_Send(ctemp);
	temp+=ctemp;

	if(hy<0)hy=32768-hy;
	ctemp=hy>>8;
	USART_Send(ctemp);
	temp+=ctemp;
	ctemp=hy;
	USART_Send(ctemp);
	temp+=ctemp;

	if(hz<0)hz=32768-hz;
	ctemp=hz>>8;
	USART_Send(ctemp);
	temp+=ctemp;
	ctemp=hz;
	USART_Send(ctemp);
	temp+=ctemp;

	USART_Send(temp%256);
	USART_Send(0xaa);
}




