/*�˶γ����ǿ�������Ŀ����������㷨����Ȩ���ڿ�����asha*/
//#include <stdio.h>
#include <math.h>
#include <Matrix.h>
//#include <ins.h>
#include <float.h>

#define  state_num 4
#define  obs_num   6
float af[6];
float Qab[36]={0};
float sita[16];
float J[24];
float zk[6];
float xkk[4];
float R[36];
float seq[4];
extern float PM[36];
extern float PP[16];
extern float FQ[16];
extern float PPA[36];
extern int yuce_only;
void acc_filterUpdate_UD(float* sita,float* zk,float* J,float* xkk,float* R,float* seq)
{ 

int i,k=0;
float PP2[state_num*state_num]={0};
float K[state_num*obs_num]={0};
float pz2[state_num*state_num]={0};
float pz[state_num*state_num]={0};
float invsita[state_num*state_num]={0};
float invJ[state_num*obs_num]={0};
float seqd2[obs_num*obs_num]={0};
float U[state_num*state_num]={0};
float D[state_num*state_num]={0};
float seqd5[state_num*obs_num]={0};
float UT[state_num*state_num]={0};
float seqd6[state_num*state_num]={0};
float seqd7[state_num*obs_num]={0};
float FW[state_num*obs_num]={0};
float seqd7T[state_num*obs_num]={0};
float FWFT[state_num*state_num]={0};
float DSubFWFT[state_num*state_num]={0};
float UDSubFWFT[state_num*state_num]={0};




af[0]=zk[0];
af[1]=zk[1];
af[2]=zk[2];
af[3]=zk[3];
af[4]=zk[4];
af[5]=zk[5];



MatrixTranspose(sita,state_num,state_num,invsita);
MatrixMultiply(sita,state_num,state_num,PP,state_num,state_num,pz);
MatrixMultiply(pz,state_num,state_num,invsita,state_num,state_num,pz2);//PK,K-1

for(i=0;i<state_num*state_num;i++)
{
pz2[i]=pz2[i]+FQ[i];
}

UD(pz2,state_num,U,D);/*��P�����UD�ֽ�*/


MatrixTranspose(J,obs_num,state_num,invJ);
MatrixTranspose(U,state_num,state_num,UT);
MatrixMultiply(D,state_num,state_num,UT,state_num,state_num,seqd6);
MatrixMultiply(seqd6,state_num,state_num,invJ,state_num,obs_num,seqd7);///F=seq7
MatrixMultiply(U,state_num,state_num,seqd7,state_num,obs_num,seqd5);///G=U*F
MatrixMultiply(J,obs_num,state_num,seqd5,state_num,obs_num,seqd2);//HPH'=seqd2
//�ֽ�HPH'+R
/**/
for(i=0;i<obs_num*obs_num;i++)
{
seqd2[i]=seqd2[i]+R[i];
}


//(HPH'+R)^-1,PK[9]
MatrixInverse(seqd2,obs_num,0);
MatrixMultiply(seqd5,state_num,obs_num,seqd2,obs_num,obs_num,K);//K=G*W^-1������
MatrixMultiply(seqd7,state_num,obs_num,seqd2,obs_num,obs_num,FW);
MatrixTranspose(seqd7,state_num,obs_num,seqd7T);
MatrixMultiply(FW,state_num,obs_num,seqd7T,obs_num,state_num,FWFT);

MatrixSub(D,FWFT,DSubFWFT,state_num,state_num);
MatrixMultiply(U,state_num,state_num,DSubFWFT,state_num,state_num,UDSubFWFT);
MatrixMultiply(UDSubFWFT,state_num,state_num,UT,state_num,state_num,PP2);
MatrixMultiply(K,state_num,obs_num,af,obs_num,1,seq);//seq[4],�˲����
if(yuce_only==1)/*�˱�־λ���ý���Ԥ�ⲻ���й��Ʊ�־λ����ins.c��DCM�����н�����λ��ͨ�����������־λ�����Խ��п����Ż�*/
{
for(i=0;i<state_num;i++)
{
seq[i]=0;
}
}
else
{

}
for(i=0;i<state_num*state_num;i++)
{
PP[i]=PP2[i];
}

}




