#ifndef _pid_h_
#define _pid_h_

//PID�ṹ��
typedef struct PID
{
	float kp;//kp
	float ki;//ki
	float kd;//kd
	double pout;//k���
	float iout;//i���
	float dout;//d���
	float now_error;			//��ǰ���
	float Last_error;			//��һ�����
	float Last_Last_error;	//���ϴ����
	float sum_of_error;     //��ʷ�����
	float set;		//����
	float now;		//��ǰ
	float out;		//���

	int pid_mode;	//PIDģʽ���ã�1Ϊλ�û�PID��2Ϊ����ʽPID
 
  float MaxOutput;//PID����޷�	
	float IntegralLimit;//I����޷�	
	float plus;		//��������ֵ
  float plus_out;	//����ʽ���ֵplus_out = last_plus_out + plus
  float last_plus_out;//�ϴ�����ʽ���ֵ
	
	float Max_Error_Data;
	
	float Small_Error_Limit;
	
	int Set_Out_Mode;
	float Set_A;
	float Set_B;
	float Set_ratio;
	float Set_alpha;
	float Last_Ud;
}PID;		

//PID�������ֵ����
void PID_limit(float *a, float PID_MAX);

//PID������Сֵ����
void PID_limitmin(float *a, float PID_MIN);

//PID�޷�����
float xianfu(float a,float max);

//PID����ֵ����
void PID_juedui(float *a);

float PID_Fabs(float ffabs);

//PID��ʼ��
void pid_init(PID*pid,float p,float i,float d,int maxout,int imaxout,int mode);

//PIDȫ����ʼ��
void pid_math_init(PID*pid,float p,float i,float d,int maxout,int imaxout,int mode,float pid_Aa,float pid_Bb,float pid_Alpha,float Limit_Data);

//pid��������
void pid_change(PID*pid,float p,float i,float d,int maxout,int imaxout,int mode);

//PID����
float pid_calc(PID*pid, float now, float set);


#endif
