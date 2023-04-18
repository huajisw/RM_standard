/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       INSTask.c/h
  * @brief      ��Ҫ����������mpu6500��������ist8310�������̬���㣬�ó�ŷ���ǣ�
  *             �ṩͨ��mpu6500��data ready �ж�����ⲿ�������������ݵȴ��ӳ�
  *             ͨ��DMA��SPI�����ԼCPUʱ�䣬�ṩע�Ͷ�Ӧ�ĺ궨�壬�ر�DMA��
  *             DR���ⲿ�жϵķ�ʽ.
  * @note       SPI �������ǳ�ʼ����ʱ����Ҫ����2MHz��֮���ȡ���������20MHz
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef INS_Task_H
#define INS_Task_H
#include "main.h"

#define USE_IST8310 //�Ƿ�ʹ��IST8310�����ƣ���ʹ��ע�Ͷ���

//#define MPU6500_USE_DATA_READY_EXIT //�Ƿ�ʹ��MPU6500���ⲿ�жϣ���ʹ��ע�Ͷ���

//#define MPU6500_USE_SPI_DMA //�Ƿ�ʹ��SPI��DMA���䣬��ʹ��ע�Ͷ���

//�������IST8310��DMA����23���ֽڣ�������ã���7���ֽڣ�Ϊ16���ֽ�
#if defined(USE_IST8310)
#define DMA_RX_NUM 23
#else
#define DMA_RX_NUM 16
#endif

//mpu6500ԭʼ�����ڻ�����buf��λ��
#ifdef MPU6500_USE_SPI_DMA
#define MPU6500_RX_BUF_DATA_OFFSET 1
#else
#define MPU6500_RX_BUF_DATA_OFFSET 0
#endif

//ist83100ԭʼ�����ڻ�����buf��λ��
#ifdef MPU6500_USE_SPI_DMA
#define IST8310_RX_BUF_DATA_OFFSET 16
#else
#define IST8310_RX_BUF_DATA_OFFSET 15
#endif

#define MPU6500_TEMPERATURE_PID_KP 1600.0f //�¶ȿ���PID��kp
#define MPU6500_TEMPERATURE_PID_KI 0.2f    //�¶ȿ���PID��ki
#define MPU6500_TEMPERATURE_PID_KD 0.0f    //�¶ȿ���PID��kd

#define MPU6500_TEMPERATURE_PID_MAX_OUT 4500.0f  //�¶ȿ���PID��max_out
#define MPU6500_TEMPERATURE_PID_MAX_IOUT 4400.0f //�¶ȿ���PID��max_iout

#define INS_DELTA_TICK 1 //������õļ��

#define INS_TASK_INIT_TIME 10 //����ʼ���� delay һ��ʱ��

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500�����¶ȵ�����TIM������ֵ������PWM���Ϊ MPU6500_TEMP_PWM_MAX - 1

//��ȡ��̬��ָ���ַ�󣬶�Ӧ��̬�ǵĵ�ַƫ���� fp32����
#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET 2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�
	
		fp32 sum_of_error;
		fp32 plus;
		fp32 plus_out;
		fp32 last_plus_out;

} PidTypeDef;

extern float fAcc[3], fGyro[3], fAngle[3];

extern void IMU_Task(void *pvParameters);

extern void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count);
extern void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3]);
extern const fp32 *get_INS_angle_point(void);
extern const fp32 *get_MPU6500_Gyro_Data_Point(void);
extern const fp32 *get_MPU6500_Accel_Data_Point(void);
extern const fp32* get_JY61_angle_Point(void);
extern const fp32* get_JY61_Gyro_Data_Point(void);
extern const fp32* get_JY61_Accel_Data_Point(void);
void WT61_Init(void);

#endif
