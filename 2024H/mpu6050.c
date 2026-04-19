#include "mpu6050.h"
#include "delay.h"

#define    MPU6050_I2C_INST		I2C_0_INST

// 增加带超时的等待宏
#define I2C_TIMEOUT  100000
#define WAIT_FOR_IDLE() { \
    uint32_t t_out = I2C_TIMEOUT; \
    while (!(DL_I2C_getControllerStatus(MPU6050_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE) && t_out--); \
}
#define WAIT_FOR_BUSY() { \
    uint32_t t_out = I2C_TIMEOUT; \
    while ((DL_I2C_getControllerStatus(MPU6050_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) && t_out--); \
}

MPU6050_DEF mpu6050;

void I2C_WriteReg(uint8_t DevAddr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
    uint8_t I2Ctxbuff[17]; 
    if (count > 16) count = 16;
    I2Ctxbuff[0] = reg_addr;
    for (uint8_t i = 0; i < count; i++) {
        I2Ctxbuff[i + 1] = reg_data[i];
    }
    WAIT_FOR_IDLE();
    DL_I2C_fillControllerTXFIFO(MPU6050_I2C_INST, &I2Ctxbuff[0], count + 1);
    DL_I2C_startControllerTransfer(MPU6050_I2C_INST, DevAddr, DL_I2C_CONTROLLER_DIRECTION_TX, count + 1);
    WAIT_FOR_BUSY();
}

void I2C_ReadReg(uint8_t DevAddr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
    WAIT_FOR_IDLE();
    DL_I2C_fillControllerTXFIFO(MPU6050_I2C_INST, &reg_addr, 1);
    DL_I2C_startControllerTransfer(MPU6050_I2C_INST, DevAddr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    WAIT_FOR_BUSY();
    DL_I2C_startControllerTransfer(MPU6050_I2C_INST, DevAddr, DL_I2C_CONTROLLER_DIRECTION_RX, count);
    for (uint8_t i = 0; i < count; i++) {
        uint32_t t_out = 100000;
        while (DL_I2C_isControllerRXFIFOEmpty(MPU6050_I2C_INST) && t_out--);
        reg_data[i] = DL_I2C_receiveControllerData(MPU6050_I2C_INST);
    }
}

void Single_WriteI2C(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data){
	I2C_WriteReg(SlaveAddress,REG_Address,&REG_data,1);
}
unsigned char Single_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address){
	uint8_t data;
	I2C_ReadReg(SlaveAddress,REG_Address,&data,1);
	return data;
}
uint8_t g_imu_addr = 0x68;

#define	SMPLRT_DIV		0x19
#define	MPU_CONFIG		0x1A
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG	        0x1C
#define	ACCEL_XOUT_H	        0x3B
#define	PWR_MGMT_1		0x6B
#define	WHO_AM_I		0x75

void mpu6050_init(void){
    DL_I2C_reset(MPU6050_I2C_INST);
    DL_I2C_enableController(MPU6050_I2C_INST);
    delay_ms(50);
    uint8_t who_am_i = 0;
    I2C_ReadReg(0x68, WHO_AM_I, &who_am_i, 1);
    if (who_am_i != 0x68) {
        delay_ms(10);
        I2C_ReadReg(0x69, WHO_AM_I, &who_am_i, 1);
        if (who_am_i == 0x68) g_imu_addr = 0x69;
    }
    Single_WriteI2C(g_imu_addr, PWR_MGMT_1, 0x00); 
    delay_ms(50); 
    Single_WriteI2C(g_imu_addr, SMPLRT_DIV, 0x09);
    Single_WriteI2C(g_imu_addr, MPU_CONFIG, 0x06);
    Single_WriteI2C(g_imu_addr, GYRO_CONFIG, 0x18);
    Single_WriteI2C(g_imu_addr, ACCEL_CONFIG, 0x18); 
}

void mpu6050_read(int16_t *gyro, int16_t *accel, float *temperature){
    uint8_t buf[14];
    int16_t temp_raw;
    I2C_ReadReg(g_imu_addr, ACCEL_XOUT_H, buf, 14);
    accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accel[2] = (int16_t)((buf[4] << 8) | buf[5]);	
    temp_raw = (int16_t)((buf[6] << 8) | buf[7]);
    gyro[0]  = (int16_t)((buf[8] << 8) | buf[9]);
    gyro[1]  = (int16_t)((buf[10] << 8) | buf[11]);
    gyro[2]  = (int16_t)((buf[12] << 8) | buf[13]);	
    *temperature = 36.53f + (float)(temp_raw / 340.0f);	
}

#define RtA 			57.324841f				
#define Sampling_Time	0.01f
float pitch2,roll2,Yaw;
float Gyro_Z_Measeure = 0;

void MPU6050_ReadDatas_Proc(void){
	static uint16_t time=0;
	mpu6050_read(mpu6050.Gyro_Original,mpu6050.Accel_Original,&mpu6050.temperature);
	if(time<100) {
        time++;
		mpu6050.Gyro_Offset[2] +=(float)mpu6050.Gyro_Original[2]/100.0f;
	} else {
		mpu6050.Gyro_Calulate[2] = mpu6050.Gyro_Original[2] - mpu6050.Gyro_Offset[2];
		mpu6050.Gyro_Average[2] = mpu6050.Gyro_Calulate[2];
	}
}

void AHRS_Geteuler(void){
	MPU6050_ReadDatas_Proc();
	Gyro_Z_Measeure = (mpu6050.Gyro_Average[2]) * 2000.0f / 32768.0f;
	Yaw += Gyro_Z_Measeure * Sampling_Time;
	if (Yaw > 180.0f)  Yaw -= 360.0f;
	if (Yaw < -180.0f) Yaw += 360.0f;
	mpu6050.Yaw = Yaw;
}
