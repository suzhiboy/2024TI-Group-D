#include "mpu6050.h"
#include "delay.h"

/* --- 软件 I2C 底层操作 (参考 OLED.h 的官方验证写法) --- */

// 设置 SCL 为高 (切换为输入，并开启上拉)
static void I2C_SCL_H(void) {
    DL_GPIO_disableOutput(GPIO_MPU6050_PORT, GPIO_MPU6050_M_SCL_PIN);
    DL_GPIO_initDigitalInputFeatures(GPIO_MPU6050_M_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

// 设置 SCL 为低 (切换为输出并拉低)
static void I2C_SCL_L(void) {
    DL_GPIO_initDigitalOutput(GPIO_MPU6050_M_SCL_IOMUX);
    DL_GPIO_clearPins(GPIO_MPU6050_PORT, GPIO_MPU6050_M_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_MPU6050_PORT, GPIO_MPU6050_M_SCL_PIN);
}

// 设置 SDA 为高 (切换为输入，并开启上拉)
static void I2C_SDA_H(void) {
    DL_GPIO_disableOutput(GPIO_MPU6050_PORT, GPIO_MPU6050_M_SDA_PIN);
    DL_GPIO_initDigitalInputFeatures(GPIO_MPU6050_M_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

// 设置 SDA 为低 (切换为输出并拉低)
static void I2C_SDA_L(void) {
    DL_GPIO_initDigitalOutput(GPIO_MPU6050_M_SDA_IOMUX);
    DL_GPIO_clearPins(GPIO_MPU6050_PORT, GPIO_MPU6050_M_SDA_PIN);
    DL_GPIO_enableOutput(GPIO_MPU6050_PORT, GPIO_MPU6050_M_SDA_PIN);
}

#define I2C_SDA_READ()  (DL_GPIO_readPins(GPIO_MPU6050_PORT, GPIO_MPU6050_M_SDA_PIN) & GPIO_MPU6050_M_SDA_PIN)

// 延迟函数 (约 100kHz)
static void SW_I2C_Delay(void) {
    delay_cycles(600); 
}

/* --- 软件 I2C 基础信号实现 --- */

void SW_I2C_Start(void) {
    I2C_SDA_H();
    I2C_SCL_H();
    SW_I2C_Delay();
    I2C_SDA_L();
    SW_I2C_Delay();
    I2C_SCL_L();
    SW_I2C_Delay();
}

void SW_I2C_Stop(void) {
    I2C_SDA_L();
    I2C_SCL_L();
    SW_I2C_Delay();
    I2C_SCL_H();
    SW_I2C_Delay();
    I2C_SDA_H();
    SW_I2C_Delay();
}

uint8_t SW_I2C_Wait_Ack(void) {
    uint16_t timeout = 0;
    I2C_SDA_H(); 
    SW_I2C_Delay();
    I2C_SCL_H();
    SW_I2C_Delay();
    while (I2C_SDA_READ()) {
        timeout++;
        if (timeout > 500) {
            SW_I2C_Stop();
            return 1;
        }
    }
    I2C_SCL_L();
    SW_I2C_Delay();
    return 0;
}

void SW_I2C_SendByte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 0x80) I2C_SDA_H(); else I2C_SDA_L();
        byte <<= 1;
        SW_I2C_Delay();
        I2C_SCL_H();
        SW_I2C_Delay();
        I2C_SCL_L();
        SW_I2C_Delay();
    }
}

uint8_t SW_I2C_ReadByte(uint8_t ack) {
    uint8_t byte = 0;
    I2C_SDA_H();
    for (uint8_t i = 0; i < 8; i++) {
        byte <<= 1;
        I2C_SCL_H();
        SW_I2C_Delay();
        if (I2C_SDA_READ()) byte++;
        I2C_SCL_L();
        SW_I2C_Delay();
    }
    if (ack) I2C_SDA_L(); else I2C_SDA_H();
    SW_I2C_Delay();
    I2C_SCL_H();
    SW_I2C_Delay();
    I2C_SCL_L();
    SW_I2C_Delay();
    return byte;
}

void I2C_WriteReg(uint8_t DevAddr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
    SW_I2C_Start();
    SW_I2C_SendByte(DevAddr << 1);
    if (SW_I2C_Wait_Ack()) return;
    SW_I2C_SendByte(reg_addr);
    SW_I2C_Wait_Ack();
    for (uint8_t i = 0; i < count; i++) {
        SW_I2C_SendByte(reg_data[i]);
        SW_I2C_Wait_Ack();
    }
    SW_I2C_Stop();
}

void I2C_ReadReg(uint8_t DevAddr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
    SW_I2C_Start();
    SW_I2C_SendByte(DevAddr << 1);
    if (SW_I2C_Wait_Ack()) return;
    SW_I2C_SendByte(reg_addr);
    SW_I2C_Wait_Ack();
    
    SW_I2C_Start();
    SW_I2C_SendByte((DevAddr << 1) | 1);
    if (SW_I2C_Wait_Ack()) return;
    for (uint8_t i = 0; i < count; i++) {
        reg_data[i] = SW_I2C_ReadByte(i < (count - 1));
    }
    SW_I2C_Stop();
}

void Single_WriteI2C(unsigned char SlaveAddress, unsigned char REG_Address, unsigned char REG_data) {
    I2C_WriteReg(SlaveAddress, REG_Address, &REG_data, 1);
}

unsigned char Single_ReadI2C(unsigned char SlaveAddress, unsigned char REG_Address) {
    uint8_t data = 0;
    I2C_ReadReg(SlaveAddress, REG_Address, &data, 1);
    return data;
}

uint8_t g_imu_addr = 0x68;
#define PWR_MGMT_1  0x6B
#define SMPLRT_DIV  0x19
#define MPU_CONFIG  0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define WHO_AM_I    0x75
#define ACCEL_XOUT_H 0x3B

void mpu6050_init(void) {
    I2C_SCL_H();
    I2C_SDA_H();
    delay_ms(150); 
    
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

void mpu6050_read(int16_t *gyro, int16_t *accel, float *temperature) {
    uint8_t buf[14];
    I2C_ReadReg(g_imu_addr, ACCEL_XOUT_H, buf, 14);
    accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accel[2] = (int16_t)((buf[4] << 8) | buf[5]);	
    int16_t temp_raw = (int16_t)((buf[6] << 8) | buf[7]);
    gyro[0]  = (int16_t)((buf[8] << 8) | buf[9]);
    gyro[1]  = (int16_t)((buf[10] << 8) | buf[11]);
    gyro[2]  = (int16_t)((buf[12] << 8) | buf[13]);	
    *temperature = 36.53f + (float)(temp_raw / 340.0f);	
}

MPU6050_DEF mpu6050;
#define Sampling_Time 0.01f
float pitch2 = 0, roll2 = 0, Yaw = 0;
float Gyro_Z_Measeure = 0;

void MPU6050_ReadDatas_Proc(void) {
    static uint16_t time = 0;
    mpu6050_read(mpu6050.Gyro_Original, mpu6050.Accel_Original, &mpu6050.temperature);
    if (time < 100) {
        time++;
        mpu6050.Gyro_Offset[2] += (float)mpu6050.Gyro_Original[2] / 100.0f;
    } else {
        mpu6050.Gyro_Calulate[2] = mpu6050.Gyro_Original[2] - mpu6050.Gyro_Offset[2];
        mpu6050.Gyro_Average[2] = mpu6050.Gyro_Calulate[2];
    }
}

void AHRS_Geteuler(void) {
    MPU6050_ReadDatas_Proc();
    Gyro_Z_Measeure = (mpu6050.Gyro_Average[2]) * 2000.0f / 32768.0f;
    Yaw += Gyro_Z_Measeure * Sampling_Time;
    if (Yaw > 180.0f)  Yaw -= 360.0f;
    if (Yaw < -180.0f) Yaw += 360.0f;
    mpu6050.Yaw = Yaw;
}
