#pragma once

void mpu6050AccInit(void);
void mpu6050AccRead(int16_t *accData);
void mpu6050AccAlign(int16_t *accData);
void mpu6050GyroInit(void);
void mpu6050GyroRead(int16_t *gyroData);
void mpu6050GyroAlign(int16_t *gyroData);

bool mpu6050Detect(uint16_t lpf, uint8_t *scale);
void mpu6050DmpLoop(void);
void mpu6050DmpResetFifo(void);
