#ifndef SERVO_H_
#define SERVO_H_

#include "stm32h7xx_hal.h" // Required for TIM_HandleTypeDef

typedef enum {
	SERVO_L = TIM_CHANNEL_1,
	SERVO_R = TIM_CHANNEL_2
} SERVO_CHANNELS;


// Function to initialize the servo driver with its timer
void servo_init(TIM_HandleTypeDef *htim);

void servo_init_micro(TIM_HandleTypeDef *htim);
void turn_servo_micro(uint8_t angle);

void turnLeft(float angle);
void turnRight(float angle);

static inline uint16_t clamp_u16(int x, int lo, int hi);
void ServoWriteUS(uint32_t channel, uint16_t pulse_us);
void ServoTurnAngle(uint32_t channel, int angle);
void servosReset();
void deadSpin();
#endif /* SERVO_H_ */
