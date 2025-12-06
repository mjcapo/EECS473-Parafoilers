#include "Servo.h"
#include "stm32h7xx_hal.h" // Include HAL definitions

// Static pointer to store the timer handle
static TIM_HandleTypeDef *servo_htim;
extern TIM_HandleTypeDef htim1;


/**
 * @brief Initializes the servo driver and starts the PWM signal.
 * @param htim: Pointer to the TIM_HandleTypeDef for the servo's timer.
 */
void servo_init(TIM_HandleTypeDef *htim) {
    servo_htim = htim;
    // Start the PWM channel required for the servo

    //ServoL
    HAL_TIM_PWM_Start(servo_htim, TIM_CHANNEL_1);

    //ServoR
    HAL_TIM_PWM_Start(servo_htim, TIM_CHANNEL_2);
}

void servo_init_micro(TIM_HandleTypeDef *htim) {
    // Start the PWM channel required for the servo

    //Servo Release
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);

    turn_servo_micro(0);
}

void turn_servo_micro(uint8_t angle)
{
    if (angle > 180) angle = 180;

    int correctedAngle = (int)angle;

    if(angle > 90) {
    	correctedAngle += 70;
    } else if (angle < 90) {
    	correctedAngle -= 70;
    }

    // Map 0–180° to 1000–2000 µs pulse width
    uint16_t pulse_us = 1000 + (correctedAngle * 1000) / 180;

    // Since 1 tick = 1 µs (timer tick = 1 MHz), counts = µs directly
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
}




// Pos Servo Functions

static inline uint16_t clamp_u16(int x, int lo, int hi) {
  if (x < lo) return (uint16_t)lo;
  if (x > hi) return (uint16_t)hi;
  return (uint16_t)x;
}

void ServoWriteUS(uint32_t channel, uint16_t pulse_us)
{
  /* Timer period is 19999 (20,000 µs). Clamp for safety. */
  uint16_t ccr = clamp_u16(pulse_us, 0, 19999);
  __HAL_TIM_SET_COMPARE(servo_htim, channel, ccr);
}


//-360 to 360
void ServoTurnAngle(uint32_t channel, int angle) {
	uint16_t pwm_pulse = (uint16_t)(1500 - angle * 0.6388);

	ServoWriteUS(channel, pwm_pulse);
}


void turnLeft(float angle) {
	angle = fabs(angle);

	//int servo_angle = angle * 2; //360 control
	int servo_angle = (int)(angle); //180 control
	ServoTurnAngle(SERVO_L, servo_angle);
	ServoTurnAngle(SERVO_R, 0);
}


void turnRight(float angle) {
	angle = fabs(angle);

	//int servo_angle = angle * 2; //360 control
	int servo_angle = (int)(angle); //180 control
	ServoTurnAngle(SERVO_R, servo_angle);
	ServoTurnAngle(SERVO_L, 0);
}

void servosReset() {
	ServoTurnAngle(SERVO_L, 0);
	ServoTurnAngle(SERVO_R, 0);
}
void deadSpin() {
	//ServoTurnAngle(SERVO_R, 360);
	ServoTurnAngle(SERVO_R, 144);
	ServoTurnAngle(SERVO_L, 0);
}

