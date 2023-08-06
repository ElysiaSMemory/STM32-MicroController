#include "stm32f10x.h"                  // Device header
#include "PWM.h"

/**
 * @brief Initialize Servo PA1
 * @param None
 * @retval None
 */
void Servo_Init(void)
{
	PWM_Init();
}

/**
 * @brief Set the angle of Servo
 * @param Angle: Angle in Degrees
 * @retval None
 */
void Servo_SetAngle(float Angle)
{
	// Given - Target
	// 0     = 500
	// 180   = 2500
	// Range
	// 180     2000
	// Formula is Angle / 180 * 2000 + 500
	PWM_SetCompare2(Angle / 180 * 2000 + 500);
}
