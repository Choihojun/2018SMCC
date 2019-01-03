/*
 * direction.h
 *
 *  Created on: 2018. 6. 28.
 *      Author: √÷»£¡ÿ
 */

#ifndef DIRECTION_RPM_H_
#define DIRECTION_RPM_H_



extern void POS_MID(void);
extern void POS_MID_School(void);

extern void Black_Line_Detect(void);
extern void Motor_dir_pwm(int pwm);
extern void Obstacle_Stop_OR_Avoid(int School_Flag);

extern volatile int Error_Past, Error_Present , Error_PaPre;
extern volatile int Left_Detected_Pos, Right_Detected_Pos;

extern volatile double duty_servo;

#define CW				0
#define CCW				1

#endif /* 0_SRC_0_APPSW_APP_INC_DIRECTION_H_ */
