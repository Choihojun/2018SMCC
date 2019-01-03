/*
 * School.h
 *
 *  Created on: 2018. 6. 29.
 *      Author: ��ȣ��
 */

#ifndef SCHOOL_H_
#define SCHOOL_H_

extern void School_Zone_Detect(void);
extern void Dotted_Line_Pos_Detect(void);
extern void Sonar_Sensor(void);
extern void Obstacle_Avoiding(void);
extern void Turn_Right(void);



extern volatile int School_Flag; // ������ ���� �� �ƿ� �÷���
extern volatile int Time_Count_School_Out,Time_Count_School_In;  // ������ �ν��� �ٷ� ������ �ƿ��� �ν����� �ʱ� ���� �ð�����

extern volatile int Dotted_Line_Pos;
extern volatile int Left_Line_Position_Que;
extern volatile int Right_Line_Position_Que;
extern volatile int Got_it;

extern volatile int Time_Count_School_Zone, Time_School_Zone_Flag;

extern volatile int IR_ADC_OFF_Flag;
#endif /* 0_SRC_0_APPSW_APP_INC_SCHOOL_H_ */
