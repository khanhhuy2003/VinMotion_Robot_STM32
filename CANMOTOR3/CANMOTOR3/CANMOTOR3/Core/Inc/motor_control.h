/*
 * motor_control.h
 *
 *  Created on: Aug 22, 2025
 *      Author: ASUS
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "main.h"
#include "fdcan.h"
extern float AngleCurrent[23];
extern float SpeedCurrent[23];
extern float TorqueCurrent[23];
extern float TempCurrent[23];
extern uint8_t TxData[8];
extern uint8_t RxData[8];
#define LAN8720_PHY_ADDRESS 0x00 // Common PHY address (verify with schematic)
#define LAN8720_BCR         0x00 // Basic Control Register address
#define LAN8720_BCR_RESET   (1 << 15) // Reset bit in BCR
#define LAN8720_RESET_PIN   GPIOB, GPIO_PIN_0 // Example reset pin (adjust as needed)
#define MAX_TX_SIZE 128
#define MAX_RX_SIZE 1024
#define MAX_MOTORS 23

//Motor calibration limited table
#define Motor1_min  -0.5
#define Motor1_max   0.5

#define Motor2_min   -0.5
#define Motor2_max   0.2

#define Motor3_min   -0.5
#define Motor3_max   0.5

#define Motor4_min  0.0
#define Motor4_max   1.0

#define Motor5_min   -0.5
#define Motor5_max   0.3

#define Motor6_min   -0.3
#define Motor6_max   0.5

#define Motor7_min  -0.5
#define Motor7_max   0.5

#define Motor8_min  -0.2
#define Motor8_max   0.5

#define Motor9_min  -0.5
#define Motor9_max  0.5

#define Motor10_min  -1.0
#define Motor10_max  0.0

#define Motor11_min  -0.3
#define Motor11_max   0.5

#define Motor12_min  -0.5
#define Motor12_max  0.4

//Motor calibration limited table---------------
#define Motor13_min  -0.5
#define Motor13_max  0.5

#define Motor14_min   -1.57   // Left_pitch 1
#define Motor14_max   3.66    // Left_pitch

#define Motor15_min   0.0   // Left_roll  2
#define Motor15_max   3.14    // Left_roll

#define Motor16_min   -2.09   // Left_yaw   3
#define Motor16_max   2.09    // Left_yaw

#define Motor17_min   -1.57    // Left_elbow 4
#define Motor17_max   0.0     // Left_elbow

#define Motor18_min   -2.44   // Left_wrist 5
#define Motor18_max   2.44    // Left_wrist

#define Motor19_min  -3.66    // Right_pitch 1
#define Motor19_max   1.57    // Right_pitch

#define Motor20_min  -3.14    // Right_roll  2
#define Motor20_max   0.0    // Right_roll

#define Motor21_min  -2.09    // Right_yaw   3
#define Motor21_max  2.09    // Right_yaw

#define Motor22_min  0.0     // Right_elbow 4
#define Motor22_max  1.57     // Right_elbow

#define Motor23_min  -2.44       // Right_wrist 5
#define Motor23_max  2.44      // Right_wrist

//---------------------------------------------------------------------
#define P_MIN -12.57f
#define P_MAX 12.57f

#define V_MIN_02 -44.0f
#define V_MAX_02 44.0f
#define KP_MIN_02 0.0f
#define KP_MAX_02 500.0f
#define KD_MIN_02 0.0f
#define KD_MAX_02 5.0f
#define T_MIN_02 -17.0f
#define T_MAX_02 17.0f

#define V_MIN_03 -20.0f
#define V_MAX_03 20.0f
#define KP_MIN_03 0.0f
#define KP_MAX_03 5000.0f
#define KD_MIN_03 0.0f
#define KD_MAX_03 100.0f
#define T_MIN_03 -60.0f
#define T_MAX_03 60.0f

#define V_MIN_04 -15.0f
#define V_MAX_04 15.0f
#define KP_MIN_04 0.0f
#define KP_MAX_04 5000.0f
#define KD_MIN_04 0.0f
#define KD_MAX_04 100.0f
#define T_MIN_04 -120.0f
#define T_MAX_04 120.0f


#define MOTOR_1	0x01
#define MOTOR_2	0x02
#define MOTOR_3	0x03
#define MOTOR_4 0x04
#define MOTOR_5	0x05
#define MOTOR_6	0x06
#define MOTOR_7	0x07
#define MOTOR_8	0x08
#define MOTOR_9	0x09
#define MOTOR_10	0x0A
#define MOTOR_11	0x0B
#define MOTOR_12	0x0C
#define MOTOR_13	0x0D
#define MOTOR_14	0x0E
#define MOTOR_15	0x0F
#define MOTOR_16	0x10
#define MOTOR_17	0x11
#define MOTOR_18	0x12
#define MOTOR_19	0x13
#define MOTOR_20	0x14
#define MOTOR_21	0x15
#define MOTOR_22	0x16
#define MOTOR_23    0x17

#define CANBUS_1    1
#define CANBUS_2    2
#define CANBUS_3    3


#define MOTOR_ENABLE_MODE       3
#define MOTOR_OPERATION_MODE    1
#define MOTOR_DISABLE_MODE      4
typedef struct {
	uint32_t targetMotorCanId :8;		//bit 7 ~ bit 0
	uint32_t data :16;					//bit 23 ~ bit 8
	uint32_t mode :5;					//bit 28 ~ bit 24
	uint32_t reserved :3;
} CanId;

typedef struct {
	float q;
	float dq;
	float tau_est;
	float temprature;
	uint32_t error_code;
	uint32_t operation_mode;
} MotorInfo;
typedef struct{

int mode;
} Motor_Receive;
/*
 * Function prototype
 */
void EnableMotorX(uint16_t motor_can_id, uint8_t bus);
void EnableAllMotor();
void DisableMotorX(uint16_t motor_can_id, uint8_t bus);
void DisableAllMotor();
void MotorReceiveCommand(uint16_t motor_id, float torque, float position, float speed,
		float kp, float kd, int canbus);
void MotorResponse(uint8_t RxData[8], int motor_can_id);
uint8_t decode_can_id(uint32_t can_id);
#endif /* INC_MOTOR_CONTROL_H_ */
