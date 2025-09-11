/*
 * motor_control.c
 *
 *  Created on: Aug 22, 2025
 *      Author: ASUS
 */


#include "motor_control.h"
#include "fdcan.h"
extern uint8_t TxData[8];
extern uint8_t RxData[8];
//FDCAN_HandleTypeDef hfdcan1;
//
//FDCAN_HandleTypeDef hfdcan2;
//
//FDCAN_HandleTypeDef hfdcan3;

extern FDCAN_TxHeaderTypeDef TxHeader;
extern FDCAN_RxHeaderTypeDef RxHeader;
float AngleCurrent[23];
float SpeedCurrent[23];
float TorqueCurrent[23];
float TempCurrent[23];
MotorConfig motor2_cfg = {T_MIN_02, T_MAX_02, V_MIN_02, V_MAX_02, KP_MIN_02, KP_MAX_02, KD_MIN_02, KD_MAX_02};
MotorConfig motor3_cfg = {T_MIN_03, T_MAX_03, V_MIN_03, V_MAX_03, KP_MIN_03, KP_MAX_03, KD_MIN_03, KD_MAX_03};
MotorConfig motor4_cfg = {T_MIN_04, T_MAX_04, V_MIN_04, V_MAX_04, KP_MIN_04, KP_MAX_04, KD_MIN_04, KD_MAX_04};
//MotorConfig motor2_cfg ;
//MotorConfig motor3_cfg ;
//MotorConfig motor4_cfg ;
/*
 * Helper_function
 */
const int can3_ids_bus[4] = {MOTOR_20, MOTOR_21, MOTOR_22, MOTOR_23};
const int can2_ids_bus[3] = {MOTOR_13, MOTOR_14, MOTOR_19};
const int can1_ids_bus[4] = {MOTOR_15,MOTOR_16,MOTOR_17,MOTOR_18};
const int can1_count = sizeof(can1_ids_bus) / sizeof(can1_ids_bus[0]);
const int can2_count = sizeof(can2_ids_bus) / sizeof(can2_ids_bus[0]);
const int can3_count = sizeof(can3_ids_bus) / sizeof(can3_ids_bus[0]);
uint32_t encode_can_id(CanId id) {
	return ((id.reserved & 0x07) << 29) | ((id.mode & 0x1F) << 24)
			| ((id.data & 0xFFFF) << 8) | ((id.targetMotorCanId & 0xFF));
}

int float_to_uint(float x, float x_min, float x_max, int bits) {
	float span = x_max - x_min;
	float offset = x_min;

	if (x > x_max)
		x = x_max;
	else if (x < x_min)
		x = x_min;
	return (int) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
}
float uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
	float span = x_max - x_min;
	return x_min + (x * span) / ((1 << bits) - 1);
}
void CAN_Transmit_1() {
//	if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData)
				!= HAL_OK) {
			//Error_Handler();
		}
//	}
}
void CAN_Transmit_2() {
	if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0) {
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData)
				!= HAL_OK) {
			//Error_Handler();
		}
	}
}
void CAN_Transmit_3() {
	if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) > 0) {
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData)
				!= HAL_OK) {
			//Error_Handler();
		}
	}
}

//////////////////
void EnableMotorX(uint16_t motor_can_id, uint8_t bus){
	CanId canid;
	canid.data = 0x00;
	canid.mode = MOTOR_ENABLE_MODE;
	canid.targetMotorCanId = motor_can_id;
	canid.reserved = 0x00;

	for(int i = 0; i < 8; i++){
		TxData[i] = 0;
	}
	uint32_t fullcanid = encode_can_id(canid);
	TxHeader.Identifier = fullcanid;

	if(bus == CANBUS_1){
		CAN_Transmit_1();

	}
	if(bus == CANBUS_2){
		CAN_Transmit_2();

	}
	if(bus == CANBUS_3){
		CAN_Transmit_3();
	}

}
/*
 * For unit test
 */
void EnableAllMotor(){
    for (int i = 0; i < can1_count; i++) {
    	EnableMotorX(can1_ids_bus[i], CANBUS_1);
    }
    for (int i = 0; i < can2_count; i++) {
    	EnableMotorX(can2_ids_bus[i], CANBUS_2);
    }
    for (int i = 0; i < can3_count; i++) {
    	EnableMotorX(can3_ids_bus[i], CANBUS_3);
    }
}
void DisableMotorX(uint16_t motor_can_id, uint8_t bus){
	CanId canid;
	canid.data = 0x00;
	canid.mode = MOTOR_DISABLE_MODE;
	canid.targetMotorCanId = motor_can_id;
	canid.reserved = 0x00;

	for(int i = 0; i < 8; i++){
		TxData[i] = 0;
	}
	uint32_t fullcanid = encode_can_id(canid);
	TxHeader.Identifier = fullcanid;

	if(bus == CANBUS_1){
		CAN_Transmit_1();

	}
	if(bus == CANBUS_2){
		CAN_Transmit_2();

	}
	if(bus == CANBUS_3){
		CAN_Transmit_3();
	}
}
void DisableAllMotor(){
    for (int i = 0; i < can1_count; i++) {
    	DisableMotorX(can1_ids_bus[i], CANBUS_1);
    }
    for (int i = 0; i < can2_count; i++) {
    	DisableMotorX(can2_ids_bus[i], CANBUS_2);
    }
    for (int i = 0; i < can3_count; i++) {
    	DisableMotorX(can3_ids_bus[i], CANBUS_3);
    }
}
//void MotorReceiveCommand(uint16_t motor_id, float torque, float position, float speed,
//		float kp, float kd, int canbus){
//	CanId canid;
//	canid.mode = 1;
//	canid.data = float_to_uint(torque, T_MIN_03, T_MAX_03, 16);
//	canid.reserved = 0;
//	canid.targetMotorCanId = motor_id;
//
//	uint16_t p = float_to_uint(position, P_MIN, P_MAX, 16);
//	uint16_t s = float_to_uint(speed, V_MIN_03, V_MIN_03, 16);
//    uint16_t kp_temp = float_to_uint(kp, KP_MIN_03, KP_MAX_03, 16);
//    uint16_t kd_temp = float_to_uint(kd, KD_MIN_03, KD_MAX_03, 16);
//	TxData[0] = (p >> 8) & 0xFF;
//	TxData[1] = p & 0xFF;
//	TxData[2] = (s >> 8) & 0xFF;
//	TxData[3] = s & 0xFF;
//	TxData[4] = (kp_temp >> 8) & 0xFF;
//	TxData[5] = kp_temp & 0xFF;
//	TxData[6] = (kd_temp >> 8) & 0xFF;
//	TxData[7] = kd_temp & 0xFF;
//
//	uint32_t fullcanid = encode_can_id(canid);
//	TxHeader.Identifier = fullcanid;
//
//	if(canbus == CANBUS_1){
//		CAN_Transmit_1();
//
//	}
//	if(canbus == CANBUS_2){
//		CAN_Transmit_2();
//
//	}
//	if(canbus == CANBUS_3){
//		CAN_Transmit_3();
//	}
//
//}
void MotorReceiveCommand(uint16_t motor_id, float torque, float position, float speed,
		float kp, float kd, int canbus, const MotorConfig *cfg){
	CanId canid;
	canid.mode = 1;
	canid.data = float_to_uint(torque, cfg->T_MIN, cfg->T_MAX, 16);
	canid.reserved = 0;
	canid.targetMotorCanId = motor_id;

    uint16_t p  = float_to_uint(position, P_MIN, P_MAX, 16);
    uint16_t v  = float_to_uint(speed, cfg->V_MIN, cfg->V_MAX, 16);
    uint16_t Kp = float_to_uint(kp, cfg->KP_MIN, cfg->KP_MAX, 16);
    uint16_t Kd = float_to_uint(kd, cfg->KD_MIN, cfg->KD_MAX, 16);
    TxData[0] = (p >> 8) & 0xFF;
    TxData[1] = p & 0xFF;
    TxData[2] = (v >> 8) & 0xFF;
    TxData[3] = v & 0xFF;
    TxData[4] = (Kp >> 8) & 0xFF;
    TxData[5] = Kp & 0xFF;
    TxData[6] = (Kd >> 8) & 0xFF;
    TxData[7] = Kd & 0xFF;
	uint32_t fullcanid = encode_can_id(canid);
	TxHeader.Identifier = fullcanid;

	if(canbus == CANBUS_1){
		CAN_Transmit_1();

	}
	if(canbus == CANBUS_2){
		CAN_Transmit_2();

	}
	if(canbus == CANBUS_3){
		CAN_Transmit_3();
	}

}
void TestMotorReceiveCommand(uint16_t motor_id, float torque, float position, float speed,
		float kp, float kd, int canbus){

	CanId canid;
	canid.mode = 1;
	canid.data = float_to_uint(torque, T_MIN_03, T_MAX_03, 16);
	canid.reserved = 0;
	canid.targetMotorCanId = motor_id;

	uint16_t p = float_to_uint(position, P_MIN, P_MAX, 16);
	uint16_t s = float_to_uint(speed, V_MIN_03, V_MIN_03, 16);
    uint16_t kp_temp = float_to_uint(kp, KP_MIN_03, KP_MAX_03, 16);
    uint16_t kd_temp = float_to_uint(kd, KD_MIN_03, KD_MAX_03, 16);
	TxData[0] = (p >> 8) & 0xFF;
	TxData[1] = p & 0xFF;
	TxData[2] = (s >> 8) & 0xFF;
	TxData[3] = s & 0xFF;
	TxData[4] = (kp_temp >> 8) & 0xFF;
	TxData[5] = kp_temp & 0xFF;
	TxData[6] = (kd_temp >> 8) & 0xFF;
	TxData[7] = kd_temp & 0xFF;

	uint32_t fullcanid = encode_can_id(canid);
	TxHeader.Identifier = fullcanid;

	CAN_Transmit_1();

}

void MotorResponse(uint8_t RxData[8], int motor_can_id){
	uint16_t currentAngle = (RxData[0] << 8) | RxData[1];
	uint16_t currentSpeed = (RxData[2] << 8) | RxData[3];
	uint16_t currentTorque = (RxData[4] << 8) | RxData[5];
	uint16_t currentTemperature = (RxData[6] << 8) | RxData[7];

	AngleCurrent[motor_can_id - 1] = uint_to_float(currentAngle, P_MIN, P_MAX, 16);
	SpeedCurrent[motor_can_id - 1] = uint_to_float(currentSpeed, V_MIN_03, V_MAX_03, 16);
	TorqueCurrent[motor_can_id - 1] = uint_to_float(currentTorque, T_MIN_03, T_MAX_03, 16);
	TempCurrent[motor_can_id - 1] = (currentTemperature) * 0.1;

}
uint8_t decode_can_id(uint32_t can_id) {
	uint8_t type_of_mode = (can_id >> 24) & 0x1F;
	uint8_t current_motor_id = (can_id >> 8) & 0xFF;

	return current_motor_id;


}
