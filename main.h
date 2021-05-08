#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

float get_angle(void);

//constants for the differents parts of the project
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define GOAL_DISTANCE 			100	//10cm
#define TRUE 					1
#define FALSE 					0
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			5.0f	//[mm]
#define KP						10.0f
#define KI 						0.1f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);
void SendInt16ToComputer(char* nom, int16_t* data);
void SendFloatToComputer(char* nom, float* data);

#ifdef __cplusplus
}
#endif

#endif
