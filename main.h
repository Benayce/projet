#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project

#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2 
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			5.0f	//[mm] because of the noise of the camera
#define KP						30.0f
#define KI 						0.5f	//must not be zero
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
