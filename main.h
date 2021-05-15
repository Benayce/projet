#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define ROTATION_COEFF			2
#define GOAL_DISTANCE 			100	//10cm
#define TRUE 					1
#define FALSE 					0
#define ERROR_THRESHOLD			5.0f	//[mm]
#define KP						10.0f
#define KI 						0.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define ANGLE_LIMITE			2	// deg
#define ROTATION	0
#define AVANCER		1
#define BUT			2
#define FIN			3
#define ETAPES		3
#define ON			1
#define OFF			0
#define DIST_MAX	600		//[mm]
#define DECALAGE90DEGRE 90
#define RAD2DEG 			360/(2*M_PI)
#define PROBLEME			4*M_PI
#define PROBLEMEANGLE	PROBLEME
#define PROBLEMEFREQ	PROBLEME

#define PAS2SON			810 //PROBLEMEANGLE*RAD2DEG+DECALGE90DEGRE

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
