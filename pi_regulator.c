#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <sensors\VL53L0X\VL53L0X.h>
#include <pi_regulator.h>
#include <son.h>

#define GOAL_DISTANCE 100	//10cm
#define TRUE 1
#define FALSE 0
#define WHEEL_DISTANCE      53.5f    //mm
#define PERIMETER_EPUCK     (M_PI * WHEEL_DISTANCE)


//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;
    uint8_t direction = FALSE;

    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        //only active once direction has been adapted
        if(direction)
        {
			speed = pi_regulator(VL53L0X_get_dist_mm(), GOAL_DISTANCE);
		}

        //computes a correction factor to let the robot rotate to be in front of the source
        //once at start to turn to sound source
        if(!direction)
        {
	        speed_correction = 360/get_angle();
	        left_motor_set_pos(-PERIMETER_EPUCK/speed_correction);
	        right_motor_set_pos(PERIMETER_EPUCK/speed_correction);
	        direction = TRUE;
		}

        //applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(speed);
		left_motor_set_speed(speed);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
