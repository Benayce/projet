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
uint8_t fini = FALSE;


//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
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
    int speed_correction = 100;
    int count = 0;
    float angle = 0;

    while(1){
        time = chVTGetSystemTime();

        //angle = get_audio_max_float(15)*360/(2*M_PI)+90;

        //computes the speed to give to the motors
        if(speed_correction < 5 && !fini)
        {
			speed = pi_regulator(VL53L0X_get_dist_mm(), GOAL_DISTANCE);
			if(speed == 0)
			{
				fini = TRUE;
			}
		}


        if(count<100){speed_correction = 450;}
        if(count>=100 && count<200){speed_correction = 200;}
        if(count >= 200){speed_correction = 0;}
        count++;
        //speed_correction = angle;

        //applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
		left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);


        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
