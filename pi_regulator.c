/*Pour cette partie nous avons réutilisé le code du TP4
 * lequel nous avons modifié pour notre projet
 *
 */

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <audio_processing.h>

#include <main.h>
#include <motors.h>
#include <sensors\VL53L0X\VL53L0X.h>
#include <pi_regulator.h>




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
    float angle = 100;
    uint8_t fini = FALSE;
    uint8_t check_angle = FALSE;
    uint8_t but = FALSE;
    uint8_t rotation = TRUE;
    uint8_t etapes = 0;

    while(1){
        time = chVTGetSystemTime();
        
        if(!check_angle)
        {
        	wait_send_to_computer();
        	check_angle = TRUE;
        }
        angle = get_angle();

        //computes the speed to give to the motors
        //only active once direction has been adapted
        if(/*abs(angle) < ANGLE_LIMITE && */!rotation && !but && !fini)
        {
			speed = pi_regulator(VL53L0X_get_dist_mm(), GOAL_DISTANCE);

			if(speed == 0){but = TRUE;}
		}
        else
        {
        	speed = 0;
        }

        if(abs(angle)>= ANGLE_LIMITE && !fini && rotation)
        {
        	speed_correction = angle;
        }
        else
        {
        	speed_correction = 0;
        	rotation = FALSE;
        }


        if(but == TRUE)
        {
        	etapes++;
        	but = FALSE;
        	rotation = TRUE;
        	chThdSleepMilliseconds(5000);
        }

        if(etapes >= 3)
        {
        	fini = TRUE;
        }

        speed_correction = 0;
        speed = 0;
		right_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
		left_motor_set_speed(speed - ROTATION_COEFF * speed_correction);


        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
