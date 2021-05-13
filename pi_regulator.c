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
#include <leds.h>

//////////////////////////////////////////

#define ROTATION	0
#define AVANCER		1
#define BUT			2
#define FIN			3
#define ETAPES		3
#define ON			1
#define OFF			0

///////////////////////////////////////



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
    uint8_t check_angle = FALSE;
    uint8_t etapes = 0;
    uint8_t led = 0;

    uint8_t state = 0;

    while(1){
        time = chVTGetSystemTime();
        
        if(!check_angle)
        {
        	set_body_led(1);
        	chThdSleepMilliseconds(5000);
        	wait_send_to_computer();
        	check_angle = TRUE;
        	set_body_led(0);
        }
        angle = get_angle();

        ///////////////////////////////////////////

        switch(state) {

           case ROTATION  :
        	   speed = 0;

               if(abs(angle) >= ANGLE_LIMITE)
               {
               	speed_correction = angle;
               }
               else
               {
               	speed_correction = 0;
               	state++;
               }
               if(speed_correction > 0)
               {
            	   set_led(LED7, ON);
            	   set_led(LED3, OFF);
               }
               else if(speed_correction < 0)
               {
            	   set_led(LED3, ON);
            	   set_led(LED7, OFF);
               }
              break; /* optional */

           case AVANCER  :
        	   speed_correction = 0;
        	   chprintf((BaseSequentialStream *)&SD3, "dist max %d\n", VL53L0X_get_dist_mm());
        	   speed = pi_regulator(VL53L0X_get_dist_mm(), GOAL_DISTANCE);

        	   if(led != ON)
        	   {
        		   led = ON;
        		   clear_leds();
        		   set_led(LED1, ON);
        	   }

        	   if(speed == 0)
        	   {
        		   state++;
        	   }

              break; /* optional */
           case BUT		:
        	   speed = 0;
        	   speed_correction = 0;
        	   if(led == ON)
        	   {
        		   led = OFF;
        		   set_led(LED1, OFF);
        		   set_body_led(ON);
        	   }
        	   chThdSleepMilliseconds(5000);
    		   set_body_led(OFF);

        	   if(etapes < ETAPES)
        	   {
        		   state = 0;
        		   etapes++;
        	   }
        	   else
        	   {
        		   state++;
        	   }

        	   break;
           case FIN		:
        	   speed = 0;
        	   speed_correction = 0;

        	   break;
        }

        /////////////////////////////////////////////

//        speed_correction = 0;
//        speed = 0;
		right_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
		left_motor_set_speed(speed - ROTATION_COEFF * speed_correction);


        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
