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

static    uint8_t state = 0;

uint8_t get_state()
{
	return state;
}


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
    uint8_t avant = OFF;
    uint8_t droite = OFF;
    uint8_t gauche = OFF;
    uint8_t arriere = OFF;
    int dist = 0;



    while(1){
        time = chVTGetSystemTime();
        
        //Cela nous donne du temps pour preparer le robot
        if(!check_angle)
        {
        	set_body_led(ON);
        	chThdSleepMilliseconds(5000);
        	//wait_send_to_computer();
        	check_angle = TRUE;
        	set_body_led(OFF);
        }



        //chprintf((BaseSequentialStream *)&SD3, "angle envoye : %f ; angle but : %f \n", angle, PROBLEMEANGLE*RAD2DEG+DECALAGE90DEGRE);
        switch(state) {

           case ROTATION  :
        	   speed = 0;
               wait_send_to_computer();
               angle = get_audio_float(ANGLE)*RAD2DEG+DECALAGE90DEGRE;
        	   //Si l'angle nous indique que pas de son est detecte, on passe au state but
        	   //qui lui va renvoyer ici avec un nouvel angle. Cela compte comme un but intermediaire
        	   if(angle == PAS2SON)
        	   {
        		   state = BUT;
        		   speed_correction = 0;
        	   }
        	   //Tant que l'angle calcule n'est pas satisfaisant, on continue de le corriger
        	   else if(abs(angle) >= ANGLE_LIMITE)
               {
               	speed_correction = angle;
               }
               //Si l'angle calcule est satisfaisant, on ne le corrige plus et on va au prochain etat
               else
               {
               	speed_correction = 0;
               	state++;
               }
               //Si le robot fait un virage a gauche, la led gauche s'allume
               if(speed_correction > 0 && !gauche)
               {
            	   clear_leds();
            	   gauche = ON;
            	   droite = OFF;
            	   set_led(LED7, gauche);
            	   set_led(LED3, droite);
               }
               //Si le robot fait un virage a droite la led droite s'allume
               else if(speed_correction < 0 && !droite)
               {
            	   clear_leds();
            	   gauche = OFF;
            	   droite = ON;
            	   set_led(LED3, droite);
            	   set_led(LED7, gauche);
               }
              break; /* optional */

           case AVANCER  :
        	   speed_correction = 0;	//on ne veut plus corriger l'angle
        	   dist = VL53L0X_get_dist_mm();

        	   //Tant que le robot voit une cible a moins que 90 cm, on applique le regulateur pi
        	   //et la led avant s'allume
        	   if(dist < DIST_MAX)
        	   {
        		   speed = pi_regulator(dist, GOAL_DISTANCE);
            	   if(avant != ON || arriere == ON)
            	   {
            		   avant = ON;
            		   arriere = OFF;
            		   clear_leds();
            		   set_led(LED1, avant);
            	   }
        	   }
        	   //Si le robot ne voit pas de cible parce qu'il l'a loupe, ou csi la cible a bouge
        	   // il s'arrete et allume les leds avant et arriere
        	   else
        	   {
        		   speed = 0;
        		   avant = ON;
        		   arriere = ON;
        		   clear_leds();
        		   set_led(LED1, avant);
        		   set_led(LED5, arriere);
        	   }

        	   //Si le robot ne bouge plus parce qu'il a atteint le but, on change d'etat
        	   if(speed == 0 && arriere != ON)
        	   {
        		   state++;
        	   }

              break; /* optional */

           case BUT		:
        	   //Le robot a atteint sa cible, il ne bouge plus
        	   speed = 0;
        	   speed_correction = 0;

        	   //On allume la body led pendant la pause du robot
        	   if(avant == ON)
        	   {
        		   clear_leds();
        		   set_body_led(ON);
        	   }
        	   chThdSleepMilliseconds(5000);
    		   set_body_led(OFF);

    		   //Si on a pas encore atteint le nombre d'etapes predefinies on retourne
    		   //a l'etat rotation
        	   if(etapes < ETAPES)
        	   {
        		   state = ROTATION;
        		   etapes++;
        	   }
        	   //Si on a atteint le nombre d'etapes predefinies on passe a l'etat fini
        	   else
        	   {
        		   state++;
        	   }

        	   break;

           case FIN		:
        	   //Le robot a fini son travail et ne bouge plus, cela est indique par l'absence de leds
        	   speed = 0;
        	   speed_correction = 0;

        	   break;
        }
        //La vitesse du robot est calculee par le regulateur PI plus haut; la correction de l'angle
        //est essentiellement un regulateur P
        chprintf((BaseSequentialStream*)&SD3,"State %d", state);
		right_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
		left_motor_set_speed(speed - ROTATION_COEFF * speed_correction);


        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
