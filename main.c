#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <audio_processing.h>
#include <arm_math.h>
#include <audio/microphone.h>
#include <pi_regulator.h>

static float angle = 100;

float get_angle(void)
{
	return angle;
}

void SendUint16ToComputer(uint16_t data1)
{
	chprintf((BaseSequentialStream *)&SD3, "distance : %d \n",data1);
}


static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}



int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //start sensor time to fligh
	VL53L0X_start();

	//inits the motors
	motors_init();

	pi_regulator_start();
	 //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);


    /* Infinite loop. */
    while (1) {

    		//test du capteur de distance
    		////SendUint16ToComputer(VL53L0X_get_dist_mm());
    		//test FFT et microphone
        wait_send_to_computer();
        //chprintf((BaseSequentialStream *)&SD3, " Pos max L = %d  ",get_pos_max());
        //chprintf((BaseSequentialStream *)&SD3, " Pos max R = %d  ",get_pos_max());
        /*chprintf((BaseSequentialStream *)&SD3, " diff de phase = %f  \n ",get_audio_max_float(13));
        chprintf((BaseSequentialStream *)&SD3, " diff de phase rad = %f  \n ",get_audio_max_float(14));
        chprintf((BaseSequentialStream *)&SD3, " phi !!!  = %f  \n ",get_audio_max_float(16));
        chprintf((BaseSequentialStream *)&SD3, " calcul !!!  = %f  \n ",get_audio_max_float(17));*/
        chprintf((BaseSequentialStream *)&SD3, " ANGLE !!!  = %f  \n ",get_audio_max_float(15)*360/(2*M_PI)+90);

        angle = get_audio_max_float(15)*360/(2*M_PI)+90;
        chThdSleepMilliseconds(1000);
   }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}



