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
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <audio_processing.h>
#include <arm_math.h>
#include <audio/microphone.h>
#include <pi_regulator.h>

static float angle = 45;

float get_angle(void)
{
	return angle;
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
    while (TRUE) {


    	//récupération de l'angle
        wait_send_to_computer();

        angle = get_audio_max_float(15)*360/(2*M_PI)+90;
        chprintf((BaseSequentialStream*)&SD3,"Angle %f", angle);
        chThdSleepMilliseconds(3000);
   }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}



