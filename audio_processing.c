#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include "arm_math.h"
#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <arm_math.h> //pas sur
#include <arm_const_structs.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micLR_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
	static float micLeft_output[FFT_SIZE];
	static float micRight_output[FFT_SIZE];

static float micLR_output[FFT_SIZE];
static float l_max;
static float r_max;
static float lr_max;

#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing

float Max_tableau(float*tab1, uint16_t size){
	float max=0;
	for (int i=0;i<size;i++){
		if (max < tab1[i]){
			max=tab1[i];
		}
	}
	return(max);
	}

void doFFT_optimized(uint16_t size, float* complex_buffer){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
	}


/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/

void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		//micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		//micLeft_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/* */
		//right right avec décalage
		add_two_buffer(micLR_cmplx_input,micRight_cmplx_input,micRight_cmplx_input,2*FFT_SIZE);

		// Left right Attention sans décalage
		//add_two_buffer(micLR_cmplx_input,micRight_cmplx_input,micLeft_cmplx_input,2*FFT_SIZE);

		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micLR_cmplx_input);

				doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
				//doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
				arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
			//	arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micLR_cmplx_input, micLR_output, FFT_SIZE);


		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3

		if(mustSend > 8){
			//signals to send the result to the computer
			l_max= Max_tableau(micLeft_output,  FFT_SIZE);
			r_max=Max_tableau(micRight_output,  FFT_SIZE);
			lr_max=Max_tableau(micLR_output,  FFT_SIZE);
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;

		//sound_remote(micLeft_output);
	}
}

void add_two_buffer(float *buffersortie,float *buffer2,float *buffer3,uint16_t size){
	for (int i=0;i<size;i++){
		int decalage =25;
		if (i+decalage<size){
			buffersortie[i]=(float)(buffer2[i] + buffer3[i+decalage])/2;
		}
		else{
			buffersortie[i]=0;//buffer2[i] ;
		}
	}

}


void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	/*if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else*/ if (name == LR_CMPLX_INPUT){
				return micLR_cmplx_input;
	}
	/*else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}*/
	else if (name == LR_OUTPUT){
			return micLR_output;
		}

	else{
		return NULL;
	}
}

float get_audio_max_float(BUFFER_NAME_t name){
 if (name == L_MAX){
			return l_max;
		}
else if (name == R_MAX){
			return r_max;
		}
else if (name == LR_MAX){
			return lr_max;
		}
else{
	return 0;
}
}
