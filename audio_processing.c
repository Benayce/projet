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
//static float micLR_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];

static float l_max;
static float r_max;
static int pos_l_max;
static int pos_r_max;
static float dif_phase;
static float dif_phase_rad;
static float Angle;
static float calcul;
static float phi;

#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing

float Max_tableau(float*tab1, uint16_t size){
	float max=0;
	int pos_max =0;
	for (int i=0;i<size;i++){
		if (max < tab1[i]){
			max=tab1[i];
			pos_max = i;
		}
	}
	if (max > MIN_VALUE_THRESHOLD){
	return(pos_max);}
	else {return 0;}
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
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){

		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/


//ETAPE 1 CALCULER LES FFT
		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);

//ETAPE 2 CALCULER LES Magnitudes et verifier si sur la meme frequence
		// Pose max => frequence

		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		pos_r_max =Max_tableau( micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		pos_l_max = Max_tableau (micLeft_output, FFT_SIZE);

// ETAPE 3 SI meme frequence reconnu regarder la difference de phase



		if(mustSend > 8){
			if((pos_r_max == pos_l_max) && (pos_r_max != 0)){
				dif_phase_rad = atan(micLeft_cmplx_input[pos_l_max*2+1] / micLeft_cmplx_input[pos_l_max*2])
						-atan(micRight_cmplx_input[pos_r_max*2+1] / micRight_cmplx_input[pos_r_max*2]);}
			dif_phase= dif_phase_rad*57.29f;
			//on a la dif de phase => avec la freq on

// ETAPE 4 transformer dephasage ang en ANGLE de sortie

			float L=0.14; // varie selon amplitude choix personnel
			float d= 0.06 / 2; // constante dist entre micro par 2
			int Fson = 1500; // Hz varie determiner par la frequence a terme
			int Vson= 340; // CONST m/s
// Problème de modulo de la diph de phase on veut entre -pi/2 et pi/2 le reste osef
			if (dif_phase_rad > M_PI/2){
				dif_phase =- 2*M_PI;
			}
			else if (dif_phase_rad < -M_PI/2){
				dif_phase =+ 2*M_PI;
				}
			if (dif_phase_rad > M_PI/2 || dif_phase_rad < -M_PI/2){}
			else{
			 phi = (dif_phase_rad * Vson / (Fson*2*M_PI));
			 calcul = phi*sqrt(4*L*L + 4*d*d - phi*phi);
			 Angle =  -acos(phi*sqrt(4*L*L + 4*d*d - phi*phi)/(4*L*d));


			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
			}
		}
		nb_samples = 0;
		mustSend++;

		//sound_remote(micLeft_output);
	}
}


void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else{
		return NULL;
	}
}

void phase_from_cmplx(){

	dif_phase_rad = atan(micLeft_cmplx_input[pos_l_max*2+1] / micLeft_cmplx_input[pos_l_max*2])-atan(micRight_cmplx_input[pos_r_max*2+1] / micRight_cmplx_input[pos_r_max*2]);
	dif_phase= dif_phase_rad*57.29f;
}


float get_audio_max_float(BUFFER_NAME_t name){
 if (name == L_MAX){
			return l_max;
		}
else if (name == R_MAX){
			return r_max;
		}
else if (name == 13){
			return dif_phase;
		}
else if (name == 14){
			return dif_phase_rad;
		}
else if (name == 15){
			return Angle;
		}
else if (name == 16){
			return phi;
		}
else if (name == 17){
			return calcul;
		}
else{
	return 0;
}
}
int get_pos_max()
{
	return pos_l_max;
}
