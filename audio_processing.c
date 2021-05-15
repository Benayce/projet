#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>

#include <chprintf.h>

#include "arm_math.h"
#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include <pi_regulator.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);


//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];


//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];

static float Angle = 0;
static int  pos_l_max;

#define MIN_VALUE_THRESHOLD	20000

//Defini arbitrairement Frequence et longeur associé
#define Fson1	1500		//Hz
#define L1		0.1		//mètre
#define Fson2	2000
#define L2		0.2
#define Fson3	3000
#define L3		0.3

#define Vson				340 			// vitesse du son en m/s
#define d				0.03	 		// distance entre les 2 micros diviser par 2
#define pas_freq 		15.6 		// 15,6 Hz de difference entre 2 indices des datas frequencielles
#define BEFORE_SEND		4			//nbr de called back de la fonction avant d'update l'angle
#define INITDEPH			10*M_PI

// Fonction à pour but de retourner la frequence dominante d'un tab de frequence
// si l'amplitude du signal à cette frequence est assez grande renvoie la position sinon renvoie 0

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
//INITIALISATION des variable
	if(get_state() == ROTATION)
	{
		 int pos_r_max;
		 // initialiser à une valeur superieur > 2_PI
		 float dif_phase_rad= INITDEPH;
		 float phi;

		/*
		*
		*	We get 160 samples per mic every 10ms
		*	So we fill the samples buffers to reach
		*	1024 samples, then we compute the FFTs.
		*
		*/

		static uint16_t nb_samples = 0;
		static uint8_t mustSend = 0;

	//ETAPE 0 loop to fill the buffers

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


	//ETAPE 1 CALCULER LES FFT

			doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
			doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);

	//ETAPE 2 CALCULER LES Magnitudes et les frequences maximales (pos_X_max)

			arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
			pos_r_max =Max_tableau( micRight_output, FFT_SIZE);
			arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
			pos_l_max = Max_tableau (micLeft_output, FFT_SIZE);

	// ETAPE 3 SI les 2 micros ont la meme frequence max (!=0) => calculer la difference de phase


			if(mustSend > BEFORE_SEND){
				if((pos_r_max == pos_l_max) && (pos_r_max > 10)){//magic number

					dif_phase_rad = atan(micLeft_cmplx_input[pos_l_max*2+1] / micLeft_cmplx_input[pos_l_max*2])
							-atan(micRight_cmplx_input[pos_r_max*2+1] / micRight_cmplx_input[pos_r_max*2]);
					Angle = 0;
				}
				else {Angle = PROBLEMEANGLE;}//Problème detecté On arrrete le robot

	// ETAPE 4 Fixer la variable L à partir de l'information de frequence

				float L=L2; // Valeur d'initialisation à SON 2
				int freq_position = (pos_l_max-2) *pas_freq ; // -2 pour que ça joue ?
				int Fson= Fson2;
				if ( (freq_position-pas_freq) < Fson1 && (freq_position+pas_freq) > Fson1 ){
								L= L1;
								Fson=Fson1;
							}
							else if ((freq_position-2*pas_freq) < Fson1 && (freq_position+pas_freq) > Fson1 ){
								L= L2;
								Fson=Fson2;
							}
							else if ((freq_position-2*pas_freq) < Fson1 && (freq_position+pas_freq) > Fson1){
								L= L3;
								Fson=Fson3;
							}


	// ETAPE 5 transformer le dephasage ang en ANGLE de sortie
	// (RAPPEL on veut des angles d'entrée dans la range [-PI/2,PI/2] )

	// Si les angles sont bien dans la range calculer la position angulaire de la cible (Angle)

				if (dif_phase_rad > M_PI/2 || dif_phase_rad < -M_PI/2){}
				else if (Angle == PROBLEMEANGLE){chBSemSignal(&sendToComputer_sem);}
				else
				{
					phi = (dif_phase_rad * Vson / (Fson*2*M_PI)); // simplifier en créant une const Vson/2_MPI
					Angle =  -acos(phi*sqrt(4*L*L + 4*d*d - phi*phi)/(4*L*d));
		//			chprintf((BaseSequentialStream*)&SD3,"Audio %f", Angle);
					//envoi le signal que l'angle est pret
					chBSemSignal(&sendToComputer_sem);
					mustSend = 0;

				}
			}
			nb_samples = 0;
			mustSend++;

		}
	}
	else{}
}


void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float get_audio_float(BUFFER_NAME_t name){
	if (name == ANGLE){
				return Angle;
	}
	else if (name == FREQ){
	return (pos_l_max)*pas_freq;
			}
	else{
		return 0;
	}
}

