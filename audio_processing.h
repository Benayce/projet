#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT,
	LR_CMPLX_INPUT,
	LR_OUTPUT,
	L_MAX,
	R_MAX,
	LR_MAX

} BUFFER_NAME_t;

float Max_tableau(float*tab1, uint16_t size);

void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

float get_audio_max_float(BUFFER_NAME_t name);

void add_two_buffer(float *buffersortie,float *buffer2,float *buffer3,uint16_t size);

void phase_from_cmplx(void);

int get_pos_max(void);


#endif /* AUDIO_PROCESSING_H */
