#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

typedef enum {
ANGLE = 1 ,
FREQ

} BUFFER_NAME_t;

float Max_tableau(float*tab1);

void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/

float get_audio_float(BUFFER_NAME_t name);

void add_two_buffer(float *buffersortie,float *buffer2,float *buffer3,uint16_t size);




#endif /* AUDIO_PROCESSING_H */
