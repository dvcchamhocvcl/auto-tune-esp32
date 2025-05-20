#ifndef Yin_h
#define Yin_h
#include <stdint.h>
#include <stdbool.h>
#define BUFFER_SIZE 512
#define HALF_BUFFER_SIZE 256

typedef struct
{
	uint8_t scale[7];
	uint16_t yinBuffer[HALF_BUFFER_SIZE]; /**< Buffer that stores the results of the intermediate processing steps of the algorithm */
	uint16_t threshold;					  /**< Allowed uncertainty in the result as a decimal (i.e 0.15 is 15%) */
	uint16_t samplingRate;
} Yin;

void Yin_init(Yin *yin, float threshold, uint16_t samplingRate);
void shift_scale_down(Yin *yin);
void shift_scale_up(Yin *yin);
float Yin_getPitch(Yin *yin, uint8_t *buffer);
float getNearestNoteFrequency(float frequency, Yin *yin, uint8_t tone_selection);
#endif
