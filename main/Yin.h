#ifndef Yin_h
#define Yin_h

#include <stdint.h>
#include <stdbool.h>

// Sampling rate set to match ESP32 ADC configuration

// For best performance with ESP32, keeping the buffer sizes the same
#define BUFFER_SIZE 512
#define HALF_BUFFER_SIZE 512

/**
 * @struct  Yin
 * @brief   Object to encapsulate the parameters for the Yin pitch detection algorithm
 */

typedef struct
{
	uint16_t naturalNotes[7];
	uint16_t numNaturalNotes;
	uint16_t yinBuffer[HALF_BUFFER_SIZE]; /**< Buffer that stores the results of the intermediate processing steps of the algorithm */
	uint16_t threshold;					  /**< Allowed uncertainty in the result as a decimal (i.e 0.15 is 15%) */
	uint16_t samplingRate;
} Yin;

/**
 * Initialise the Yin pitch detection object
 * @param yin        Yin pitch detection object to initialise
 * @param threshold  Allowed uncertainty (e.g 0.05 will return a pitch with ~95% probability)
 */
void Yin_init(Yin *yin, float threshold, uint16_t samplingRate);

/**
 * Runs the Yin pitch detection algorithm
 * @param  yin    Initialised Yin object
 * @param  buffer Buffer of samples to analyse
 * @return        Fundamental frequency of the signal in Hz. Returns -1 if pitch can't be found
 */
float Yin_getPitch(Yin *yin, uint8_t *buffer);

/**
 * Gets the nearest musical note frequency to a given frequency
 * @param frequency The frequency to find the nearest note for
 * @return The frequency of the nearest musical note
 */
float getNearestNoteFrequency(float frequency, Yin *yin);

#endif
