#include "Yin.h"
#include <math.h>
/* ------------------------------------------------------------------------------------------
--------------------------------------------------------------------------- PRIVATE FUNCTIONS
-------------------------------------------------------------------------------------------*/

// Define log2 function for ESP32 compatibility if not available
#ifndef log2
#define log2(x) (log(x) * M_LOG2E)
#endif

/**
 * Step 1: Calculates the squared difference of the signal with a shifted version of itself.
 * @param buffer Buffer of samples to process.
 *
 * This is the Yin algorithms tweak on autocorellation. Read http://audition.ens.fr/adc/pdf/2002_JASA_YIN.pdf
 * for more details on what is in here and why it's done this way.
 */
void Yin_difference(Yin *yin, uint8_t *buffer)
{
	const uint8_t *bufferIEnd = buffer + yin->halfBufferSize;
	/* Calculate the difference for difference shift values (tau) for the half of the samples */
	for (unsigned char tau = 0; tau < yin->halfBufferSize - 1; tau++)
	{
		uint32_t result = 0;
		for (uint8_t *bufferI = buffer; bufferI < bufferIEnd; bufferI += 4)
		{
			result += pow((int16_t)(*bufferI) - (int16_t)bufferI[tau], 2);
		}
		yin->yinBuffer[tau] = result >> 6;
	}
}

/**
 * Step 2: Calculate the cumulative mean on the normalised difference calculated in step 1
 * @param yin #Yin structure with information about the signal
 *
 * This goes through the Yin autocorellation values and finds out roughly where shift is which
 * produced the smallest difference
 */
void Yin_cumulativeMeanNormalizedDifference(Yin *yin)
{
	uint32_t runningSum = 0;
	yin->yinBuffer[0] = 1;

	/* Sum all the values in the autocorellation buffer and nomalise the result, replacing
	 * the value in the autocorellation buffer with a cumulative mean of the normalised difference */
	for (unsigned char tau = 1; tau < yin->halfBufferSize - 1; tau++)
	{
		runningSum += yin->yinBuffer[tau];
		yin->yinBuffer[tau] = (((uint32_t)yin->yinBuffer[tau] * tau) << 10) / runningSum;
	}
}

/**
 * Step 3: Search through the normalised cumulative mean array and find values that are over the threshold
 * @return Shift (tau) which caused the best approximate autocorellation. -1 if no suitable value is found over the threshold.
 */
int16_t Yin_absoluteThreshold(Yin *yin)
{
	unsigned char tau;

	/* Search through the array of cumulative mean values, and look for ones that are over the threshold
	 * The first two positions in yinBuffer are always so start at the third (index 2) */
	for (tau = 2; tau < yin->halfBufferSize - 1; tau++)
	{
		if (yin->yinBuffer[tau] < yin->threshold)
		{
			while (tau + 1 < yin->halfBufferSize && yin->yinBuffer[tau + 1] < yin->yinBuffer[tau])
			{
				tau++;
			}
			break;
		}
	}

	/* if no pitch found */
	if (tau >= yin->halfBufferSize - 1 || yin->yinBuffer[tau] >= yin->threshold)
	{
		return -1;
	}

	return tau;
}

/**
 * Step 5: Interpolate the shift value (tau) to improve the pitch estimate.
 * @param  yin         [description]
 * @param  tauEstimate [description]
 * @return             [description]
 *
 * The 'best' shift value for autocorellation is most likely not an interger shift of the signal.
 * As we only autocorellated using integer shifts we should check that there isn't a better fractional
 * shift value.
 */
float Yin_parabolicInterpolation(Yin *yin, int16_t tauEstimate)
{
	float betterTau;
	int16_t x0;
	int16_t x2;

	/* Calculate the first polynomial coeffcient based on the current estimate of tau */
	if (tauEstimate < 1)
	{
		x0 = tauEstimate;
	}
	else
	{
		x0 = tauEstimate - 1;
	}

	/* Calculate the second polynomial coeffcient based on the current estimate of tau */
	if (tauEstimate + 1 < yin->halfBufferSize)
	{
		x2 = tauEstimate + 1;
	}
	else
	{
		x2 = tauEstimate;
	}

	/* Algorithm to parabolically interpolate the shift value tau to find a better estimate */
	if (x0 == tauEstimate)
	{
		if (yin->yinBuffer[tauEstimate] <= yin->yinBuffer[x2])
		{
			betterTau = tauEstimate;
		}
		else
		{
			betterTau = x2;
		}
	}
	else if (x2 == tauEstimate)
	{
		if (yin->yinBuffer[tauEstimate] <= yin->yinBuffer[x0])
		{
			betterTau = tauEstimate;
		}
		else
		{
			betterTau = x0;
		}
	}
	else
	{
		float s0, s1, s2;
		s0 = yin->yinBuffer[x0];
		s1 = yin->yinBuffer[tauEstimate];
		s2 = yin->yinBuffer[x2];
		betterTau = tauEstimate + (s2 - s0) / (2 * (2 * s1 - s2 - s0));
	}

	return betterTau;
}

/* ------------------------------------------------------------------------------------------
---------------------------------------------------------------------------- PUBLIC FUNCTIONS
-------------------------------------------------------------------------------------------*/

/**
 * Initialise the Yin pitch detection object
 * @param yin        Yin pitch detection object to initialise
 * @param threshold  Allowed uncertainty (e.g 0.05 will return a pitch with ~95% probability)
 */
void Yin_init(Yin *yin, float threshold, uint16_t bufferSize)
{
	/* Initialise the fields of the Yin structure passed in */
	yin->bufferSize = bufferSize;
	yin->halfBufferSize = bufferSize / 2;
	yin->threshold = threshold * 1024;
	yin->numNaturalNotes = 7;
	yin->naturalNotes[0] = 0;  // C
	yin->naturalNotes[1] = 2;  // D
	yin->naturalNotes[2] = 4;  // E
	yin->naturalNotes[3] = 5;  // F
	yin->naturalNotes[4] = 7;  // G
	yin->naturalNotes[5] = 9;  // A
	yin->naturalNotes[6] = 11; // B
}

/**
 * Runs the Yin pitch detection algortihm
 * @param  yin    Initialised Yin object
 * @param  buffer Buffer of samples to analyse
 * @return        Fundamental frequency of the signal in Hz. Returns -1 if pitch can't be found
 */
float Yin_getPitch(Yin *yin, uint8_t *buffer)
{
	/* Step 1: Calculates the squared difference of the signal with a shifted version of itself. */
	Yin_difference(yin, buffer); // 1000

	/* Step 2: Calculate the cumulative mean on the normalised difference calculated in step 1 */
	Yin_cumulativeMeanNormalizedDifference(yin); // 16

	/* Step 3: Search through the normalised cumulative mean array and find values that are over the threshold */
	int16_t tauEstimate = Yin_absoluteThreshold(yin); // ~1

	/* Step 5: Interpolate the shift value (tau) to improve the pitch estimate. */
	if (tauEstimate != -1)
		return YIN_SAMPLING_RATE / Yin_parabolicInterpolation(yin, tauEstimate);

	return -1;
}

/**
 * Gets the nearest musical note frequency to a given frequency
 * @param frequency The frequency to find the nearest note for
 * @return The frequency of the nearest musical note
 */
float getNearestNoteFrequency(float frequency, Yin *yin)
{
	// Calculate the number of semitones from A4 (440 Hz)
	float semitonesFromA4 = 12.0f * log2(frequency / 440.0f);

	// Convert to semitones from C0 (C in octave 0)
	// A4 is 9 semitones above C4, and C4 is 4 octaves above C0
	// So A4 is 4*12 + 9 = 57 semitones above C0
	float semitonesFromC0 = semitonesFromA4 + 57.0f;

	// Find the octave (integer division)
	int octave = (int)(semitonesFromC0 / 12.0f);

	// Find the note within the octave (floating point position)
	float noteInOctave = semitonesFromC0 - (octave * 12.0f);

	// Find the closest natural note
	float minDistance = 12.0f; // Initialize with maximum possible distance
	int closestNoteOffset = 0; // Default to C

	for (int i = 0; i < 7; i++)
	{
		float distance = fabs(noteInOctave - yin->naturalNotes[i]);

		// Handle wrapping around the octave boundaries
		if (distance > 6.0f)
		{
			distance = 12.0f - distance;
		}

		if (distance < minDistance)
		{
			minDistance = distance;
			closestNoteOffset = yin->naturalNotes[i];
		}
	}

	// Calculate the semitones from A4 to the nearest natural note
	float semitonesToNearest = (octave * 12.0f) + closestNoteOffset - 57.0f;

	// Convert back to frequency (440 Hz * 2^(semitones/12))
	return 440.0f * pow(2.0f, semitonesToNearest / 12.0f);
}
