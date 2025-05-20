#include "Yin.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
void Yin_difference(Yin *yin, uint8_t *buffer)
{
	const uint8_t *bufferIEnd = buffer + HALF_BUFFER_SIZE;
	for (unsigned char tau = 0; tau < HALF_BUFFER_SIZE - 1; tau++)
	{
		uint32_t result = 0;
		for (uint8_t *bufferI = buffer; bufferI < bufferIEnd; bufferI += 4)
		{
			result += pow((int16_t)(*bufferI) - (int16_t)bufferI[tau], 2);
		}
		yin->yinBuffer[tau] = result >> 6;
	}
}

void Yin_cumulativeMeanNormalizedDifference(Yin *yin)
{
	uint32_t runningSum = 0;
	yin->yinBuffer[0] = 1;
	for (unsigned char tau = 1; tau < HALF_BUFFER_SIZE - 1; tau++)
	{
		runningSum += yin->yinBuffer[tau];
		yin->yinBuffer[tau] = (((uint32_t)yin->yinBuffer[tau] * tau) << 10) / runningSum;
	}
}

int16_t Yin_absoluteThreshold(Yin *yin)
{
	unsigned char tau;

	/* Search through the array of cumulative mean values, and look for ones that are over the threshold
	 * The first two positions in yinBuffer are always so start at the third (index 2) */
	for (tau = 2; tau < HALF_BUFFER_SIZE - 1; tau++)
	{
		if (yin->yinBuffer[tau] < yin->threshold)
		{
			while (tau + 1 < HALF_BUFFER_SIZE && yin->yinBuffer[tau + 1] < yin->yinBuffer[tau])
			{
				tau++;
			}
			break;
		}
	}

	/* if no pitch found */
	if (tau >= HALF_BUFFER_SIZE - 1 || yin->yinBuffer[tau] >= yin->threshold)
	{
		return -1;
	}

	return tau;
}

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
	if (tauEstimate + 1 < HALF_BUFFER_SIZE)
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

void Yin_init(Yin *yin, float threshold, uint16_t samplingRate)
{
	yin->scale[0] = 1;
	yin->scale[1] = 2;
	yin->scale[2] = 4;
	yin->scale[3] = 6;
	yin->scale[4] = 8;
	yin->scale[5] = 9;
	yin->scale[6] = 11;
	yin->threshold = threshold * 1024;
	yin->samplingRate = samplingRate;
}

void shift_scale_up(Yin *yin)
{
	if (yin->scale[6] == 11)
	{
		for (int i = 6; i > 0; i--)
		{
			yin->scale[i] = yin->scale[i - 1] + 1;
		}
		yin->scale[0] = 0;
	}
	else
	{
		for (int i = 0; i < 7; i++)
		{
			yin->scale[i] = yin->scale[i] + 1;
		}
	}
}
void shift_scale_down(Yin *yin)
{
	if (yin->scale[0] == 0)
	{
		for (int i = 0; i < 6; i++)
		{
			yin->scale[i] = yin->scale[i + 1] - 1;
		}
		yin->scale[6] = 11;
	}
	else
	{
		for (int i = 0; i < 7; i++)
		{
			yin->scale[i] = yin->scale[i] - 1;
		}
	}
}
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
		return yin->samplingRate / Yin_parabolicInterpolation(yin, tauEstimate);
	return -1;
}

float getNearestNoteFrequency(float frequency, Yin *yin, uint8_t scale)
{
	float semitonesFromA4 = 12.0f * log2(frequency / 440.0f);

	float semitonesFromC0 = semitonesFromA4 + 57.0f;
	int octave = (int)(semitonesFromC0 / 12.0f);
	float noteInOctave = semitonesFromC0 - (octave * 12.0f);
	float minDistance = 12.0f; // Initialize with maximum possible distance
	int closestNoteOffset = 0; // Default to C

	for (int i = 0; i < 7; i++)
	{
		float distance = fabs(noteInOctave - yin->scale[i]);
		if (distance > 6.0f)
		{
			distance = 12.0f - distance;
		}

		if (distance < minDistance)
		{
			minDistance = distance;
			closestNoteOffset = yin->scale[i];
		}
	}
	float semitonesToNearest = (octave * 12.0f) + closestNoteOffset - 57.0f;
	return 440.0f * pow(2.0f, semitonesToNearest / 12.0f);
}
