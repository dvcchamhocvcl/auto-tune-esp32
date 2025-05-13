#include <stdio.h>             /** C standard library */
#include <math.h>              /** Math functions */
#include "freertos/FreeRTOS.h" /** FreeRTOS libraries (tasks, delays, etc.) */
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h" /** GPIO control library */
#include <driver/dac.h>
#include <driver/adc.h>
#include "driver/i2s.h"
#include "driver/dac.h"
#include "esp_adc_cal.h"
#include "esp_task_wdt.h"
#include "Yin.h" /** Yin pitch detection algorithm */

// GPIO pins
#define MIC_INPUT_PIN 32
#define I2S_NUM I2S_NUM_0
#define SAMPLE_RATE 44100
#define I2S_DOUT 25
#define I2S_BCLK 26
#define I2S_LRC 27
#define MODE_SWITCH 4 // GPIO4 for mode switch (can be adjusted)

// For pitch detection and correction
#define PROCESSING_BUFFER_SIZE 512
#define PITCH_DETECTION_THRESHOLD 0.05f // Adjustable based on desired detection sensitivity
#define MIN_VALID_FREQUENCY 78.0f       // Minimum frequency to consider valid (in Hz)

typedef enum
{
    MODE_AUTOTUNE = 0, // Automatic pitch correction
    MODE_BYPASS = 1    // Manual pitch shifting
} OperationMode;

// Global variables
static Yin yinDetector;                                  // Yin pitch detector
static uint8_t processingBuffer[PROCESSING_BUFFER_SIZE]; // Buffer for audio analysis
static volatile bool bufferFilled = false;
static volatile OperationMode currentMode = MODE_AUTOTUNE;
static int8_t manualPitchShift = 0; // -128 to 127 for manual pitch shift
static QueueHandle_t audioQueue;    // Queue for audio samples

// Task for pitch processing
void PitchProcessTask(void *pvParams)
{
    esp_task_wdt_add(NULL);

    uint8_t playbackSpeed = 128; // Neutral speed (no shift)
    float targetFrequency = 0.0f;
    float frequency = 0.0f;
    while (1)
    {
        // Wait for buffer to be filled with audio samples
        if (bufferFilled)
        {
            // Detect pitch using Yin algorithm
            frequency = Yin_getPitch(&yinDetector, processingBuffer);

            // Only use valid detected frequencies
            if (frequency >= MIN_VALID_FREQUENCY)
            {
                // In autotune mode, correct pitch to nearest note
                if (currentMode == MODE_AUTOTUNE)
                {
                    // Find the nearest musical note frequency
                    targetFrequency = getNearestNoteFrequency(frequency, &yinDetector);

                    // Adjust playback speed to match the target frequency
                    // Ratio is target/current, scaled to 8-bit unsigned range around 128 (neutral)
                    playbackSpeed = (uint8_t)fmin(255, fmax(1, round(targetFrequency * 128.0 / frequency) + manualPitchShift));
                }
            }
            else
            {
                // No valid pitch detected, use manual shift only
                playbackSpeed = (uint8_t)fmin(255, fmax(1, 128 + manualPitchShift));
            }

            // In manual shift mode
            if (currentMode == MODE_BYPASS)
            {
                playbackSpeed = (uint8_t)fmin(255, fmax(1, 128 + manualPitchShift));
            }

            // Reset buffer filled flag for next cycle
            bufferFilled = false;

            // Debug output
            printf("Freq: %.2f, Target: %.2f, Speed: %d\n", frequency, targetFrequency, playbackSpeed);
        }

        // Send current playback speed to audio processing queue
        xQueueSend(audioQueue, &playbackSpeed, 0);

        // Give time for other tasks
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz update rate
        esp_task_wdt_reset();
    }
}

// Task for reading the mode switch
void SwitchReadTask(void *pvParams)
{
    // Configure GPIO for mode switch
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MODE_SWITCH),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    while (1)
    {
        // Read switch state
        if (gpio_get_level(MODE_SWITCH) == 0)
        {
            currentMode = MODE_AUTOTUNE;
        }
        else
        {
            currentMode = MODE_BYPASS;
        }

        // Check switch every 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Original mic reading task, modified to include pitch shifting
void MicRead(void *pvParams)
{
    esp_task_wdt_add(NULL);
    dac_output_enable(DAC_CHANNEL_1);

    // Create circular buffers for audio processing
    uint8_t *recordBuffer = (uint8_t *)malloc(PROCESSING_BUFFER_SIZE * sizeof(uint8_t));
    if (recordBuffer == NULL)
    {
        // Handle allocation failure
        vTaskDelete(NULL);
        return;
    }

    static uint16_t inputPosition = 0;
    static uint16_t outputPosition = 0;
    static uint16_t processingPosition = 0;
    uint8_t currentPlaybackSpeed = 128; // Start with neutral speed

    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);

    while (1)
    {
        // Read audio sample from ADC
        int rawValue = adc1_get_raw(ADC1_CHANNEL_4);
        uint8_t soundSample = (uint8_t)(rawValue >> 4); // Convert to 8-bit

        // Store in record buffer
        recordBuffer[inputPosition] = soundSample;
        inputPosition = (inputPosition + 1) & (PROCESSING_BUFFER_SIZE - 1); // Wrap around

        // Adjust playback position based on speed
        outputPosition += currentPlaybackSpeed;

        // Output the pitch-shifted audio through DAC
        dac_output_voltage(DAC_CHANNEL_1, recordBuffer[outputPosition >> 7]);

        // Fill processing buffer for pitch detection when needed
        if (!bufferFilled)
        {
            processingBuffer[processingPosition++] = soundSample;

            // When buffer is full, signal pitch processing task
            if (processingPosition >= PROCESSING_BUFFER_SIZE)
            {
                processingPosition = 0;
                bufferFilled = true;
            }
        }

        // Ensure WDT doesn't time out
        esp_task_wdt_reset();
    }
}

void ReadMic(void *pvParams)
{
    esp_task_wdt_add(NULL);
    dac_output_enable(DAC_CHANNEL_1);
}

// Function to adjust manual pitch shift value
void setPitchShift(int8_t shift)
{
    manualPitchShift = shift;
}

void app_main()
{
    // Initialize Yin pitch detector
    Yin_init(&yinDetector, PITCH_DETECTION_THRESHOLD);

    // Create queue for communication between tasks
    audioQueue = xQueueCreate(10, sizeof(uint8_t));

    // Create tasks
    xTaskCreatePinnedToCore(&MicRead, "mic", 4096, NULL, 5, NULL, 1);            // Higher priority on core 1
    xTaskCreatePinnedToCore(&PitchProcessTask, "pitch", 4096, NULL, 3, NULL, 0); // Lower priority on core 0
    xTaskCreatePinnedToCore(&SwitchReadTask, "switch", 2048, NULL, 2, NULL, 0);  // Lowest priority on core 0

    // Example of changing pitch shift from main context (could be tied to buttons/encoder)
    setPitchShift(0); // Raise pitch slightly
}