#include <stdio.h>             /** C standard library */
#include <math.h>              /** Math functions */
#include "freertos/FreeRTOS.h" /** FreeRTOS libraries (tasks, delays, etc.) */
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h" /** GPIO control library */
#include <driver/dac.h>
#include <driver/adc.h>
#include "driver/i2s.h"
#include "esp_timer.h"
#include "driver/dac.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "Yin.h" /** Yin pitch detection algorithm */

#define MIC_INPUT_PIN 32
#define I2S_NUM I2S_NUM_0
#define SAMPLE_RATE 16000

uint16_t samplingRate = SAMPLE_RATE;
// For pitch detection and correction
#define PROCESSING_BUFFER_SIZE 512
#define PITCH_DETECTION_THRESHOLD 0.05f // Adjustable based on desired detection sensitivity
#define MIN_VALID_FREQUENCY 78.0f       // Minimum frequency to consider valid (in Hz)

// GPIO pin for mode switch (autotune/pitch shift)
#define MODE_SWITCH 4 // GPIO4 for mode switch (can be adjusted)

static const char *TAG = "AutoTune";
typedef enum
{
    MODE_AUTOTUNE = 0,    // Automatic pitch correction
    MODE_MANUAL_SHIFT = 1 // Manual pitch shifting
} OperationMode;

// Global variables
static Yin yinDetector;                                  // Yin pitch detector
static uint8_t processingBuffer[PROCESSING_BUFFER_SIZE]; // Buffer for audio analysis
static uint8_t recordBuffer[PROCESSING_BUFFER_SIZE];     // Circular buffer for audio samples
static volatile bool bufferFilled = false;
static volatile OperationMode currentMode = MODE_AUTOTUNE;
static int8_t manualPitchShift = 0;              // -128 to 127 for manual pitch shift
static volatile uint16_t inputPosition = 0;      // Position for writing new samples
static volatile uint16_t outputPosition = 0;     // Position for reading samples
static volatile uint16_t processingPosition = 0; // Position for filling the processing buffer
static volatile uint8_t playbackSpeed = 128;     // Neutral speed (no shift)
static volatile bool isrReady = false;           // Flag to indicate ISR is properly initialized

// Cache value for ADC channel to avoid runtime calls
static adc1_channel_t adc_channel = ADC1_CHANNEL_4;

// This function will be called at high frequency for audio sampling
// MUST be minimal and fast - no printf, ESP_LOG, or blocking calls
void IRAM_ATTR SampleTimerCallback(void *arg)
{
    if (!isrReady)
    {
        return;
    }

    // Use the regular API but make it faster with these tricks:
    // 1. Cache the ADC channel in a static variable
    // 2. Use a lower sample rate (8kHz instead of 20kHz)
    // 3. Use simplified processing
    int rawValue = adc1_get_raw(adc_channel);
    uint8_t soundSample = (uint8_t)(rawValue >> 4); // Convert to 8-bit

    // Store in record buffer
    recordBuffer[inputPosition] = soundSample;
    inputPosition = (inputPosition + 1) & (PROCESSING_BUFFER_SIZE - 1); // Wrap around

    // Adjust playback position based on speed
    outputPosition += playbackSpeed;

    // Output the pitch-shifted audio through DAC - use regular API
    dac_output_voltage(DAC_CHANNEL_1, recordBuffer[outputPosition >> 7]);

    // Fill processing buffer for pitch detection when needed - simplified logic
    if (!bufferFilled)
    {
        processingBuffer[processingPosition] = soundSample;
        processingPosition++;

        // When buffer is full, signal pitch processing task
        if (processingPosition >= PROCESSING_BUFFER_SIZE)
        {
            processingPosition = 0;
            bufferFilled = true;
        }
    }
}

// Task for pitch processing
void PitchProcessTask(void *pvParams)
{
    esp_task_wdt_add(NULL);

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
            if (currentMode == MODE_MANUAL_SHIFT)
            {
                playbackSpeed = (uint8_t)fmin(255, fmax(1, 128 + manualPitchShift));
            }

            // Reset buffer filled flag for next cycle
            bufferFilled = false;

            // Debug output (safe to do in a task)
            // ESP_LOGI(TAG, "Freq: %.2f, Target: %.2f, Speed: %d", frequency, targetFrequency, playbackSpeed);
        }

        // Give time for other tasks
        vTaskDelay(pdMS_TO_TICKS(10)); // 50Hz update rate
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
            currentMode = MODE_MANUAL_SHIFT;
        }

        // Check switch every 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Function to adjust manual pitch shift value
void setPitchShift(int8_t shift)
{
    manualPitchShift = shift;
}

void app_main()
{
    // Initialize Yin pitch detector with appropriate sampling rate
    Yin_init(&yinDetector, PITCH_DETECTION_THRESHOLD, samplingRate);

    // Configure ADC - use standard API but cache the channel
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(adc_channel, ADC_ATTEN_DB_11);

    // Enable DAC
    dac_output_enable(DAC_CHANNEL_1);

    // Initialize buffers to mid-level (128)
    for (int i = 0; i < PROCESSING_BUFFER_SIZE; i++)
    {
        recordBuffer[i] = 128;
        processingBuffer[i] = 128;
    }

    // Create tasks
    xTaskCreatePinnedToCore(&PitchProcessTask, "pitch", 4096, NULL, 3, NULL, 0); // Lower priority on core 0
    xTaskCreatePinnedToCore(&SwitchReadTask, "switch", 2048, NULL, 2, NULL, 0);  // Lowest priority on core 0

    // Create high-precision timer for audio sampling
    esp_timer_handle_t sampleTimer;
    const esp_timer_create_args_t sampleTimerconf = {
        .callback = &SampleTimerCallback,
        .name = "audio_sampler",
        .dispatch_method = ESP_TIMER_ISR // ISR for high performance
    };

    esp_timer_create(&sampleTimerconf, &sampleTimer);

    // Set initial shift
    setPitchShift(0);

    // Mark ISR as ready to process
    isrReady = true;

    // Start timer with calculated interval
    uint32_t timer_interval = 1000000 / SAMPLE_RATE;
    esp_timer_start_periodic(sampleTimer, timer_interval);
}