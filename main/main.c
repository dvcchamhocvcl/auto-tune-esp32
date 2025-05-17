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

#define SAMPLE_RATE 10000
#define PITCH_DETECTION_THRESHOLD 0.05f // Adjustable based on desired detection sensitivity
#define MIN_VALID_FREQUENCY 62.0f       // Minimum frequency to consider valid (in Hz)
#define MODE_SWITCH 4                   // GPIO4 for mode switch (can be adjusted)

typedef enum
{
    MODE_AUTOTUNE = 0,    // Automatic pitch correction
    MODE_MANUAL_SHIFT = 1 // Manual pitch shifting
} OperationMode;

// Global variables
static Yin yinDetector;                       // Yin pitch detector
static uint8_t processingBuffer[BUFFER_SIZE]; // Buffer for audio analysis
static uint8_t recordBuffer[BUFFER_SIZE];     // Circular buffer for audio samples
static volatile bool bufferFilled = false;
static volatile OperationMode currentMode = MODE_AUTOTUNE;
static volatile uint16_t inputPosition = 0;      // Position for writing new samples
static volatile uint16_t outputPosition = 0;     // Position for reading samples
static volatile uint16_t processingPosition = 0; // Position for filling the processing buffer
static volatile uint8_t playbackSpeed = 128;     // Neutral speed (no shift)
static volatile bool isrReady = false;           // Flag to indicate ISR is properly initialized
uint16_t samplingRate = SAMPLE_RATE;
static adc1_channel_t adc_channel = ADC1_CHANNEL_4;
static volatile float targetFrequency = 0.0f;
static volatile float frequency = 0.0f;

#define TAG "AutoTune"
// This function will be called at high frequency for audio sampling
// MUST be minimal and fast - no printf, ESP_LOG, or blocking calls
void IRAM_ATTR SampleTimerCallback(void *arg)
{
    if (!isrReady)
    {
        return;
    }
    int rawValue = adc1_get_raw(adc_channel);
    uint8_t soundSample = (uint8_t)(rawValue >> 1); // Convert to 8-bit

    // Store in record buffer
    recordBuffer[inputPosition] = soundSample;
    inputPosition = (inputPosition + 1) & (BUFFER_SIZE - 1); // Wrap around

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
        if (processingPosition >= BUFFER_SIZE)
        {
            processingPosition = 0;
            bufferFilled = true;
        }
    }
}

// Task for pitch processing
void IRAM_ATTR PitchProcessTask(void *arg)
{
    if (bufferFilled)
    {
        frequency = Yin_getPitch(&yinDetector, processingBuffer);
        if (frequency >= MIN_VALID_FREQUENCY)
        {
            if (currentMode == MODE_AUTOTUNE)
            {
                targetFrequency = getNearestNoteFrequency(frequency, &yinDetector);
                playbackSpeed = (uint8_t)fmin(255, fmax(1, round(targetFrequency * 128.0 / frequency)));
            }
        }
        else
        {
            // No valid pitch detected, use manual shift only
            playbackSpeed = (uint8_t)fmin(255, fmax(1, 128));
        }

        // In manual shift mode
        if (currentMode == MODE_MANUAL_SHIFT)
        {
            playbackSpeed = (uint8_t)fmin(255, fmax(1, 128));
        }

        // Reset buffer filled flag for next cycle
        bufferFilled = false;

        // Debug output (safe to do in a task)
        // ESP_LOGI(TAG, "Freq: %.2f, Target: %.2f, Speed: %d", frequency, targetFrequency, playbackSpeed);
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

void app_main()
{
    // Initialize Yin pitch detector with appropriate sampling rate
    Yin_init(&yinDetector, PITCH_DETECTION_THRESHOLD, samplingRate);

    // Configure ADC - use standard API but cache the channel
    adc1_config_width(ADC_WIDTH_BIT_9);
    adc1_config_channel_atten(adc_channel, ADC_ATTEN_DB_11);

    // Enable DAC
    dac_output_enable(DAC_CHANNEL_1);

    // Initialize buffers to mid-level (128)
    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        recordBuffer[i] = 128;
        processingBuffer[i] = 128;
    }

    // Create tasks
    // xTaskCreatePinnedToCore(&PitchProcessTask, "pitch", 4096, NULL, 3, NULL, 1); // Lower priority on core 0
    xTaskCreatePinnedToCore(&SwitchReadTask, "switch", 2048, NULL, 2, NULL, 0); // Lowest priority on core 0

    // Create high-precision timer for audio sampling
    esp_timer_handle_t sampleTimer;
    esp_timer_handle_t pitchTimer;
    const esp_timer_create_args_t sampleTimerconf = {
        .callback = &SampleTimerCallback,
        .name = "audio_sampler",
        .dispatch_method = ESP_TIMER_ISR};

    const esp_timer_create_args_t pitchTimerconf = {
        .callback = &PitchProcessTask,
        .name = "pitch_process",
        .dispatch_method = ESP_TIMER_TASK};

    esp_timer_create(&sampleTimerconf, &sampleTimer);
    esp_timer_create(&pitchTimerconf, &pitchTimer);

    // Mark ISR as ready to process
    isrReady = true;

    // Start timer with calculated interval
    esp_timer_start_periodic(sampleTimer, 1000000 / SAMPLE_RATE);
    esp_timer_start_periodic(pitchTimer, BUFFER_SIZE * 1000000 / SAMPLE_RATE);
}