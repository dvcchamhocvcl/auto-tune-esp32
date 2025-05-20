#include <stdio.h>             /** C standard library */
#include <math.h>              /** Math functions */
#include "freertos/FreeRTOS.h" /** FreeRTOS libraries (tasks, delays, etc.) */
#include "freertos/task.h"
#include "driver/gpio.h" /** GPIO control library */
#include <driver/dac.h>
#include <driver/adc.h>
#include "esp_timer.h"
#include "sevenseg.h"
// #include "driver/dac.h"
// #include "esp_adc_cal.h"
#include "Yin.h" /** Yin pitch detection algorithm */

#define SAMPLE_RATE 20000
#define PITCH_DETECTION_THRESHOLD 0.05f // Adjustable based on desired detection sensitivity
#define MIN_VALID_FREQUENCY 62.0f       // Minimum frequency to consider valid (in Hz)
#define MODE_SWITCH 4                   // GPIO4 for mode switch (can be adjusted)

// Global variables
static Yin yinDetector;                       // Yin pitch detector
static uint8_t processingBuffer[BUFFER_SIZE]; // Buffer for audio analysis
static uint8_t recordBuffer[BUFFER_SIZE];     // Circular buffer for audio samples
static volatile bool bufferFilled = false;
static volatile uint16_t inputPosition = 0;      // Position for writing new samples
static volatile uint16_t outputPosition = 0;     // Position for reading samples
static volatile uint16_t processingPosition = 0; // Position for filling the processing buffer
static volatile uint8_t playbackSpeed = 128;     // Neutral speed (no shift)
static volatile bool isrReady = false;           // Flag to indicate ISR is properly initialized
uint16_t samplingRate = SAMPLE_RATE;
static adc1_channel_t adc_channel = ADC1_CHANNEL_4;
static volatile float targetFrequency = 0.0f;
static volatile float frequency = 0.0f;
static uint8_t tone_selection = 0; // Make sure this is accessible where needed

void IRAM_ATTR SampleInput(void *arg)
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
void PitchProcessTask(void *arg)
{
    while (1)
    {
        if (bufferFilled)
        {
            frequency = Yin_getPitch(&yinDetector, processingBuffer);
            if (frequency >= MIN_VALID_FREQUENCY)
            {
                targetFrequency = getNearestNoteFrequency(frequency, &yinDetector, tone_selection);
                // playbackSpeed = (uint8_t)fmin(255, fmax(1, round(targetFrequency * 128.0 / frequency)));
                playbackSpeed = (uint8_t)round(targetFrequency * 128.0 / frequency);
            }
            bufferFilled = false;
        }
        vTaskDelay(5);
    }
}

void display_7seg_task(void *arg)
{
    while (1)
    {
        for (int i = 0; i <= 6; i++)
        {
            display_7seg(i);
            vTaskDelay(50);
        }
    }
}

void button_task(void *arg)
{
    // Configure button GPIOs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 22) | (1ULL << 23),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Enable pull-up resistors
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    int last_state_22 = 1, last_state_23 = 1;

    while (1)
    {
        int state_22 = gpio_get_level(22);
        int state_23 = gpio_get_level(23);

        // Button pressed = logic LOW (assuming pull-up and button to GND)
        if (last_state_23 == 1 && state_23 == 0)
        {
            if (tone_selection == 11)
            {
                tone_selection = 0;
            }
            else
            {
                tone_selection++;
            }
            display_7seg(tone_selection);
            shift_scale_up(&yinDetector);
        }
        if (last_state_22 == 1 && state_22 == 0)
        {
            if (tone_selection == 0)
            {
                tone_selection = 11;
            }
            else
            {
                tone_selection--;
            }
            display_7seg(tone_selection);
            shift_scale_down(&yinDetector);
        }

        last_state_22 = state_22;
        last_state_23 = state_23;

        vTaskDelay(pdMS_TO_TICKS(10)); // 20ms debounce
    }
}

void app_main()
{
    init_7seg();
    display_7seg(0);
    // Initialize Yin pitch detector with appropriate sampling rate
    Yin_init(&yinDetector, PITCH_DETECTION_THRESHOLD, samplingRate);
    adc1_config_width(ADC_WIDTH_BIT_9);
    adc1_config_channel_atten(adc_channel, ADC_ATTEN_DB_11);
    dac_output_enable(DAC_CHANNEL_1);
    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        recordBuffer[i] = 128;
        processingBuffer[i] = 128;
    }

    esp_timer_handle_t sampleTimer;
    const esp_timer_create_args_t sampleTimerconf = {
        .callback = &SampleInput,
        .name = "audio_sampler",
        .dispatch_method = ESP_TIMER_ISR};

    // xTaskCreatePinnedToCore(display_7seg_task, "display_7seg_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(button_task, "button_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(PitchProcessTask, "PitchProcessTask", 4096, NULL, 5, NULL, 1);
    esp_timer_create(&sampleTimerconf, &sampleTimer);

    // Mark ISR as ready to process
    isrReady = true;

    // Start timer with calculated interval
    esp_timer_start_periodic(sampleTimer, 1000000 / SAMPLE_RATE);
}