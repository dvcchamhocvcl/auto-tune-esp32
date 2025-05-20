#include "sevenseg.h"
#include "freertos/FreeRTOS.h"
static const uint8_t hex_patterns[] = {
    SEG_A,
    SEG_AS,
    SEG_B,
    SEG_C,
    SEG_CS,
    SEG_D,
    SEG_DS,
    SEG_E,
    SEG_F,
    SEG_FS,
    SEG_G,
    SEG_GS};

void init_7seg()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SHIFT_DATA_PIN) |
                        (1ULL << SHIFT_CLOCK_PIN) |
                        (1ULL << SHIFT_LATCH_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Initialize all pins to low
    gpio_set_level(SHIFT_DATA_PIN, 0);
    gpio_set_level(SHIFT_CLOCK_PIN, 0);
    gpio_set_level(SHIFT_LATCH_PIN, 0);
}

void display_7seg(uint8_t value)
{
    uint8_t pattern = hex_patterns[value];

    // Latch low to prepare for new data
    gpio_set_level(SHIFT_LATCH_PIN, 0);

    // Shift out the pattern, MSB first
    for (int i = 0; i <= 7; i++)
    {
        // Set data bit
        gpio_set_level(SHIFT_DATA_PIN, (pattern >> i) & 0x01);
        gpio_set_level(SHIFT_CLOCK_PIN, 1);
        vTaskDelay(5);
        gpio_set_level(SHIFT_CLOCK_PIN, 0);
    }

    // Latch high to update display
    gpio_set_level(SHIFT_LATCH_PIN, 1);
}
